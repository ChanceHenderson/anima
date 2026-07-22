//Anima
//A brushless flywheel blaster
//Based off of the Spirit codebase, by wonderboy
//Updated for Anima by Chance
//Software revision 1.0.0
//FOR RP2040

#include <Arduino.h>
#include "anima.h"
#include <Adafruit_SSD1306.h>
#include <bits/stdc++.h>
#include <ClickButton.h>
#include <PIO_DShot.h>
#include <Servo.h>
#include <stdio.h>
#include <Wire.h>
#include "FreeSansBoldOblique24pt7b.h"
#include "hardware/irq.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "hardware/watchdog.h"
#include <stdint.h>
#include <iostream>
#include <string>
#include <sstream>
#include "flywheelmotor.h"
#include "pidcontrol.h"
#include "basicpwmcontrol.h"
#include "esc_passthrough.h"

// Mutex for safe motor access between cores
auto_init_mutex(motor_mutex);
// Mutex for saving
auto_init_mutex(save_mutex);

typedef struct
{
    uint32_t timestamp;
    uint16_t left_rpm;
    uint16_t right_rpm;
    uint16_t left_throttle;
    uint16_t right_throttle;
} LogMessage;


void load_profile(uint8_t profile_num) 
{
  Profile loaded_profile;  
  // Try to load from filesystem
  if (LittleFS.begin()) 
  {
    Serial.println("LittleFS Begin successful");
    if (LittleFS.exists(PROFILE_PATHS[profile_num]))
    {
      Serial.println("Profile file exists");
      File f = LittleFS.open(PROFILE_PATHS[profile_num], "r");
      if (f)
      {
        if (f.size() == sizeof(Profile))
        {
          f.read((uint8_t*)&loaded_profile, sizeof(Profile));
          f.close();
          if (loaded_profile.profile_rpm >= MIN_RPM && loaded_profile.profile_rpm <= MAX_RPM &&
              loaded_profile.fire_rate >= MIN_FIRE_RATE && loaded_profile.fire_rate <= MAX_FIRE_RATE &&
              loaded_profile.post_shot_rev_ms >= MIN_POST_SHOT_REV_MS && loaded_profile.post_shot_rev_ms <= MAX_POST_SHOT_REV_MS)
          {
            LittleFS.end();
            // Apply settings
            profile_rpm = loaded_profile.profile_rpm;
            fire_rate = loaded_profile.fire_rate;
            post_shot_rev_duration_ms = loaded_profile.post_shot_rev_ms;
            use_idle = loaded_profile.use_idle;
            rev_is_auto = loaded_profile.rev_is_auto;
            return;
          }
          else
          {
            Serial.println("Profile file contains invalid values, using defaults");
          }
        }
        else if (f.size() != sizeof(Profile))
        {
          Serial.println("Profile file size incorrect, using defaults");
          f.close();
          LittleFS.remove(PROFILE_PATHS[profile_num]); // Delete corrupted file
        }
      }
      else
      {
        Serial.println("Profile file open failed, using defaults");
      }
      if (f) f.close();
    }
    LittleFS.end();
  }
  else
  {
    Serial.println("LittleFS Begin failed");
  }
  // If we get here, use defaults
  memcpy_P(&loaded_profile, &DEFAULT_PROFILES[profile_num], sizeof(Profile));
  profile_rpm = loaded_profile.profile_rpm;
  fire_rate = loaded_profile.fire_rate;
  post_shot_rev_duration_ms = loaded_profile.post_shot_rev_ms;
  use_idle = loaded_profile.use_idle;
  rev_is_auto = loaded_profile.rev_is_auto;
}

void check_battery()
{
  voltage_index++;
  voltage_readings[voltage_index%10] = (float)analogRead(PIN_VOLTAGE_IN) * 3.3 / 4096.0 * 11.0;;
  float average_voltage = 0.0;
  float divisor = (float)std::min(voltage_index, 10);
  for (int i = 0; i < divisor; i++) 
  {
      average_voltage += voltage_readings[i];
  }
  average_voltage /= divisor;
  voltage = average_voltage;
}

void save_current_profile(uint8_t profile_num) 
{
  Profile current_settings = {
    profile_rpm,
    fire_rate,
    post_shot_rev_duration_ms,
    use_idle,
    rev_is_auto
  };
  
  if (LittleFS.begin())
  {
    Serial.println("LittleFS Begin successful for save");
    File f = LittleFS.open(PROFILE_PATHS[profile_num], "w");
    if (f)
    {
      Serial.println("Profile file opened for writing");
      f.write((uint8_t*)&current_settings, sizeof(Profile));
      f.close();
    }
    LittleFS.end();
  }
  else
  {
    Serial.println("LittleFS Begin failed for save");
  }
}

void set_solenoid_throttle(uint16_t throttle) 
{
  mutex_enter_blocking(&motor_mutex);
  solenoid_throttle_target = throttle;
  mutex_exit(&motor_mutex);
}

void extendNoid() 
{
  if (USE_ESC_SOLENOID)
  {
    set_solenoid_throttle(nextSolenoidOn);
    if (nextSolenoidOn == SOLENOID_ON_HIGH) 
    {
        nextSolenoidOn = SOLENOID_ON_LOW;
    } 
    else
    {
        nextSolenoidOn = SOLENOID_ON_HIGH;
    }
  }
  else
  {
    digitalWrite(PIN_SOLENOID_MOSFET, HIGH);
  }
}

void retractNoid() 
{
  if (USE_ESC_SOLENOID)
  {
  set_solenoid_throttle(SOLENOID_OFF);
  }
  else
  {
    digitalWrite(PIN_SOLENOID_MOSFET, LOW);
  }
}

void fireNoid() 
{
    extendNoid();
    delay(noid_extend_ms);
    retractNoid();
    delay(noid_retract_ms);
}

void reset()
{
    // Use watchdog to reboot - handles multi-core cleanup automatically
    watchdog_reboot(0, 0, 0);
    while(1); // Wait for watchdog to trigger reset
}

void fire(bool rev_trigger_auto)
{
    fireNoid();
    last_fire_timestamp = millis();
    shot_count += 1;
    update_display = true;
    
    // Apply fire rate delay (only for non-full-auto modes)
    if (!rev_trigger_auto && (mode == SEMI || mode == BINARY))
    {
        delay(MAX_ROF_DELAY);
    }
    else if (rev_trigger_auto || mode == AUTO)
    {
        delay(single_shot_delay);
    }
}

// Safe wrapper functions for Core 1 to access motor data
void set_target_rpm(uint32_t rpm) 
{
    mutex_enter_blocking(&motor_mutex);
    if (left_motor != nullptr) 
    {
        left_motor->control_algorithm->set_target_rpm(rpm);
    }
    if (right_motor != nullptr) 
    {
        right_motor->control_algorithm->set_target_rpm(rpm);
    }
    mutex_exit(&motor_mutex);
}


void setup()
{
    // Core 0 runs setup() by default on Arduino
    Serial.ignoreFlowControl(true);
    Serial.printf("Core 0: Starting setup on Core 0\n");
    // If all three buttons are held, format LittleFS
    if (digitalRead(PIN_MENU_IN) == LOW && digitalRead(PIN_FIRE_IN) == LOW && digitalRead(PIN_REV_IN) == LOW) {
        impending_format = true;
    }
    if (digitalRead(PIN_MENU_IN) == LOW && digitalRead(PIN_FIRE_IN) == HIGH)
    {
      current_profile = PROFILE_LOW;
    } 
    else if (digitalRead(PIN_MENU_IN) == HIGH && digitalRead(PIN_FIRE_IN) == LOW) 
    {
      current_profile = PROFILE_MED;
    } 
    else if (digitalRead(PIN_MENU_IN) == LOW && digitalRead(PIN_FIRE_IN) == LOW) 
    {
      current_profile = PROFILE_T;
      menu.longClickTime = 500;
    }
    else
    {
      current_profile = PROFILE_HIGH;
    }
    load_profile(current_profile);
    Serial.printf("Core 0: Loaded profile %d\n", current_profile);
    if (tuning.load()) Serial.println("Core 0: Loaded PID tuning constants from flash");
    else Serial.println("Core 0: Using default PID tuning constants");
    if (noid_settings.load()) Serial.println("Core 0: Loaded solenoid settings from flash");
    else Serial.println("Core 0: Using default solenoid settings");
    // Launch Core 1 for UI and control logic
    multicore_launch_core1(core1_main);
}

// Initialize motors and solenoid (called from Core 1 after mode selection)
void initialize_motors() 
{
  left_motor = new FlywheelMotor(PIN_LEFT_MOTOR, MOTOR_POLES,  new PIDControl(tuning.left, THROTTLE_UPDATE_US));
  right_motor = new FlywheelMotor(PIN_RIGHT_MOTOR, MOTOR_POLES, new PIDControl(tuning.right, THROTTLE_UPDATE_US));
  if (USE_ESC_SOLENOID)
  {
    solenoid_dshot = new BidirDShotX1(PIN_SOLENOID_OUT, 600);
    solenoid_dshot->sendThrottle(SOLENOID_OFF);
  }
    // Enable extended telemetry on both motors
    for (int i = 0; i < 5000; i++) 
    {
        left_motor->get_dshot_instance()->sendThrottle(0);
        right_motor->get_dshot_instance()->sendThrottle(0);
        uint32_t left_packet;
        uint32_t right_packet;
        BidirDshotTelemetryType bdirtype_left = left_motor->get_dshot_instance()->getTelemetryRaw(&left_packet);
        BidirDshotTelemetryType bdirtype_right = right_motor->get_dshot_instance()->getTelemetryRaw(&right_packet);
        delayMicroseconds(200);
    }

    for (int i = 0; i < 20; i++) 
    {
        uint32_t left_packet;
        uint32_t right_packet;
        delayMicroseconds(100);
        BidirDshotTelemetryType bdirtype_left = left_motor->get_dshot_instance()->getTelemetryRaw(&left_packet);
        BidirDshotTelemetryType bdirtype_right = right_motor->get_dshot_instance()->getTelemetryRaw(&right_packet);        
        delayMicroseconds(200);
    }
    motors_initialized = true;
}


void rev_down() 
{
  if (use_idle && !presently_idling) 
  {
    presently_idling = true;
    set_target_rpm(IDLE_RPM);
  } 
  else if (!use_idle) 
  {
    set_target_rpm(0);
  }
  revved = false;
  last_rev_timestamp = millis();
  safety_timer = 0;
}

void main_loop() 
{
  if (low_batt) 
  {
    set_target_rpm(0);
  }
  //read button states
  trig.Update();
  menu.Update();
  rev.Update();
  //menu button handling
  //if menu button is held switch to settings screen. if it is pressed on the main screen, change fire mode
  if (!trig.depressed && menu.clicks != 0)
  {
    if (menu.clicks < 0)
    {
      update_display = true;
      settings = !settings;
      menu.clicks = 0;
      trig.clicks = 0;
      //update target RPM in case it was changed on settings screen
      if (!settings)
      {
        if (use_idle)
        {
          set_target_rpm(IDLE_RPM);
          presently_idling = true;
        }
      }
      else
      {
        set_target_rpm(0);  // Stop motors in settings mode
      }
      
    } 
    else if (menu.clicks > 0 && !settings && !lock)
    {
      update_display = true;
      mode++;

      if (mode > BINARY)
      {
        mode = SEMI;
      }
    }
  }

  //trigger button handling
  //if not in settings mode, handle rev and fire triggers
  if (!settings)
  {
    // ========== REV TRIGGER HANDLING ==========
    // Rev trigger held: spin up to profile RPM
    if (!rev_is_auto && rev.depressed && !menu.depressed && !low_batt)
    {
      manual_rev_active = true;
      if (use_idle && presently_idling)
      {
        // Tournament mode: transition from idle to full speed
        presently_idling = false;
      }
      set_target_rpm(profile_rpm);
    }
    else
    {
      manual_rev_active = false;
    }
    
    // ========== FIRE TRIGGER HANDLING ==========
    if (((rev_is_auto && rev.depressed) || trig.depressed) && !menu.depressed && safety_timer < REV_SAFETY_TIMEOUT && !low_batt) 
    {
      // Spin up if not already spun up
      if (!revved)
      {
        if (use_idle && presently_idling)
        {
          // Tournament mode: transition from idle to full speed
          presently_idling = false;
        }
        set_target_rpm(profile_rpm);
        
        // Wait for spinup
        unsigned long spinup_start = millis();
        while (current_rpm_left <= (profile_rpm - SPINUP_RPM_THRESHOLD) || 
               current_rpm_right <= (profile_rpm - SPINUP_RPM_THRESHOLD))
        {
          if (millis() - spinup_start > REV_FAIL_TIMER)
          {
            // Failed to spin up - abort
            rev_down();
            while (trig.depressed)
            {
              trig.Update();
              delay(5);
            }
            break;
          }
          delayMicroseconds(100);
        }
        
        // Check if we successfully spun up
        if (millis() - spinup_start <= REV_FAIL_TIMER)
        {
          revved = true;
          spin_down_timer = millis();
          safety_timer = millis() - last_rev_timestamp;
        }
      }
      else
      {
        // Already revved, update timers
        spin_down_timer = millis();
        safety_timer = millis() - last_rev_timestamp;
      }
      
      // Fire according to mode
      if (revved) 
      {
        if (rev_is_auto && rev.depressed)
        {
          // If rev trigger is also fire trigger, fire continuously while held
          fire(true);
        }
        else
        {
          switch (mode) 
          {
            case SEMI:
            case BINARY:
              // Fire once then set flag that prevents additional shots until trigger is released
              if (!fired) 
              {
                fire(false);
                fired = true;
              }
              break;
            case AUTO:
              // Full auto - fire continuously while trigger held
              fire(false);
              break;
          }
        }
      }
    } 
    else 
    {
      // Fire trigger not pressed
      
      // Safety timeout check
      if (safety_timer >= REV_SAFETY_TIMEOUT)
      {
        rev_down();
        while (trig.depressed)
        {
          trig.Update();
          delay(5);
        }
      }

      // Binary mode: fire on trigger release
      if (fired && revved && mode == BINARY) 
      {
        fire(false);
      }
      
      // Reset fired flag for next trigger pull
      fired = false;
      
      // ========== SPINDOWN LOGIC ==========
      // Keep wheels spun if:
      // - Rev trigger is held, OR
      // - Fire trigger is held, OR
      // - Within post_shot_rev_duration_ms of last fire
      bool should_stay_spun = manual_rev_active ||
                             trig.depressed ||
                             (millis() - last_fire_timestamp < post_shot_rev_duration_ms);
      
      if (!should_stay_spun)
      {
        rev_down();
      }
      
      retractNoid();
    }

    //if trigger is held whilst menu is held, lock/unlock the fire mode
    if (menu.depressed && trig.clicks < 0) 
    {
      update_display = true;
      lock = !lock;
      menu.clicks = 0;
      trig.clicks = 0;
    }

    //update battery voltage reading every BATTERY_CHECK_MS milliseconds
    //only when not revving to prevent low readings due to sag
    if (millis() % BATTERY_CHECK_MS == 0 && !revved) 
    {
      check_battery();
      if (voltage < LOW_BATTERY_THRESHOLD)
      {
        low_batt = true;
      }
      else
      {
        low_batt = false;
      }
      noid_extend_ms = constrain(map(voltage, 17.0, 14.6, noid_settings.ms_full, noid_settings.ms_dead),
                                 min(noid_settings.ms_full, noid_settings.ms_dead),
                                 max(noid_settings.ms_full, noid_settings.ms_dead));
    }

    if (millis() % SCREEN_UPDATE_MS == 0)
    {
      update_display = true;
    }

    //write the display buffer if flag is set & only when not revving
    //prevents screen write delay from affecting operation
    if (update_display && !revved)
    {
      display_main();
      update_display = false;
    }
  } 
  else 
  {
    //settings menu
    //update screen every second for runtime display
    if (millis() % 1000 == 0) 
    {
      update_display = true;
    }

    //menu press switches selected parameter
    if (menu.clicks > 0) 
    {
      if (trig.depressed)
      {
        // Reset current profile to defaults
        Profile defaultProfile;
        memcpy_P(&defaultProfile, &DEFAULT_PROFILES[current_profile], sizeof(Profile));

        // Apply default values
        profile_rpm = defaultProfile.profile_rpm;
        fire_rate = defaultProfile.fire_rate;
        post_shot_rev_duration_ms = defaultProfile.post_shot_rev_ms;

        // Save to EEPROM
        //EEPROM.put(current_profile * sizeof(Profile), defaultProfile);

        // Visual feedback
        oled.clearDisplay();
        oled.setCursor(0, 28);
        oled.print(F("Profile Reset!"));
        oled.display();
        delay(1000);  // Show message briefly

        update_display = true;
        menu.clicks = 0;  // Clear the click so it doesn't change selection
        trig.clicks = 0;
      }
      else 
      {
        update_display = true;
        selected++;

        if (selected > 10)
        {
          selected = 1;
        }
        menu.clicks = 0;
        trig.clicks = 0;
      }
    }

    //trigger press changes parameter value
    if (trig.clicks > 0)
    {
      update_display = true;
      switch (selected)
      {
        case 1:
          profile_rpm += RPM_STEP_SIZE;
          if (profile_rpm > MAX_RPM)
          {
            profile_rpm = MIN_RPM;
          }
          break;
        case 2:
          fire_rate += 1;
          if (fire_rate > MAX_FIRE_RATE)
          {
            fire_rate = MIN_FIRE_RATE;
          }
          single_shot_delay = max(ceil((1000/fire_rate) - (noid_extend_ms+noid_retract_ms)), 0);
          break;
        case 3:
          post_shot_rev_duration_ms += 100;
          if (post_shot_rev_duration_ms > MAX_POST_SHOT_REV_MS)
          {
            post_shot_rev_duration_ms = MIN_POST_SHOT_REV_MS;
          }
          break;
        case 4:
          use_idle = !use_idle;
          break;
        case 5:
          rev_is_auto = !rev_is_auto;
          break;
        case 6:
        {
          // Save current profile to filesystem
          save_requested = true;
          bool done = false;
          while (!done)
          {
            mutex_enter_blocking(&save_mutex);
            if (!save_requested) done = true;
            mutex_exit(&save_mutex);
            delay(10);
          }
          // Visual feedback
          oled.clearDisplay();
          oled.setCursor(0, 28);
          oled.print(F("Profile Saved!"));
          oled.display();
          delay(1000);  // Show message briefly
          break;
        }
        case 7:
          // PID autotune - takes over the screen and buttons until it finishes
          run_pid_autotune();
          trig.clicks = 0;
          menu.clicks = 0;
          rev.clicks = 0;
          update_display = true;
          break;
        case 8:
          noid_settings.ms_full += 1;
          if (noid_settings.ms_full > NOID_MS_MAX)
          {
            noid_settings.ms_full = NOID_MS_MIN;
          }
          break;
        case 9:
          noid_settings.ms_dead += 1;
          if (noid_settings.ms_dead > NOID_MS_MAX)
          {
            noid_settings.ms_dead = NOID_MS_MIN;
          }
          break;
        case 10:
        {
          // Save solenoid settings to filesystem
          noid_save_requested = true;
          bool done = false;
          while (!done)
          {
            mutex_enter_blocking(&save_mutex);
            if (!noid_save_requested) done = true;
            mutex_exit(&save_mutex);
            delay(10);
          }
          // Visual feedback
          oled.clearDisplay();
          oled.setCursor(0, 28);
          oled.print(F("Noid Saved!"));
          oled.display();
          delay(1000);  // Show message briefly
          break;
        }
      }
      menu.clicks = 0;
      trig.clicks = 0;
    }

    
    if (!DUAL_STAGE_TRIGGER)
    {
      //rev press changes parameter value the other way
      if (rev.clicks > 0)
      {
        update_display = true;
        switch (selected)
        {
          case 1:
            profile_rpm -= RPM_STEP_SIZE;
            if (profile_rpm < MIN_RPM)
            {
              profile_rpm = MAX_RPM;
            }
            break;
          case 2:
            fire_rate -= 1;
            if (fire_rate < MIN_FIRE_RATE)
            {
              fire_rate = MAX_FIRE_RATE;
            }
            single_shot_delay = max(ceil((1000/fire_rate) - (noid_extend_ms+noid_retract_ms)), 0);
            break;
          case 3:
            post_shot_rev_duration_ms -= 100;
            if (post_shot_rev_duration_ms < MIN_POST_SHOT_REV_MS)
            {
              post_shot_rev_duration_ms = MAX_POST_SHOT_REV_MS;
            }
            break;
          case 4:
            use_idle = !use_idle;
            break;
          case 5:
            rev_is_auto = !rev_is_auto;
            break;
          case 8:
            noid_settings.ms_full -= 1;
            if (noid_settings.ms_full < NOID_MS_MIN)
            {
              noid_settings.ms_full = NOID_MS_MAX;
            }
            break;
          case 9:
            noid_settings.ms_dead -= 1;
            if (noid_settings.ms_dead < NOID_MS_MIN)
            {
              noid_settings.ms_dead = NOID_MS_MAX;
            }
            break;
        }
        menu.clicks = 0;
        trig.clicks = 0;
        rev.clicks = 0;
      }

      // hold rev or trigger to rapidly change RPM setting
      if (rev.clicks < 0 && selected == 1)
      {
        while (rev.depressed)
        {
          profile_rpm -= RPM_STEP_SIZE;
          if (profile_rpm < MIN_RPM)
          {
            profile_rpm = MAX_RPM;
          }
          rev.Update();
          display_settings(selected);
          delay(30);
        }
      }
    }
    if (trig.clicks < 0 && selected == 1)
    {
      while (trig.depressed)
      {
        profile_rpm += RPM_STEP_SIZE;
        if (profile_rpm > MAX_RPM)
        {
          profile_rpm = MIN_RPM;
        }
        trig.Update();
        display_settings(selected);
        delay(30);
      }
    }
    retractNoid();
    shot_count = 0;

    if (update_display)
    {
      display_settings(selected);
      update_display = false;
    }
  }
}

void rev_up() 
{
  // Transition from idle to full speed in tournament mode
  if (use_idle && presently_idling) 
  {
    presently_idling = false;
    set_target_rpm(profile_rpm);
  }
  
  if (!revved) 
  {
    // Record trigger down event timestamp
    unsigned long last_trigger_down_ms = millis();

    // Wait for motors to reach speed
    bool fail = false;
    
    while (current_rpm_left <= (profile_rpm - SPINUP_RPM_THRESHOLD) || 
           current_rpm_right <= (profile_rpm - SPINUP_RPM_THRESHOLD)) 
    {
      if (millis() - last_trigger_down_ms > REV_FAIL_TIMER) 
      {
        fail = true;
        break;
      }
      delayMicroseconds(100);
    }
    
    if (fail) 
    {
      // Failed to reach speed setpoint in time - shut down motors
      rev_down();
      while (trig.depressed) 
      {
        trig.Update();
        delay(5);
      }
    } 
    else 
    {
      // Successfully spun up
      revved = true;
      spin_down_timer = millis();
      safety_timer = millis() - last_rev_timestamp;
    }
  } 
  else 
  {
    // Already revved, just update timers
    spin_down_timer = millis();
    safety_timer = millis() - last_rev_timestamp;
  }
}

//main (firing) screen display output
void display_main()
{
  byte rpm_thousands = profile_rpm / 1000;
  byte rpm_hundreds = (profile_rpm % 1000) / 100;
  byte countH = 60;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print(noid_extend_ms);
  oled.print("ms");

  oled.drawFastHLine(55, 1, 23, 1);
  oled.drawFastHLine(55, 7, 23, 1);
  oled.drawFastVLine(54, 2, 5, 1);
  oled.drawFastVLine(78, 2, 5, 1);
  oled.drawFastVLine(79, 3, 2, 1);
  int battery_ticks = min(10, (int)((voltage - 14.8) / 0.2));
  for (int b = 0; b < battery_ticks; b++)
  {
    oled.drawFastVLine(57 + (b * 2), 3, 2, 1);
  }

  oled.setCursor(99, 0);
  oled.print(voltage, 1);
  oled.print(F("V"));
  oled.drawFastHLine(0, 9, 128, 1);

  if (!low_batt) {
    oled.setFont(&FreeSansBoldOblique24pt7b);
    countH = 50;
    if (shot_count > 999) shot_count = 0;
    if (shot_count > 9) countH -= 14;
    if (shot_count > 99) countH -= 14;
    oled.setCursor(countH, 47);
    oled.print(shot_count);
  } 
  else
  {
    oled.setCursor(0, 26);
    oled.print(F("BATTERY\n CRITICAL!"));
  }

  countH = 80;
  oled.setFont();
  oled.drawFastHLine(0, 53, 128, 1);
  int throttle_thousands = profile_rpm / 1000;
  int throttle_hundreds = (profile_rpm % 1000) / 100;
  if (low_batt) 
  {
    throttle_thousands = 0;
  }
  if (throttle_thousands > 9)
  {
    countH -= 6;
  }
  oled.setCursor(countH, 56);
  oled.print(throttle_thousands);
  oled.print(F("."));
  oled.print(throttle_hundreds);
  oled.print(F("K RPM"));
  oled.setCursor(0, 56);

  if (!low_batt)
  {
    switch (mode)
    {
      case SEMI:
        oled.print(F("Semi"));
        break;
      case AUTO:
        oled.print(F("Auto"));
        break;
      case BINARY:
        oled.print(F("Binary"));
        break;
    }
    if (lock)
    {
      oled.print(F("*"));
    }
  }
  else
  {
    oled.print(F("STOP"));
  }
  oled.display();
}

//settings screen display output
void display_settings(byte selected)
{
  if (selected > 7)
  {
    // page 2: solenoid settings
    oled.clearDisplay();
    oled.setFont();
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print(F("Noid ms (full): "));
    oled.setCursor(96, 0);
    if (selected == 8) oled.setTextColor(0, 1);
    oled.print(noid_settings.ms_full);
    oled.setTextColor(1, 0);
    oled.setCursor(0, 9);
    oled.print(F("Noid ms (dead): "));
    oled.setCursor(96, 9);
    if (selected == 9) oled.setTextColor(0, 1);
    oled.print(noid_settings.ms_dead);
    oled.setTextColor(1, 0);
    oled.setCursor(0, 18);
    oled.print(F("Save noid cfg: "));
    oled.setCursor(96, 18);
    if (selected == 10) oled.setTextColor(0, 1);
    oled.print(F("Save"));
    oled.setTextColor(1, 0);
    oled.setCursor(0, 54);
    oled.print(F("Page 2/2"));
    oled.display();
    return;
  }
  oled.clearDisplay();
  oled.setFont();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print(F("RPM: "));
  oled.setCursor(96, 0);
  if (selected == 1) oled.setTextColor(0, 1);
  oled.print(profile_rpm);
  oled.setTextColor(1, 0);
  oled.setCursor(0, 9);
  oled.print(F("Fire Rate: "));
  oled.setCursor(96, 9);
  if (selected == 2) oled.setTextColor(0, 1);
  oled.print(fire_rate);
  oled.setTextColor(1, 0);
  oled.setCursor(0, 18);
  oled.print(F("Post-fire rev: "));
  oled.setCursor(96, 18);
  if (selected == 3) oled.setTextColor(0, 1);
  oled.print(post_shot_rev_duration_ms);
  oled.setTextColor(1, 0);
  oled.setCursor(0, 27);
  oled.print(F("Idle: "));
  oled.setCursor(96, 27);
  if (selected == 4) oled.setTextColor(0, 1);
  if (use_idle) oled.print(F("Yes"));
  else oled.print(F("No"));
  oled.setTextColor(1, 0);
  oled.setCursor(0, 36);
  oled.print(F("Rev trigger auto: "));
  oled.setCursor(96, 36);
  if (selected == 5) oled.setTextColor(0, 1);
  if (rev_is_auto) oled.print(F("Yes"));
  else oled.print(F("No"));
  oled.setTextColor(1, 0);
  oled.setCursor(0, 45);
  oled.print(F("Save Profile: "));
  oled.setCursor(96, 45);
  if (selected == 6) oled.setTextColor(0, 1);
  oled.print(F("Save"));
  oled.setTextColor(1, 0);
  oled.setCursor(0, 54);
  oled.print(F("Tune PID: "));
  oled.setCursor(96, 54);
  if (selected == 7) oled.setTextColor(0, 1);
  oled.print(F("Go"));
  oled.setTextColor(1, 0);
  oled.display();
}

void core1_main()
{
  if (!USE_ESC_SOLENOID)
  {
    pinMode(PIN_SOLENOID_MOSFET, OUTPUT);
    digitalWrite(PIN_SOLENOID_MOSFET, LOW);
  }
  menu.debounceTime = 20;
  trig.debounceTime = 5;
  rev.debounceTime = 5;
  trig.longClickTime = 332;   //how long to wait before locking fire mode when menu and trigger held
  menu.longClickTime = 1500;  //how long to wait before switching to settings mode when menu button held
  trig.multiclickTime = 0;
  menu.multiclickTime = 0;
  rev.multiclickTime = 0;
  analogReadResolution(12);
  //startup animation
  Wire1.setSDA(PIN_OLED_SDA);
  Wire1.setSCL(PIN_OLED_SCL);
  Wire1.begin();
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.setTextColor(1);
  oled.setTextWrap(false);
  if (impending_format) 
  {
    oled.setTextSize(1);
    oled.clearDisplay();
    oled.setCursor(0, 12);
    oled.print(F("All triggers held"));
    oled.setCursor(0, 26);
    oled.print(F("Profiles will reset"));
    oled.setCursor(0, 40);
    oled.print(F("in 5 seconds!"));
    oled.setCursor(0, 54);
    oled.print(F("Turn off now to stop."));
    oled.display();
    delay(5000);
    // Signal core 0 to perform the format. Use volatile flag for cross-core sync.
    mutex_enter_blocking(&save_mutex);
    format_requested = true;
    mutex_exit(&save_mutex);
    // Wait for core 0 to clear the flag. Poll without taking the mutex to avoid deadlock
    while (format_requested)
    {
      delay(100);
    }
    if (format_successful == 1)
    {
      // Visual feedback for format
      oled.clearDisplay();
      oled.setCursor(0, 24);
      oled.print(F("Defaults Restored!"));
      oled.setCursor(0, 38);
      oled.print(F("Rebooting...."));
      oled.display();
      delay(5000);
      reset(); 
    }
    else if (format_successful == 2)
    {
      oled.clearDisplay();
      oled.setCursor(0, 24);
      oled.print(F("Format failed!"));
      oled.setCursor(0, 38);
      oled.print(F("Rebooting...."));
      oled.display();
      delay(5000);
      reset(); 
    }
    
  }
  oled.setTextSize(2);
  oled.setCursor(0, 0);
  oled.clearDisplay();
  oled.drawBitmap(0, 0, splash, 128, 64, 1);
  oled.setCursor(8, 40);
  oled.display();
  String str;
  switch (current_profile) 
  {
    case PROFILE_LOW:
      str = "Low  Power";
      break;
    case PROFILE_MED:
      str = "Mid  Power";
      break;
    case PROFILE_T:
      str = "Tournament";
      break;
    case PROFILE_HIGH:
    default:
      str = "High Power";
      break;
  }
  if (digitalRead(PIN_REV_IN) == LOW)
  {
    oled.clearDisplay();
    oled.setFont();
    oled.setTextSize(1);
    oled.setCursor(0, 20);
    oled.print("Passthrough mode");
    oled.setCursor(0, 50);
    oled.print("Menu to reset");
    oled.display();
    
    attachInterrupt(digitalPinToInterrupt(PIN_MENU_IN), reset, FALLING);
    // Passthrough mode - don't initialize DSHOT motors
    uint8_t pins[4] = {PIN_ESC_CHANNEL_1, PIN_ESC_CHANNEL_2, PIN_ESC_CHANNEL_3, PIN_ESC_CHANNEL_4};
    while(true)
    {
        beginPassthrough(pins, 4);
        while (processPassthrough()) {}
    }
  } 
  oled.print(str);
  oled.display();
  oled.clearDisplay();
  initialize_motors();
  noid_extend_ms = constrain(map(voltage, 17.0, 14.6, noid_settings.ms_full, noid_settings.ms_dead),
                             min(noid_settings.ms_full, noid_settings.ms_dead),
                             max(noid_settings.ms_full, noid_settings.ms_dead));
  if (use_idle)
  {
    set_target_rpm(IDLE_RPM);
    presently_idling = true;
  } 
  while (true) 
  {
    main_loop();
  }
} 


// Core 0 loop - continuously send motor throttle commands
void loop()
{
    // Allow formatting/save requests to be handled even before motors are initialized
    mutex_enter_blocking(&save_mutex);
    if (save_requested)
    {
        save_current_profile(current_profile);
        save_requested = false;
    }
    if (tuning_save_requested)
    {
        if (tuning.save()) Serial.println("PID tuning constants saved");
        else Serial.println("PID tuning constants save FAILED");
        tuning_save_requested = false;
    }
    if (noid_save_requested)
    {
        if (noid_settings.save()) Serial.println("Solenoid settings saved");
        else Serial.println("Solenoid settings save FAILED");
        noid_save_requested = false;
    }
    if (format_requested)
    {
      if (LittleFS.begin())
      {
        LittleFS.format();
        LittleFS.end();
        format_successful = 1;
      }
      else
      {
        format_successful = 2;
      }
      format_requested = false;
    }
    mutex_exit(&save_mutex);
    if (!motors_initialized) 
    {
        delay(100);
        return;
    }
  // Core 0's only job: send throttle to motors every 200us
  // Use mutex to safely access motors while Core 1 may be setting target RPM
    mutex_enter_blocking(&motor_mutex);
    if (left_motor != nullptr) 
    {
        left_motor->send_throttle();
        current_rpm_left = left_motor->get_current_rpm();
    }
    if (right_motor != nullptr) 
    {
        right_motor->send_throttle();
        current_rpm_right = right_motor->get_current_rpm();
    }
  if (solenoid_dshot != nullptr)
  {
    solenoid_dshot->sendThrottle(solenoid_throttle_target);
  }
    mutex_exit(&motor_mutex);
    delayMicroseconds(200);
}

/////////////////////////////// PID AUTOTUNE ///////////////////////////////
// Runs on core 1 (the UI core). Core 0 keeps streaming throttle packets the
// whole time, so the tuner works by swapping control algorithms in and out
// under the motor mutex and watching the RPM values core 0 publishes.
//
// Each wheel is tuned independently, but always with BOTH wheels spinning:
// the motors share one battery, so the combined draw is part of the plant
// being tuned.
//
// Sequence (trigger = consent, menu = abort, at every step):
//   1/3  Feed-forward sweep: 9 open-loop throttle points, log-fit each
//        wheel's throttle->RPM curve
//   2/3  Relay tests: bang-bang both wheels around each grid target
//        (nominally 12k/20k/30k RPM, limited to what the wheels can reach)
//        to measure each wheel's ultimate gain Ku and period Tu, then
//        Ziegler-Nichols "some overshoot" gains per wheel per target.
//        The result is a gain grid; PIDControl interpolates it for whatever
//        RPM a profile asks for (gain scheduling).
//   3/3  Step-response check at the highest grid target (the hardest case
//        for oscillation). Backs a wheel's gains off if it overshoots or
//        oscillates, bumps them up if sluggish.
// The result is shown and only written to flash if the user accepts it.

#define TUNE_MAX_RPM 45000            // hard abort if either wheel exceeds this
#define TUNE_MIN_VOLTAGE 13.6f        // hard abort if the pack sags below this mid-spin
#define TUNE_FF_STABILIZE_MS 1500     // per-point settling time in the feed-forward sweep
#define TUNE_FF_SAMPLE_MS 300         // per-point measurement time
#define TUNE_RELAY_CROSSINGS 8        // up-crossings recorded (early cycles are discarded)
#define TUNE_RELAY_TIMEOUT_MS 12000
#define TUNE_STEP_SAMPLE_MS 1500      // step-response observation window
#define TUNE_MAX_VERIFY_ITERATIONS 3
#define TUNE_GRID_MAX_FRACTION 0.85f  // grid targets must sit below this fraction of measured max RPM
#define TUNE_MIN_TARGET 8000
static const uint32_t TUNE_GRID_NOMINAL[MAX_GAIN_POINTS] = {12000, 20000, 30000};

typedef struct
{
    float os_pct;          // peak above target during spinup
    float band_pct;        // peak-to-peak wobble over the last 300 ms, % of target
    uint32_t t90_ms;       // time to first reach 90% of target
} StepMetrics;

// One wheel's relay-test state
typedef struct
{
    float base;            // feed-forward throttle at the target
    float h;               // relay amplitude
    bool high;
    bool done;
    int crossings;
    uint32_t crossing_us[TUNE_RELAY_CROSSINGS];
    uint32_t cyc_min[TUNE_RELAY_CROSSINGS];
    uint32_t cyc_max[TUNE_RELAY_CROSSINGS];
    uint32_t cur_min;
    uint32_t cur_max;
    float Ku;
    float Tu;
} RelayChannel;

static const char* tune_abort_reason = nullptr;

// Swap both motors' control algorithms under the mutex and free the old ones.
// Core 0 only touches control_algorithm inside the same mutex, so this is safe
// without pausing the motor loop.
static void tune_swap_algorithms(FlywheelControlAlgorithm* new_left, FlywheelControlAlgorithm* new_right)
{
    mutex_enter_blocking(&motor_mutex);
    FlywheelControlAlgorithm* old_left = left_motor->control_algorithm;
    FlywheelControlAlgorithm* old_right = right_motor->control_algorithm;
    left_motor->control_algorithm = new_left;
    right_motor->control_algorithm = new_right;
    mutex_exit(&motor_mutex);
    delete old_left;
    delete old_right;
}

static void tune_set_throttle_lr(uint32_t throttle_left, uint32_t throttle_right)
{
    mutex_enter_blocking(&motor_mutex);
    left_motor->control_algorithm->set_throttle(throttle_left);
    right_motor->control_algorithm->set_throttle(throttle_right);
    mutex_exit(&motor_mutex);
}

static void tune_set_throttle(uint32_t throttle)
{
    tune_set_throttle_lr(throttle, throttle);
}

// Safety net for every loop that has the wheels moving:
// overspeed, user abort via menu button, and battery collapse.
static bool tune_guard()
{
    static uint32_t last_batt_check = 0;
    if (current_rpm_left > TUNE_MAX_RPM || current_rpm_right > TUNE_MAX_RPM)
    {
        tune_abort_reason = "Overspeed!";
        return false;
    }
    if (digitalRead(PIN_MENU_IN) == LOW)
    {
        tune_abort_reason = "User abort";
        return false;
    }
    if (millis() - last_batt_check >= BATTERY_CHECK_MS)
    {
        last_batt_check = millis();
        check_battery();
        // lower bar than LOW_BATTERY_THRESHOLD - healthy packs sag under load
        if (voltage < TUNE_MIN_VOLTAGE)
        {
            tune_abort_reason = "Battery sagged";
            return false;
        }
    }
    return true;
}

static void tune_screen(const char* l1, const char* l2, const char* l3, const char* l4, const char* footer)
{
    oled.clearDisplay();
    oled.setFont();
    oled.setTextSize(1);
    if (l1) { oled.setCursor(0, 0);  oled.print(l1); }
    if (l2) { oled.setCursor(0, 14); oled.print(l2); }
    if (l3) { oled.setCursor(0, 28); oled.print(l3); }
    if (l4) { oled.setCursor(0, 41); oled.print(l4); }
    if (footer) { oled.setCursor(0, 54); oled.print(footer); }
    oled.display();
}

static void tune_wait_release()
{
    while (digitalRead(PIN_FIRE_IN) == LOW || digitalRead(PIN_MENU_IN) == LOW) delay(10);
    delay(50); // debounce
}

// Wait on whatever is on screen. Trigger = yes, menu = no/abort.
static bool tune_consent_wait()
{
    tune_wait_release();
    while (true)
    {
        if (digitalRead(PIN_FIRE_IN) == LOW) { tune_wait_release(); return true; }
        if (digitalRead(PIN_MENU_IN) == LOW) { tune_wait_release(); tune_abort_reason = "User abort"; return false; }
        delay(10);
    }
}

static bool tune_consent(const char* l1, const char* l2, const char* l3, const char* l4 = nullptr)
{
    tune_screen(l1, l2, l3, l4, "Trig=GO   Menu=QUIT");
    return tune_consent_wait();
}

// Phase 1: open-loop throttle sweep with both wheels spinning; log-fit
// RPM = a*ln(throttle) + b for each wheel separately.
static bool tune_ff_sweep(TuningConstants &cand, uint32_t &max_rpm_left, uint32_t &max_rpm_right)
{
    const int num_points = 9;
    int throttles[num_points];
    int rpms_left[num_points];
    int rpms_right[num_points];

    for (int i = 0; i < num_points; i++)
    {
        int throttle = (i + 1) * 200;
        throttles[i] = throttle;

        oled.clearDisplay();
        oled.setFont();
        oled.setTextSize(1);
        oled.setCursor(0, 0);  oled.print(F("1/3 Feed-forward"));
        oled.setCursor(0, 14); oled.print(F("Point ")); oled.print(i + 1); oled.print(F("/9 Thr ")); oled.print(throttle);
        oled.setCursor(0, 54); oled.print(F("Menu=ABORT"));
        oled.display();

        tune_set_throttle(throttle);

        uint32_t t0 = millis();
        while (millis() - t0 < TUNE_FF_STABILIZE_MS)
        {
            if (!tune_guard()) return false;
            delay(5);
        }

        // sample every 5 ms so the average spans many telemetry updates
        uint32_t sum_left = 0, sum_right = 0;
        int samples = 0;
        t0 = millis();
        while (millis() - t0 < TUNE_FF_SAMPLE_MS)
        {
            if (!tune_guard()) return false;
            sum_left += current_rpm_left;
            sum_right += current_rpm_right;
            samples++;
            delay(5);
        }
        rpms_left[i] = sum_left / samples;
        rpms_right[i] = sum_right / samples;

        if (i == 0 && (rpms_left[0] < 500 || rpms_right[0] < 500))
        {
            tune_abort_reason = "No RPM signal";
            return false;
        }
        // wheels may differ, but a large gap at the same throttle means a
        // dying motor/ESC, junk telemetry, or something rubbing
        int32_t diff = (int32_t)rpms_left[i] - (int32_t)rpms_right[i];
        if (diff < 0) diff = -diff;
        if (diff > (rpms_left[i] + rpms_right[i]) / 8)
        {
            tune_abort_reason = "Wheel mismatch";
            return false;
        }
        if (i > 0 && (rpms_left[i] <= rpms_left[i - 1] || rpms_right[i] <= rpms_right[i - 1]))
        {
            tune_abort_reason = "RPM not rising";
            return false;
        }
    }
    tune_set_throttle(0);

    std::pair<double, double> fit_left = fitLog(throttles, rpms_left, num_points);
    std::pair<double, double> fit_right = fitLog(throttles, rpms_right, num_points);
    if (fit_left.first < 500.0 || fit_left.first > 100000.0 ||
        fit_right.first < 500.0 || fit_right.first > 100000.0)
    {
        tune_abort_reason = "Bad FF fit";
        return false;
    }
    cand.left.ff_exponent = (float)fit_left.first;
    cand.left.ff_offset = -(float)fit_left.second;
    cand.right.ff_exponent = (float)fit_right.first;
    cand.right.ff_offset = -(float)fit_right.second;
    max_rpm_left = rpms_left[num_points - 1];
    max_rpm_right = rpms_right[num_points - 1];
    return true;
}

// Set up one wheel's relay channel. Returns false if there isn't enough
// throttle headroom to oscillate around this target.
static bool tune_relay_setup(RelayChannel &ch, const MotorTune &mt, uint32_t target)
{
    ch.base = mt.feed_forward((float)target);
    // Relay amplitude: 30% of the baseline, limited by the throttle range.
    // On these log-shaped curves a 12k target can sit near base=190, so the
    // low side is allowed down to 100 - the wheel only visits it briefly at
    // the bottom of each oscillation.
    ch.h = 0.30f * ch.base;
    if (ch.h > 1900.0f - ch.base) ch.h = 1900.0f - ch.base;
    if (ch.h > ch.base - 100.0f)  ch.h = ch.base - 100.0f;
    ch.high = true;
    ch.done = false;
    ch.crossings = 0;
    ch.cur_min = UINT32_MAX;
    ch.cur_max = 0;
    ch.Ku = 0.0f;
    ch.Tu = 0.0f;
    return (ch.h >= 20.0f && ch.base >= 130.0f && ch.base <= 1850.0f);
}

// Advance one wheel's relay state machine; returns the throttle to command.
static uint32_t tune_relay_step(RelayChannel &ch, uint32_t rpm, uint32_t target)
{
    if (ch.done) return (uint32_t)ch.base;  // hold steady while the other wheel finishes
    if (rpm < ch.cur_min) ch.cur_min = rpm;
    if (rpm > ch.cur_max) ch.cur_max = rpm;
    if (!ch.high && rpm + 100 < target)
    {
        ch.high = true;
    }
    else if (ch.high && rpm > target + 100)
    {
        ch.crossing_us[ch.crossings] = micros();
        ch.cyc_min[ch.crossings] = ch.cur_min;
        ch.cyc_max[ch.crossings] = ch.cur_max;
        ch.crossings++;
        ch.cur_min = UINT32_MAX;
        ch.cur_max = 0;
        ch.high = false;
        if (ch.crossings >= TUNE_RELAY_CROSSINGS) ch.done = true;
    }
    return (uint32_t)(ch.high ? ch.base + ch.h : ch.base - ch.h);
}

// Average the settled cycles into Ku and Tu.
static bool tune_relay_finish(RelayChannel &ch)
{
    float period_sum = 0.0f;
    float amp_sum = 0.0f;
    int n = 0;
    for (int i = 3; i < TUNE_RELAY_CROSSINGS; i++)
    {
        period_sum += (float)(ch.crossing_us[i] - ch.crossing_us[i - 1]) * 1e-6f;
        amp_sum += (float)(ch.cyc_max[i] - ch.cyc_min[i]) / 2.0f;
        n++;
    }
    ch.Tu = period_sum / n;
    float amp = amp_sum / n;
    if (amp < 100.0f || ch.Tu < 0.004f || ch.Tu > 2.0f) return false;
    // Describing-function ultimate gain, corrected for the +/-100 RPM
    // switching hysteresis (amp always includes the hysteresis band, so the
    // raw formula would underestimate Ku for small oscillations)
    float amp_eff = sqrtf(amp * amp - 100.0f * 100.0f);
    if (amp_eff < 30.0f) return false; // oscillation buried in the hysteresis band
    ch.Ku = 4.0f * ch.h / (PI * amp_eff);
    return true;
}

// Ziegler-Nichols "some overshoot" rule - biased toward reaction time.
// Kd uses the classic Tu/8 rather than Tu/3: derivative of quantized eRPM
// is noisy, so we take the milder braking action.
static GainPoint tune_zn_gains(const RelayChannel &ch, uint32_t target)
{
    GainPoint g;
    g.rpm = (float)target;
    g.kp = 0.33f * ch.Ku;
    if (g.kp > 10.0f) g.kp = 10.0f;
    g.ki = 2.0f * g.kp / ch.Tu;         // Ti = Tu/2
    if (g.ki > 200.0f) g.ki = 200.0f;
    g.kd = g.kp * ch.Tu / 8.0f;         // Td = Tu/8
    if (g.kd > 0.005f) g.kd = 0.005f;
    return g;
}

// Phase 2 (one grid point): run both wheels' relays in parallel and store
// the resulting gains into each wheel's grid.
static bool tune_relay_point(TuningConstants &cand, uint32_t target, int point_index)
{
    RelayChannel chl, chr;
    if (!tune_relay_setup(chl, cand.left, target) || !tune_relay_setup(chr, cand.right, target))
    {
        tune_abort_reason = "Relay range bad";
        return false;
    }

    char line[24];
    snprintf(line, sizeof(line), "%lu RPM...", (unsigned long)target);
    tune_screen("2/3 Relay test", line, nullptr, nullptr, "Menu=ABORT");

    uint32_t t0 = millis();
    while (!(chl.done && chr.done))
    {
        if (!tune_guard()) { tune_set_throttle(0); return false; }
        if (millis() - t0 > TUNE_RELAY_TIMEOUT_MS)
        {
            tune_set_throttle(0);
            tune_abort_reason = "Relay timeout";
            return false;
        }
        uint32_t throttle_left = tune_relay_step(chl, current_rpm_left, target);
        uint32_t throttle_right = tune_relay_step(chr, current_rpm_right, target);
        tune_set_throttle_lr(throttle_left, throttle_right);
        delayMicroseconds(500);
    }
    tune_set_throttle(0);

    if (!tune_relay_finish(chl) || !tune_relay_finish(chr))
    {
        tune_abort_reason = "Osc unusable";
        return false;
    }
    cand.left.points[point_index] = tune_zn_gains(chl, target);
    cand.right.points[point_index] = tune_zn_gains(chr, target);
    return true;
}

// Phase 3: one closed-loop step test, metrics recorded per wheel.
// The candidate PIDControl instances must already be installed.
static bool tune_step_response(uint32_t target, StepMetrics &ml, StepMetrics &mr)
{
    // make sure we're starting from (near) rest
    uint32_t t0 = millis();
    while (current_rpm_left > 1000 || current_rpm_right > 1000)
    {
        if (!tune_guard()) return false;
        if (millis() - t0 > 4000) { tune_abort_reason = "No spindown"; return false; }
        delay(10);
    }

    ml.t90_ms = 0;
    mr.t90_ms = 0;
    uint32_t peak_left = 0, peak_right = 0;
    uint32_t band_min_left = UINT32_MAX, band_max_left = 0;
    uint32_t band_min_right = UINT32_MAX, band_max_right = 0;

    set_target_rpm(target);
    t0 = millis();
    while (millis() - t0 < TUNE_STEP_SAMPLE_MS)
    {
        if (!tune_guard()) { set_target_rpm(0); return false; }
        uint32_t rpm_left = current_rpm_left;
        uint32_t rpm_right = current_rpm_right;
        uint32_t elapsed = millis() - t0;
        if (rpm_left > peak_left) peak_left = rpm_left;
        if (rpm_right > peak_right) peak_right = rpm_right;
        if (ml.t90_ms == 0 && rpm_left >= (target * 9) / 10) ml.t90_ms = elapsed;
        if (mr.t90_ms == 0 && rpm_right >= (target * 9) / 10) mr.t90_ms = elapsed;
        if (elapsed >= TUNE_STEP_SAMPLE_MS - 300)
        {
            if (rpm_left < band_min_left) band_min_left = rpm_left;
            if (rpm_left > band_max_left) band_max_left = rpm_left;
            if (rpm_right < band_min_right) band_min_right = rpm_right;
            if (rpm_right > band_max_right) band_max_right = rpm_right;
        }
        delayMicroseconds(500);
    }
    set_target_rpm(0);

    if (ml.t90_ms == 0 || mr.t90_ms == 0)
    {
        tune_abort_reason = "Never hit 90%";
        return false;
    }
    ml.os_pct = peak_left > target ? (float)(peak_left - target) * 100.0f / (float)target : 0.0f;
    mr.os_pct = peak_right > target ? (float)(peak_right - target) * 100.0f / (float)target : 0.0f;
    ml.band_pct = (float)(band_max_left - band_min_left) * 100.0f / (float)target;
    mr.band_pct = (float)(band_max_right - band_min_right) * 100.0f / (float)target;
    return true;
}

// Nudge one wheel's whole gain grid based on its step-test behavior.
// Returns true if anything changed (another verification pass is needed).
static bool tune_adjust(MotorTune &mt, const StepMetrics &m)
{
    if (m.os_pct > 10.0f || m.band_pct > 8.0f)
    {
        mt.scale_gains(0.8f, 0.85f);
        return true;
    }
    if (m.os_pct < 1.0f && m.t90_ms > 350)
    {
        mt.scale_gains(1.2f, 1.0f);
        return true;
    }
    return false;
}

// The full tuning sequence. Returns false if aborted (reason in tune_abort_reason).
static bool tune_run()
{
    TuningConstants cand = tuning;
    uint32_t max_rpm_left = 0, max_rpm_right = 0;
    char l2[24], l3[24];

    if (!tune_ff_sweep(cand, max_rpm_left, max_rpm_right)) return false;

    snprintf(l2, sizeof(l2), "L %dlnT-%d", (int)cand.left.ff_exponent, (int)cand.left.ff_offset);
    snprintf(l3, sizeof(l3), "R %dlnT-%d", (int)cand.right.ff_exponent, (int)cand.right.ff_offset);
    if (!tune_consent("1/3 FF fit done", l2, l3)) return false;

    // Build the gain grid: nominal targets both wheels can comfortably reach
    // (the relay needs headroom above the target to oscillate).
    uint32_t max_rpm = min(max_rpm_left, max_rpm_right);
    uint32_t max_ok = (uint32_t)(TUNE_GRID_MAX_FRACTION * (float)max_rpm);
    uint32_t grid[MAX_GAIN_POINTS];
    int n_grid = 0;
    for (int i = 0; i < MAX_GAIN_POINTS; i++)
    {
        if (TUNE_GRID_NOMINAL[i] <= max_ok) grid[n_grid++] = TUNE_GRID_NOMINAL[i];
    }
    if (n_grid == 0)
    {
        if (max_ok < TUNE_MIN_TARGET)
        {
            tune_abort_reason = "Wheels too slow";
            return false;
        }
        grid[n_grid++] = max_ok;
    }

    int n_used = 0;
    for (int i = 0; i < n_grid; i++)
    {
        // Pre-check throttle headroom so an out-of-range point is skipped
        // instead of killing the whole tune
        RelayChannel probe_left, probe_right;
        if (!tune_relay_setup(probe_left, cand.left, grid[i]) ||
            !tune_relay_setup(probe_right, cand.right, grid[i]))
        {
            snprintf(l2, sizeof(l2), "Skip %lu RPM", (unsigned long)grid[i]);
            tune_screen("2/3 Relay tests", l2, "(no headroom)", nullptr, nullptr);
            delay(1500);
            continue;
        }
        snprintf(l2, sizeof(l2), "Relay %d/%d", i + 1, n_grid);
        snprintf(l3, sizeof(l3), "at %lu RPM", (unsigned long)grid[i]);
        if (!tune_consent("2/3 Relay tests", l2, l3)) return false;
        if (!tune_relay_point(cand, grid[i], n_used)) return false;
        grid[n_used] = grid[i];
        n_used++;
    }
    if (n_used == 0)
    {
        tune_abort_reason = "No usable targets";
        return false;
    }
    cand.left.num_points = n_used;
    cand.right.num_points = n_used;

    // Verify at the highest grid target - the hardest case for overshoot
    // and oscillation. Gain scheduling covers everything below it.
    uint32_t verify_target = grid[n_used - 1];

    StepMetrics metrics_left, metrics_right;
    for (int iter = 0; iter < TUNE_MAX_VERIFY_ITERATIONS; iter++)
    {
        tune_swap_algorithms(new PIDControl(cand.left, THROTTLE_UPDATE_US),
                             new PIDControl(cand.right, THROTTLE_UPDATE_US));

        snprintf(l2, sizeof(l2), "Try %d at %lu", iter + 1, (unsigned long)verify_target);
        if (!tune_consent("3/3 Step test", l2, "Full spinup!")) return false;
        tune_screen("3/3 Step test", "Running...", nullptr, nullptr, "Menu=ABORT");
        if (!tune_step_response(verify_target, metrics_left, metrics_right)) return false;

        if (iter < TUNE_MAX_VERIFY_ITERATIONS - 1)
        {
            bool adjusted_left = tune_adjust(cand.left, metrics_left);
            bool adjusted_right = tune_adjust(cand.right, metrics_right);
            if (adjusted_left || adjusted_right) continue;
        }
        break;
    }

    // results + save prompt
    float kp_left, ki_left, kd_left, kp_right, ki_right, kd_right;
    cand.left.gains_at((float)verify_target, kp_left, ki_left, kd_left);
    cand.right.gains_at((float)verify_target, kp_right, ki_right, kd_right);
    oled.clearDisplay();
    oled.setFont();
    oled.setTextSize(1);
    oled.setCursor(0, 0);  oled.print(F("L Kp=")); oled.print(kp_left, 2); oled.print(F(" OS=")); oled.print(metrics_left.os_pct, 1); oled.print(F("%"));
    oled.setCursor(0, 10); oled.print(F("R Kp=")); oled.print(kp_right, 2); oled.print(F(" OS=")); oled.print(metrics_right.os_pct, 1); oled.print(F("%"));
    oled.setCursor(0, 20); oled.print(F("t90 L=")); oled.print(metrics_left.t90_ms); oled.print(F(" R=")); oled.print(metrics_right.t90_ms);
    oled.setCursor(0, 32); oled.print(F("Grid ")); oled.print(n_used); oled.print(F(" pt, top ")); oled.print((unsigned long)grid[n_used - 1]);
    oled.setCursor(0, 54); oled.print(F("Trig=SAVE Menu=NO"));
    oled.display();
    bool save_it = tune_consent_wait();
    tune_abort_reason = nullptr;  // declining the save is not an abort

    if (save_it && cand.is_sane())
    {
        tuning = cand;
        mutex_enter_blocking(&save_mutex);
        tuning_save_requested = true;
        mutex_exit(&save_mutex);
        uint32_t t0 = millis();
        while (tuning_save_requested && millis() - t0 < 3000) delay(10);
        tune_screen("Saved!", nullptr, nullptr, nullptr, nullptr);
        delay(800);
    }
    else if (save_it)
    {
        tune_screen("Result failed", "sanity checks -", "keeping old values", nullptr, nullptr);
        delay(2000);
    }
    return true;
}

void run_pid_autotune()
{
    tune_abort_reason = nullptr;
    retractNoid();
    set_target_rpm(0);

    if (!tune_consent("PID AUTOTUNE", "Wheels WILL spin!", "Remove darts + mag", "Tunes 12k-30k grid"))
    {
        tune_abort_reason = nullptr;
        return;
    }

    // fill the voltage running average with fresh readings before deciding
    for (int i = 0; i < 10; i++)
    {
        check_battery();
        delay(20);
    }
    if (voltage < LOW_BATTERY_THRESHOLD)
    {
        tune_screen("PID AUTOTUNE", "Battery too low", "to tune safely", nullptr, "Trig=OK");
        tune_consent_wait();
        tune_abort_reason = nullptr;
        return;
    }

    // open-loop control for the measurement phases (frees the current PIDControls)
    tune_swap_algorithms(new BasicPWMControl(0), new BasicPWMControl(0));

    bool finished = tune_run();

    // stop everything and restore closed-loop control with whatever `tuning`
    // now holds (the new constants if the user saved, the old ones otherwise)
    tune_swap_algorithms(new PIDControl(tuning.left, THROTTLE_UPDATE_US),
                         new PIDControl(tuning.right, THROTTLE_UPDATE_US));
    set_target_rpm(0);

    if (!finished)
    {
        tune_screen("Tune stopped:", tune_abort_reason ? tune_abort_reason : "unknown", nullptr, nullptr, "Trig=OK");
        tune_consent_wait();
        tune_abort_reason = nullptr;
    }
}

std::pair<double,double> fitLog(
    const int *x,
    const int *y,
    int n)
{
    double sumU = 0.0;
    double sumY = 0.0;
    double sumUU = 0.0;
    double sumUY = 0.0;

    for (int i = 0; i < n; ++i)
    {
        double u = std::log(x[i]);   // natural log
        sumU  += u;
        sumY  += y[i];
        sumUU += u * u;
        sumUY += u * y[i];
    }

    double denom = n * sumUU - sumU * sumU;

    double a = (n * sumUY - sumU * sumY) / denom;
    double b = (sumY - a * sumU) / n;

    return {a, b};
}
