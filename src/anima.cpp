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
    Serial.begin(115200);
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
    // Launch Core 1 for UI and control logic
    multicore_launch_core1(core1_main);
}

// Initialize motors and solenoid (called from Core 1 after mode selection)
void initialize_motors() 
{
  left_motor = new FlywheelMotor(PIN_ESC_1_OUT, MOTOR_POLES,  new PIDControl(Kp, Ki, Kd, feed_forward_curve_offset, feed_forward_curve_exponent, 200));
  right_motor = new FlywheelMotor(PIN_ESC_2_OUT, MOTOR_POLES, new PIDControl(Kp, Ki, Kd, feed_forward_curve_offset, feed_forward_curve_exponent, 200));
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
      noid_extend_ms = map(voltage, 17.0, 14.6, 16, 26);
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

        if (selected > 6) 
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
      menu.clicks = 0;
      trig.clicks = 0;
    }

    
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
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.print(F("RPM: "));
  oled.setCursor(96, 0);
  if (selected == 1) oled.setTextColor(0, 1);
  oled.print(profile_rpm);
  oled.setTextColor(1, 0);
  oled.setCursor(0, 11);
  oled.print(F("Fire Rate: "));
  oled.setCursor(96, 11);
  if (selected == 2) oled.setTextColor(0, 1);
  oled.print(fire_rate);
  oled.setTextColor(1, 0);
  oled.setCursor(0, 22);
  oled.print(F("Post-fire rev: "));
  oled.setCursor(96, 22);
  if (selected == 3) oled.setTextColor(0, 1);
  oled.print(post_shot_rev_duration_ms);
  oled.setTextColor(1, 0);
  oled.setCursor(0, 33);
  oled.print(F("Idle: "));
  oled.setCursor(96, 33);
  if (selected == 4) oled.setTextColor(0, 1);
  if (use_idle) oled.print(F("Yes"));
  else oled.print(F("No"));
  oled.setTextColor(1, 0);
  oled.setCursor(0, 44);
  oled.print(F("Rev trigger auto: "));
  oled.setCursor(96, 44);
  if (selected == 5) oled.setTextColor(0, 1);
  if (rev_is_auto) oled.print(F("Yes"));
  else oled.print(F("No"));
  oled.setTextColor(1, 0);
  oled.setCursor(0, 55);
  oled.print(F("Save Profile: "));
  oled.setCursor(96, 55);
  if (selected == 6) oled.setTextColor(0, 1);
  oled.print(F("Save"));
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
      uint8_t pins[4] = {PIN_ESC_1_OUT, PIN_SOLENOID_OUT, PIN_ESC_2_OUT, PIN_UNUSED_ESC_OUT};
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
  noid_extend_ms = map(voltage, 17.0, 14.6, 17, 26);
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

void tune_feed_forward() 
{
  motors_initialized = false; // Pause motor loop while swapping algorithms
  delayMicroseconds(500); // Ensure motor loop is paused
  FlywheelControlAlgorithm* ff_algo_left = left_motor->control_algorithm;
  FlywheelControlAlgorithm* ff_algo_right = right_motor->control_algorithm;
  left_motor->control_algorithm = new BasicPWMControl(0);
  right_motor->control_algorithm = new BasicPWMControl(0);
  motors_initialized = true; // Resume motor loop with new algorithms

  int throttle_rpms[9] = {0,0,0,0,0,0,0,0,0};
  for (int i = 1; i < 10; i++) 
  {
    oled.clearDisplay();
    oled.setFont();
    oled.setTextSize(1);
    oled.setCursor(0, 5);
    oled.print("Feed forward tuning");
    oled.setCursor(0, 20);
    oled.print("Step 1: RPM test");
    oled.setCursor(0, 35);
    oled.print("Phase ");
    oled.print(i);
    oled.print("/9");
    oled.setCursor(0, 50);
    oled.print("Press trigger to start");
    oled.display();
    
    while (digitalRead(PIN_FIRE_IN) == HIGH)
    {
        delay(10);
    }

    oled.clearDisplay();
    oled.setFont();
    oled.setTextSize(1);
    oled.setCursor(30, 20);
    oled.print("Throttle: ");
    oled.print(i * 200);
    oled.setCursor(0, 40);
    oled.print("TEST IN PROGRESS!");
    oled.display();

    mutex_enter_blocking(&motor_mutex);
    left_motor->control_algorithm->set_throttle(i * 200);
    mutex_exit(&motor_mutex);

    delay(1500); // Wait for motor to stabilize
    uint32_t rpm_sum = 0;
    for (int j = 0; j < 50; j++) 
    {
      rpm_sum += current_rpm_left;
      delayMicroseconds(200);
    }

    mutex_enter_blocking(&motor_mutex);
    left_motor->control_algorithm->set_throttle(0);
    mutex_exit(&motor_mutex);

    throttle_rpms[i-1] = rpm_sum / 50;
  }
  
  oled.clearDisplay();
  oled.setFont();
  oled.setTextSize(1);
  oled.setCursor(20, 0);
  oled.print("Test Complete!");
  oled.setCursor(0, 12);
  oled.print("200: ");
  oled.print(throttle_rpms[0]);
  oled.setCursor(64, 12);
  oled.print("400: ");
  oled.print(throttle_rpms[1]);
  oled.setCursor(0, 24);
  oled.print("600: ");
  oled.print(throttle_rpms[2]);
  oled.setCursor(64, 24);
  oled.print("800: ");
  oled.print(throttle_rpms[3]);
  oled.setCursor(0, 36);
  oled.print("1000: "); 
  oled.print(throttle_rpms[4]);
  oled.setCursor(64, 36);
  oled.print("1200: ");
  oled.print(throttle_rpms[5]);
  oled.setCursor(0, 48);
  oled.print("1400: ");
  oled.print(throttle_rpms[6]);
  oled.setCursor(64, 48);
  oled.print("1600: ");
  oled.print(throttle_rpms[7]);
  oled.setCursor(0, 60);
  oled.print("1800: ");
  oled.print(throttle_rpms[8]);
  oled.setCursor(64, 60);
  oled.print("Trigger...");
  oled.display();

  while (digitalRead(PIN_FIRE_IN) == HIGH)
  {
      delay(10);
  }

  int x[9] = {200, 400, 600, 800, 1000, 1200, 1400, 1600, 1800};
  std::pair<double, double> log_params = fitLog(x, throttle_rpms, 9);

  oled.clearDisplay();
  oled.setFont();
  oled.setTextSize(1);
  oled.setCursor(10, 0);
  oled.print("Tuning complete!");
  oled.setCursor(0, 12);
  oled.print("Estimated curve: ");
  oled.setCursor(0, 24);
  oled.print("R=");
  oled.print(int(log_params.second));
  oled.print("ln(T)+");
  oled.print(int(log_params.first));
  oled.display();

  motors_initialized = false; // Pause motor loop while swapping algorithms
  delayMicroseconds(500); // Ensure motor loop is paused
  delete left_motor->control_algorithm;
  delete right_motor->control_algorithm;
  left_motor->control_algorithm = ff_algo_left;
  right_motor->control_algorithm = ff_algo_right;
  motors_initialized = true; // Resume motor loop with new algorithms
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
