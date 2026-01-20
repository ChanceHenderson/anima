//Anima
//A brushless flywheel blaster
//Based off of the Spirit codebase, by wonderboy
//Updated for Anima by Chance
//Software revision 1.0.0
//FOR RP2040

#include <Arduino.h>
#include "anima.h"
#include <Adafruit_SSD1306.h>
#include <ClickButton.h>
//#include <EEPROM.h>
#include <PIO_DShot.h>
#include <Servo.h>
#include <stdio.h>
#include <Wire.h>
#include "FreeSansBoldOblique24pt7b.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "hardware/watchdog.h"
#include "hardware/pwm.h"
#include <stdint.h>
#include <iostream>
#include <string>
#include <sstream>
#include "flywheelmotor.h"
#include "pidcontrol.h"
#include "esc_passthrough.h"

// Mutex for safe motor access between cores
auto_init_mutex(motor_mutex);

typedef struct
{
    uint32_t timestamp;
    uint16_t left_rpm;
    uint16_t right_rpm;
    uint16_t left_throttle;
    uint16_t right_throttle;
} LogMessage;


void load_profile(byte profile) 
{
  /*
  if (EEPROM.read(profile * sizeof(Profile)) == 0xFF) {
    Profile default_profile;
    memcpy_P(&default_profile, &DEFAULT_PROFILES[profile], sizeof(Profile));
    EEPROM.put(profile * sizeof(Profile), default_profile);
  }

  // Load profile from EEPROM
  Profile loaded_profile;
  EEPROM.get(profile * sizeof(Profile), loaded_profile);

  // Apply settings
  profile_rpm = loaded_profile.profile_rpm;
  fire_rate = loaded_profile.fire_rate;
  post_shot_rev_duration_ms = loaded_profile.post_shot_rev_ms;
  */
}

void check_battery()
{
}

void save_current_profile() 
{
  Profile current_settings = {
    profile_rpm,
    fire_rate,
    post_shot_rev_duration_ms
  };
  //EEPROM.put(current_profile * sizeof(Profile), current_settings);
}

void extendNoid() 
{
    set_solenoid_pwm(nextSolenoidOn);
    if (nextSolenoidOn == SOLENOID_ON_HIGH) 
    {
        nextSolenoidOn = SOLENOID_ON_LOW;
    } 
    else
    {
        nextSolenoidOn = SOLENOID_ON_HIGH;
    }
}

void retractNoid() 
{
    set_solenoid_pwm(SOLENOID_OFF);
}

void fireNoid() 
{
    extendNoid();
    delay(noid_extend_duration_ms);
    retractNoid();
}

// Safe wrapper functions for Core 1 to access motor data
void safe_set_target_rpm(uint32_t rpm) 
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

void safe_get_rpm_and_throttle(uint16_t* left_rpm, uint16_t* right_rpm, uint16_t* left_throttle, uint16_t* right_throttle) 
{
    mutex_enter_blocking(&motor_mutex);
    if (left_motor != nullptr) 
    {
        *left_rpm = left_motor->get_current_rpm();
        *left_throttle = left_motor->control_algorithm->get_throttle();
    }
    if (right_motor != nullptr) 
    {
        *right_rpm = right_motor->get_current_rpm();
        *right_throttle = right_motor->control_algorithm->get_throttle();
    }
    mutex_exit(&motor_mutex);
}

void setup()
{
    // Core 0 runs setup() by default on Arduino
    Serial.begin(115200);
    Serial.printf("Core 0: Starting setup on Core 0\n");
    // Launch Core 1 for UI and control logic
    multicore_launch_core1(core1_main);
}

// Initialize motors and solenoid (called from Core 1 after mode selection)
void initialize_motors() 
{
    left_motor = new FlywheelMotor(PIN_ESC_1_OUT, MOTOR_POLES,  max_acceleration_rpm_per_us, max_deceleration_rpm_per_us, new PIDControl(Kp, Ki, Kd, feed_forward_curve_offset, feed_forward_curve_exponent, 200));
    right_motor = new FlywheelMotor(PIN_ESC_2_OUT, MOTOR_POLES, max_acceleration_rpm_per_us, max_deceleration_rpm_per_us, new PIDControl(Kp, Ki, Kd, feed_forward_curve_offset, feed_forward_curve_exponent, 200));
    // Initialize PWM for solenoid (high-frequency servo, 480Hz)
    gpio_set_function(PIN_SOLENOID_OUT, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(PIN_SOLENOID_OUT);
    channel_num = pwm_gpio_to_channel(PIN_SOLENOID_OUT);
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, PWM_CLOCK_DIV);
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(slice_num, &config, true);
    
    // Set initial position to center (1500us)
    pwm_set_chan_level(slice_num, channel_num, PWM_CENTER);
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
  if (tourney && !idle) 
  {
    idle = true;
    staged_rpm = IDLE_RPM;
    safe_set_target_rpm(staged_rpm);
  } 
  else if (!tourney) 
  {
    staged_rpm = 0;
    safe_set_target_rpm(staged_rpm);
  }
  revved = false;
  last_rev_timestamp = millis();
  safety_timer = 0;
}

void main_loop() 
{
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
        staged_rpm = profile_rpm;
        safe_set_target_rpm(staged_rpm);
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
  //if not in settings mode, while trigger is pressed, spin up the flywheels and fire
  if (!settings)
  {
    //if in tourney mode and flywheel speed set to idle RPM, spin up the flywheels for pre-rev
    if (tourney && idle) 
    {
      
    }
    // Rev trigger pressed but nothing else is
    if (rev.depressed && !menu.depressed && !trig.depressed && !low_batt)
    {
      manual_rev_active = true;
      staged_rpm = profile_rpm;
      safe_set_target_rpm(staged_rpm);
    }
    else if (!rev.depressed)
    {
      manual_rev_active = false;
    }
    //make sure we haven't been revving for longer than the safety timeout and battery voltage is OK
    if (trig.depressed && !menu.depressed && safety_timer < REV_SAFETY_TIMEOUT && !low_batt) 
    {
      rev_up();
      if (revved) 
      {
        switch (mode) 
        {
          case SEMI:
          case BINARY:
            //fire once then set flag that prevents additional shots until trigger is released
            if (!fired) 
            {
              fire();
              fired = true;
            }
            break;
          case AUTO:
            trig.Update();
            if (trig.depressed) 
            {
              fire();
            }
            break;
        }
      }
    } 
    else 
    {

      if (safety_timer >= REV_SAFETY_TIMEOUT)
      {
        rev_down();
        while (trig.depressed)
        {
          trig.Update();
          delay(5);
        }
      }

      //if in binary or reverse mode, fire on trigger release
      if (fired && revved) 
      {
        if (mode == BINARY) 
        {
          fire();
        }
      }
      fired = false;
      if (millis() - spin_down_timer >= post_shot_rev_duration_ms && !manual_rev_active)
      {
        rev_down();
      }
      retractNoid();
    }

    //if trigger is held whilst menu is held, lock/unlock the fire mode
    if (menu.depressed && trig.clicks < 0) {
      update_display = true;
      lock = !lock;
      menu.clicks = 0;
      trig.clicks = 0;
    }

    //update battery voltage reading every BATTFREQ milliseconds
    //only when not revving to prevent low readings due to sag
    if (millis() % BATTERY_CHECK_MS == 0 && !revved) {
      update_display = true;
      check_battery();
      single_shot_delay = map(voltage, 17, 15, 17, 26);
    }

    //write the display buffer if flag is set & only when not revving
    //prevents screen write delay from affecting operation
    if (update_display && !revved) {
      display_main();
      update_display = false;
    }
  } else {
    //settings menu
    //update screen every second for runtime display
    if (millis() % 1000 == 0) {
      update_display = true;
    }

    //menu press switches selected parameter
    if (menu.clicks > 0) {
      if (trig.depressed) {
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
      } else {
        update_display = true;
        selected++;

        if (selected > 3) {
          selected = 1;
        }
        menu.clicks = 0;
        trig.clicks = 0;
      }
    }

    //trigger press changes parameter value
    if (trig.clicks > 0) {
      update_display = true;
      switch (selected) {
        case 1:
          profile_rpm += RPM_STEP_SIZE;
          if (profile_rpm > MAX_RPM) {
            profile_rpm = MIN_RPM;
          }
          break;
        case 2:
          fire_rate += 1;
          if (fire_rate > MAX_FIRE_RATE) {
            fire_rate = MIN_FIRE_RATE;
          }
          single_shot_delay = ceil((1000 - (fire_rate * noid_extend_ms)) / fire_rate);
          break;
        case 3:
          post_shot_rev_duration_ms += 100;
          if (post_shot_rev_duration_ms > 2000) {
            post_shot_rev_duration_ms = 0;
          }
          break;
      }
      menu.clicks = 0;
      trig.clicks = 0;
    }

    //rev press changes parameter value the other way
    if (rev.clicks > 0) {
      update_display = true;
      switch (selected) {
        case 1:
          profile_rpm -= RPM_STEP_SIZE;
          if (profile_rpm < MIN_RPM) {
            profile_rpm = MAX_RPM;
          }
          break;
        case 2:
          fire_rate -= 1;
          if (fire_rate < MIN_FIRE_RATE) {
            fire_rate = MAX_FIRE_RATE;
          }
          single_shot_delay = ceil((1000 - (fire_rate * noid_extend_ms)) / fire_rate);
          break;
        case 3:
          post_shot_rev_duration_ms -= 100;
          if (post_shot_rev_duration_ms < 0) {
            post_shot_rev_duration_ms = 2000;
          }
          break;
      }
      menu.clicks = 0;
      trig.clicks = 0;
      rev.clicks = 0;
    }
    retractNoid();
    shot_count = 0;

    if (update_display) {
      display_settings(selected);
      update_display = false;
    }
  }
}

void rev_up() {
  //if in tourney mode and flywheel speed set to idle RPM, first update the targetRPM to the actual RPM
  if (tourney && idle) {
    idle = false;
    staged_rpm = profile_rpm;
    safe_set_target_rpm(staged_rpm);
  }
  if (!revved) {
    //Record trigger down event timestamp
    long last_trigger_down_ms = millis();

    //Wait for motors to reach speed
    bool fail = false;
    
    while (current_rpm_left <= (profile_rpm - SPINUP_RPM_THRESHOLD) || (current_rpm_right <= (profile_rpm - SPINUP_RPM_THRESHOLD))) {
      if (millis() - last_trigger_down_ms > REV_FAIL_TIMER) {
        fail = true;
        break;
      }
      delayMicroseconds(100);
    }
    if (fail) {
      //Whoops. We exited because motors failed to reach speed setpoint in a reasonable time
      //Shut down motors
      rev_down();
      while (trig.depressed) {
        trig.Update();
        delay(5);
      }
    } else {
      revved = true;  //say it's ready to fire, close everything out so it'll go to the fire control code
      spin_down_timer = millis();
      safety_timer = millis() - last_rev_timestamp;
    }

  } else {
    spin_down_timer = millis();
    safety_timer = millis() - last_rev_timestamp;
  }
}

//main (firing) screen display output
void display_main() {
  byte rpm_thousands = profile_rpm / 1000;
  byte rpm_hundreds = (profile_rpm % 1000) / 100;
  byte countH = 60;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print(noid_extend_ms);
  oled.print("ms");

  oled.setCursor(99, 0);
  oled.print(voltage, 1);
  oled.print(F("V"));
  oled.drawFastHLine(0, 9, 128, 1);

  if (!low_batt) {
    oled.setFont(&FreeSansBoldOblique24pt7b);

    countH = 50;
    if (shot_count > 999) {
      shot_count = 0;
    }
    if (shot_count > 9) {
      countH -= 14;
    }
    if (shot_count > 99) {
      countH -= 14;
    }
    oled.setCursor(countH, 47);
    oled.print(shot_count);
  } else {
    oled.setCursor(0, 26);
    oled.print(F("BATTERY\n CRITICAL!"));
  }

  countH = 105;
  oled.setFont();
  oled.drawFastHLine(0, 53, 128, 1);
  if (low_batt) {
    throttle_thousands = 0;
  }
  if (throttle_thousands > 9) {
    countH -= 6;
  }
  oled.setCursor(countH, 56);
  oled.print(throttle_thousands);
  oled.print(F("."));
  oled.print(throttle_hundreds);
  oled.setCursor(0, 56);

  if (!low_batt) {
    switch (mode) {
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
    if (lock) {
      oled.print(F("*"));
    }
  } else {
    oled.print(F("STOP"));
  }

  oled.display();
}

//settings screen display output
void display_settings(byte selected) {
  //run timer
  unsigned int t = millis() / 1000;
  byte hours = t / 3600;
  t %= 3600;
  byte minutes = t / 60;
  t %= 60;
  byte seconds = t;

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.print(F("0"));
  oled.print(hours);
  oled.print(F(":"));
  if (minutes < 10) {
    oled.print(F("0"));
  }
  oled.print(minutes);
  oled.print(F(":"));
  if (seconds < 10) {
    oled.print(F("0"));
  }
  oled.print(seconds);
  oled.drawFastHLine(0, 9, 128, 1);
  oled.setCursor(0, 11);
  oled.print(F("RPM: "));
  oled.setCursor(96, 11);
  if (selected == 1) {
    oled.setTextColor(0, 1);
  }
  oled.print(profile_rpm);
  oled.setTextColor(1, 0);
  oled.setCursor(0, 25);
  oled.print(F("Fire Rate: "));
  oled.setCursor(96, 25);
  if (selected == 2) {
    oled.setTextColor(0, 1);
  }
  oled.print(fire_rate);
  oled.setTextColor(1, 0);
  oled.setCursor(0, 39);
  oled.print(F("Post-fire rev (ms): "));
  oled.setCursor(96, 39);
  if (selected == 3) {
    oled.setTextColor(0, 1);
  }
  oled.print(post_shot_rev_duration_ms);
  oled.setTextColor(1, 0);
  oled.display();
}

void core1_main() {
  menu.debounceTime = 20;
  trig.debounceTime = 5;
  rev.debounceTime = 5;
  trig.longClickTime = 332;   //how long to wait before locking fire mode when menu and trigger held
  menu.longClickTime = 1500;  //how long to wait before switching to settings mode when menu button held
  trig.multiclickTime = 0;
  menu.multiclickTime = 0;
  rev.multiclickTime = 0;
  //startup animation
  Wire.setSDA(PIN_OLED_SDA);
  Wire.setSCL(PIN_OLED_SCL);
  Wire.begin();
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.setCursor(0, 0);
  oled.setTextColor(1);
  oled.setTextSize(2);
  oled.setTextWrap(false);
  oled.clearDisplay();
  oled.drawBitmap(0, 0, splash, 128, 64, 1);
  oled.setCursor(8, 40);
  oled.display();
  String str;
  if (digitalRead(PIN_MENU_IN) == LOW && digitalRead(PIN_FIRE_IN) == HIGH) {
    current_profile = PROFILE_LOW;
    str = "Low  Power";
  } else if (digitalRead(PIN_MENU_IN) == HIGH && digitalRead(PIN_FIRE_IN) == LOW) {
    current_profile = PROFILE_MED;
    str = "Mid  Power";
  } else if (digitalRead(PIN_MENU_IN) == LOW && digitalRead(PIN_FIRE_IN) == LOW) {
    current_profile = PROFILE_T;
    menu.longClickTime = 500;
    str = "Tournament";
    tourney = true;
    idle = true;
  } else {
    str = "High Power";
    current_profile = PROFILE_HIGH;
  }

  load_profile(current_profile);
  oled.print(str);
  oled.display();
  oled.clearDisplay();
  noid_extend_ms = map(voltage, 17.0, 14.6, 17, 26);
  while (true) 
  {
    main_loop();
  }
} 


// Core 0 loop - continuously send motor throttle commands
void loop()
{
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
    mutex_exit(&motor_mutex);
    delayMicroseconds(200);
}