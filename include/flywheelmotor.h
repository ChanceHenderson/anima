#ifndef FLYWHEELMOTOR_H
#define FLYWHEELMOTOR_H
#include <Arduino.h>
#include <PIO_DShot.h>
#include <stdint.h>
#include "flywheelcontrolalgorithm.h"



class FlywheelMotor {
private:
    volatile unsigned int target_rpm;
    volatile unsigned int actual_rpm;
    
    float voltage = 0.0f;
    BidirDShotX1* dshot;
    uint32_t last_voltage_check = 0;
    uint32_t packets_sent = 0;
    uint32_t last_good_rpm_timestamp = 0;
    uint32_t bad_rpm_reads = 0;
    uint32_t max_acceleration_rpm_per_us;
    uint32_t max_deceleration_rpm_per_us;
    uint32_t motor_poles;
    float current;
    float temp;
    uint32_t lastStatus;
    uint32_t stress;
    uint32_t update_rpm();
public:
    FlywheelControlAlgorithm* control_algorithm;
    FlywheelMotor(uint8_t esc_pin, uint32_t motor_poles, float max_acceleration_rpm_per_us, float max_deceleration_rpm_per_us, FlywheelControlAlgorithm* control_algorithm);
    void send_throttle();
    float get_current_voltage() { return this->voltage; }
    volatile uint16_t get_current_rpm() { return this->actual_rpm; }
    uint32_t get_bad_rpm_reads() { return bad_rpm_reads;  }
    void reset_bad_rpm_reads() { bad_rpm_reads = 0; }
    BidirDShotX1* get_dshot_instance() { return dshot;  }
};
#endif