#include <Arduino.h>
#include "flywheelmotor.h"
#include <PIO_DShot.h>
#include <stdint.h>

uint32_t calculate_time_difference_us(uint32_t start, uint32_t end)
{
    if (end >= start) return end - start;  // normal case
    else return (UINT32_MAX - start) + end + 1; // micros overflows approx every 70 minutes - handle that
}

FlywheelMotor::FlywheelMotor(uint8_t esc_pin, uint32_t motor_poles, float max_acceleration_rpm_per_us, float max_deceleration_rpm_per_us, FlywheelControlAlgorithm* control_algorithm)
{
    this->dshot = new BidirDShotX1(esc_pin, 600);
    this->control_algorithm = control_algorithm;
    this->motor_poles = motor_poles;
    this->actual_rpm = 0;
    this->max_acceleration_rpm_per_us = max_acceleration_rpm_per_us;
    this->max_deceleration_rpm_per_us = max_deceleration_rpm_per_us;
}

uint32_t FlywheelMotor::update_rpm()
{
    uint32_t rpm = 0;
    uint32_t returnValue = 0;
    uint32_t now = micros();
    uint32_t time_since_last_update = calculate_time_difference_us(now, this->last_good_rpm_timestamp);
	BidirDshotTelemetryType bdir_type = dshot->getTelemetryPacket(&returnValue);
	switch (bdir_type) {
        case BidirDshotTelemetryType::ERPM:
            rpm = returnValue / (this->motor_poles / 2);
            if (
                rpm >= 0 &&               // can't have negative RPM
                rpm <= 50000 //&&           // we can't go over 50krpm so if it says we did, it lied
                //rpm >= this->actual_rpm - (this->max_deceleration_rpm_per_us * time_since_last_update) && // it only decelerates so fast - if it says it did more than that, it lied
                //rpm <= this->actual_rpm + (this->max_acceleration_rpm_per_us * time_since_last_update)    // it only accelerates so fast - if it says it did more than that, it lied
                ) 
            {
                this->actual_rpm = rpm;
                this->last_good_rpm_timestamp = now;
            }
            else
            {
                bad_rpm_reads++;
            }
            break;
        case BidirDshotTelemetryType::VOLTAGE:
            voltage = (float)returnValue / 4;
            break;
        case BidirDshotTelemetryType::CURRENT:
            current = returnValue;
            break;
        case BidirDshotTelemetryType::TEMPERATURE:
            temp = returnValue;
            break;
        case BidirDshotTelemetryType::STATUS:
            lastStatus = returnValue;
            break;
        case BidirDshotTelemetryType::STRESS:
            stress = returnValue & ESC_STATUS_MAX_STRESS_MASK;
            break;
    }
    return this->actual_rpm;
}

void FlywheelMotor::send_throttle() 
{
    update_rpm();
    //Serial.println("Actual RPM: " + String(this->actual_rpm));
    uint32_t throttle = this->control_algorithm->calc_next_throttle(this->actual_rpm);
    //Serial.println("Calculated throttle: " + String(throttle));
    this->dshot->sendThrottle(throttle);
    this->packets_sent++;
}