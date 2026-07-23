#include <Arduino.h>
#include "flywheelmotor.h"
#include <PIO_DShot.h>
#include <stdint.h>

uint32_t calculate_time_difference_us(uint32_t start, uint32_t end)
{
    if (end >= start) return end - start;  // normal case
    else return (UINT32_MAX - start) + end + 1; // micros overflows approx every 70 minutes - handle that
}

FlywheelMotor::FlywheelMotor(uint8_t esc_pin, uint32_t motor_poles, FlywheelControlAlgorithm* control_algorithm)
{
    this->dshot = new BidirDShotX1(esc_pin, 600);
    this->control_algorithm = control_algorithm;
    this->motor_poles = motor_poles;
    this->actual_rpm = 0;
}

uint32_t FlywheelMotor::update_rpm()
{
    uint32_t rpm = 0;
    uint32_t returnValue = 0;
    uint32_t now = micros();
	BidirDshotTelemetryType bdir_type = dshot->getTelemetryPacket(&returnValue);
	switch (bdir_type) {
        case BidirDshotTelemetryType::ERPM:
        {
            rpm = returnValue / (this->motor_poles / 2);
            if (rpm > RPM_MAX_VALID) // we can't go over 50krpm so if it says we did, it lied
            {
                bad_rpm_reads++;
                break;
            }
            // Plausibility gate: flywheel inertia bounds how fast the real RPM
            // can move, so a sample outside the physically reachable window
            // since the last accepted sample is corruption (an AM32 hiccup or
            // a bit flip that survived the 4-bit checksum), not motion. The
            // window grows with elapsed time, so after a telemetry gap it
            // opens up and re-syncs on its own.
            uint32_t elapsed_us = calculate_time_difference_us(this->last_good_rpm_timestamp, now);
            uint32_t window = RPM_WINDOW_BASE + (elapsed_us / 1000) * RPM_WINDOW_SLEW_PER_MS;
            uint32_t last = this->actual_rpm;
            uint32_t diff = (rpm > last) ? (rpm - last) : (last - rpm);
            if (diff <= window)
            {
                this->actual_rpm = rpm;
                this->last_good_rpm_timestamp = now;
                outlier_count = 0;
            }
            else
            {
                // A genuine step change (hard jam, lost sync) shows up as
                // repeated outliers that agree with EACH OTHER - accept after
                // enough consecutive agreeing samples instead of locking out.
                // Isolated corruption never agrees with itself twice.
                uint32_t outlier_diff = (rpm > last_outlier_rpm) ? (rpm - last_outlier_rpm) : (last_outlier_rpm - rpm);
                if (outlier_count > 0 && outlier_diff <= RPM_OUTLIER_AGREE)
                {
                    outlier_count++;
                }
                else
                {
                    outlier_count = 1;
                }
                last_outlier_rpm = rpm;
                if (outlier_count >= RPM_OUTLIER_RESYNC)
                {
                    this->actual_rpm = rpm;
                    this->last_good_rpm_timestamp = now;
                    outlier_count = 0;
                }
                else
                {
                    bad_rpm_reads++;
                }
            }
            break;
        }
        case BidirDshotTelemetryType::VOLTAGE:
            voltage = (float)returnValue / 4;
            last_edt_frame_ms = millis();
            break;
        case BidirDshotTelemetryType::CURRENT:
            current = returnValue;
            last_edt_frame_ms = millis();
            break;
        case BidirDshotTelemetryType::TEMPERATURE:
            temp = returnValue;
            last_edt_frame_ms = millis();
            break;
        case BidirDshotTelemetryType::STATUS:
            lastStatus = returnValue;
            last_edt_frame_ms = millis();
            break;
        case BidirDshotTelemetryType::STRESS:
            stress = returnValue & ESC_STATUS_MAX_STRESS_MASK;
            last_edt_frame_ms = millis();
            break;
    }
    return this->actual_rpm;
}

// Ask the ESC to interleave extended telemetry (temperature/voltage/current/
// status/stress) frames into the bidir DShot stream. EDT is opt-in: without
// DShot command 13 the ESC only ever sends eRPM frames. sendRaw11Bit sets the
// telemetry-request bit, which the command requires (sendThrottle does not).
// Only call while the motor is stopped - ESCs ignore commands while spinning.
void FlywheelMotor::enable_edt()
{
    for (int i = 0; i < 10; i++)
    {
        dshot->sendRaw11Bit(DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE);
        delayMicroseconds(200);
    }
    last_edt_enable_ms = millis();
}

// EDT resets whenever the ESC power cycles (e.g. a brownout mid-session), so
// if the extended frames stop arriving, re-send the enable burst. Call this
// only while the motor is stopped. Rate-limited so an ESC that simply doesn't
// support EDT only sees a short burst every 10 s.
void FlywheelMotor::maybe_reenable_edt()
{
    if (millis() - last_edt_frame_ms < 10000) return;  // frames still flowing
    if (millis() - last_edt_enable_ms < 10000) return; // recently attempted
    enable_edt();
}

void FlywheelMotor::send_throttle() 
{
    update_rpm();
    uint32_t throttle = this->control_algorithm->calc_next_throttle(this->actual_rpm);
    this->dshot->sendThrottle(throttle);
    this->packets_sent++;
}