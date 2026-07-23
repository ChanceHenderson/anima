#ifndef FLYWHEELMOTOR_H
#define FLYWHEELMOTOR_H
#include <Arduino.h>
#include <PIO_DShot.h>
#include <stdint.h>
#include "flywheelcontrolalgorithm.h"

// eRPM plausibility filtering. The bidir DShot checksum is only 4 bits, so
// roughly 1 in 16 corrupted frames decodes "valid" with a bogus value - and
// AM32 occasionally sends junk eRPM outright. A flywheel has inertia, so a
// sample the wheel could not physically have reached since the last accepted
// sample is corruption, not motion.
#define RPM_MAX_VALID 50000        // hard ceiling - nothing we run can exceed this
#define RPM_WINDOW_BASE 1000       // always-allowed deviation (quantization/jitter), rpm
#define RPM_WINDOW_SLEW_PER_MS 300 // window growth per ms since last good sample (full-throttle spinup measures ~150-200 rpm/ms)
#define RPM_OUTLIER_AGREE 3000     // consecutive outliers within this of each other are "agreeing", rpm
#define RPM_OUTLIER_RESYNC 3       // agreeing outliers needed to accept a genuine step change



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
    uint32_t motor_poles;
    float current = 0.0f;
    float temp = 0.0f;
    uint32_t lastStatus = 0;
    uint32_t stress = 0;
    uint32_t last_edt_frame_ms = 0;   // when the last extended-telemetry frame arrived (0 = never)
    uint32_t last_edt_enable_ms = 0;  // when we last sent the EDT enable burst
    uint32_t outlier_count = 0;       // consecutive eRPM samples outside the plausibility window
    uint32_t last_outlier_rpm = 0;    // most recent rejected sample, for the agreement check
    uint32_t update_rpm();
public:
    FlywheelControlAlgorithm* control_algorithm;
    FlywheelMotor(uint8_t esc_pin, uint32_t motor_poles, FlywheelControlAlgorithm* control_algorithm);
    void send_throttle();
    // Extended DShot Telemetry (EDT) is opt-in per the spec and resets on
    // every ESC power cycle - enable_edt() sends the enable burst (only call
    // while the motor is stopped), maybe_reenable_edt() re-sends it if the
    // extended frames ever stop (e.g. the ESC browned out and rebooted).
    void enable_edt();
    void maybe_reenable_edt();
    float get_current_voltage() { return this->voltage; }
    volatile uint16_t get_current_rpm() { return this->actual_rpm; }
    float get_temperature() { return this->temp; }
    float get_current() { return this->current; }
    uint32_t get_status() { return this->lastStatus; }
    uint32_t get_stress() { return this->stress; }
    uint32_t get_last_edt_frame_ms() { return this->last_edt_frame_ms; }
    uint32_t get_bad_rpm_reads() { return bad_rpm_reads;  }
    void reset_bad_rpm_reads() { bad_rpm_reads = 0; }
    BidirDShotX1* get_dshot_instance() { return dshot;  }
};
#endif