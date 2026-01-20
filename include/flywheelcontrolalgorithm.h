#pragma once
#include <stdint.h>
class FlywheelControlAlgorithm {
public:
    virtual ~FlywheelControlAlgorithm() = default;

    // Set throttle (0..max)
    virtual void set_throttle(uint32_t throttle) = 0;

    // Get current throttle value
    virtual uint32_t get_throttle() { return throttle; }

    // Get current target RPM
    virtual uint32_t get_target_rpm() { return target_rpm; }

    // Update throttle
    virtual uint32_t calc_next_throttle(uint32_t current_rpm) = 0;

    // Command target RPM for closed-loop control
    virtual void set_target_rpm(uint32_t rpm) = 0;


protected:
    uint32_t target_rpm = 0;
    uint32_t throttle = 0;
};