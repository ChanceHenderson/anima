#pragma once
#include <stdint.h>
#include "flywheelcontrolalgorithm.h"

class BasicPWMControl : public FlywheelControlAlgorithm {
public:
    BasicPWMControl(uint32_t initial_throttle) {
        this->throttle = initial_throttle;
    }
    // Called at 5 kHz
    uint32_t calc_next_throttle(uint32_t current_rpm) override {return this->throttle;}
    
    void set_throttle(uint32_t throttle) override {
        this->throttle = throttle;
    }
    
    void set_target_rpm(uint32_t rpm) override {
        this->throttle = rpm;
    };
};