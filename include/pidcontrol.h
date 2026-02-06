#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "flywheelcontrolalgorithm.h"


// PID and Filter Window Sizes
#define INTEGRAL_WINDOW 25
#define RPM_FILTER_WINDOW 6


typedef struct {
    float integral_buffer[INTEGRAL_WINDOW];
    uint8_t index;
    uint8_t filled;
    float last_measured;
    unsigned long last_time;
    float integral_sum;
} PIDState;


typedef struct {
    uint8_t index;
    uint8_t filled;
    uint32_t buffer[RPM_FILTER_WINDOW];
} RPMFilterState;


class PIDControl : public FlywheelControlAlgorithm {
public:
    PIDControl(float Kp, float Ki, float Kd, float feed_forward_curve_offset, float feed_forward_curve_exponent, uint32_t update_frequency_us)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->feed_forward_curve_offset = feed_forward_curve_offset;
        this->feed_forward_curve_exponent = feed_forward_curve_exponent;
        this->update_frequency_us = update_frequency_us;
        reset_pid();
        reset_filter();
    }

    void set_throttle(uint32_t throttle) override
    {
        this->throttle = throttle;
    }

    uint32_t calc_next_throttle(uint32_t current_rpm) override
    {
        if (this->target_rpm == 0)
        {
            reset_pid();
            reset_filter();
            throttle = 0;
        }
        else
        {
            throttle = calculate_pid(current_rpm);
        }
        return throttle;
    }

    void set_target_rpm(uint32_t rpm) override
    {
        target_rpm = rpm;
    }
    
    void reset_pid()
    {
        pidstate.last_measured = 0.0f;
        pidstate.last_time = 0;
        pidstate.index = 0;
        pidstate.filled = 0;
        pidstate.integral_sum = 0.0f;
        for (uint8_t i = 0; i < INTEGRAL_WINDOW; i++) pidstate.integral_buffer[i] = 0.0f;
    }

    void reset_filter()
    {
        filter.index = 0;
        filter.filled = 0;
        for (uint8_t i = 0; i < RPM_FILTER_WINDOW; i++) filter.buffer[i] = 0;
    }

    uint32_t calculate_pid(uint32_t current_rpm)
    {
        uint32_t target_rpm = this->target_rpm;

        // 1) Filter RPM measurement
        current_rpm = filter_rpm(current_rpm);
        float m = (float)current_rpm;
        float t = (float)target_rpm;

        // 2) Time step
        unsigned long now = micros();
        float dt;
        if (pidstate.last_time == 0) 
        {
            dt = this->update_frequency_us * 1e-6f;  // first call: assume nominal dt
        } 
        else
        {
            dt = (now - pidstate.last_time) * 1e-6f;
            if (dt <= 1e-6f) dt = 1e-6f;
        }
        pidstate.last_time = now;

        // 3) Error
        float error = t - m;

        // ===================== Moving-window Integral =====================
        // Subtract oldest, add newest
        pidstate.integral_sum -= pidstate.integral_buffer[pidstate.index];
        pidstate.integral_buffer[pidstate.index] = error * dt;  // scale by dt
        pidstate.integral_sum += pidstate.integral_buffer[pidstate.index];

        pidstate.index = (pidstate.index + 1) % INTEGRAL_WINDOW;
        if (pidstate.filled < INTEGRAL_WINDOW) pidstate.filled++;

        float integral = pidstate.integral_sum;  // moving-window integral

        // ===================== Derivative (on measurement) =====================
        float derivative = -(m - pidstate.last_measured) / dt;
        pidstate.last_measured = m;

        // ===================== Feed-forward + PID =====================
        float pid_effort = (Kp * error) + (Ki * integral) + (Kd * derivative);
        float ff = expf((t + feed_forward_curve_offset) / feed_forward_curve_exponent);
        float output = ff + pid_effort;
        //float output = pid_effort;
        // ===================== Clamp and Anti-Windup =====================
        bool saturated_high = false;
        bool saturated_low = false;
        float max_throttle = 2000.0f;
        if (output > max_throttle) 
        {
            output = max_throttle;
            saturated_high = true;
        } 
        else if (output < 0.0f) 
        {
            output = 0.0f;
            saturated_low = true;
        }

        // Undo integral step if saturated in direction of error
        if ((saturated_high && error > 0.0f) || (saturated_low && error < 0.0f))
        {
            // Remove the newest sample from integral_sum to prevent windup
            pidstate.integral_sum -= pidstate.integral_buffer[(pidstate.index + INTEGRAL_WINDOW - 1) % INTEGRAL_WINDOW];
            pidstate.integral_buffer[(pidstate.index + INTEGRAL_WINDOW - 1) % INTEGRAL_WINDOW] = 0.0f;
        }
        return (uint32_t)(output + 0.5f); // floats truncate when being cast to int - e.g 1.9 becomes 1. by adding 0.5 first, we accomplish rounding to the nearest int instead.

    }


    uint32_t filter_rpm(uint32_t new_sample)
    {
        filter.buffer[filter.index] = new_sample;
        filter.index = (filter.index + 1) % RPM_FILTER_WINDOW;
        if (filter.filled < RPM_FILTER_WINDOW) filter.filled++;

        uint64_t sum = 0;
        for (uint8_t i = 0; i < filter.filled; i++) sum += filter.buffer[i];
        return (uint32_t)(sum / filter.filled);
    }

private:
    float Kp;
    float Ki;
    float Kd;
    float feed_forward_curve_offset;
    float feed_forward_curve_exponent;
    uint32_t update_frequency_us;
    PIDState pidstate = {0};
    RPMFilterState filter = {0};
};