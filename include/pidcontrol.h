#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "flywheelcontrolalgorithm.h"
#include "motortune.h"

// Closed-loop flywheel control: exponential feed-forward carries the load,
// PID trims around it. Gains are interpolated from the MotorTune gain grid
// at every target change (gain scheduling), so the loop stays crisp at 12k
// and stable at 30k with one tune.
//
// Design notes (v2, replacing the moving-window implementation):
//  - True accumulating integral. Its job is to absorb feed-forward error
//    (battery sag, wheel wear), so it is clamped to a small authority band
//    and frozen while the output is saturated in the direction of the error
//    (conditional integration). During a full-throttle spinup it stays
//    parked at zero - no windup bulge to burn off at the top.
//  - Fixed nominal dt. The loop is fixed-rate; measuring dt just multiplied
//    quantization noise on jittery iterations (the old code could divide the
//    derivative by as little as 1 us).
//  - Derivative on measurement, double-filtered: the RPM moving average plus
//    a first-order low-pass on the derivative itself. It brakes overshoot
//    without amplifying eRPM quantization noise.

// RPM measurement moving-average window
#define RPM_FILTER_WINDOW 6
// Max integral contribution in throttle counts, either direction. It only
// needs to trim feed-forward error, never to lift the whole load.
#define PID_INTEGRAL_LIMIT 400.0f
// First-order low-pass coefficient for the derivative term. At a 5 kHz loop
// this puts the cutoff around 40 Hz.
#define PID_D_LPF_ALPHA 0.05f

typedef struct {
    uint8_t index;
    uint8_t filled;
    uint32_t buffer[RPM_FILTER_WINDOW];
} RPMFilterState;

class PIDControl : public FlywheelControlAlgorithm {
public:
    PIDControl(const MotorTune &tune, uint32_t update_period_us)
    {
        this->tune = tune;
        this->dt = (float)update_period_us * 1e-6f;
        this->tune.gains_at(0.0f, kp, ki, kd);  // start at the low end of the schedule
        reset_state();
    }

    void set_throttle(uint32_t throttle) override
    {
        this->throttle = throttle;
    }

    void set_target_rpm(uint32_t rpm) override
    {
        if (rpm == target_rpm) return;
        target_rpm = rpm;
        if (rpm > 0)
        {
            tune.gains_at((float)rpm, kp, ki, kd);
            ff_throttle = tune.feed_forward((float)rpm);
        }
        // The integral carries over between nonzero targets - it holds the
        // battery/feed-forward trim, which doesn't change with the setpoint.
    }

    uint32_t calc_next_throttle(uint32_t current_rpm) override
    {
        if (target_rpm == 0)
        {
            reset_state();
            throttle = 0;
            return 0;
        }

        float m = (float)filter_rpm(current_rpm);
        float error = (float)target_rpm - m;

        // Derivative on (already averaged) measurement, then low-passed.
        // First sample after a reset has no history - report zero rather
        // than a spike.
        float d_raw = has_last_measured ? -(m - last_measured) / dt : 0.0f;
        has_last_measured = true;
        last_measured = m;
        d_filtered += PID_D_LPF_ALPHA * (d_raw - d_filtered);

        float p_term = kp * error;
        float d_term = kd * d_filtered;

        // Conditional integration: probe the output with the current
        // integral, and only integrate when we aren't pushing further into
        // a saturated output.
        float out = ff_throttle + p_term + integral + d_term;
        bool sat_high = out >= 2000.0f;
        bool sat_low = out <= 0.0f;
        if (!((sat_high && error > 0.0f) || (sat_low && error < 0.0f)))
        {
            integral += ki * error * dt;
            if (integral > PID_INTEGRAL_LIMIT) integral = PID_INTEGRAL_LIMIT;
            else if (integral < -PID_INTEGRAL_LIMIT) integral = -PID_INTEGRAL_LIMIT;
            out = ff_throttle + p_term + integral + d_term;
        }

        if (out > 2000.0f) out = 2000.0f;
        else if (out < 0.0f) out = 0.0f;
        throttle = (uint32_t)(out + 0.5f); // +0.5 rounds instead of truncating
        return throttle;
    }

    void reset_state()
    {
        integral = 0.0f;
        d_filtered = 0.0f;
        last_measured = 0.0f;
        has_last_measured = false;
        filter.index = 0;
        filter.filled = 0;
        for (uint8_t i = 0; i < RPM_FILTER_WINDOW; i++) filter.buffer[i] = 0;
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
    MotorTune tune;
    float kp = DEFAULT_KP;
    float ki = DEFAULT_KI;
    float kd = DEFAULT_KD;
    float ff_throttle = 0.0f;
    float dt;
    float integral = 0.0f;
    float d_filtered = 0.0f;
    float last_measured = 0.0f;
    bool has_last_measured = false;
    RPMFilterState filter = {0};
};
