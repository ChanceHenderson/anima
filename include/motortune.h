#pragma once
#include <math.h>
#include <stdint.h>

// Per-wheel tuning data: a feed-forward curve plus a small grid of PID gains
// keyed by target RPM. PIDControl interpolates this grid whenever the target
// changes (gain scheduling), so one tune covers the whole RPM range instead
// of being correct at only one setpoint.

#define MAX_GAIN_POINTS 3

// Hand-tuned defaults: Plus motors, Robo Spirit wheels, 4S battery.
// Used whenever the tuning file is missing or fails validation.
// NOTE: Ki is NOT the old windowed-integral value (1.6) - the controller now
// uses a true accumulating integral, so Ki means throttle per (RPM * s) of
// accumulated error. 4.0 trims a 100-throttle feed-forward error from a
// 100 RPM residual in ~0.25 s without threatening stability.
#define DEFAULT_KP 0.6f               // throttle per RPM
#define DEFAULT_KI 4.0f               // throttle per (RPM * s)
#define DEFAULT_KD 0.0003f            // throttle*s per RPM
#define DEFAULT_FF_OFFSET 56074.0f    // RPM = -offset + exponent * ln(throttle)
#define DEFAULT_FF_EXPONENT 12952.0f

typedef struct
{
    float rpm;
    float kp;
    float ki;
    float kd;
} GainPoint;

class MotorTune {
public:
    float ff_offset = DEFAULT_FF_OFFSET;
    float ff_exponent = DEFAULT_FF_EXPONENT;
    uint8_t num_points = 1;
    GainPoint points[MAX_GAIN_POINTS] = {
        { 20000.0f, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD },
        { 0.0f, 0.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f, 0.0f }
    };

    // throttle needed to hold `rpm` according to the feed-forward curve
    float feed_forward(float rpm) const
    {
        return expf((rpm + ff_offset) / ff_exponent);
    }

    // Piecewise-linear interpolation of the gain grid, clamped at both ends.
    void gains_at(float rpm, float &kp, float &ki, float &kd) const
    {
        if (num_points == 0)
        {
            kp = DEFAULT_KP;
            ki = DEFAULT_KI;
            kd = DEFAULT_KD;
            return;
        }
        const GainPoint &last = points[num_points - 1];
        if (num_points == 1 || rpm <= points[0].rpm)
        {
            kp = points[0].kp;
            ki = points[0].ki;
            kd = points[0].kd;
            return;
        }
        if (rpm >= last.rpm)
        {
            kp = last.kp;
            ki = last.ki;
            kd = last.kd;
            return;
        }
        for (uint8_t i = 0; i + 1 < num_points; i++)
        {
            if (rpm <= points[i + 1].rpm)
            {
                float f = (rpm - points[i].rpm) / (points[i + 1].rpm - points[i].rpm);
                kp = points[i].kp + f * (points[i + 1].kp - points[i].kp);
                ki = points[i].ki + f * (points[i + 1].ki - points[i].ki);
                kd = points[i].kd + f * (points[i + 1].kd - points[i].kd);
                return;
            }
        }
        kp = last.kp;
        ki = last.ki;
        kd = last.kd;
    }

    // Multiply every grid point's gains (used by the autotune verification pass)
    void scale_gains(float pi_factor, float d_factor)
    {
        for (uint8_t i = 0; i < num_points; i++)
        {
            points[i].kp *= pi_factor;
            points[i].ki *= pi_factor;
            points[i].kd *= d_factor;
        }
    }

    void set_defaults()
    {
        ff_offset = DEFAULT_FF_OFFSET;
        ff_exponent = DEFAULT_FF_EXPONENT;
        num_points = 1;
        points[0] = { 20000.0f, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD };
        points[1] = { 0.0f, 0.0f, 0.0f, 0.0f };
        points[2] = { 0.0f, 0.0f, 0.0f, 0.0f };
    }

    bool is_sane() const
    {
        if (!(isfinite(ff_offset) && isfinite(ff_exponent))) return false;
        if (ff_exponent < 500.0f || ff_exponent > 100000.0f) return false;
        if (ff_offset < 0.0f || ff_offset > 500000.0f) return false;
        if (num_points < 1 || num_points > MAX_GAIN_POINTS) return false;
        for (uint8_t i = 0; i < num_points; i++)
        {
            const GainPoint &g = points[i];
            if (!(isfinite(g.rpm) && isfinite(g.kp) && isfinite(g.ki) && isfinite(g.kd))) return false;
            if (g.rpm < 1000.0f || g.rpm > 50000.0f) return false;
            if (g.kp <= 0.0f || g.kp > 50.0f) return false;
            if (g.ki < 0.0f || g.ki > 1000.0f) return false;
            if (g.kd < 0.0f || g.kd > 0.1f) return false;
            if (i > 0 && g.rpm <= points[i - 1].rpm) return false;
        }
        return true;
    }
};
