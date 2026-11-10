#include "PID.h"
#include <Arduino.h>

PID_Controller::PID_Controller()
    : kp(0.0f), ki(0.0f), kd(0.0f),
      output_ramp_limit(0.0f), back_calculation_gain(0.0f),
      error_prev(0.0f), output_prev(0.0f), integral_prev(0.0f),
      proportional_output(0.0f), derivative_output(0.0f),
      timestamp_prev(0), last_update_interval(0.0f), last_update_output(0.0f),
      has_sample(false), back_calculation_pending(false)
{
}

PID_Controller::PID_Controller(float kp, float ki, float kd, float ramp_limit,
                               float back_calculation_gain)
    : kp(kp), ki(ki), kd(kd), output_ramp_limit(ramp_limit),
      back_calculation_gain(back_calculation_gain),
      error_prev(0.0f), output_prev(0.0f), integral_prev(0.0f),
      proportional_output(0.0f), derivative_output(0.0f),
      timestamp_prev(0), last_update_interval(0.0f), last_update_output(0.0f),
      has_sample(false), back_calculation_pending(false)
{
}

void PID_Controller::configure(float kp, float ki, float kd, float ramp_limit,
                               float back_calculation_gain)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->output_ramp_limit = ramp_limit;
    this->back_calculation_gain = back_calculation_gain;
    back_calculation_pending = false;
    last_update_interval = 0.0f;
}

float PID_Controller::update(float error)
{
    const uint32_t timestamp_now = micros();
    back_calculation_pending = false;
    last_update_interval = 0.0f;

    if (!has_sample)
    {
        error_prev = error;
        proportional_output = kp * error;
        derivative_output = 0.0f;
        output_prev = proportional_output;
        last_update_output = output_prev;
        timestamp_prev = timestamp_now;
        has_sample = true;
        return output_prev;
    }

    const uint32_t elapsedUs = timestamp_now - timestamp_prev;
    if (elapsedUs == 0) // if the time hasn't advanced, return the previous output to avoid division by zero
        return output_prev;

    const float Ts = static_cast<float>(elapsedUs) * 1.0e-6f;

    float proportional = kp * error;
    float integral = integral_prev + ki * Ts * 0.5f * (error + error_prev);
    float derivative = kd * (error - error_prev) / Ts;
    proportional_output = proportional;
    derivative_output = derivative;

    const float unconstrained_output = proportional + integral + derivative;
    float output = unconstrained_output;

    // Limit the output rate of change.
    if (output_ramp_limit > 0)
    {
        float output_rate = (output - output_prev) / Ts;
        if (output_rate > output_ramp_limit)
            output = output_prev + output_ramp_limit * Ts;
        else if (output_rate < -output_ramp_limit)
            output = output_prev - output_ramp_limit * Ts;
    }

    if (back_calculation_gain > 0.0f && ki != 0.0f)
        integral += back_calculation_gain * Ts * (output - unconstrained_output);

    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    last_update_interval = Ts;
    last_update_output = output;
    back_calculation_pending = true;

    return output;
}

void PID_Controller::backCalculate(float applied_output)
{
    if (back_calculation_pending && back_calculation_gain > 0.0f && ki != 0.0f)
        integral_prev += back_calculation_gain * last_update_interval * (applied_output - last_update_output);

    output_prev = applied_output;
    back_calculation_pending = false;
}

float PID_Controller::getProportionalOutput() const
{
    return proportional_output;
}

float PID_Controller::getIntegralOutput() const
{
    return integral_prev;
}

float PID_Controller::getDerivativeOutput() const
{
    return derivative_output;
}

void PID_Controller::reset()
{
    error_prev = 0.0f;
    output_prev = 0.0f;
    integral_prev = 0.0f;
    proportional_output = 0.0f;
    derivative_output = 0.0f;
    timestamp_prev = 0;
    last_update_interval = 0.0f;
    last_update_output = 0.0f;
    has_sample = false;
    back_calculation_pending = false;
}
