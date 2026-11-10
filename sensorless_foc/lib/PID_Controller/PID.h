#pragma once

/*
 * Back-calculation PID controller with output ramp limiting.
 *
 * Usage:
 *   PID_Controller pid(kp, ki, kd, rampLimit, backCalculationGain);
 *   float output = pid.update(target - measured);
 *   appliedOutput = limitOutput(output); // optional downstream output limiter
 *   pid.backCalculate(appliedOutput); // after a downstream output limiter
 *   pid.reset(); // Clear controller history before starting again.
 *
 * ramp_limit limits output change per second; 0 disables it and a positive
 * value enables it. 
 * 
 * back_calculation_gain is Kaw in 1/s; it tracks output
 * saturation by correcting the integral. It defaults to 0 (disabled).
 * 
 * The first update returns the P term without applying the output ramp.
 * A second update in the same micros() tick returns the previous output.
 */

class PID_Controller
{
public:
    PID_Controller();

    PID_Controller(float kp, float ki, float kd, float ramp_limit,
                   float back_calculation_gain = 0.0f);

    // Updates parameters without resetting history.
    void configure(float kp, float ki, float kd, float ramp_limit,
                   float back_calculation_gain = 0.0f);

    // The first call returns the P term without applying the output ramp.
    // A call in the same micros() tick returns the previous output.
    float update(float error);

    // Feeds back an output limited downstream of this PID.
    void backCalculate(float applied_output);

    float getProportionalOutput() const;
    float getIntegralOutput() const;
    float getDerivativeOutput() const;

    // Clears controller history; the next update uses the first-call behavior.
    void reset();

private:
    float kp;
    float ki;
    float kd;

    float output_ramp_limit;
    float back_calculation_gain;
    float error_prev;
    float output_prev;
    float integral_prev;
    float proportional_output;
    float derivative_output;
    unsigned timestamp_prev;
    float last_update_interval;
    float last_update_output;
    bool has_sample;
    bool back_calculation_pending;
};
