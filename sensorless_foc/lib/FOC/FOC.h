#pragma once

/*
 * FOC_Controller usage:
 * 1. Create an instance and call configure() with the PWM pins, motor parameters,
 *    and current-sensor parameters.
 *    If using the SMO, call setSMOParameters() after configure() to provide
 *    the motor resistance and inductance. The observer runs in active modes.
 * 2. Velocity and current limits default to +/-150 rad/s and +/-0.5 A.
 *    Optionally override them and configure the PID(s), then call
 *    calibrateSensor(); it calibrates current zero before encoder alignment.
 *    Do not enable closed-loop control if calibration fails.
 * 3. Set a target, select a mode, and call run() repeatedly from loop().
 *    Entering FOC_MODE_STOPPED clears targets and controller/observer history;
 *    set a new target before starting a closed-loop mode again.
 *
 * Example (velocity mode):
 *   FOC_Controller foc;
 *   void setup() {
 *       foc.configure(PIN_A, PIN_B, PIN_C, pole_pairs, 1, bus_voltage,
 *                     CURRENT_PIN_A, CURRENT_PIN_B, shunt_resistor_ohm,
 *                     amp_gain);
 *       // Optional: override the default velocity and current limits.
 *       // foc.setVelocityLimits(-max_velocity, max_velocity); //  default +/-150 rad/s
 *       // foc.setCurrentLimits(-max_current, max_current); // default +/-0.5 A
 *       // foc.setVelocityFilterCutoff(10.0f); // optional, defaults to 10 Hz
 *       foc.setVelPID(kp, ki, kd, ramp_limit);
 *       if (foc.calibrateSensor()) {
 *           foc.setTargetVelocity(target_velocity);
 *           foc.setMode(FOC_MODE_VELOCITY_CLOSED_LOOP);
 *       }
 *   }
 *   void loop() { foc.run(); }
 *
 * For position mode, configure the position PID and use setTargetPosition() with
 * FOC_MODE_POSITION_CLOSED_LOOP instead.
 * 
 * default 10.0Hz 100.0Hz
 */

#include "LowPassFilter.h"
#include "AS5600.h"
#include "PID.h"

typedef enum {
    FOC_MODE_STOPPED = 0,
    FOC_MODE_POSITION_CLOSED_LOOP = 1,
    FOC_MODE_VELOCITY_CLOSED_LOOP = 2,
    FOC_MODE_CURRENT_LOOP = 3
} FOCMode;

class FOC_Controller
{
public:
    FOC_Controller();

    // Current filter cutoff is optional and defaults to 100 Hz.
    // Configure the motor and required current sensor before calibration or control.
    // sensor_dir sets the positive sensor direction and must be 1 or -1.
    void configure(int pwm_pin_A, int pwm_pin_B, int pwm_pin_C,
                   int pole_pairs, int sensor_dir,
                   float bus_voltage,
                   int adc_pinA, int adc_pinB,
                   float shunt_resistor_ohm, float amp_gain);

    // Sets the SMO phase resistance (ohms) and inductance (henries).
    // Call after configure() when using the SMO.
    void setSMOParameters(float phase_resistance_ohm,
                          float phase_inductance_h);

    // Velocity and current limits have defaults and can be overridden.
    // Entering FOC_MODE_STOPPED clears targets and controller/observer history
    // while retaining motor, sensor, calibration, and controller configuration.
    bool setMode(FOCMode mode);

    // Overrides the default signed velocity limits in rad/s; min must be <= max.
    void setVelocityLimits(float min_velocity, float max_velocity);

    // Overrides the default signed current limits in A; min must be <= max.
    void setCurrentLimits(float min_current, float max_current);

    // Sets the velocity feedback low-pass cutoff in Hz; call after configure().
    // cutoff_hz must be positive and finite. Changing it does not reset the filter.
    void setVelocityFilterCutoff(float cutoff_hz);

    // Sets both current feedback filter cutoffs in Hz without resetting them.
    void setCurrentFilterCutoff(float cutoff_hz);

    // Velocity PID output is an Iq reference in A; ramp_limit is A/s.
    void setVelPID(float kp, float ki, float kd, float ramp_limit);

    // Position PID output is a velocity reference in rad/s; ramp_limit is rad/s^2.
    void setPosPID(float kp, float ki, float kd, float ramp_limit);

    // Current PID output is voltage in V; ramp_limit is V/s.
    void setCurPID(float kp, float ki, float kd, float ramp_limit);

    void setTargetVelocity(float target_velocity);

    void setTargetPosition(float target_position);

    // only used for debugging current closed-loop control, not for normal operation
    void SetTargetCurrent(float target_current);

    float getActualVelocity() const;

    // Returns the turn-tracked angle with turns before the latest position-mode
    // entry excluded.
    double getActualPosition() const;

    // Calibrates current zero before encoder alignment; returns false if the
    // encoder cannot be read during alignment.
    bool calibrateSensor();

    // Runs the selected mode; sensor read failures produce zero phase voltage.
    void run();

private:
    static int controller_count;
    int controller_id;
    FOCMode mode;

    /* PWM */
    int pwm_pin_A, pwm_pin_B, pwm_pin_C;
    int ledc_channel_A, ledc_channel_B, ledc_channel_C;

    /* Motor and inverter parameters */
    int pole_pairs;
    float bus_voltage;
    float min_velocity, max_velocity;
    float min_current, max_current;
    float limitVelocity(float velocity) const;
    float limitCurrent(float current) const;

    /* Position sensor */
    Sensor_AS5600 sensor;
    int sensor_dir;
    double position_offset; // record how many loops have passed since the last position reset
    float zero_electric_angle;

    /* Current Sensor */
    int adc_pinA, adc_pinB;                // pins for current sensing
    float shunt_resistor_ohm;              // shunt resistor value in ohms
    float amp_gain;                        // amplifier gain
    float volts_to_amps_ratio;             // conversion ratio from volts to amps. unit: amps/volt
    float raw_current_a, raw_current_b;    // raw current values in amps
    float current_a, current_b, current_c; // filtered current values in amps
    float zero_drift_a, zero_drift_b;      // zero drift values for calibration
    LowPassFilter current_a_lpf;
    LowPassFilter current_b_lpf;
    void calibrateCurrentSensorZeroDrift();
    void updateCurrentSensor();

    /* Position loop */
    double target_position, actual_position;
    PID_Controller vel_pid;

    /* Velocity loop */
    float target_velocity, actual_velocity;
    PID_Controller pos_pid;
    LowPassFilter vel_lpf;

    /* Current loop */
    float target_current;             // Target q-axis current in amps. Used for debugging current closed-loop control.
    float Ialpha, Ibeta;              // Latest alpha- and beta-axis currents, in amps.
    float Iq, Id, error_Iq, error_Id; // Latest q- and d-axis currents, in amps.
    float current_ud_pid_output;
    float current_voltage_scale;
    float current_uq_applied, current_ud_applied;
    void currentLoop(float target_Iq, float target_Id); // A -> V; input Iq, Id; output Uq, Ud
    PID_Controller Iq_pid, Id_pid;

    /* Inverter */
    float setPhaseVoltage(float Uq, float Ud, float angle_el);

    /* internal parameters */
    float m_angle, e_angle; // Latest mechanical and electrical angles, in radians.
    float Uq, Ud, error;    // Latest q- and d-axis voltages, in volts.
    float Ualpha, Ubeta;    // Latest alpha- and beta-axis voltages, in volts.

    /* SMO */
    float Rs;                     // Phase resistance in ohms.
    float Ls;                     // Phase inductance in henries.
    float Ealpha, Ebeta;          // Latest estimated back-EMF in volts.
    float EstIalpha, EstIbeta;    // Latest estimated alpha- and beta-axis currents, in amps.
    float SMO_Est_theta;          // Latest estimated electrical angle, in radians.
    unsigned smo_last_time = 0;   // Last time SMO was updated, in microseconds.
    bool smo_initialized = false; // Indicates whether the SMO has been initialized.
    void ClearSMO();           // clear the SMO state variables
    void resetRuntimeState();
    void SMO_estimate();

    void runPositionClosedLoop();
    void runVelocityClosedLoop();
    void runVelocityLoop(float velocity_reference);
    void runCurrentClosedLoop();
};