#include "FOC.h"
#include <Arduino.h>

int FOC_Controller::controller_count = 0;

#define _3PI_2 (4.71238898038f)
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f

// normalize angle into [0, 2PI]
static float normalize_angle(float angle)
{
    float a = fmod(angle, 2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
}

template <typename T>
static inline T clamp(T val, T low, T high)
{
    if (val < low)
        val = low;
    else if (val > high)
        val = high;

    return val;
};

static float calculateBackCalculationGain(float kp, float ki)
{
    // Use the integral time as the tracking time constant.
    return kp != 0.0f ? fabsf(ki / kp) : 0.0f;
}

void FOC_Controller::configure(int pwm_pin_A, int pwm_pin_B, int pwm_pin_C, 
    int pole_pairs, int sensor_dir, 
    float bus_voltage,
    int adc_pinA, int adc_pinB,
    float shunt_resistor_ohm, float amp_gain)
{
    this->pwm_pin_A = pwm_pin_A;
    this->pwm_pin_B = pwm_pin_B;
    this->pwm_pin_C = pwm_pin_C;
    this->pole_pairs = pole_pairs;
    this->sensor_dir = sensor_dir;
    this->bus_voltage = bus_voltage;
    this->min_velocity = 0.0f;
    this->max_velocity = 0.0f;
    this->min_current = 0.0f;
    this->max_current = 0.0f;
    this->velocity_limits_configured = false;
    this->current_limits_configured = false;
    this->target_velocity = 0;
    this->target_position = 0;
    this->target_current = 0;
    this->actual_velocity = 0;
    this->actual_position = 0;
    this->position_offset = 0;
    this->zero_electric_angle = 0.0f;
    this->current_ud_pid_output = 0.0f;
    this->current_voltage_scale = 1.0f;
    this->current_uq_applied = 0.0f;
    this->current_ud_applied = 0.0f;
    this->adc_pinA = adc_pinA;
    this->adc_pinB = adc_pinB;
    this->shunt_resistor_ohm = shunt_resistor_ohm;
    this->amp_gain = amp_gain;
    this->volts_to_amps_ratio = 1.0f / shunt_resistor_ohm / amp_gain;

    this->mode = FOC_MODE_STOPPED;
    vel_pid.reset();
    pos_pid.reset();
    Iq_pid.reset();
    Id_pid.reset();
    vel_lpf.reset();
    current_a_lpf.reset();
    current_b_lpf.reset();

    ledc_channel_A = controller_id * 3 + 0;
    ledc_channel_B = controller_id * 3 + 1;
    ledc_channel_C = controller_id * 3 + 2;

    ledcSetup(ledc_channel_A, 30000, 8); // pwm频道, 频率, 精度
    ledcSetup(ledc_channel_B, 30000, 8); // pwm频道, 频率, 精度
    ledcSetup(ledc_channel_C, 30000, 8); // pwm频道, 频率, 精度
    ledcAttachPin(pwm_pin_A, ledc_channel_A);
    ledcAttachPin(pwm_pin_B, ledc_channel_B);
    ledcAttachPin(pwm_pin_C, ledc_channel_C);

    pinMode(adc_pinA, INPUT);
    pinMode(adc_pinB, INPUT);

    AS5600Status sensorStatus = sensor.begin(Wire, 19, 18, 400000UL);
    if (sensorStatus != AS5600Status::Ok) {
        Serial.printf("AS5600 initialization failed: %u\n",
                      static_cast<unsigned>(sensorStatus));
        return;
    }

    // vel_pid = PID_Controller{.P = 2, .I = 0, .D = 0, .ramp = 100000, .limit = bus_voltage / 2};
    vel_pid.configure(2, 0, 0, 100000, calculateBackCalculationGain(2, 0));
    pos_pid.configure(2, 0.8, 0, 100000, calculateBackCalculationGain(2, 0.8f));
    Iq_pid.configure(2, 0.8, 0, 100000, calculateBackCalculationGain(2, 0.8f));
    Id_pid.configure(2, 0.8, 0, 100000, calculateBackCalculationGain(2, 0.8f));

    vel_lpf.configure(10.0f);
    current_a_lpf.configure(100.0f);
    current_b_lpf.configure(100.0f);
}

bool FOC_Controller::setMode(FOCMode mode)
{
    const bool requires_velocity_limits =
        mode == FOC_MODE_POSITION_CLOSED_LOOP ||
        mode == FOC_MODE_VELOCITY_CLOSED_LOOP;
    const bool requires_current_limits =
        mode == FOC_MODE_POSITION_CLOSED_LOOP ||
        mode == FOC_MODE_VELOCITY_CLOSED_LOOP ||
        mode == FOC_MODE_CURRENT_LOOP;

    if ((requires_velocity_limits && !velocity_limits_configured) ||
        (requires_current_limits && !current_limits_configured))
    {
        this->mode = FOC_MODE_STOPPED;
        setPhaseVoltage(0, 0, 0);
        return false;
    }

    if (this->mode != mode)
    {
        if (mode == FOC_MODE_POSITION_CLOSED_LOOP)
        {
            pos_pid.reset();
            vel_pid.reset();
            position_offset = sensor.getAngle() - sensor.getMechanicalAngle();
            actual_position = sensor.getAngle() - position_offset;
        }
        else if (mode == FOC_MODE_VELOCITY_CLOSED_LOOP)
        {
            vel_pid.reset();
        }
        else if (mode == FOC_MODE_CURRENT_LOOP)
        {
            Iq_pid.reset();
            Id_pid.reset();
        }
    }

    this->mode = mode;
    setPhaseVoltage(0, 0, 0);
    return true;
}

void FOC_Controller::setVelocityLimits(float min_velocity, float max_velocity)
{
    this->min_velocity = min_velocity;
    this->max_velocity = max_velocity;
    velocity_limits_configured = true;
    target_velocity = limitVelocity(target_velocity);
}

void FOC_Controller::setCurrentLimits(float min_current, float max_current)
{
    this->min_current = min_current;
    this->max_current = max_current;
    current_limits_configured = true;
    target_current = limitCurrent(target_current);
}

float FOC_Controller::limitVelocity(float velocity) const
{
    return velocity_limits_configured
               ? clamp(velocity, min_velocity, max_velocity)
               : velocity;
}

float FOC_Controller::limitCurrent(float current) const
{
    return current_limits_configured
               ? clamp(current, min_current, max_current)
               : current;
}

float FOC_Controller::setPhaseVoltage(float Uq, float Ud, float e_angle)
{
    // Vector magnitude limit
    // Keep the phase-voltage vector within the linear range of centered SPWM.
    const float voltage_magnitude = sqrtf(Uq * Uq + Ud * Ud);
    const float voltage_limit = bus_voltage * 0.5f;
    float voltage_scale = 1.0f;
    if (voltage_magnitude > voltage_limit)
    {
        voltage_scale = voltage_limit / voltage_magnitude;
        Uq *= voltage_scale;
        Ud *= voltage_scale;
    }

    // Wrap to [0, 2π]
    e_angle = normalize_angle(e_angle);

    // inverse park transform: Ud, Uq -> Ualpha, Ubeta
    float Ualpha, Ubeta;
    Ualpha = Ud * cos(e_angle) - Uq * sin(e_angle);
    Ubeta = Ud * sin(e_angle) + Uq * cos(e_angle);

    // inverse clarke transform: Ualpha, Ubeta -> Ua, Ub, Uc
    float Ua, Ub, Uc;
    Ua = Ualpha;
    Ub = (sqrt(3) * Ubeta - Ualpha) / 2;
    Uc = (-Ualpha - sqrt(3) * Ubeta) / 2;

    // spwm
    float dc_bias = bus_voltage / 2;
    float duty_a, duty_b, duty_c;
    Ua += dc_bias, Ub += dc_bias, Uc += dc_bias;
    duty_a = clamp(Ua / bus_voltage, 0.0f, 1.0f);
    duty_b = clamp(Ub / bus_voltage, 0.0f, 1.0f);
    duty_c = clamp(Uc / bus_voltage, 0.0f, 1.0f);

    // output pwm
    ledcWrite(ledc_channel_A, duty_a * 255);
    ledcWrite(ledc_channel_B, duty_b * 255);
    ledcWrite(ledc_channel_C, duty_c * 255);
    return voltage_scale;
}

void FOC_Controller::setTargetVelocity(float target_velocity)
{
    this->target_velocity = limitVelocity(target_velocity);
}

void FOC_Controller::setTargetPosition(float target_position)
{
    this->target_position = target_position;
}

void FOC_Controller::setVelPID(float kp, float ki, float kd, float ramp_limit)
{
    vel_pid.configure(kp, ki, kd, ramp_limit, calculateBackCalculationGain(kp, ki));
}

void FOC_Controller::setPosPID(float kp, float ki, float kd, float ramp_limit)
{
    pos_pid.configure(kp, ki, kd, ramp_limit, calculateBackCalculationGain(kp, ki));
}

void FOC_Controller::setCurPID(float kp, float ki, float kd, float ramp_limit)
{
    const float back_calculation_gain = calculateBackCalculationGain(kp, ki);
    Iq_pid.configure(kp, ki, kd, ramp_limit, back_calculation_gain);
    Id_pid.configure(kp, ki, kd, ramp_limit, back_calculation_gain);
}

void FOC_Controller::setVelocityFilterCutoff(float cutoff_hz)
{
    vel_lpf.configure(cutoff_hz);
}

void FOC_Controller::setCurrentFilterCutoff(float cutoff_hz)
{
    current_a_lpf.configure(cutoff_hz);
    current_b_lpf.configure(cutoff_hz);
}

float FOC_Controller::getActualVelocity() const
{
    return actual_velocity;
}

double FOC_Controller::getActualPosition() const
{
    return actual_position;
}

void FOC_Controller::run()
{
    // read the sensor data even if the foc is stopped
    if (sensor.update() != AS5600Status::Ok) {
        setPhaseVoltage(0, 0, 0);
        return;
    }

    actual_position = sensor.getAngle() - position_offset;
    actual_velocity = vel_lpf.update(sensor_dir * sensor.getVelocity());

    if (mode == FOC_MODE_POSITION_CLOSED_LOOP ||
        mode == FOC_MODE_VELOCITY_CLOSED_LOOP ||
        mode == FOC_MODE_CURRENT_LOOP)
        updateCurrentSensor();

    if (mode == FOC_MODE_POSITION_CLOSED_LOOP)
        runPositionClosedLoop();
    else if (mode == FOC_MODE_VELOCITY_CLOSED_LOOP)
        runVelocityClosedLoop();
    else if (mode == FOC_MODE_CURRENT_LOOP)
        runCurrentClosedLoop();
    else
        setPhaseVoltage(0, 0, 0);

    static uint32_t lastPrintTime = 0;
    if (millis() - lastPrintTime >= 200) {
        lastPrintTime = millis();
        // serial printing blocks the FOC loop
        // if (mode == FOC_MODE_CURRENT_LOOP) {
        //     Serial.printf(
        //         "chennel 0: %f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
        //         /* 0 */ actual_position, 
        //         /* 1 */ actual_velocity, 
        //         /* 2 */ target_current, 
        //         /* 3 */ Iq, 
        //         /* 4 */ Id, 
        //         /* 5 */ Uq, 
        //         /* 6 */ Ud,
        //         /* 7 */ error_Iq,
        //         /* 8 */ Iq_pid.getProportionalOutput(),
        //         /* 9 */ Iq_pid.getIntegralOutput(),
        //         /* 10 */ Iq_pid.getDerivativeOutput(),
        //         /* 11 */ current_uq_applied, 
        //         /* 12 */ error_Id,
        //         /* 13 */ Id_pid.getProportionalOutput(),
        //         /* 14 */ Id_pid.getIntegralOutput(),
        //         /* 15 */ Id_pid.getDerivativeOutput(),
        //         /* 16 */ current_ud_pid_output, 
        //         /* 17 */ current_ud_applied,
        //         /* 18 */ current_voltage_scale, 
        //         /* 19 */ current_a, 
        //         /* 20 */ current_b, 
        //         /* 21 */ e_angle);
        // } else {
        //     Serial.printf("chennel 0: %f,%f,%f,%f,%f,%f,%f\n",
        //                   actual_position,
        //                   actual_velocity,
        //                   target_current,
        //                   Iq,
        //                   Id,
        //                   Uq,
        //                   Ud);
        // }
    }
}

void FOC_Controller::runPositionClosedLoop()
{
    const float position_error =
        sensor_dir * static_cast<float>(target_position - actual_position);
    const float requested_velocity = pos_pid.update(position_error);
    const float velocity_reference = limitVelocity(requested_velocity);
    pos_pid.backCalculate(velocity_reference);
    runVelocityLoop(velocity_reference);
}


void FOC_Controller::runVelocityClosedLoop()
{
    runVelocityLoop(limitVelocity(target_velocity));
}

void FOC_Controller::runVelocityLoop(float velocity_reference)
{
    error = velocity_reference - actual_velocity;

    const float requested_current = vel_pid.update(error);
    target_current = limitCurrent(requested_current);
    vel_pid.backCalculate(target_current);

    currentLoop(target_current, 0.0f);
}

void FOC_Controller::currentLoop(float target_Iq, float target_Id)
{
    m_angle = sensor.getMechanicalAngle();
    e_angle = normalize_angle(sensor_dir * pole_pairs * m_angle - zero_electric_angle);

    // clark transform: Ia, Ib -> I_alpha, I_beta
    float I_alpha = current_a;
    float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

    // park transform: I_alpha, I_beta -> Id, Iq
    Iq = I_beta * cos(e_angle) - I_alpha * sin(e_angle);
    Id = I_alpha * cos(e_angle) + I_beta * sin(e_angle);

    error_Iq = target_Iq - Iq;
    error_Id = target_Id - Id;

    Uq = Iq_pid.update(error_Iq);
    Ud = Id_pid.update(error_Id);
    current_ud_pid_output = Ud;

    // Ud = 0;
    const float voltage_scale = setPhaseVoltage(Uq, Ud, e_angle);
    current_voltage_scale = voltage_scale;
    current_uq_applied = Uq * voltage_scale;
    current_ud_applied = Ud * voltage_scale;
    Iq_pid.backCalculate(Uq * voltage_scale);
    Id_pid.backCalculate(Ud * voltage_scale);
}

void FOC_Controller::runCurrentClosedLoop()
{
    target_current = limitCurrent(target_current);
    currentLoop(target_current, 0.0f);
}

void FOC_Controller::SetTargetCurrent(float target_current)
{
    this->target_current = limitCurrent(target_current);
}

FOC_Controller::FOC_Controller()
{
    controller_id = controller_count++;
    zero_electric_angle = 0.0f;
}

bool FOC_Controller::calibrateSensor()
{
    // Ensure the motor is unpowered while measuring the current-sensor offsets.
    setPhaseVoltage(0, 0, 0);
    calibrateCurrentSensorZeroDrift();
    current_a_lpf.reset();
    current_b_lpf.reset();

    // Align the sensor to the motor's electrical angle.
    setPhaseVoltage(3, 0, _3PI_2);
    delay(1000);
    if (sensor.update() != AS5600Status::Ok) {
        setPhaseVoltage(0, 0, _3PI_2);
        Serial.println("AS5600 update failed during alignment.");
        return false;
    }
    zero_electric_angle = normalize_angle(
        (float)(sensor_dir * pole_pairs) * sensor.getMechanicalAngle());
    setPhaseVoltage(0, 0, _3PI_2);
    return true;
}

/* Current Sensor ----------------------------- */

// ESP32 ADC Parameters
#define ADC_VOLTAGE 3.3f
#define ADC_RESOLUTION 4095.0f
#define ADC_CONV (ADC_VOLTAGE / ADC_RESOLUTION) // Voltage = ADC reading * ADC_CONV

void FOC_Controller::calibrateCurrentSensorZeroDrift()
{
    // Calibrate the zero drift of the current sensor.
    // Read multiple samples to calculate zero drift
    const int num_samples = 1000;
    float sum_a = 0.0f, sum_b = 0.0f;

    for (int i = 0; i < num_samples; ++i)
    {
        sum_a += analogRead(adc_pinA) * ADC_CONV * volts_to_amps_ratio;
        sum_b += analogRead(adc_pinB) * ADC_CONV * volts_to_amps_ratio;
    }

    zero_drift_a = sum_a / num_samples;
    zero_drift_b = sum_b / num_samples;
}

void FOC_Controller::updateCurrentSensor()
{
    // read raw values from ADC and convert to amps
    raw_current_a = analogRead(adc_pinA) * ADC_CONV * volts_to_amps_ratio - zero_drift_a;
    raw_current_b = analogRead(adc_pinB) * ADC_CONV * volts_to_amps_ratio - zero_drift_b;

    // Apply low-pass filter to the current measurements
    current_a = current_a_lpf.update(raw_current_a);
    current_b = current_b_lpf.update(raw_current_b);
    current_c = 0 - current_a - current_b;
}