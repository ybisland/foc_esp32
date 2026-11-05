#include "FOC.h"
#include "serial_command.h"
#include <Arduino.h>

/**
 * DengFOC V4 hardware serial control for the FOC controller.
 */

#define BUS_VOLTAGE (12.6f)
#define DIR (1)
#define POLE_PAIRS (7)

#define M0_PIN_A (32)
#define M0_PIN_B (33)
#define M0_PIN_C (25)
#define M0_CURRENT_ADC_PIN_A (39)
#define M0_CURRENT_ADC_PIN_B (36)
#define M0_SHUNT_RESISTOR_OHM (0.01f)
#define M0_CURRENT_AMP_GAIN (50.0f)

FOC_Controller m0_foc;
static bool sensors_calibrated = false;

void setup()
{
    Serial.begin(115200);

    // DengFOC V4 board motor enable
    pinMode(12, OUTPUT);
    digitalWrite(12, HIGH);

    m0_foc.configure(
        M0_PIN_A, M0_PIN_B, M0_PIN_C,
        POLE_PAIRS, DIR,
        BUS_VOLTAGE,
        M0_CURRENT_ADC_PIN_A, M0_CURRENT_ADC_PIN_B,
        M0_SHUNT_RESISTOR_OHM, M0_CURRENT_AMP_GAIN);
    // Set hardware-appropriate limits before selecting a closed-loop mode.
    // m0_foc.setVelocityLimits(min_velocity_rad_s, max_velocity_rad_s);
    // m0_foc.setCurrentLimits(min_iq_a, max_iq_a);
    m0_foc.setVelocityFilterCutoff(1.0f);

    sensors_calibrated = m0_foc.calibrateSensor();
    if (!sensors_calibrated)
        Serial.println("Sensor calibration failed; closed-loop commands are disabled.");
    m0_foc.setMode(FOC_MODE_STOPPED);

    // Velocity PID output ramp is Iq in A/s; position PID output ramp is speed in rad/s^2.
    m0_foc.setVelPID(1.0f, 0.8f, 0.0f, 6.0f);
    m0_foc.setPosPID(1.0f, 0.8f, 0.0f, 6.0f);

    serial_command_init(&m0_foc);
}

void loop()
{
    if (sensors_calibrated)
        receive_serial_commands();
    m0_foc.run();
}
