#pragma once

#include <Arduino.h>
#include <Wire.h>

enum class AS5600Status : uint8_t {
    Ok,
    NotInitialized,
    InvalidConfig,
    I2CError,
    ShortRead
};

/**
 * Minimal AS5600 driver for Arduino-ESP32.
 *
 * Call begin() once, then call update() periodically before reading the
 * cached angle or velocity. Multi-turn angle is accumulated only while the
 * device is powered and update() observes every move of less than half a turn.
 */
class Sensor_AS5600 {
public:
    Sensor_AS5600();

    // Initializes the I2C controller; sensor communication is checked by update().
    AS5600Status begin(TwoWire& wire,
                       int sdaPin,
                       int sclPin,
                       uint32_t frequencyHz = 400000UL);

    // Reads the angle; the first successful call seeds the angle and velocity state.
    AS5600Status update();

    // Latest single-turn mechanical angle in [0, 2*pi), in radians.
    float getMechanicalAngle() const;

    // Turn-tracked sensor angle in radians; seeded from the first sample.
    // Not retained across reset.
    double getAngle() const;

    // Average angular velocity since the previous successful update, rad/s.
    float getVelocity() const;


private:
    AS5600Status readRawAngle(uint16_t& rawAngle);

    // IIC controller and pins; frequency in Hz.
    TwoWire* wire_;
    int sdaPin_;
    int sclPin_;
    uint32_t frequencyHz_;

    bool initialized_;
    bool hasSample_;
    int32_t fullRotations_;
    float mechanicalAngle_;
    float velocity_;
    uint32_t prevSampleUs_;
};
