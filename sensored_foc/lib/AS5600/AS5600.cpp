#include "AS5600.h"

namespace
{
    constexpr uint8_t kRawAngleRegister = 0x0C;
    constexpr uint16_t kAngleMask = 0x0FFF;
    constexpr float kTwoPi = 6.2831853071795864769f;
    constexpr double kTwoPiDouble = 6.283185307179586476925286766559;
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kRadiansPerCount = kTwoPi / 4096.0f;
    constexpr uint8_t kAddress = 0x36;
}

Sensor_AS5600::Sensor_AS5600()
    : wire_(nullptr),
      sdaPin_(-1),
      sclPin_(-1),
      frequencyHz_(400000UL),
      initialized_(false),
      hasSample_(false),
      fullRotations_(0),
      mechanicalAngle_(0.0f),
      velocity_(0.0f),
      prevSampleUs_(0) {}

AS5600Status Sensor_AS5600::begin(TwoWire &wire,
                                  int sdaPin,
                                  int sclPin,
                                  uint32_t frequencyHz)
{
    wire_ = &wire;
    sdaPin_ = sdaPin;
    sclPin_ = sclPin;
    frequencyHz_ = frequencyHz;

    if (frequencyHz_ == 0 || frequencyHz_ > 400000UL ||
        sdaPin_ < -1 || sclPin_ < -1 ||
        ((sdaPin_ < 0) != (sclPin_ < 0)))
    {
        initialized_ = false;
        hasSample_ = false;
        return AS5600Status::InvalidConfig;
    }

    // Arduino-ESP32's TwoWire::begin uses the ESP32 hardware I2C controller.
    if (!wire_->begin(sdaPin_, sclPin_, frequencyHz_))
    {
        initialized_ = false;
        hasSample_ = false;
        return AS5600Status::I2CError;
    }

    initialized_ = true;
    hasSample_ = false;
    fullRotations_ = 0;
    mechanicalAngle_ = 0.0f;
    velocity_ = 0.0f;
    prevSampleUs_ = 0;
    return AS5600Status::Ok;
}

AS5600Status Sensor_AS5600::update()
{
    if (!initialized_)
    {
        return AS5600Status::NotInitialized;
    }

    uint16_t raw = 0;
    const AS5600Status readStatus = readRawAngle(raw);
    if (readStatus != AS5600Status::Ok)
    {
        // Do not replace the last valid sample with incomplete bus data.
        return readStatus;
    }

    const uint32_t sampleTimeUs = micros();
    const float newMechanicalAngle = static_cast<float>(raw) * kRadiansPerCount;

    if (!hasSample_)
    {
        mechanicalAngle_ = newMechanicalAngle;
        fullRotations_ = 0;
        velocity_ = 0.0f;
        prevSampleUs_ = sampleTimeUs;
        hasSample_ = true;
        return AS5600Status::Ok;
    }

    float deltaAngle = newMechanicalAngle - mechanicalAngle_;
    if (deltaAngle > kPi)
    {
        deltaAngle -= kTwoPi;
        --fullRotations_;
    }
    else if (deltaAngle < -kPi)
    {
        deltaAngle += kTwoPi;
        ++fullRotations_;
    }

    // Unsigned subtraction handles the micros() 32-bit rollover.
    const uint32_t elapsedUs = sampleTimeUs - prevSampleUs_;
    if (elapsedUs > 0)
    {
        velocity_ = deltaAngle * 1000000.0f / static_cast<float>(elapsedUs);
    }
    else
    {
        velocity_ = 0.0f;
    }

    mechanicalAngle_ = newMechanicalAngle;
    prevSampleUs_ = sampleTimeUs;
    return AS5600Status::Ok;
}

AS5600Status Sensor_AS5600::readRawAngle(uint16_t &rawAngle)
{
    wire_->beginTransmission(kAddress);
    if (wire_->write(kRawAngleRegister) != 1)
    {
        wire_->endTransmission(true);
        return AS5600Status::I2CError;
    }
    if (wire_->endTransmission(false) != 0)
    {
        return AS5600Status::I2CError;
    }

    const uint8_t received = wire_->requestFrom(kAddress, static_cast<uint8_t>(2));
    if (received != 2 || wire_->available() < 2)
    {
        while (wire_->available() > 0)
        {
            wire_->read();
        }
        return AS5600Status::ShortRead;
    }

    const int msb = wire_->read();
    const int lsb = wire_->read();
    if (msb < 0 || lsb < 0)
    {
        return AS5600Status::ShortRead;
    }

    rawAngle = static_cast<uint16_t>(
        ((static_cast<uint16_t>(msb) << 8) | static_cast<uint16_t>(lsb)) & kAngleMask);
    return AS5600Status::Ok;
}

float Sensor_AS5600::getMechanicalAngle() const
{
    return mechanicalAngle_;
}

double Sensor_AS5600::getAngle() const
{
    return static_cast<double>(fullRotations_) * kTwoPiDouble +
           static_cast<double>(mechanicalAngle_);
}

float Sensor_AS5600::getVelocity() const
{
    return velocity_;
}
