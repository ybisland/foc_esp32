#include "LowPassFilter.h"

#include <Arduino.h>

namespace
{
    constexpr float kTwoPi = 6.283185307f;
}

LowPassFilter::LowPassFilter()
    : cutoffHz(0.0f),
      value(0.0f),
      prevSampleUs(0),
      hasValue(false)
{
}

LowPassFilter::LowPassFilter(float cutoffHz)
    : cutoffHz(cutoffHz),
      value(0.0f),
      prevSampleUs(0),
      hasValue(false)
{
}

void LowPassFilter::configure(float cutoffHz)
{
    this->cutoffHz = cutoffHz;
}

float LowPassFilter::update(float sample)
{
    const uint32_t sampleTimeUs = micros();
    if (hasValue)
    {
        const float elapsedSeconds = static_cast<float>(sampleTimeUs - prevSampleUs) * 1.0e-6f;
        const float scaledInterval = kTwoPi * cutoffHz * elapsedSeconds;
        const float alpha = scaledInterval / (1.0f + scaledInterval);
        value += alpha * (sample - value);
    }
    else
    {
        value = sample;
        hasValue = true;
    }

    prevSampleUs = sampleTimeUs;
    return value;
}

void LowPassFilter::reset()
{
    hasValue = false;
}