#pragma once

#include <stdint.h>

class LowPassFilter {
public:
    LowPassFilter();

    // The caller must provide a positive, finite cutoffHz.
    explicit LowPassFilter(float cutoffHz);

    // Changes the cutoff frequency without resetting the filter.
    void configure(float cutoffHz);

    // The caller must provide finite samples within the range of float arithmetic.
    float update(float sample);

    // Discards the output so the next sample initializes it again.
    void reset();

private:
    float cutoffHz;
    float value;
    uint32_t prevSampleUs;
    bool hasValue;
};