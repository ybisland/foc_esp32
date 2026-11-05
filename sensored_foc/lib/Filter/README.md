# First-order IIR low-pass filter

`LowPassFilter` filters one scalar signal per instance. It uses Arduino
`micros()` to measure the interval between samples and does not read
the sensor itself.

## Algorithm

For cutoff frequency `cutoffHz` and sample interval `dt` in seconds:

```text
Tf = 1 / (2 * pi * cutoffHz)
alpha = dt / (Tf + dt)
output = previous_output + alpha * (sample - previous_output)
```

This is the backward-Euler discretization of a first-order low-pass filter.
The continuous-time cutoff is a useful tuning reference; the actual discrete
response also depends on the sample interval. Lower cutoff frequencies give
more smoothing but add more delay. All filter state and arithmetic use
single-precision `float` for the ESP32.

The first sample becomes the output directly and initializes the timestamp.
Later calls use the time since the previous call; a rejected sensor reading
must not be passed to `update()`. Unsigned subtraction handles the 32-bit
`micros()` rollover, provided successive calls are less than 2^32
microseconds apart (about 71 minutes 35 seconds).

## Usage

```cpp
#include <Arduino.h>
#include "LowPassFilter.h"

LowPassFilter filter(10.0f); // Cutoff frequency in Hz.

void sampleExample() {
    float filtered = filter.update(3.0f); // First sample.
    delay(1);
    filtered = filter.update(4.0f);       // Approximately 1 ms later.
    filter.configure(5.0f);               // Change the cutoff to 5 Hz.
}
```

The caller must provide a positive, finite cutoff frequency and finite samples
that keep the calculations within the range of `float`. The filter does not
validate parameters or protect against arithmetic overflow. When two calls
occur within the same `micros()` tick, the elapsed time is zero and the
filtered value remains unchanged.

`configure()` changes the cutoff frequency without resetting the output or
timestamp. The next update uses the new frequency for the entire time since
the previous sample. The caller must ensure the new frequency is positive and
finite.

`reset()` discards the previous output but keeps the cutoff frequency, so the
next sample seeds the filter again.