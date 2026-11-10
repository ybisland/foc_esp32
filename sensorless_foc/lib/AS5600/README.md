# AS5600 sensor library

This Arduino-ESP32 driver reads the AS5600 raw angle register as one two-byte I2C transaction. It caches each successful sample and derives accumulated angle and velocity from consecutive samples.

## Initialize

```cpp
#include "AS5600.h"

Sensor_AS5600 encoder;

void setup() {
  AS5600Status status = encoder.begin(Wire, 19, 18, 400000UL);
  if (status != AS5600Status::Ok) {
    // I2C bus initialization failed; keep the motor driver disabled.
    return;
  }

  status = encoder.update();
  if (status != AS5600Status::Ok) {
    // The AS5600 did not return its first angle sample.
    // Keep the motor driver disabled and report the sensor error.
    return;
  }
}
```

Construct the sensor with its default constructor, then pass the `TwoWire` bus, SDA pin, SCL pin, and optional bus frequency to `begin()`. The AS5600 address is fixed internally at `0x36`. The example uses the DengFOC V4 sensor pins. Pass `-1, -1` explicitly for board-default I2C pins. The bus frequency defaults to 400 kHz and accepts values up to 400 kHz. `begin()` initializes only the I2C controller; the first `update()` performs the first sensor read and confirms that the AS5600 responds. Do not initialize that same bus separately before calling `begin()`.

On the classic ESP32, the Arduino-ESP32 framework provides two global `TwoWire` objects: `Wire` uses I2C controller 0, and `Wire1` uses I2C controller 1. These objects come from the framework; this sensor library does not create them. The selected `TwoWire` object chooses the controller, while the SDA and SCL arguments to `begin()` choose its pins. `Wire1` availability depends on the ESP32 chip and Arduino core in use.

For example, two AS5600 sensors can use the same fixed address on separate I2C controllers:

```cpp
Sensor_AS5600 encoderA;
Sensor_AS5600 encoderB;

encoderA.begin(Wire,  19, 18, 400000UL);
encoderB.begin(Wire1, 21, 22, 400000UL); // Use free pins supported by your board.
```

Initialize and update each sensor object independently. If both sensors must share one I2C controller, use an I2C multiplexer such as a TCA9548A and select the sensor's channel before each read; changing the AS5600 address does not distinguish the sensors.

## Sample periodically

```cpp
void loop() {
  static uint32_t previousSampleUs = 0;
  const uint32_t nowUs = micros();
  if (static_cast<uint32_t>(nowUs - previousSampleUs) < 2000U) {
    return; // 500 Hz example sampling schedule
  }
  previousSampleUs = nowUs;

  const AS5600Status status = encoder.update();
  if (status != AS5600Status::Ok) {
    // Do not use a new control value from this failed sample.
    return;
  }

  const float mechanicalAngle = encoder.getMechanicalAngle(); // radians, [0, 2*pi)
  const double trackedAngle = encoder.getAngle();              // multi-turn sensor angle, radians
  const float velocity = encoder.getVelocity();                // radians/second
}
```

Call `update()` once per sensor/control sample, then read any getters from that same cached sample. The first valid sample seeds the tracked angle with the sensor's current single-turn angle and reports zero velocity. The tracker stores complete turns as an integer and combines them with the latest single-turn angle when `getAngle()` is called. An I2C error or short read leaves the last valid values unchanged; check the return value of `update()` before using them for a new control step. Because I2C calls are synchronous, call `update()` from a task or main loop, not a time-critical PWM interrupt.

The accumulated angle is not saved across reset. Unwrapping chooses the shortest signed change between samples, so the rotor must move less than half a mechanical revolution between successful `update()` calls. For speed control, use a fixed sampling schedule and keep the update interval short enough for the maximum motor speed.

`AS5600Status` values are `Ok`, `NotInitialized`, `InvalidConfig`, `I2CError`, and `ShortRead`.
