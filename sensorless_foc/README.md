# Serial commands

The serial port runs at 115200 baud. Send one command per line, ending with LF,
CR, or CRLF. Leading and trailing spaces are allowed. Send `help` to print the
available commands.

| Command | Unit | Effect |
| --- | --- | --- |
| `mode stop` | — | Stops closed-loop output. |
| `mode velocity` | — | Enters velocity mode after successful calibration. |
| `speed 5.0` | radians/second | Sets the velocity target; effective in velocity mode. |
| `cutoff 10.0` | Hz | Sets the velocity feedback filter cutoff; must be positive and finite. |
| `velpid <kp> <ki> <kd> <ramp>` | — | Configures the velocity PID. |
| `curpid <kp> <ki> <kd> <ramp>` | — | Configures the current PID. |

PID commands are optional; the current FOC PID settings remain in use until a
`velpid` or `curpid` command changes them. PID values must be finite. `ramp`
must be nonnegative; zero disables the output ramp limit. After startup, the
controller remains stopped until a mode command is received.

The AS5600 remains available for sensor-angle comparison and is still used for
commutation and velocity feedback. The `e_angle` and `SMO_Est_theta` telemetry
values can be compared; the SMO angle is not yet used to control the motor.

Control commands reply with `OK` or an `ERR` message; `help` prints the command
list. Values must be finite numbers, and each command must fit within 63
characters, excluding the line ending.

According to the test, the motor will not jigger when the cutoff is less than 10Hz.