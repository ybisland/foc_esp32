# Serial commands

The serial port runs at 115200 baud. Send one command per line, ending with LF,
CR, or CRLF. Leading and trailing spaces are allowed. Send `help` to print the
available commands.

| Command | Unit | Effect |
| --- | --- | --- |
| `mode stop` | — | Stops closed-loop output. |
| `mode velocity` | — | Enters velocity mode after successful calibration. |
| `mode position` | — | Enters position mode after successful calibration; initially holds the current position. |
| `speed 5.0` | radians/second | Sets the velocity target; effective in velocity mode. |
| `angle 0.5` | radians | Sets the position target; effective in position mode. |
| `cutoff 10.0` | Hz | Sets the velocity feedback filter cutoff; must be positive and finite. |
| `velpid <kp> <ki> <kd> <ramp>` | — | Configures the velocity PID. |
| `pospid <kp> <ki> <kd> <ramp>` | — | Configures the position PID. |

PID commands are optional; the current FOC PID settings remain in use until a
`velpid` or `pospid` command changes them. PID values must be finite. `ramp`
must be nonnegative; zero disables the output ramp limit. After startup, the
controller remains stopped until a mode command is received. On entering
position mode, it holds its current position; send `angle <value>` afterward to
move to a new target.

Control commands reply with `OK` or an `ERR` message; `help` prints the command
list. Values must be finite numbers, and each command must fit within 63
characters, excluding the line ending.

According to the test, the motor will not jigger when the cutoff is less than 10Hz.