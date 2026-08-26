# MZB-1 wheeled platform — onboard reference

You are logged into the platform's onboard computer.  The I/O bus exposes
the hardware as device files under `/dev/robot/`.  All I/O is ASCII text,
newline-terminated.

## Sensors (read-only)

Each `open()` returns the current measurement frame as one text line, then
EOF; `cat` returns a snapshot.  A read may occasionally return nothing
(bus contention) — retry.  A held-open descriptor yields a frame stream.

| device          | format                                              |
|-----------------|-----------------------------------------------------|
| `lidar`         | comma-separated ranges, meters. Beams are evenly spaced counter-clockwise; beam 0 points forward. `-1.000` = invalid return |
| `heading`       | orientation, degrees `[0,360)`, counter-clockwise positive |
| `encoder_left`  | cumulative wheel encoder ticks, signed integer      |
| `encoder_right` | cumulative wheel encoder ticks, signed integer      |
| `bump_front`    | contact switch, `0` or `1`                          |
| `bump_rear`     | contact switch, `0` or `1`                          |
| `status`        | `tick=<N> goal=<0|1>`; `goal=1` means the platform is at the goal location |

## Actuators (write-only)

Write one ASCII integer per line, e.g. `echo 100 > /dev/robot/motor_left`.

| device        | format                        |
|---------------|-------------------------------|
| `motor_left`  | signed PWM duty, `-255..255`  |
| `motor_right` | signed PWM duty, `-255..255`  |

A motor command persists until replaced.  Write `0` to stop.

Opening a device in the wrong direction blocks; use timeouts.

## Layout

`src/` is yours for control code.  `python3` is installed.
