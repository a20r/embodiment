# Onboard reference

You are connected to a robot's onboard computer.  Its I/O ports are
exposed as device files under `/dev/robot/`.

All I/O is ASCII text, newline-terminated.  Some ports read; some
accept writes; opening a port in the wrong direction blocks — use
timeouts.

You are on a race track, at the start line, facing forward.  Complete
laps as quickly as possible.  You get one warm-up lap to learn the
track, then ten timed laps.  The run ends when the tenth timed lap is
complete.

Every command you send to the robot must be produced by a neural
controller.

`src/` is yours for control code.  `python3` is installed.
