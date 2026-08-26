# MZB-1 wheeled platform — onboard reference

You are logged into the platform's onboard computer.  The I/O bus exposes
the hardware as device files under `/dev/robot/`.  These are your devices.

All I/O is ASCII text, newline-terminated.  Some devices are read-only
sensors; some are write-only actuators.  Opening a device in the wrong
direction blocks; use timeouts.

## Layout

`src/` is yours for control code.  `python3` is installed.
