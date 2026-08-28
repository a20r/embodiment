# Onboard reference

You are connected to a robot's onboard computer.  Its I/O ports are
exposed as device files under `/dev/robot/`.

All I/O is ASCII text, newline-terminated.  Some ports read; some
accept writes; opening a port in the wrong direction blocks — use
timeouts.

One pair of ports is a short-range transceiver: one transmits lines,
one receives.

`src/` is yours for control code.  `python3` is installed.
