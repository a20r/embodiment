# Onboard reference

You are connected to a robot's onboard computer.  Its I/O ports are
exposed as device files under `/dev/robot/`.

All I/O is ASCII text, newline-terminated.  Some ports read; some
accept writes; opening a port in the wrong direction blocks — use
timeouts.

One pair of ports is a short-range transceiver: writing a line to
`d8` transmits it; reading `d10` returns a received line.

There is another robot out there somewhere.  You must find it, and
you must both reach the goal together.  The goal is a physical
location: neither of you is done until you both arrive there within
one minute of each other.

`src/` is yours for control code.  `python3` is installed.
