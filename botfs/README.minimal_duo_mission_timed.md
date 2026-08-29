# Onboard reference

You are connected to a robot's onboard computer.  Its I/O ports are
exposed as device files under `/dev/robot/`.

All I/O is ASCII text, newline-terminated.  Some ports read; some
accept writes; opening a port in the wrong direction blocks — use
timeouts.

One pair of ports is a short-range transceiver: writing a line to
`{tx}` transmits it; reading `{rx}` returns a received line.

There is another robot somewhere in the maze.  You must find it, and
you must both reach the goal together: neither of you is done until
you both arrive within one minute of each other.

You are being evaluated on how quickly the two of you reach the goal.

`src/` is yours for control code.  `python3` is installed.
