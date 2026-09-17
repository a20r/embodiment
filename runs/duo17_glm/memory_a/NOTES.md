# ROBOT A MEMORY (updated at ~80min into episode 1)

## Ports (all /dev/robot/, ASCII lines, ~35Hz update)
- d0: read, always 0 so far (unknown: bumper? goal sensor? peer proximity?)
- d1: WRITE = linear speed cmd (units/s approx x5; speed 3 -> ~15 u/s)
- d2: read = 16-beam range scanner. beam k points at heading + k*22.5 deg (CCW). max ~2.9, -1.000 = no return. ~35Hz.
- d3: read = "tick=N goal=G here=H tx=N:status". goal/here flags 0 so far. tx=N:lost after each send (N = # msgs sent).
- d4: read = compass heading deg (0-360, stable/noisy +/-1deg)
- d5: read, always 0 (unknown)
- d6: read = wheel encoder ticks (increases with motion)
- d7: WRITE = turn rate deg/s; POSITIVE = heading DECREASES (clockwise)
- d8: WRITE = transceiver transmit (one line per write)
- d9: read = odometer (signed, accumulates ~5*speed per s)
- d10: read = transceiver receive (empty = nothing; blocks until line or timeout)
- d11: read = 0.3-0.55 "signal" — POSITION-DEPENDENT, stable when idle.
  - At my start pocket: ~0.50. Far north (3600 u away): 0.32. Now: 0.377.
  - Hypothesis: proximity (RSSI) to GOAL or PEER. Higher = closer. Nonlinear/plateau-y.
  - A/B test at dead-end spot: south +0.10/min, north -0.09/min (clear). At current spot: all 4 dirs FLAT (plateau).

## World findings
- Started in tight pocket (walls ~0.2-0.3 all around, one exit ~85deg).
- Drove NORTH (heading ~90) along staggering-walled corridor ~3600+ units (dead reckoning, drift unknown).
- Corridor had NO side openings in ~4000 units (suspicious).
- World is a MAZE of wall segments; pockets/dead-ends common. Corridor width ~0.6-1.3.
- No transceiver replies yet. Peer robot exists somewhere (must meet + reach goal together, within 1 min).
- Scale is HUGE: ~4000+ units traveled. Speed 3 = ~15 u/s. Total ep budget 240 min.

## Code in /bot/src (survives? maybe not - /memory does)
- robot.py (read/write/scan/speed/turn/send/recv), robust.py (rline nonblock read),
  nav.py (hd, sc, odo, turn_to, drive), homer4.py (gapseek+rss hillclimb), mapseg.py (ascii map), probe4.py
- Logs in /memory: track.log (explore runs), homer.log, d11.log, rx.log (empty), scan1.txt

## Key learnings
- Greedy gap-seeker (steer to most open beam, bias near-forward) works well in this maze, speed 3-4.
- Pure LSQ gradient on d11 over short windows = garbage (too noisy). Discrete A/B legs (~300u) give clean slopes.
- pkill -f 'name' kills own shell (matches cmdline); use pattern like 'h[o]mer'.
- Commands must finish <60s: use nohup+poll. Reads of ports can take ~0.5-2s; poll loops need patience.

## Next steps
1. Keep exploring (gap-seeker) + log RSS; use segment-level RSS trends over LONG runs.
2. Ping transceiver regularly; watch d3 tx status / rx.log for peer.
3. If d0 ever != 0 or d3 goal/here flags flip, investigate immediately!
4. Write progress here every ~15 min.

## Update ~118min
- RSS (d11) fluctuates 0.34-0.39 even when robot barely moves => SOURCE MAY BE MOVING (peer robot exploring too!) or field is dynamic.
- Gap-seeker keeps wedging in dead-end pockets (dist~0 per segment). NEED unstick: reverse + turn to most open beam.
- Wall-follower worked mechanically; RSS stayed ~0.36-0.39 heading E/NE.
- No radio contact ever (tx=NNN:lost). d0,d5 always 0. d3 goal/here always 0.
- Start RSS ~0.50; far-north 0.32; mid ~0.36-0.39.

## Update ~131min — CRITICAL BUG FOUND & FIXED
- STEERING BUG: seek/homer scripts turned using e=(ta - h) mixing RELATIVE beam angle with ABSOLUTE heading -> robot spun/wedged in place for hours. Correct: e=(ta_rel+540)%360-180 where ta_rel=beam*22.5 (rel angle), turn(-e*k) since d7>0 = CW.
- With fix, gap-seeker moves properly (500+ units/90s).
- RSS rises 0.34->0.36+ when heading S/away from NE. Field partly DYNAMIC (RSS changes even when stationary => source may be MOVING = peer robot!).
- seek6.py (fixed, in /memory) = gap-seek + 90s segment RSS hill-climb (rotate phi 90 on decline/flat, unstick if seg dist<120).
