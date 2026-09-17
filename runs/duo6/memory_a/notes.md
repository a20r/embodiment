# Robot ports (confirmed ep1)
- /dev/robot/* are FIFOs; read ports: d1..d9 (some), write: d0,d10,d11. Open with timeout; wrong direction blocks.
- d0: TX (write line), d4: RX (read line, empty if none)
- d1: heading deg (noisy ±3)
- d2: ? (0), d5: ~0.13 noisy const?, d7: 0, d8: 0/-1, d9: 0
- d3: lidar 16 rays CSV, -1.000 = dropout. Range max ~2.3?
- d6: "tick=N goal=0" tick ~2.5/s? goal flag!
- d10 = LEFT wheel, d11 = RIGHT wheel (same sign): both + -> translate (~0.0015 u/s per unit cmd), d10-d11 -> rotate CW/heading+ (~0.9 deg/s per unit diff)
- cmd 10/10 -> ~0.015 u/s. Corridors ~0.3-0.7 wide. Try higher cmds.
## Findings ep1 (cont)
- d5 ~ euclidean distance to bot A (scale unsure, fit S~6.7 A-units per d5 unit; noisy). Changes as A moves.
- Radio d4: bot A streams "HELLO from bot A t=<unix> x= y= h=<deg>" ~1Hz. Pipe QUEUES msgs (drain fast!). Read >=2Hz.
- Radio seems long/unlimited range (contact for 10+ min while A far). Earlier silence unexplained (A idle at start? starts moving/broadcasting after first contact?).
- A ignores text commands (HELP/STOP/COME/GOAL?/FOLLOW).
- A's frame: units ~6.7 per d5-unit; A speed ~0.3-0.5 A-units/s; A pose range seen: x 7..-35, y -3..6.
- Our drive: cmd60 ~0.11 lidar-units/s. Corridor ~0.3.
- d6 tick ~100/s. goal=0 always so far.
- STRATEGY: explore for goal (d6), log A poses + d5 + odom; then lure A (unknown how). Maybe A attracted to radio/our broadcasts?
