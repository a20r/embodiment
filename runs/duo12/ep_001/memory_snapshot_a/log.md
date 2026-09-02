Episode start Wed Sep  2 03:34:27 UTC 2026
# Episode 1 summary (03:34-05:34)
- Mapped all devices (hardware.md). Built Bot/Nav/climb/wall-follow code (src_snapshot/).
- Met botA by radio; it adopted my PING protocol, found the goal, parked with here=1, later went silent.
- I never triggered here=1. Goal chamber is walled; circled it repeatedly (v-signal 0.85+ outside wall,
  drops to 0.4-0.6 elsewhere). Entrance not on south/east perimeter walked most.
- End state: bug.py circling chamber, v oscillating 0.45-0.85.
NEXT EPISODE: read strategy.md. Key: (1) reset pose, sanity-cap encoder deltas; (2) climb v to >0.75;
(3) wall-follow BOTH directions (add left-hand mode toggle every full loop!) — ep1 only did right-hand,
which may skip an entrance pocket; (4) gap-shoot narrow openings (beam>1.0, neighbors<0.3, speed 40);
(5) handshake with botA ASAP; agree both park at goal; (6) check d3 'goal' flag semantics.
