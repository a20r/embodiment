# ROBOT NOTES (persisted across episodes) — read this first!

## Port map (/dev/robot/dN, FIFOs; one line per open; use O_NONBLOCK + select, see /memory/rio.py)
- d7 WRITE: LEFT wheel speed cmd (float; 1 => ~5 enc units/s, 100 => ~500/s, linear). Left wheel ~5% faster than right.
- d1 WRITE: RIGHT wheel speed cmd (same scale). Robot is DIFFERENTIAL DRIVE, wheelbase ~350 enc units.
- d6 READ: LEFT wheel encoder (cumulative, noise +-2). d9 READ: RIGHT wheel encoder.
- d4 READ: compass heading deg, CCW-positive (left wheel fwd => heading decreases). noise +-3 deg.
- d2 READ: 16-beam lidar, comma-separated, beam i at 22.5*i deg CCW from front (beam 0 = front, 4 = left, 8 = back, 12 = right). -1 = dropout. Units: ~9000 enc units per 1.0. Min reading when touching wall ~0.09. Noise ~+-0.03.
- d3 READ: status "tick=N goal=0 here=0" (100 ticks/s). goal/here flags presumably become 1 at goal / when other robot present.
- d11 READ: scalar 0.2-0.5, changes over time even when we're still -> probably signal/proximity to OTHER robot (unconfirmed).
- d0, d5 READ: always 0 so far (bump sensors?).
- d8 WRITE: radio TX (plain text). d10 READ: radio RX (empty line if nothing). Short range.
- Writes to a FIFO with no reader give ENXIO (e.g. reading-ports).
## World
- Start: tiny enclosure, walls ~0.2-0.3 lidar units around, opening at +90deg (beam 4) at 2.7. Maze-like?
- Speed: use cmd 20-100. Motor commands persist until changed (no watchdog). Always write 0 to stop!
## Episode 1 progress (see also timeline below)
- CONFIRMED: d11 = signal strength to OTHER robot (jumped 0.3 -> 0.93 when it came near). Use as hot/cold meter.
- Other robot TALKS on radio: "Hello from robot at approx (x,y). I am mapping. Where are you? Reply please." It reports its own odometry coords (its frame).
- Lidar scale (free motion): ~1700 enc units per lidar unit (slip inflates encoder counts when pushing walls).
- Collision: touching a wall => wheels slip, encoders count but no motion. Keep >=0.15 clearance. front() must not stop on side beams 1/15 unless < 0.3.
- Corridors ~0.5 wide, axis-aligned (0/90/180/270). Robot radius ~0.1 (min lidar 0.09).
- Code saved in /memory/src (ctl.py=turn/forward/scan, explore2.py=grid explorer w/ right-hand rule).
## Episode 1 (later) — coordination facts
- Robot B is an LLM agent. Radio msgs TRUNCATE at ~250 chars: keep each line <240. B replies with prefix "B:".
- B's frame: odometry x+=d*cos(hdg), y+=d*sin(hdg), 1820 ticks/unit, B started ~(0,0). B explored bbox x[-3,1] y[-0.5,3.4]: rooms + 0.5-wide corridors, NO goal there.
- I (A) was located by B at (-2.55,0.45) in B's frame = south end of a dead-end N-S corridor (x~-2.7, y 0.1..2.9), only exit east at its top. My start room ~(-2.6,0.5)?
- Agreed protocol: whoever sees goal=1 stays ON goal, broadcasts "GOAL AT (x,y)" every 5s; other homes in with d11.
- Explorer explore2.py args: <speed> <x0> <y0>; writes explore2.pid (kill with `kill $(cat explore2.pid)` — NEVER pkill -f with a pattern that appears in your own command line!)
## Episode 1 end-game state (03:37)
- A parked at TOP (north end) of dead-end corridor (B frame ~(-2.8,2.7)), facing N; east opening there. route.py runs in bg: watches rx.log for "ROUTE: N 1.2, E 0.6" legs (dir+lidar units) from B and executes them, reports goal state. txloop.py broadcasts tx_msg.txt lines every ~10s. rxlog.py logs radio to rx.log.
- If new episode: read this, re-create /bot/src from /memory/src, contact B (short msgs), agree on shared landmark + ROUTE protocol. Goal still unknown at this time.
## FINAL NOTES episode 1 (~03:46, budget nearly out; episode may continue autonomously)
- B's FRONTIERS (B frame): all NORTH y>3.5: F1(-2.9,3.8) F2(-2.4,3.75) F3(-2.0,4.0) F4(-1.7,4.0) F5(-1.2,3.6); east wall gaps x~0.95 at y 3.1,2.6,2.1,0.9. West x<-3.4 & south y<-0.5 are walls. B was seen going to (-4.9,5.3),(-3.4,5.0) = far NW beyond its bbox.
- My explorer wedges in narrow pockets (front-stop from side beams); ESCAPE patch added (backup+turn). Better next time: front stop only on beam 0 (<0.25) and beams 1/15 only if <0.2.
- Running stack at end: route.py (master: starts explore2.py as child; on rx "ROUTE: N 1.2, E 0.6" kills explorer & executes legs; reports goal) + txloop.py + rxlog.py.
- NEXT EPISODE QUICKSTART: cp /memory/src/*.py /bot/src/; start rxlog.py & txloop.py (edit tx_msg.txt, lines <240 chars); talk to B; ask B for goal location/ROUTE legs from a shared landmark; drive with ctl.turn/ctl.forward (v=40-60, stopfront 0.27). goal flag = d3 "goal=1".
- 03:52 A context budget exhausted; left autonomous stack running (explore2 wandering near x-2..-3,y1..2.5 due to odometry drift; goal=0). B silent since 03:44 (last seen far NW (-3.3,4.5)). If you are a new episode: goal NOT yet found by A; frontiers are NORTH (y>3.5 in B frame). Try to get B to send a ROUTE from a shared landmark.
- 04:05 B FOUND GOAL at (-5.4,5.0) B-frame (far NW). INDICATOR IS d3 here=1 (not goal). A heading NW autonomously (explore2 TGT homing).
- 04:23 A homing (d11-gradient explorer) still far from goal (d11~0.26), B parked on goal at (-5.4,5.0) B-frame (far NW). Stack running autonomously.
- 04:47 explorer now biased N then W (compass-based) + d11 trend; B silent since 04:34, assumed parked on goal.
- 05:04 plan: explorer navigates to corridor-top landmark (-2.8,2.85) then injects B's ROUTE (N1.2,W.3,N.3,W.3,N.3,W.4,N.25,W.95,N.55) for route.py
- 05:11 A stuck near (-2.2,1.9) own frame (pocket), goal not reached; B parked on goal broadcasting route from corridor top. Stack left running.
