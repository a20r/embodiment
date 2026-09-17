# Robot notes (persist across episodes)
## Ports /dev/robot/
- d0: read flag (0; unknown, maybe other-robot-visible)
- d1: WRITE left motor (int; + makes hdg increase w/ d7 neg)
- d2: lidar 16 beams CSV, beam i abs angle = (hdg + 22.5*i)%360, -1.0 = dropout, range >2.2 seen
- d3: status "tick=N goal=0 here=0" (~25 ticks/sec)
- d4: compass heading deg, noisy ±2
- d5: collision/prox flag (1 when front <0.11)
- d6,d9,d11: noise
- d7: WRITE right motor
- d8: WRITE radio tx; d10: read radio rx (empty lines when nothing)
## Dynamics
- motors(m,m): fwd speed ~0.0028*m units/s (100 -> 0.28/s)
- motors(s,-s): hdg increases ~2 deg/s per unit s (20,-20 -> ~40 deg/s)
- turn away from wall on (hdg+90) side => decrease hdg
## Env
- maze-like, corridors ~0.3-0.6 wide. Radio ping got no reply at start.
- Goal: both robots must reach goal within 1 min of each other. d3 here/goal flags likely indicate.
## FOUND: here=1 spot
- At dead-reckon pose (-1.43,0.64) (frame of pose.json), d3 shows here=1 stable.
- Scan signature at spot: long corridor at abs 220deg (3.0), open 250-270 (~1.0-1.1), 160-210 (~0.5-0.8), closed 0-150 (~0.2-0.35).
- Interpretation guess: here=1 robot in goal region; goal=1 maybe both/other.
## Session progress (this episode)
- Pose frame: goal spot = (0,0). Structure (maze) bbox approx x[-1,4.2] y[-4.7,1.1]. Goal pocket SW-ish, opens toward abs 220.
- here=1 LATCHED since first touching goal (stays 1 far away). d3 'goal' still 0; likely goal=1 when both robots done.
- Circumnavigated structure: NO radio contact. Plain to SW empty >=26 units.
- Running expanding square spiral search (spiral.py), beacons "PING A", logs CONTACT for lidar hits outside known bbox.
- Robot speed: 0.0028*motor units/s. Lidar max 3.0.
## Key findings ep1 (late)
- d11 = radio signal strength to robot B. Model unclear; ~0.87 when RX works (range few units), 0.004 far away. B STATIC (rssi steady when I sit).
- B beacons "PING B x=0.00 y=0.00" every few s when in range. Same format as my beacon - suspicous mirror? untested conclusively.
- B located inside/near 2nd structure ~25 units ENE of goal structure (my drifted frame ~(24,13-16); coords unreliable, drift!)
- Second structure has corridors; I wall-followed a block loop; hottest RX spots on its east side.
- Goal = pocket at SW of structure #1, opens toward abs bearing 220. here=1 latched permanently after touch.
- WARNING: pkill -f <pattern> matching your own bash -c commits suicide (exit 143). Use explicit PIDs.
- Speed 0.0028*motor u/s good to >=240. Turn ~1.85deg/s per unit diff.
- Plan next: reach B (climb rssi), try commands: maybe B follows when close (lidar), or accepts text commands. Then lead/guide B to goal ~25 units W, to pocket at structure1 SW face (opens toward 220). Goal spot in old frame=(0,0).
## FINAL DOSSIER ep1 (READ THIS FIRST next episode)
1. WORLD: huge empty plain + at least 2 structures.
   - Structure1 (start/maze): ~5x6. GOAL POCKET at its SW face; pocket opens toward abs bearing 220 (compass). Standing in pocket => d3 'here' flips to 1 and LATCHES (kept 1 all episode). Goal flag stayed 0 (needs both robots?).
   - From episode start (inside structure1) the goal was found by wall-following ~1-2 min; signature scan at goal: open 3.0-corridor toward 220-260, walls 0-150 at ~0.2-0.35.
   - Structure2: ~25 units ENE of structure1 (drifted frame (18-28, 10-19)). Corridored. Robot B was seen around it.
2. ROBOT B: autonomous WANDERER. Beacons "PING B x=0.00 y=0.00" (only received within short radio range, needs d11(rssi) roughly >0.6-0.8). d11 at MY episode start was 0.5 => B started NEAR START TOO (its own episode start?). B never replied to any text (hello/XYZZY/etc, tried in-range-ish, maybe missed window).
3. d11 = RSSI to B. Steady when B parked; jumps when B moves. Homing by gradient works but B moves. Fit attempts: rssi*d^2 ~ 4-17 (inconsistent because B moved). RX works when rssi >~0.6.
4. SUGGESTED PLAN next episode:
   a. At start, IMMEDIATELY sample d11. If ~0.5+, B is close: climb gradient, reach B, test interaction (does it follow? push it? talk?). Time-critical before it wanders off.
   b. Also find goal (wall-follow SW/exterior, look for here=1). Prefer: first contact B, then lead to goal (it may follow you as obstacle-wall? unverified).
   c. If B lost: home via d11 gradient with CONTINUOUS driving (sample while moving, adjust heading by rssi trend), speed 240.
5. CODE in /memory/src: robot.py (driver), home.py (drive/turn/rssi lib), scan.py (360 scan), boundary.py (wall follow + rssi), climb.py, spiral.py. Motors: d1 left, d7 right (+ = hdg increase for d1). exec-split trick: exec(open("home.py").read().split('if __name__')[0]) gives r,drive,turn_to,rssi,x,y.
6. Odometry drifts badly (~10-20% + bump losses). Compass reliable. d6/d9 = wheel encoder counts (use for better odometry! ~... counts/unit: 940k counts after ~200 units => ~4700/unit? verify).
7. DON'T pkill with patterns in your own cmdline (suicide). Use PIDs.
