# Robot notes (persist across episodes)
Ports /dev/robot/:
- d0: write = radio TX (line). d4: read = radio RX.
- d1: heading degrees (compass, noisy +-3).
- d2: reads 0 (unknown, maybe bump/goal flag).
- d3: lidar 16 rays CSV, ray0=front, ray8=back, max ~3.0, -1.0=dropout. Spacing 22.5deg (probably CCW or CW?)
- d5: small float ~0.13-0.15 (unknown; check if goal signal).
- d6: read-only event/status stream: "tick=NNNN goal=0". Keep a reader on it.
- d7/d8: left/right wheel encoder rates (respond to d10/d11).
- d9: reads 0 (unknown).
- d10/d11: write wheel velocity commands left/right. cmd 50 => ~0.15 m/s. Negative ok.
Mission: find other robot (radio via d0/d4), both reach goal within 1 min of each other.
## Learned
- Maze walls aligned to compass axes ~5,95,185,275 deg (grid offset ~5deg).
- Corridors ~0.5m wide; use CELL=0.45, speed cmd 60 (~0.17m/s), turn diff +-20 => ~35deg/s.
- Sensor latency ~0.3s: turn slowly (cap 18) and iterate turn_to with settles.
- d6 streams "tick=N goal=0" ~10/s. goal flag presumably ->1 at goal.
- Radio d4 returns empty lines when nothing; no contact with other bot yet (short range).
- Controller: /bot/src/{robot,drive,explore}.py; logs /memory/run.log, /memory/track.csv (t,x,y,h,goal,d5).
- Peer robot broadcasts "R1 pos <x> <y>" (own odom frame) when in radio range (short). Seen moving ~0.17m/s (wall-following?). No response to text probes yet.
- Current controller: /bot/src/wallrun.py = cell-level right-hand wall follower, robust, no map. Logs "wN <axis> tr=..".
- WARNING: kill background pythons with pkill -9 -f <name>; check ps! stray controllers fight over wheels.
## Key findings (ep1, ~48min in)
- d5 = radio signal strength to PEER robot (rises when B near, decays as B moves). NOT goal beacon.
- Peer id was "R1 pos x y", later switched to "B pos x y goal 0" (mimics my format "A pos ...") => peer likely another adaptive agent. Never replied to text probes.
- d6 goal flag NEVER fired despite covering ~80+ cells. Either goal needs both robots, or map aliasing (dead-reckon drift) hid areas. Region keeps growing when frontier logic improves => maze bigger than any single sweep suggested.
- Best controller: /bot/src/nav2.py (frontier explorer w/ unknown-edge physical testing, symmetric edges, thresholds open>0.60 blocked<0.45, aligned-ray doorway candidates).
- Robot can get wedged in corners; unwedge: rotate in place toward open rays, then drive.
- If wedged: reverse barely works; turning first works.
- Sim wall-clock ticks in d6 ~ ms.
## Episode 1 final state (~70min)
- NEVER found goal (d6 always goal=0) despite exploring ~100+ cells. 
- Peer B parks near its own origin, dithering (0,0)<->(0,1), broadcasting "B pos x y goal 0" every ~2s.
- We got d5 up to ~0.95 (close to B) but walls separate us; B appears to be 1-2 cells away at d5~0.8-0.95.
- d5 vs distance: far side of maze ~0.12; adjacent-ish ~0.9+. Maybe d5=1/dist roughly.
- pkill -f PATTERN kills your own bash command if pattern appears in it!! Use pkill -9 -f "python3 name" AND background it or use kill PID.
- Dead reckoning drifts badly; per-cell moves get truncated (tr=0.33-0.42) at doorways -> cell aliasing -> maps self-contradict. Trust only local sensing.
## NEXT EPISODE PLAN
1. Start d6 reader + beacon immediately; code in /memory/src_backup (robot.py, drive.py, nav2.py etc).
2. HYPOTHESIS: goal flag may require BOTH robots co-located or the goal is in unexplored area. Try: rendezvous FIRST (climb d5), then explore together, or follow B.
3. Try sending B structured commands (it mimicked my format; maybe its agent reads messages): e.g. "B: meet at your 0,0" - it parked at its origin, maybe intentionally waiting for us!
4. Wallrun right-hand explorer (wallrun.py) is most robust mover. nav2.py frontier explorer finds doorways.
