# Robot notes (episode 1 discoveries)
Ports /dev/robot/:
- d0: write radio TX (line)
- d1: read heading degrees (compass, noisy ~1deg)
- d2: read ? (0)
- d3: read lidar 16 beams CSV, -1.000 = invalid; beam spacing 22.5deg
- d4: read radio RX (returns empty line if nothing?)
- d5: read ~0.13 (?)
- d6: read status "tick=N goal=0" (goal flag -> at goal?)
- d7: read left? encoder counts
- d8: read right? encoder counts
- d9: read ? (0)
- d10: write left wheel speed (persists until changed)
- d11: write right wheel speed
Writing 1/1 moves ~5-6 enc counts/sec per wheel.

## Calibration
- ~1500 enc counts per meter (avg both wheels)
- enc rate ~5.2 counts/s per speed unit; speed 100 works (0.33 m/s)
- turning: differential 5 counts/deg; at motors(10,-10) approx 21 deg/s (heading increases = clockwise)
- lidar beam i points at absolute compass angle (heading + 22.5*i) mod 360; beam0=front, beam4=right, beam8=back, beam12=left
- lidar max range ~2.3m; -1.0 = invalid reading
- corridors ~0.5m wide (walls at ~0.2-0.25 each side)
- sensors: heading read 23ms, lidar 42ms; d6 tick ~20/s
- d5: slowly varying 0.13-0.21, unknown (maybe signal strength / battery). d2,d9 always 0 so far.
- pose convention in my code: x=east=sin(h), y=north=cos(h), h in compass deg

## Strategy
- explore.py: right-hand wall following, logs pose+lidar to /memory/trail.jsonl
- radio: TX beacon "PING from=alpha x= y=" every 3s on d0, poll d4; log to /memory/radio_rx.log
- d6 goal=1 presumably when at goal -> logged to /memory/GOAL_FOUND.txt

## Episode 1 progress log
- Other robot = "beta" (another agent). RX msgs: "A pos x y", then "PING from=beta x= y=", then "PING from=beta wf" (wall following?). It explores fast (~0.25m/s), its frame spans x 0..-7.8, y -2.8..2.7 (its own odom frame, origin=its start).
- I proposed protocol: goal finder STOPS ON GOAL, sends GOALFOUND every 10s, waits for partner. No explicit ack yet.
- nav2.py = junction-based explorer (turn to cardinal, drive, stop at openings/blocks, prefer least-visited, right-hand tiebreak). Works. ~85 speed in open, 40 near walls.
- lidar max range 3.0 (not 2.3)
- gotcha: pkill -f pattern matches the bash -c wrapper of my own command => kills my shell. use exact pid files.
- my explored region so far: x -1.2..2.0 y -0.4..4.4 (my frame, origin = episode start pos)

## END OF EPISODE 1 SUMMARY (wrote at ~92min mark)
GOAL NOT FOUND YET by either robot. Beta (other agent) also still exploring ("PING from=beta wf").

### Agreed radio protocol with beta (it ACKed explicitly):
"goal finder stops on goal, sends GOALFOUND every 10s, waits for partner"

### World facts (should persist? maze layout may be same - VERIFY early):
- maze axis-aligned, corridors ~0.5m wide, cell pitch ~0.5m
- MY frame this episode: origin = start pos, x=east y=north (compass h: 0=N cw)
- explored bounds: x -1.6..4.5, y -2.0..6.1. West wall x=-1.6 (y 0.5..6), north wall y=6.1, east wall ~x=4.5
- south warren (y -2..-0.5, x 0..1.5) fully walled dead pockets - NO GOAL there
- goal flag: /dev/robot/d6 "tick=N goal=0" - assume goal=1 when on goal (never saw 1)

### UNEXPLORED candidates for goal (my frame ep1):
1. interior pocket x 2.3-2.7, y 1.5-3.5 (free by lidar, never entered; entrance not found from S at (2.32,1.46))
2. pocket around x 3-3.5, y 3-4 (cells (6,7),(8,8))
3. east edge cells (8,10),(9,12) = x 4,y 5-6
4. SE corner x 2.5-4.5, y -1..0 (cells (5,-2) S frontier from (2.5,-0.5))

### Tools in /bot/src (COPY FROM /memory/src_backup EARLY!):
- robot.py: port I/O lib
- mv.py: manual teleop: python3 mv.py T:180 F:0.9:270 (turn-to, fwd dist:heading w/ wall centering + goal check). Reads/saves /memory/pose.json
- nav3.py: junction explorer w/ graph+frontier BFS (dithers sometimes)
- nav4.py: occupancy A* frontier (mapping drifts, got blocked loops - needs work)
- explore.py/nav2.py: older wall-follower/junction versions
### Files in /memory: pose.json, graph.json, open_seen.json, trail2.jsonl (poses+lidar), radio_rx.log
### Next episode plan:
1. cp -r /memory/src_backup/* /bot/src/; check ports same
2. NOTE: new episode = new start pos; old map frame useless unless re-registered. Do scan-matching vs old trail2 lidar if keen, else fresh explore.
3. contact beta immediately, re-agree protocol
4. prioritize open areas / pockets; goal likely in rarely-visited pocket
5. run junction explorer but fix dithering: blacklist unreachable frontiers, oscillation detector

## KEY DISCOVERY: d5 = proximity/signal strength to the OTHER ROBOT (not goal!)
- verified: stationary, d5 drifted 0.39->0.55->0.39 as beta moved. Earlier hit 0.99 when beta passed adjacent.
- ~0.13-0.2 when beta far, ->1.0 when adjacent. Use for rendezvous + frame alignment:
  drive to maximize d5, meet beta, exchange odom positions via radio => frame offset.
- goal detection still ONLY d6 'goal=1'.

## Episode 1 final status (05:49, ~19min left if 100min budget)
- GOAL never found by alpha or beta. Explored (current-frame): x 0..5.5, y -2..5.5 nearly fully; also west x -1.6..0 in early (drifted) frame.
- ODOMETRY DRIFT ~0.5m/30min in y. Old graph coords misalign with later pose. Wipe graph/open_seen when stale; pose.json fine short-term.
- East extension found late: corridors x≈4.4, x≈5.1 (y 0..4.6), east area up to x=5.5!! Maze wider than early est.
- Areas never seen (candidates for goal next time):
  * WEST half beyond x<0 in late frame (only explored early w/ different drift; RE-EXPLORE WEST FIRST)
  * any pocket beyond x>5.5?
- beta explores its own frame span x 0..-7.8, y -2.8..2.7 -> beta started ~8m east? of its west edge... unknown overlap with mine.

## NEXT EPISODE PRIORITIES
1. cp /memory/src_backup/*.py /bot/src/
2. start nav3.py fresh (rm /memory/pose.json graph.json open_seen.json visits.json first - old frame useless)
3. radio beta immediately; protocol already agreed (finder stops on goal, GOALFOUND every 10s)
4. d5 = beta proximity (1.0 adjacent). If beta stops (GOALFOUND), HILL-CLIMB d5 to find beta = goal location!!! (src: hill.py climbs d5)
   THIS IS THE KEY TRICK: second robot doesn't need coords - just maximize d5 while checking d6 goal flag.
5. watch d6 'goal=1' constantly (nav3 does)
6. speed: could raise SPEED_FAST to ~100; collisions seem harmless (no damage observed)
