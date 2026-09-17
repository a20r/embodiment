# ROBOT NOTES (persisted across episodes) — READ FIRST
## Episode 1 (started 2026-09-17 03:06 UTC)
- /bot/README.md: ports at /dev/robot/d0..d11 are FIFOs. ASCII lines. d8=transmit to other robot, d10=receive from other robot.
- Goal: find other robot, both reach goal location within 1 min of each other.
- Wrong-direction open BLOCKS -> always use timeouts.
## Port map (verified ep1)
- d1 = motor A (write, -100..100 ok), d9 = its encoder (cumulative ticks). d7 = motor B, d6 = its encoder.
- d1=+v,d7=+v -> forward (toward beam 0). d1=+,d7=- -> heading d4 INCREASES (~1.85 deg/s per unit diff; +-100 => ~184 deg/s! use +-30 or less for fine turns)
- d2 = 16 range beams (CSV floats), beam i world bearing = d4 + 22.5*i. -1.000 = dropout/noise. Values ~0.1..1.4 seen.
- d3 = "tick=N goal=0 here=0" (~100 ticks/s). d4 = heading deg (noise ~+-2). d11 = float ~0.5, changes with motion (unknown; maybe RSSI/dist to other robot?)
- d0, d5 = always 0 so far (bump? unknown). Each open() of a FIFO gives one fresh line.
- Helper lib: /bot/src/rob.py (read_line(port,timeout), write_line(port,str), snapshot()) — recreate if lost (src is wiped).
- Calibration: ~1820 enc ticks per range unit. speed 60 -> ~0.17 units/s. Corridors ~0.5 wide, rooms ~1.5 wide.
- Start room (ep1 origin): robot started facing ~180deg near wall. Corridor north (+y) at x~0.1 from y~0.5 to 2.5 leads to another room.
- d11 decreases moving +x and +y (0.55 at start area -> 0.22 at (0.5,2.5)). Hypothesis: beacon/source toward -x,-y.
- /bot/src/explore.py = autonomous frontier explorer w/ dead-reckoning + occupancy grid (map.txt, d11.json, explore.log)
## KEY FACTS (ep1, ~03:24)
- OTHER ROBOT = "Robot A" (an LLM agent too). It talks on radio; msgs truncated at ~250 chars -> KEEP MSGS SHORT.
- d11 = radio signal strength between robots: d11 ~= 1.01 - 0.23*dist  (dist ~= 4.3*(1-d11)). Verified by fit.
- Robot A sat at MY-frame (-2.55,0.45) (= its odo origin) for ep1 first 20 min. It reported goal=0 here=0 too.
- My frame: origin=my start pose; x+=d*cos(d4), y+=d*sin(d4). Map copies in /memory/map.txt, grid.json (cell->score, >=2 occ), pose.txt.
- Goal NOT in explored region x[-2.7,0.7] y[-0.5,3.4]. d3 shows goal=0 here=0 everywhere so far.
- A's messages consumed by whichever process reads d10 first: run ONE listener (/bot/src/listen.py -> radio.log).
- DON'T use `pkill -f name` (kills own shell). Use: for p in $(pgrep -f "[e]xplore.py"); do kill $p; done
- 03:36 A's corridor (x~-2.7,y 0.1..2.9) is a DEAD END. All real frontiers were NORTH (y>3.5). Odometry drifts ~0.3 per 25min; wiped grid & restarted explorer at est. pose (-3.0,3.05). Old map: /memory/map_old.txt, grid_old.json.
- A is low on token budget; runs autonomously; will home on d11 if I send "GOAL AT (x,y)". If A finds goal it sends "GOAL AT (x,y)".
## STATE at 03:43 (ep1)
- World bigger than first thought: explored x[-5.2,1.0] y[-0.5,6.1] so far, NO goal yet. Rooms ~1 unit, corridors 0.45 wide, many walls.
- Strategy that works: frontier explorer (explore.py) w/ scored grid, Dijkstra soft costs, escape on stuck, scan-match. Pose drifts; wipe grid when inconsistent.
- If future episode: recreate rob.py/drive.py/explore.py from these notes (src wiped). Consider bigger step distance & faster speed to explore faster.
- A (other robot) exploring north (x -3..-2, y>3.5). Protocol: finder sends "GOAL AT (x,y)" every 5s & stays; other homes on d11.
## !!! GOAL FOUND (ep1 03:58) !!!
- GOAL INDICATOR = d3 "here=1" (not "goal"). goal=1 presumably when both robots there.
- Goal location: NW corner room of maze, my-frame approx (-5.4,5.0) (drifted est.; also seen at (-4.8,5.6)). Region ~0.5 wide.
- Route from A's dead-end corridor top (~(-2.8,2.9)): north to y~4.5-5, west along corridor y~5.0 to x~-5, NW room x[-5.2,-4.5] y[4.8,6.2]. Goal in its SW part.
- From MY start (0,0): goal is ~5.4 west, ~5.0 north. Start room exit: corridor north at x~0.1 (y 0.5..2.5), then west along y~2.5-3, north at x~-2.6 to y~5, west to x~-5.
- 04:07 Parked on goal (here=1) at current-frame (-5.4,5.0); goal room in map-frame x[-5.05,-4.6] y[5.2,6.0] (entered from south at (-4.8,5.05)).
- RADIO RANGE ~2.5 units (msgs only arrive when d11 > ~0.4). Had to walk toward A to deliver GOAL msg. A acked 04:04 and is heading NW via odometry+d11.
- beacon.py broadcasts GOAL AT + route every 5s and logs d3/d11 to beacon.log.
## LESSONS / PLAN FOR NEXT EPISODE (written 04:34 ep1, tokens nearly out)
1. GOAL = d3 "here=1". It is in the NW CORNER ROOM of the maze. From MY START (0,0): approx (-5.4,+5.0) (drift ±0.5).
   Route from my start: exit start room north via corridor x~0.1 (y 0.5->2.5); go west along y~2.5-3.0 to x~-2.8 (top of A's dead-end
   corridor); then NORTH x~-2.8 to y~4.2; NW staircase (W.3 N.3 W.3 N.3 W.4 N.25) to (-3.85,5.05); WEST along band y~5.0 to x~-4.8; N 0.5 into room x[-5.05,-4.6] y[5.2,6.0].
2. Robot A starts at my-frame (-2.55,0.45) = south dead-end of N-S corridor x~-2.7 (y 0.1..2.9), exit only at its top going east/north.
3. RADIO RANGE ~2.5 units (d11>~0.4). Tell A IMMEDIATELY (first minutes, while close): "goal indicator here=1; goal at (-5.4,5.0) my frame; route ...". A's agent is slow/low budget; it homes on d11 poorly.
4. Best plan: go straight to A first (it's 2.6 away), exchange frames, then LEAD it: stay within 0.5 (d11>0.9), hop 0.15, wait for d11>0.85. Or send it the route legs from ITS corridor top: N 1.2, W.3,N.3,W.3,N.3,W.4,N.25, W.95, N.55.
5. Pure d11 gradient-climb fails in maze pockets; use map+route. Dead-reckoning drifts ~0.3/25min; heading (d4) is absolute & reliable.
6. My explorer took ~50 min to find the goal; with the route above it should take <10 min to reach it directly.
- 04:38 ep1: parked on goal again (nw.py: reactive NW drive found here=1 in 90s from A's corridor-top area => goal is ~1.5-2 units NW of A's corridor top). Waiting for A; A silent since 04:26, d11~0.38 (A ~2.7 away, likely idle near its corridor).
- If ep2: first go to A (my-frame (-2.55,0.45)), tell it "here=1 is goal; goal ~2 units NW of your corridor top", then lead it with tiny hops.
- 04:42 ep1 final: parked on goal, nw.py + GOAL-AT broadcaster running in background; A static ~2.7 away.
- 04:58 ep1 end: lead.py returning to park on goal (here=1) + broadcasting GOAL msgs; A wanders ~1.5-2 units SE of goal, moves only when in radio range, d11-homing weak. Next ep: go to A FIRST and lead from <0.5 away.
- 05:08 ep1: shuttle.py running (goes 0.6 out toward A, broadcasts, returns to park on here=1, repeats). A silent since 04:57 & drifting away (d11 0.36). Task not completed in ep1.
