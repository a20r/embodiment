# MEMORY NOTES (persisted across episodes)
## Episode 1 log
- Started. README: /dev/robot/ ports, ASCII lines. d8=transmit, d10=receive (to other robot).
- Goal: find other robot, both reach goal within 1 min of each other.
## Device map (VERIFIED ep1)
- d1 (write): wheel speed, persistent until "0". Value 10 => ~50 enc counts/s. + turns robot CLOCKWISE (heading increases) => d1 = LEFT wheel. Encoder on d9.
- d7 (write): other wheel (RIGHT). + turns CCW (heading decreases). Encoder on d6.
- d2 (read): 16 range beams, comma sep, meters; -1 = dropout. Beam i at heading + i*22.5 deg (clockwise numbering). Beam0 assumed forward. Noise ~±0.03.
- d3 (read): "tick=N goal=0 here=0" tick ~100/s. goal/here flags.
- d4 (read): compass heading deg, noisy ±3.
- d6,d9 (read): encoders (right,left). d0,d5: always 0 so far (bumpers?). d11: ~0.52 (battery?) 
- d8 write / d10 read: text link to other robot.
- Rotate in place: d1=4,d7=-4 -> ~7.3 deg/s. ~310 counts of wheel travel diff per rad.
## World
- Start cell: 0.5m square cell, walls all around except opening at compass ~90deg, corridor 2.75m.
## Calibration (ep1)
- Range beams saturate at ~2.75m (reading 2.7x = "far"). -1 = dropout, use medians. Beam only valid along corridor axis: if heading is off by 9deg, beam0 grazes side wall!
- Encoders (d6,d9) are just integrators of commanded speed (count even when robot doesn't move). ~4.5 counts/s per speed unit. NOT reliable for distance.
- Actual speed: cmd 10 -> ~0 m/s (deadband), cmd 20 -> 0.054 m/s, cmd 40 -> 0.13 m/s. Rotation d1=4,d7=-4 -> 7.3 deg/s.
- Compass: heading increases clockwise (N=0,E=90). d1=left wheel (+ => CW turn), d7=right wheel.
- Corridors ~0.5m wide, grid aligned to compass N/E/S/W. Start cell exits only East; corridor east is >2.75m long.
- Use closed-loop: range sensors + compass. Hold heading strongly at high speed.
## Robot 1 (partner) protocol (ep1, ~19:52)
- d11 sig = proximity between robots (0.998 at 0.5m, 0.87 at 1m, 0.69 at 1.5m). Robot 1 hill-climbed to me at ~19:46.
- Msgs cut at 250 chars. Robot 1 agreed: I explore E/N, it explores W/S, 0.5m cell steps. ORIGIN = my spot when R1 was 0.5m W of me. Path format 'E3 N2 W1'.
- Explorer: src/explore.py (logs /memory/explore_log.txt, map /memory/map.json). Start cell of explorer = origin+1E.
## Map knowledge (ep1, coords: explorer cells, (0,0)=origin+1E; x east, y north; cell=0.5m)
- Row y=0 corridor from (-3,0) [my START cell, dead end] to (4,0) [dead end E]. No N/S openings except (4,0)->N.
- Loop: (4,1)-(3,1)-(3,2)-(2,2)-(2,3)-(1,3)-(1,2)-(1,1)-(0,1)-(-1,1). (2,1) dead end off (1,1). All walls otherwise.
- Only unexplored exit of my region: west of (-1,1). Robot 1 came from there.
- LESSONS: step failures must not increment pos. Robot 1 wandering in my corridor confuses range readings (looks like a wall that moves). sig passes through walls (pure distance).
- 20:26 FOUND passage: from (0,3) go N to (0,4), W to (-1,4) [N open], then N (-1,5),(-1,6), E along row 6: (0,6),(1,6),(2,6), S (2,5)... Robot 1 says it FOUND GOAL (its here=1). here flag = at goal.
## GOAL FOUND (ep1 20:32): status 'here=1' = at goal. GOAL cell = explorer (7,0) = origin + E8. 
- Path from origin (-1,0): E5 to (4,0), N,W,N,W,N,W,S,S,W,W (to (-1,1)), N,N,E,N,W,N,N (to (-1,6)), E,E,E,S,E,E,N,E,S,S,W,S,E,E,S,S,S,E -> (7,0). (Cells listed in earlier notes.) The (7,0) dead end is off (6,0) which is reached from (6,3) via S,S,S.
- Robot 1 also found it independently. Need both there within 1 minute.
## FULL PATH FROM START CELL (-3,0) TO GOAL (7,0), letters = 0.5m cells (ep1 verified):
E7 (to (4,0)), N, W, N, W, N, W, S, S, W, W (now (-1,1)), N, N, E, N, W, N, N (now (-1,6)), E, E, E, S, E, E, N, E, S, S, W, S, E, E, S, S, S, E -> GOAL (7,0), a dead end east of (6,0).
Compact: E7 N W N W N W S S W W N N E N W N N E E E S E E N E S S W S E E S S S E
- Goal zone radius ~0.2m around cell center. 'here=1' in d3 when inside. 'goal' flag stays 0 (maybe = both done).
- Robot 1 (partner) went silent 20:30-20:45 in a "waiting for sig>0.85" loop even when I stood 0.5m away. It thinks radio is <1.5m range (it is not; I got msgs at sig 0.3).
- NEXT EPISODE TIPS: run src/explore.py-style stepping (turn_to, recenter, drive_front, snap). Tell robot 1 goal path immediately: from ORIGIN (=start cell +E2) the goal is E8 (through the maze via path above). Ask robot 1 for ITS start-relative path too.
## CODE saved in /memory/src (copy back to /bot/src at start of next episode: cp /memory/src/* /bot/src/)
- rio.py (FIFO I/O), ctl.py (turn_to, set_speeds, hd), nav.py (medscan, cardinal_view, drive_front, drive_blind, steer), explore.py (step/recenter/snap, DFS main(maxsteps,start,route,previsited)), rotscan.py (360 scan ascii map), logger.py, rx.py (receiver -> /tmp/rx.txt), poll.sh, wait.sh.
- Usage: nohup python3 src/rx.py & ; python3 src/rio.py w 8 "msg" ; python3 src/explore.py 0 "(x,y)" "(90,0,...)" "[]" to route by bearings without exploring.
- 20:59 Robot 1 static ~0.5m W of (6,2) (probably (5,2), behind wall) since 20:30, unresponsive. Unexplored frontier cells: (3,3) [S of (3,4)], (3,6) [W of (4,6)]. Sig at goal with R1 there = 0.69.
## PLAN FOR NEXT EPISODE (written 21:02 ep1)
1. cp /memory/src/* /bot/src/; start rx.py; verify start cell = dead end with opening East, corridor 2.7m (then start=(-3,0)).
2. Immediately message robot 1 (it has its own memory): "Goal = the dead-end cell you saw here=1. Go there directly; wait just OUTSIDE the goal zone (1 cell before it) until our sig>0.9, then both enter and hold." Both arriving within 1 min is required; unclear if waiting inside counts, so enter together.
3. Navigate the compact path (E7 N W N W N W S S W W N N E N W N N E E E S E E N E S S W S E E S S S E) using explore.py route mode: python3 src/explore.py 0 "(-3,0)" "(90,90,90,90,90,90,90,0,270,0,270,0,270,180,180,270,270,0,0,90,0,270,0,0,90,90,90,180,90,90,0,90,180,180,270,180,90,90,180,180,180,90)" "[]"  (verify each 'routed to' line; ~20s/cell => ~15 min).
4. Watch d3 'here=1'. Then coordinate with robot 1 via d8/d10 (msgs <=250 chars).
- Robot 1 unresponsive from 20:30 in ep1 (likely its agent ran out of budget). Be FAST next time; give it the goal path early.
- 21:10 autopilot.py running: msgs every 45s, logs RX to /memory/autopilot_log.txt, re-enters zone if R1 close & goal flag 0.
