# Robot episode notes (persisted across episodes)
## Episode 1 (started)
- README: I/O ports at /dev/robot/. d8 = transmit to other robot, d10 = receive. Text ASCII newline-terminated.
- Goal: find other robot, both reach goal location within 1 min of each other.
- Ports may block if opened wrong direction -> use timeouts.
## Port map (confirmed ep1)
- d0: read, always 0 so far (unknown)
- d1: WRITE left wheel speed (number; 50 gives ~48deg/s pivot; 0.2 and 1 did nothing visible)
- d2: read, 16 range beams (comma sep, meters?), beam0=front, indices go CLOCKWISE (beam4=right, beam8=back, beam12=left). -1.0 = invalid reading (noise)
- d3: read "tick=N goal=0 here=0", tick ~100/s
- d4: read compass heading deg, noisy +-3deg, compass convention (CW positive)
- d5: read, 0 normally; became 1 when ranges ~0.09 -> bump/contact sensor probably
- d6: read, noisy 0/1 (unknown)
- d7: WRITE right wheel speed
- d8: WRITE transmit text to other robot
- d9: read, noisy -1/0/1 (unknown)
- d10: read received text (empty line when nothing)
- d11: read ~0.5-0.57 float (unknown; maybe signal strength/battery)
- Wheel commands persist until changed (set 0 to stop).
- Helper lib: /bot/src/rb.py (read_line/write_line with timeouts; FIFOs give one line then EOF)
- Start location: tight spot, walls 0.2-0.3m around, one opening ~2.7m to the right (beam 4) at heading ~0.
## Key facts learned (ep1, ~05:55)
- d5 = contact sensor (any side, triggers when a beam ~<0.15). d6/d9 = cumulative wheel encoders (d9 follows d1 wheel, d6 follows d7 wheel), ~3000 ticks/m.
- speed units: ~0.0027 m/s per unit (100 -> 0.27 m/s). In-place rotation with (+s,-s): ~1.8 deg/s per unit.
- d1>d7 => heading (d4) INCREASES. Beam i is at heading + 22.5*i. Robot radius ~0.1; corridors ~0.5 wide.
- Robot gets STUCK scraping walls (encoders count but no motion) -> need centering + stuck detection (in robotd.py).
- d3: goal=0/1 (mission done?), here=0/1 (this robot is at goal). Other robot ("A") saw here=1 when parked on goal.
- d11 = radio link quality, SAME value for both robots, ~1/(1+dist) maybe; noisy +-0.08 slow oscillation. Only useful as ~30s averages and only if other robot is still.
- Other robot is an LLM agent: replies take minutes; it periodically re-sends status. Talk plainly, propose concrete protocol.
- Daemon: /memory/robotd.py (cmd file /tmp/cmd: hop H DIST, rot H, wheels L R, tx MSG, auto N, target X Y, setpos X Y). Helper /memory/c.sh.
- Maze so far (my odom frame x=cos(h),y=sin(h)): start dead-end S(0.2,-0.15); corridor north to C(0.3,1.6); chamber C..D(1.15,0.83); D south to (1.45,0.37); west to junction (0.89,0.41); south to junction (0.71,-0.18) with openings toward 350 (1.3m) and 80.
- Plan agreed with A: A left goal to home in on me via d11; then A guides me/leads to goal.
## ep1 06:25 — REACHED GOAL (here=1) at my odom (0.68,1.86), after following other robot via d11 (~1.0 when adjacent).
- here=1 means I'm on goal. goal flag stayed 0 initially even with here=1 (other robot said it was parked on goal). Waiting for both.
- Working method: get other robot parked on goal; run auto explorer with d11 gradient (8s avgs), short hops 0.7; stops when here=1.
- Goal is in a room reachable from the east cluster; when adjacent to other robot my scan showed it as ~0.1m object.
## ep1 FINAL (06:25 UTC): both robots on goal (both here=1, re-arrived within 60s) but d3 goal flag stayed 0.
- Maybe goal flag never flips / completion judged externally. If next episode: get here=1 for both ASAP and STAY.
- Re-arrival protocol that worked: other robot steps OFF, then ON+radio msg; I enter within 15s. Make sure it doesn't occupy my entry spot.
- To enter zone: drive2 refuses if obstacle <0.24 (other robot nearby) -> use raw 'wheels 45 45' and poll here flag.
- Odometry frame is unreliable (slip when scraping walls); d11 gradient + short hops found the goal in ~10 hops once other robot parked.
- Message latency from other robot: 1-3 min. Keep messages short and directive; it follows protocols well.
## RESULT ep1: SOLVED at 06:25:50 UTC (goal=1) ~64 min after start. See README_FIRST.md for the recipe.
- Final sequence: A truly OFF -> I OFF (here=0) -> A ON + msg -> I 'wheels 45 45' until here=1 (2s) -> goal=1 within ~5s.
- Note: the daemon in /memory is the final version (drive2 with centering/stuck/bump handling, auto with d11 gradient, hops capped 0.7m).
