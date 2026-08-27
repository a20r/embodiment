# Robot notes (episode 1)
Ports /dev/robot/ (named pipes, ASCII lines):
- d0: read, int, usually 0 (bump sensor? unknown)
- d1: read, 16 lidar ranges CSV, ~0.2-0.8 units, -1.0 = dropout/no return (noisy)
- d2: read, LEFT wheel encoder (counts, +fwd)
- d3: read, compass heading degrees 0-360, noise ~+-2deg
- d4: read, "tick=N goal=0" status; tick ~50/s. goal flag -> reach goal when 1?
- d5: read, int, usually 0 (unknown)
- d6: WRITE left wheel speed (float; persists until changed)
- d7: WRITE right wheel speed
- d8: read, RIGHT wheel encoder
Calibration: wheels +-v spin: heading rate ~0.95*v deg/s, encoder ~2.5*v ticks/s.
Speed 100 works fine. Reads: each read gets latest line (head -1 with timeout).
Env: small ranges (0.2-0.8) -> maybe narrow maze corridors.
More findings:
- d5=1 front bump (when pressed against wall). d0 likely rear bump.
- Lidar beams: 16, beam i points at (robot frame) fwd + (i-4)*22.5deg CW... i.e. beam4=forward, beam8=right, beam12=back, beam0=left. Verified by rotation shift test (cw 90 -> shift 4) and forward test.
- Compass d3: clockwise-increasing bearing; forward motion doesn't change it.
- Distance: ~333 encoder ticks per lidar unit (rough). speed v -> 2.5*v ticks/s -> v/133 units/s.
- Turning: wheels(+v,-v) = clockwise = heading increases at ~0.95v deg/s.
- Corridors ~0.2-0.4 units wide; robot radius ~0.12 (bump at front lidar ~0.13). Lidar max seen ~0.86.
- Strategy: wall following + check d4 goal flag.
CORRECTION: forward = beam 3 (not 4). right=beam7, back=beam11, left=beam15.
Beam i bearing = forward + (i-3)*22.5 deg clockwise.
Lidar max ~0.9 (capped). Contact when front beam ~0.13. Reverse works.
Encoders integrate commanded speed even when stalled vs wall (unreliable when bumping).
Episode 1 run: right-wall follower (src/follow.py) started ~15:01, works well.
Bash commands limited to 60s (sleep<=50). Reading pipes races with controller (2 readers).
Trajectory so far spans x 0..16, y -1..17 units (dead reckoning w/ ang=heading; offset arbitrary).
d0 always 0 so far (rear bump? unknown). d5 front bump confirmed.
Working code saved: /memory/rc.py, /memory/follow3.py (wall follower with
periodic escape every 100s + side switch + stuck detection; checks d4 goal flag).
Usage next episode: cp /memory/*.py /bot/src/; cd /bot/src; nohup python3 follow3.py &
Maze is LARGE: traveled y range ~40+ units. Corridors + open rooms + pillars.
Goal flag never fired yet as of 15:29 ep1 (~30min in).

=== END OF EPISODE 1 SUMMARY (wallclock ran out, goal NOT found) ===
What works (verified):
- rc.py helpers: lidar/heading/enc/wheels/turn_to. follow3.py: wall follower
  with bump recovery, stuck detection, escape every 100s + side switch.
- Robot moved continuously for ~35 min without failures. goal flag (d4) never fired.

Key facts (re-verify quickly but trust these):
- d6/d7 write left/right wheel speed (persist). d2/d8 encoders (~2.5 ticks per
  unit speed per sec; ~333 ticks per lidar unit). d3 compass deg CW-positive.
- d1: 16 lidar beams; beam 3 = FORWARD, beam 7 = right, 11 = back, 15 = left.
  -1 = dropout. Contact at front reading ~0.13. d5=1 front bump. d0 unknown always 0.
- Speed 45 cruising works; 100 ok for turns. wheels(+v,-v)=turn right (heading increases).

Maze: HUGE (explored span roughly 60 x 45 units this episode; corridors ~0.2-0.5
wide, open rooms, pillars). Episode 1 spent too long orbiting small wall loops
before adding escapes.

ADVICE FOR NEXT EPISODE:
1. cp /memory/rc.py /memory/follow3.py /bot/src/; cd /bot/src; nohup python3 follow3.py > follow.out 2>&1 &
   Then poll: sleep 50; tail follow.log; head -1 /dev/robot/d4  (goal=1 means done).
2. START IMMEDIATELY (5 min setup max). Don't redo calibration.
3. Consider improving coverage: episode 1 wandered locally. Ideas:
   - Bias escapes toward unexplored odometry quadrants (track visited grid in the
     controller, choose escape direction toward least-visited side).
   - Or pick one compass direction (e.g. keep heading ~180-270) and push far, since
     goal is probably far from start. Ep1 start region explored heavily: within
     ~15 units of start (many pillar rooms). Far SW (-60,-45 in run3 frame) reached at end.
   - Trajectory of run3 in /memory/traj_ep1_run3.txt (start3 frame, start=(0,0)).
     Run3 went: south ~35, then west ~60. Unknown if same map/start next episode.
4. Watch out: commands killed at 60s (sleep<=50). Don't pkill with pattern matching
   your own command line. Two readers on one pipe race (controller + your reads OK
   since controller retries).
5. Log odometry grid to /memory periodically so a killed episode still leaves a map.
