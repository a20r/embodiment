# Robot notes (episode 1)
Ports /dev/robot/ (all FIFOs, ASCII lines):
- d0: READ lidar, 16 beams csv, max range 3.0, -1.0 = dropout. Beams fixed to robot frame.
- d1: WRITE wheel speed A (left?). numeric.
- d2: READ status "tick=N goal=0". tick ~50/s. goal flag presumably ->1 at goal.
- d3: WRITE radio tx. No replies seen at start position (short range - retry near beacons).
- d4: READ "0" (bump? unknown)
- d5: READ radio rx (empty so far)
- d6: WRITE wheel speed B (right?). d1=+,d6=- => heading (d10) DECREASES (~9-14 deg/s at 5).
- d7: READ gives occasional "1" (unknown)
- d8: READ odometer tick count (increases when wheels move)
- d9: READ "0" (unknown)
- d10: READ compass heading deg, noisy +-2
Start pos: heading ~178. d0 start: 0.28,0.25,0.35,0.60,3.0,0.65,0.32,0.26,0.27,0.26,0.30,0.51,1.30,0.53,0.33,0.22 (beam4=3.0 open)
Forward (both +): beams 0,1,15 decreased, 5,10 increased => forward ~ beam 0? TBD.
Speed 5 both wheels = slow (~0.05 m/s?). Try higher speeds.

## Key findings (ep1, later)
- Sensors are ON-DEMAND: each open of the fifo yields one fresh line. A held-open
  stream does NOT keep producing while stationary. Use open/read/close per poll
  (see /memory/src/robolib2.py).
- Calibration: odo (d8) m/tick = 0.00176. wheel speed 50 both = 0.42 m/s.
  speed ~0.0085 m/s per unit. Turn: (30,-30) = -57 deg/s (heading DECREASES).
  d1=left wheel, d6=right wheel. Positive both = forward.
- Lidar d0: beam0 = forward, beams CCW, 22.5 deg apart, max 3.0, -1=dropout (reuse last).
- Robot can wedge into wall corners: wheels spin, odo counts, but lidar static.
  Detect stuck via LIDAR variance over ~4s, not odometry!
- pkill -f with script name in the SAME bash command kills your own shell. Use kill by PID.
- Environment: maze, corridors ~0.5-0.6 m wide. goal flag in d2 ('goal=1' expected at goal).
- Radio d3/d5 silent so far ('hello','ping' etc. no reply).
- Strategy running: right-hand wall follow (explore3.py).
- d7 = LEFT wheel encoder, d8 = RIGHT wheel encoder (both counters; d8 negative when wheel reverses).
- d4, d9 remain 0 always so far (bump? beacon?).
- Dead reckoning gets corrupted by wheel-slip when wedged. Use both encoders + lidar checks.

## EPISODE 1 RESULT: SOLVED
- GOAL reached at t=812s of explore5.py run; d2 flipped to goal=1.
- Winning approach: right-hand wall follow + P-control on side distance,
  lidar-variance stuck recovery, and "DASH" (on loop detection, drive toward
  most-open lidar beam for up to 14s) to escape wall-islands. See /memory/src/explore5.py.
- Goal position (dead-reckoned, origin = wherever ex5 started, which was
  ~mid-maze after prior wandering): x=-17.7, y=-19.9. Coordinates are relative,
  not absolute; maze is large (>20m span). Compass headings absolute though:
  the goal lay generally SOUTH-WEST (heading ~180-270 territory) of where I
  spent most time. From the true start position the robot first wandered near
  origin; goal was far away.
- Radio d3/d5: never received anything the whole episode. Probably unused.
- d4, d9 stayed 0 the whole episode (never learned their meaning).
- Timing: total ~75 min incl. all experimentation; the final controller alone
  took ~13.5 min from its start to goal. Next time: run explore5.py immediately.

## QUICKSTART FOR NEXT EPISODE
1. cp /memory/src/robolib2.py /memory/src/explore5.py /bot/src/
2. cd /bot/src && nohup python3 explore5.py > /tmp/e.out 2>&1 &
3. Poll: grep -aE 'GOAL|RX|DASH|RECOVER' /memory/telemetry.log | tail
4. Beware: never put a running script's name inside a bash command that also
   calls pkill/pgrep -f with that pattern (kills your own shell). Use pgrep -f 'name[5]'.
5. If a background controller is killed, wheels keep last speed: write 0 to d1 and d6.
