# Robot notes (episode 1 findings)
Ports /dev/robot/:
- d0: read, single int (0 so far). unknown (bump?)
- d1: read, 16 lidar ranges CSV, max ~3.0, -1.0 = invalid/noise. BODY-FIXED.
  Beam i body angle ~= (i-4)*22.5 deg CCW (idx4 = FRONT; idx increases CCW/left).
- d2: read, heading deg, CCW positive, noisy ~+-2deg
- d3: read, receiver (line) from other robot
- d4: write, LEFT wheel speed (float; 5 gives ~0.55 units/s fwd w/ d5)
- d5: write, RIGHT wheel speed. d4>d5 -> heading decreases (turn right/CW)
- d6: unknown, blocks both read & write so far
- d7: read, single int 0. unknown
- d9: read, "tick=N goal=0". ticks ~100/4s? actually ~25/s
Goal: both robots must reach goal within 1 min of each other. goal flag in d9?
Speeds persist until changed; write 0 to stop.
No position sensor found yet (only heading + lidar).
CONFIRMED (ep1):
- FRONT = lidar idx0. beam i body angle = +i*22.5deg CCW (idx4 left, idx8 back, idx12 right)
- yaw rate deg/s = 0.905*(d5-d4)  (d5>d4 turns CCW/left)
- d0 = bump flag (1 when touching wall)
- positive d4=d5 drives toward idx0; speed at 15,15 ~0.5-0.6 u/s (maybe capped)
- lidar max range 3.0, occasional -1.0 invalid readings; readings can be steppy
- v = 0.085*cmd per wheel avg (cmd 5->0.43u/s, 15->1.0u/s, cap ~1.05 at 30)
- yaw = 0.905*(d5-d4) deg/s
- d3 idle: emits empty lines constantly (nonblock read, filter blanks)
- corridors ~0.5-0.6 wide; bump at front lidar ~0.07
- pkill -f matches your own bash command string -- kill by ps/awk instead!
- controller: /bot/src/ctrl.py (right-wall follow, logs to /memory/run.log)
- d6 = beacon signal (scalar ~0.1-0.2 so far, position dependent, climb it!). ctrl2.py seeks gradient + wall-follow fallback.
EP1 later findings:
- d6 signal at same dead-reckoned spot changed over time (0.15 -> 0.20) =>
  source likely MOVING => probably distance to the OTHER ROBOT, not a static goal.
  Or dead-reckoning drift. s range seen: 0.11-0.21.
- s model fit ~ C/d (p=1) but data too flat to localize source; fits unstable.
- Dead-reckoning inflates distance when grinding walls; use short straight bursts.
- No RX ever received yet (radio short-range). d3 idle = blank lines.
- Strategy that covered most ground: ctrl3.py right/left wall-follow with
  s-trend-based side switching. Reached s~0.21 near dead-reckoned (-20,-14)
  region (frame of ctrl3 start).
- Controllers in /bot/src (ctrl3.py best; home.py = stop-and-probe homing, flails
  when s locally flat).
