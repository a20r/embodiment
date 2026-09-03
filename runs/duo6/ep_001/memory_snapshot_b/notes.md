# Robot notes (ep1)
Ports /dev/robot/:
- d0: radio TX (write line)
- d4: radio RX (read; gives empty lines when nothing? short-range)
- d1: compass heading deg (noisy +-5). d10 fwd increases heading, d11 fwd decreases.
- d2: unknown, reads 0
- d3: lidar 16 beams CSV, max 3.000, -1.000 = dropout. beam i angle = i*22.5deg?
      forward ~ between beam 15 and 0 (both shrink when driving fwd). likely beam0=forward.
- d5: response pipe: after each write to d10/d11 it emits a number ~0.13 (meaning? maybe ack/battery)
- d6: status "tick=N goal=0". tick ~ 240/s? goal flag.
- d7: left encoder counts (d10=left wheel speed)
- d8: right encoder counts (d11=right wheel speed)
- d9: reads 0, unknown (bump?)
Motion: write signed speed to d10/d11 (velocity setpoints, persist until changed).
 speed 10 both => ~0.02 units/s fwd. turn rate ~ (d10-d11) deg/s approx.
 encoders ~4-5 counts per unit speed per sec.
Maze walls at 0.1-3.0 lidar units.
Other robot exists; must both reach goal within 1 min of each other.

## Ep1 progress
- explore.py wall-follower (left-hand, beam12 side) works. Speed base up to 90 cmd.
- Odometry: UPC=0.001565 units/enc count; compass blend. Pose logged /memory/traj.jsonl (resumes from last line on restart).
- Maze extents seen so far: x -1..9.5, y -11..6. Corridors 0.3-1 wide.
- Radio: TX beacon 1/s; RX (d4) polls empty line when no msg. Nothing received yet.
- d2,d9 remain 0 everywhere so far. d5 emits ~0.13 per motor write (drained in thread).
- CAUTION: pkill -f matches your own bash -c command string; use [b]racket trick.
- RADIO CONTACT t=1787978283: "PING bot x=2.19 y=1.00 h=93.7" (their frame).
  Our pose then ~ (2.5,-1.7) our frame. Radio is short range -> other bot was near there.
- Code copies in /memory: robot.py explore.py occmap.py plot.py.
  Quickstart: mkdir -p /bot/src; cp /memory/*.py /bot/src/; cd /bot/src; nohup python3 explore.py &
- Maze is LARGE: x -30..+9.5, y -11..+6 so far (units). Speeds: cmd50=0.34u/s, cmd100~0.76u/s max~1.9u/s(cmd>=256).

## KEY FINDINGS
- d5 = range/signal to OTHER ROBOT (bot B) — rises when close. >0.9 = radio decodes reliably.
- Bot B is another agent; chases us via its range sensor. Neither knows goal.
- PLAN (in effect): A (me) wall-follow explores, throttles when d5<0.45; B chases.
  On d6 goal=1: stop, broadcast GOALFOUND, B homes in. Both must arrive within 1 min.
- convoy.py implements this. Radio log /memory/radio_rx.log.
