# Robot notes (episode 1)
Ports (/dev/robot/):
- d0: read, always 0 so far (unknown)
- d1: WRITE drive speed. speed 2 ~= 0.33 m/s (rough)
- d2: read lidar CSV 16 beams. Beam i points at compass dir (heading + 22.5*i) mod 360.
      Max range ~2.6-2.7 (capped), -1.0 = invalid sometimes. Noisy.
- d3: read status "tick=N goal=0 here=0". tick ~ 1/s? here likely =1 at goal.
- d4: read compass heading deg, noisy +-2. POSITIVE d7 turn DECREASES heading.
- d5: read, 0 so far
- d6: read counter, counts during ROTATION (~5.2 counts/deg)
- d7: WRITE turn rate (approx deg/s, positive decreases heading)
- d8: WRITE tx radio. d10: read rx radio (empty line if nothing). Other robot exists;
      no reply to "hello" yet (short range).
- d9: read counter, counts during DRIVE: ~5 counts per (speed*sec). m/count between 0.006-0.015 (bad calib)
- d11: read float 0.35-0.52, position dependent. Hypothesis: beacon signal (RSSI to other robot or goal?)
Goal: both robots must be at goal location within 1 min of each other.
Env: maze, corridors ~0.5m wide.
Driving veers: heading drifts + when driving; use closed-loop heading hold.
Code in /bot/src (rob.py helpers, explore.py) - COPY TO /memory each episode!

## MOTOR TRUTH (important!)
d1 (v) and d7 (w) are MIXED channels, both affect translation AND rotation:
- Translation ~ 0.09*(w - v) m/s small-signal (positive = toward beam0=front); saturates ~0.45
- Heading rate ~ (v - 0.8*w) deg/s (positive = heading increases)
- FORWARD STRAIGHT: (v,w)=(16,20) gives ~0.19 m/s, trim v +- (0.9*err) to hold compass heading H (err=norm(H-h), v=16+clip)
- PURE-ish TURN: v=0, w=turnrate → heading DECREASES ~0.8*w deg/s, little translation at w>=10 (nonlinear, w=5 translates a lot! avoid small w alone)
- (v,w)=(5,5)=nothing. Motors nonlinear/stick-slip: sometimes stuck at low differential; use bigger values.
- d9 = integral of v (5 counts per v*s); d6 = integral of |w|. TACHOMETERS not ground truth (slip!).
Use lidar+compass as ground truth. Robot can get wedged; escape by reversing (v,w)=(20,16).

## d11 = RSSI to other robot "A" (MOVING source!)
- Confirmed: standing still, sig rose 0.03->0.29 then fell (A patrols the maze).
- sig ~0.5 when A within ~1 cell; radio range even shorter: got ONE rx msg when sig~0.48:
  "PING A x=-0.60 y=-1.82"  (A broadcasts its dead-reckoned pose)
- A did NOT respond to our hello/PING B yet (maybe out of range when we sent).
## Strategy state (end ep1):
- pounce.py: wait till sig>0.18 then greedy gradient chase w/ short probes. Running.
- Unknown: how goal is revealed. status d3 "goal=0 here=0" never changed yet.
- Code copies: rob2.py (good primitives), pounce.py, chase.py, fine.py, seek.py in /memory.
- rxlog.sh logs radio to /memory/rx.log (run it first thing!).
## TODO next episode:
1. start rxlog.sh; 2. run pounce.py-like chase; when radio contact, keep exchanging
   messages - try "GOAL?", follow A, or stay in radio range and see if A leads somewhere.
3. Maybe A follows a fixed patrol loop: could map sig peaks over time.

## Episode 1 final summary
- Never reached goal; never got 2nd radio msg. Spent most time learning motor model
  (see MOTOR TRUTH) - that knowledge is the big win, don't re-derive it.
- Biggest inefficiency: no global map/odometry -> wandered blindly. NEXT TIME:
  build junction graph with dead-reckoning (compass + commanded fwd time at 0.19m/s
  is decent) right from the start, and record sig(d11) everywhere, timestamped.
- A's RSSI (d11) field is steep: 0.5 adjacent, ~0.01 at >3-4 cells. Passes of A
  observed at: episode-start cell (t~0-300s), and once ~t=1788317460-1788317540.
- Suggested ep2 plan:
  1) start rxlog.sh, 2) run hunt.py while ALSO logging pose, 3) when radio contact:
     stop, spam pings + "GOAL?", record everything A says, try following it
     (it may lead to goal); watch d3 for here=1/goal=1 and note pose when it flips.
  4) If A replies never, hypothesis: goal revealed by proximity handshake; just STAY
     with A (keep sig>0.4) as long as possible and observe status.
