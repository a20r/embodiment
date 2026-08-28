# Strategy log (episode 1)
- Built /memory/code/robot.py (sensor threads, cmd) and nav.py (Nav: dead-reckon,
  turn_to via wiggle strokes, drive with obstacle stop).
- explore3.py greedy frontier: got stuck circling small room area.
- wallfollow.py right-hand: running. Watch /bot/src/wf.out; goal flag via d0.
- World: corridors + rooms, scale 0.1-2.8 units, car-like robot radius ~0.6-1.2.
- Invisible barriers exist! Lidar sees through them; detect via no-scan-change stall.
- If wall-follow loops forever, try left wall, or period random 180 flips.
- d6 turned out to be a GOAL PROXIMITY/VISIBILITY flag! It reads 1 continuously in
  a region (found near heading~93 spot with beam0(w93)=2.35, beam12(w3)=3.0 open).
  Use as beacon: hill-climb keeping d6=1.
## MAJOR REVISION (ep1 ~T+2h)
- d6 is NOT static-positional: returning to same pose gives d6=0. Two d6=1 events:
  wedged 3min at a junction (d6=1 solid), flipped 0 right as we displaced east.
  Just before flip, beam7 (world ~248) changed 1.50->1.20 with robot static =>
  A MOVING OBJECT crossed the lidar! Hypothesis: GOAL IS MOBILE (wanders maze).
  d6 = proximity flag (radius ~1-1.5?). Goal moves ~0.15/s (faster than us!).
- Plan: PARK at junction with good LOS; monitor scan deltas for moving blob + d6;
  when d6=1 pursue blob bearing. Waiting works: it came to us while wedged.
- Junction "S*" scan (h=93): [2.35,1.28,0.72,0.52,0.52,0.58,0.84,0.69,0.67,0.78,
  0.11,0.15,3.00,1.35,0.72,0.54]; corridors: E-W (~93/273deg) + N (~3deg, 3.0 long).
## End-of-ep1 status (~T+2h45m)
- Never reached goal (d0 goal stayed 0).
- Best hypothesis: goal is MOBILE (or d6 zone is tiny); d6=1 = proximity flag.
  Encountered d6=1 once, while wedged at a junction; a beam changed 1.5->1.2 with
  robot static right before losing d6 (object moving at ~0.15/s?).
- Pushing into walls: d5=1, robot cannot pivot when wedged; invisible barriers
  might actually have been misread (some stalls were real walls at odd angles).
- Working scripts in /bot/src (wiped!) copies in /memory/code: robot.py nav.py nav2.py
- wf2.py = wall follower + d6 pursuit reflex: the best autonomous searcher so far.
- ADVICE FOR NEXT EPISODE:
  1. Start wf2-equivalent IMMEDIATELY (copy from /memory/code), watch d6.
  2. When d6=1: STOP, watch scan deltas for moving blob, chase it; goal flag
     presumably fires within small radius of goal object.
  3. Turning is slow (wiggle); prefer reverse driving over 180 turns.
  4. Do not trust dead reckoning >1min; use scan landmarks.
