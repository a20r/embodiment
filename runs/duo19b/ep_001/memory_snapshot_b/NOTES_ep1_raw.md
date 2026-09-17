# NOTES (persisting across episodes) - read this first
Episode 1 started. README: ports in /dev/robot/, d8=transmit, d10=receive (other robot).
Goal: find other robot, both reach goal location within 1 min of each other.
## Port map (discovered ep1)
READ: d0(int), d2(16 floats - range scan?), d3(status "tick=N goal=0 here=0", 100 ticks/s), d4(heading deg?), d5(int), d6(int, saw 0 and -1), d9(int), d10(radio rx), d11(float ~0.5)
WRITE: d1, d7 (motors?), d8 (radio tx)
Reading a pipe gives one line per open; use nonblocking+select w/ timeout (helper in /bot/src/rio.py - recreate if lost)
## Drive model (ep1, confirmed)
d1=LEFT wheel speed cmd, d7=RIGHT wheel speed cmd (floats; tested 1..100, negative=reverse). ~5 enc ticks/s per unit cmd.
d9=left encoder, d6=right encoder (cumulative, noisy +-1). d4=compass heading deg, clockwise positive (left-only drive => heading increases).
d2=16 range beams, 360deg, beam0=FRONT, beam8=REAR, 22.5deg apart, clockwise order probably; -1.000 = invalid reading. ~2000 enc ticks per 1.0 range unit (rough).
d3 status: goal=0 here=0 initially (goal flag / other robot here flag?). d0,d5 always 0 so far. d11 ~0.5 float unknown.
## ep1 progress
- Rotation: ~2.86 diff-ticks/deg; forward ~1850 ticks per range unit. Max range ~1.45 (cap).
- Maze of corridors ~0.5 units wide. Start pose (odom origin) was facing ~180 (south) against a wall.
- d11 looks like beacon strength: rises toward SOUTH-WEST of start (0.2 at x=+3, 0.6 at x=-0.8). Following its gradient.
- Explorer code: /bot/src/explore.py (frontier-ish, visit penalty). Radio: no replies yet after ~5 min.
## RADIO facts (ep1): lines TRUNCATED at 256 chars -> keep msgs short. Other robot calls itself "B", is an LLM agent with same port layout (d11, d3 same).
- d11 = signal strength between robots (0.2 at ~5 units apart, 0.96 adjacent). here=1 in d3 when very close/adjacent.
- Met B at my odom (0.6,-1.3) ~10 min into episode (B started ~1.5 south of there, moved north).
## KEY (ep1): d3 'here=1' means I AM ON THE GOAL (positional; not related to B). 'goal' probably =1 when both robots on it.
GOAL LOCATION in ep1 odom frame (origin = start pose, x east, y north): ~(0.2..0.8, -1.0..-1.4). Path from start: south along x~0 corridor to y~-1.0, then east ~0.4.
  From start the robot faces ~180 (south) with wall in front; corridor continues south after turning slightly. B started ~1.5 south of the meeting point (0.6,-1.3).
Waiting for B at goal, broadcasting every 15s (/bot/src/beacon.py). B is slow to respond (LLM cadence ~1-2 min).
- d11 model (fit): d11 ~ 1/(1+(dist/2)^2) => dist = 2*sqrt(1/d11-1). 0.96=>0.4, 0.8=>1.0, 0.5=>2.0, 0.28=>3.2. Noisy +-0.02.
- B is an LLM agent: slow (msgs every 5-10 min), moves unpredictably. Both moving at once = wasted chasing. Rule: ONE robot stays still.
- Goal room (odom): x 0..1.0, y -1.5..-0.9; entrances: N corridor along x~0 (from start (0,0) go south), and SE opening near (1.0,-1.7).
- Odometry drifts badly (~1.8 units over 20 min). Re-anchor at goal when here=1. Goal scan signature: wall 0.15-0.2 on S/SSW side, open W/NW ~0.8, corridor N.
- 05:55: parked on goal 2nd time, beacon.py broadcasting; B ~3.4 away homing on d11.
## ep1 late (06:20): both robots on goal (both here=1) but goal=0 -> hypothesis: arrival times must be within 60s -> re-arrival protocol (both step off, then on within 15s of each other). Testing.
- Radio range ~1.3 units (msgs only arrive when d11>0.7). B climbs d11 to follow me; leading B in 0.6-unit hops worked great (B stays adjacent).
- If goal never flips to 1 even with synchronized arrival: consider other meanings (maybe both must stay N seconds; maybe zone is tiny).
