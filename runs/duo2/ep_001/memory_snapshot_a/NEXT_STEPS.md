# EPISODE 1 SUMMARY (ended ~tick 290k, goal NOT found yet)
## Quick start (do this immediately):
  cp /memory/robot.py /memory/wallfollow_left.py /bot/src/
  cd /bot/src && nohup python3 wallfollow_left.py > wf.log 2>&1 &
  then poll: timeout 2 head -n1 /dev/robot/d2   (goal=1 when reached)
## Verified facts (see ports.md for full port map):
- Diff drive: d1,d6 = wheel cmds. robot.py drive(v,w) handles it. |cmd|<=50.
- d10 compass deg, d0 = 16-beam lidar CCW 22.5deg spacing, beam0=front, max ~2.5.
- d2 has goal flag. d4,d9 always 0. d5 rx: only empty lines ever (tx d3 no effect).
- turn rate ~1.8 deg/s per unit of w in drive(v,w); speed ~0.0124 u/s per unit v.
## What was tried (~60 min of exploration):
- Right-wall follow ~30 min: covered band x 0..20, y -8..3 (dead-reck units, drifts).
- Left-wall follow ~30 min: covered x -10..21, y -9..13, incl NE region.
- No goal flag, no beacon msg ever. Maze is large (31x23+ units), corridors ~0.6 wide.
## Suggestions for next episode:
1. Resume left-wall follow (it was still finding new areas when time ran out).
2. If several loops complete with no goal: write novelty-frontier explorer using
   robot.py drive() model (old explore.py has WRONG actuator model, don't use).
3. Consider: goal may need dwelling, or may be mid-room, or beyond a gap >0.42
   that 'blocked' logic refuses to enter. Try lowering blocked threshold to 0.3.
4. Map: /memory/map_snapshot.txt + track.log full trajectory (W x y lines).
5. Check d4/d9 occasionally - could be pickup/button sensors.
## Final state ep1: left-wall follower ran to end of episode, still discovering
## new cells as far as (3,18),(12,12) NE/N. Goal never seen. Maze > 31x27 units.
## Resume left-wall following FIRST, it had not finished a full loop yet.
## Update: episode continued far beyond estimate. Left-wall + DETACH-every-150s
## explored to x -13..27, y -29..18 (dead-reck). Still goal=0 at tick ~654k.
## wallfollow_left.py now includes DETACH logic (crosses open space to hop walls).

# FINAL STATE EP1 (powered down at tick ~720k, goal NEVER reached)
- Explored (dead-reck units): x -20..28, y -30..18. See map_snapshot.txt.
- Total ~110 min of driving. Right-wall 30min, left-wall+DETACH rest.
- GOAL FLAG NEVER FIRED. RECONSIDER ASSUMPTIONS NEXT TIME:
  1. Maybe goal needs something OTHER than location: try dwelling/stopping in
     distinctive spots (large rooms/dead-ends), driving over specific features.
  2. Transceiver: d5 emitted ONLY empty lines all episode; d3 tx of many strings
     got no reply. Maybe beacon has short range - log d5 nonstop while exploring
     (controller does poll it every 25s; consider every loop).
  3. d4/d9 stayed 0 always - watch them during exploration; they may flip near
     something interesting (they're read-streams like d2).
  4. Dead reckoning drifts; consider building real occupancy map w/ scan matching,
     frontier exploration instead of wall-following (coverage was slow).
  5. Consider systematically sweeping the interior (wall-following only hugs
     walls; goal may be mid-corridor intersection or mid-room).
- Controller quality: wallfollow_left.py ran hours without crashing; FROZEN
  recovery + DETACH work. Speeds: drive(38,0) cruise, blocked<0.38 turn-in-place.
