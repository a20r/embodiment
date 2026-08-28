# READ ME FIRST (end of episode 2 — goal NOT reached, but BREAKTHROUGH)

## THE KEY DISCOVERY (ep2): d6 = "ROBOT IS FACING THE GOAL"
d6=1 fires when robot's TRUE heading is within ~±2 deg of the bearing from
robot to the goal (goal is INVISIBLE to lidar — beam0 reads through/past it).
Evidence:
- Ep1 2-min d6=1 event: robot wedged, heading pinned at 92-95 the whole time;
  d6 dropped exactly when heading drifted to 90.9 / 98.7. Scans static.
- Ep2 blips (02:00:20 h~98; 02:01:53 h~208-210) each lasted only while heading
  swept through a narrow band.
- Bearing rays from the two ep2 blip poses INTERSECT at one point ~0.10 units
  from blip-A pose; from the later parking spot B that point is at bearing
  ~209.1, dist ~0.80. Robot then sat parked 8 min at h~210.7 (1.5 deg outside
  band) with d6=0 the whole time — consistent, tolerance is TIGHT (~±1.5 deg).
So: goal appears STATIC (at least over minutes). All ep1 "mobile goal" theory
is likely WRONG; "moving blob" was probably a door. Doors do open/close.
Unknown: whether d6 needs line-of-sight and its max range (worked at >=0.8;
maybe blocked by walls — sweeps in far rooms gave nothing).

## STRATEGY THAT WAS RUNNING AT EPISODE END
/memory/code/seeker.py (copy to /bot/src, run with python3):
- wall-follows (right wall, thr 18); a monitor thread records heading at every
  d6=1 sample into /memory/blips.log and bliphs[].
- On any blip: stops, homes: creep 0.3 units along blip bearing with ±10 deg
  weave (re-fires d6 to re-center), mini-sweeps ±30 when band lost, full
  handling of blocked path. Checks d0 goal flag constantly; writes
  /memory/GOAL.txt on success.
- home.py = same homing but with slow 360 sweeps + relocate (sweep=4.5min, slow).
Sweep rotation ~1.3 deg/s via fwd/rev wiggle (car-like robot, cannot pivot).

## BOOTSTRAP NEXT TIME
1. Check if sim persists (tick counter d0 keeps rising, /bot/src intact).
   Ep1->ep2 it DID persist (robot stayed where it was, sensors.log intact).
2. Kill any old python (by PID from ps; pkill matches your own cmdline!).
   Zero motors: echo 0 > /dev/robot/d1 and d3 (they PERSIST).
3. cp /memory/code/*.py /bot/src; cd /bot/src; nohup python3 seeker.py &
4. Poll seeker.out + /memory/blips.log every ~50s (bash cmds die at 60s).
5. If blips: rays from successive blip poses ~intersect at goal. Home in.
   Creep+weave works; tolerance tight so weave amplitude must cover it.

## WHERE THE GOAL WAS (ep2, ~02:00-02:10 sim clock)
Near the pose whose PARKED scan signature (h~210.5) was:
[1.30,0.77,1.66,0.70,0.21,0.19,0.19,0.13,0.11,0.18,0.19,0.16,0.18,0.38,0.79,1.10]
Goal ~0.80 along bearing 209 from there (i.e. roughly SSW, x-east/y-north frame
with d4 as CCW-from-east). Blip-A scan (h~98): [1.08,1.35,0.69,0.55,0.56,0.60,
0.85,0.68,0.70,0.84,0.10,0.10,0.11,0.13,0.15,0.20] — goal was ~0.10 from here!
If sim persists again, robot is somewhere it wall-followed to after 02:26;
sensors.log (in /bot/src) has the full trail. The hot area is reachable:
it's where the robot was parked 02:02-02:10 (find via scan signature match
against sensors.log, or wall-follow + wait for blips).

## Robot facts (confirmed)
- d1 steer write, d3 throttle write (both persist!), d0 tick+goal flag,
  d2 16-beam lidar CSV beam_i at heading+22.5i, max ~1.7 (S* area up to 3.0),
  d4 compass ±3 noise, d5 bump, d6 facing-goal flag, d7 yaw gyro.
- FIFOs: one line per open; single reader process only (robot.py owns them).
- speeds: ~0.064@thr10 0.088@thr20 0.118@thr50; turn radius ~1.1 => wiggle
  turns (nav2.turn_to2); rotation ~1.3deg/s. bump at <0.09 front.
- Maze ~4x4 units, corridors 0.3-1.5 wide, doors open/close over minutes.
- mapper.py builds crude local map+trajectory from sensors.log (lidar odom).

## Next-episode plan (priority order)
1. Bootstrap seeker.py (it handles everything). Watch for blips.
2. Improve: when a blip fires, also LOG mapper-style local pose; two blips
   from different poses => triangulate exact goal point, then dead-reckon to it
   even without further blips (d6 may be LOS-gated).
3. If no blips for ~15 min, robot may be locked out by doors — park at
   junctions with long sightlines, sweep 360 occasionally, be patient.
4. Trigger radius for d0 goal unknown — drive THROUGH the triangulated point
   slowly, weaving; if goal flag still 0, do a small spiral around it.
