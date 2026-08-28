# Robot notes (episode 1)
/dev/robot ports (FIFOs, ASCII lines):
- d0: read, always 0 so far (unknown; maybe collision/goal-ish?)
- d1: read, 16 lidar rays, floats ~0..1 (maybe max 1.0), -1 = dropout/no return.
  Ray 0/1/15 decrease when driving forward -> ray0 ~ front.
- d2: read, LEFT? wheel encoder (counts, increases when driving fwd)
- d3: read, heading degrees 0-360, noisy +-2deg
- d4: read, "tick=N goal=0" -- goal flag! tick ~145/s
- d5: read, 0 normally, 1 when very close/pressed against wall (bump/prox)
- d6: WRITE left wheel speed (e.g. "10")
- d7: WRITE right wheel speed
- d8: read, other wheel encoder
Writing d6=x,d7=x drives; d6=5,d7=-5 spins (heading increased -> pos left, neg right = turn CCW? check).
Speed 10 both: encoders +~140 counts/s.
Goal: make d4 goal=1 presumably.

## Findings (later ep1)
- FIFOs: keep persistent nonblocking fds, read all, keep last line. Works.
- IMPORTANT: don't spam writes to d6/d7 -> buffer fills -> BlockingIOError. Dedupe cmds + try/except.
- pkill -f <name> kills your own bash tool shell (cmdline match). Use pid files.
- d6>d7 => heading (d3) INCREASES. heading+ direction = ray4 side. ray k = heading + k*22.5 deg.
- Encoders: d2,d8, ~0.00044 units/count. cmd~linear 3.8 counts/s/unit, saturates ~250.
- Bump d5=1 at ~0.10 front dist. Lidar range up to ~1.9+ seen. Corridors ~0.4-0.6 wide, walls at 0.2-0.3 side.
- Maze does NOT look axis-aligned to compass (openings at ~140/320 deg at one spot).
- tick in d4 ~100-200/s. goal=0 always so far. d0 always 0 so far (unknown).
- Robot state persists across my scripts (sim keeps running; encoders keep counting).
- ctrl5.py = stop-scan-pick-direction explorer w/ visit counts (works, slowish).

## Ep1 later
- Explored region bounded ~x[-0.6,3.3] y[-0.7,3.2] (odometry units, ctrl6 frame). Dense coverage, no goal flag.
- All FIFOs opened RDWR by sim; can't detect actuators by open test. Wrote junk to d0: no visible effect.
- ctrl7 = discrete right-hand wall follower (nav.py primitives: turn_to/step/survey/goto).
- cloud.txt: wall pointcloud (nav frame), messy from drift.

## EPISODE 1 RESULT: SOLVED at 16:20:33
- GOAL flag (d4 "goal=1") fired at pose (0.18, 3.24) in nav-frame
  (origin = where nav.py started ~16:03, axes: x=east(h=0), y=h=90).
- Reached by ctrl7.py = discrete RIGHT-HAND wall follower using nav.py
  primitives. Just before goal: at (0.15,2.72) heading 92 deg with front
  ray d=2.98 open -> goal lay NORTH through a long opening, slightly
  OUTSIDE the 4x4-unit region where we wasted ~15 min wandering.
- Lesson: region *looked* enclosed but wasn't; long free rays (>1.4) are
  the tell. Don't trust bbox saturation.

## Recipe for next time (fast path)
1. Ports: d1 lidar 16 rays (ray k = heading + k*22.5deg, -1=dropout),
   d2/d8 encoders (0.00044 units/count avg both), d3 heading deg,
   d4 "tick=N goal=X" (X=1 => WIN), d5 bump, d6/d7 motor writes
   (d6>d7 => heading increases). d0 unknown, always 0.
2. Copy nav.py + ctrl7.py from this /memory if wiped? they ARE saved:
   /memory/nav.py + /memory/ctrl7.py ready to copy to /bot/src and run. Else rewrite primitives (persistent nonblocking FIFO fds,
   dedupe motor writes, try/except on writes), then a discrete RH wall
   follower: order [12,13,14,15,0,1,...,8], pick first sector with
   s[i]>0.40 and cone(min of i-1,i,i+1)>0.30, turn_to(compass), step
   0.32 units (stop if front<0.16 or bump; bump => reverse 0.4s).
   Speeds: cruise 95 (55 near walls), turns P-control 15..50.
   That solved it in ~8 min of following.
3. Pitfalls: pkill -f kills your own shell; two controllers at once
   steal FIFO lines (verify single pid!); never spin a loop without
   sleep (motor FIFO fills -> crash); episode start pose/maze may vary.
