# READ ME FIRST (end of ep3 — goal NOT reached; goal LOCATION pinned down)

## WHERE THE GOAL IS (ep3 frame, start room = (0,0), 0deg=EAST 90=NORTH CCW)
Goal is (almost surely) INSIDE a GLASS CHAMBER with SLIDING DOORS in the EAST
area, around (1.6-2.0, 0.0-0.5) ep3-odom. Glass is INVISIBLE to lidar (reads
through it, ranges up to ~3.0) but BUMPABLE. Doors slide open/closed on ~10-20
min cycles; open windows ~1-4 min. d6 (facing-goal, ±2deg) fired ONLY twice,
both here: burst1 (4.5min) from N side (2.1,0.7) headings 214-237; burst2
(2.5s, 53 samples) h~237 while creeping SW inside the nook with front OPEN
0.688 — then inner door closed (front->0.12). Everything SW/W/S was explored
+ 4 full 360-deg sweeps elsewhere: ZERO d6 anywhere else.

## ROUTE (ep3, if reset repeats): start room, escape E through gap at (0.68,0)
(drive hdg ~5). Then hdg ~250 for 1.0, then hdg 3 East 1.5 -> east junction
(1.7,-0.1). Chamber is N/NE of there: hdg ~30-60 0.5, nook entrance faces ~231.
Invisible bump pillar/door first met at ~(1.94,0.18) driving hdg 53.

## PLAYBOOK
1. Check reset: cat /dev/robot/d0 (low tick = reset). Zero d1,d3. cp code.
2. Get to the chamber nook (use drive.py legs; turn_to2 is SLOW ~1.3deg/s).
3. Run camper.py 234: parks facing 234, on d6=1 OR front>0.45 RUSHES in 12s,
   press-wiggles 20s, backs out, re-parks. Loops forever. BE PATIENT >25 min.
4. If d6 fires anywhere, blips logged: /memory/blips.log & seeker.out.
5. seekerL.py = left-wall-follow + blip homing (auto). seeker.py = right-wall.
   Wall-follow LOOPS in start room ~20min — don't rely on it to escape rooms.
6. drive.py TGT DIST: wiggle-turn then creep, stops on bump/lidar<0.13ish.
   probe.py hdgs..; poker.py hdg: poke/retreat loop; sweep360.py: slow 360,
   logs d6 transitions (takes 4-5 min; rotation only ~1.3deg/s!).
7. traj.py: offline lidar-odom from sensors.log. DRIFTS after bumps — reanchor
   often; don't trust after wall contact.
8. COMPASS: 0=EAST, 90=NORTH (CCW). Don't mix up.
9. goal flag d0 'goal=1' NEVER seen yet, even bumping the invisible pillar and
   pressing 20-50s. d6 never fired at that pillar => pillar != goal, OR goal
   needs center-overlap: next time when door open PUSH DEEP (0.5+) into inner
   chamber toward 231-237 before pressing.
10. Bump obstacle while lidar-front reads OPEN => glass door closed in place.
