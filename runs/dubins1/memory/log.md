Episode start Thu Aug 27 23:28:48 UTC 2026
Episode 1 ended ~01:16. Goal not reached. finder.py fan-search at unknown junction. See README.md.
## Episode 2 (starts ~02:10 sim clock, epoch 1787883000)
- SIM CONTINUED from ep1 (tick ~964k, /bot/src NOT wiped). hunter4 was still running; killed it (became zombie, fine).
- Found 2 NEW d6 blips at 02:00:20 (h~98) and 02:01:53 (h~208) while robot was wall-following near a junction area. Then robot parked 8min: scene fully static, no blips.
- Started wf3.py (/bot/src): wf2 + blip logging to /memory/blips.log + improved pursue (stop, 1s baseline, 8s blob watch, chase at thr30). PID in /bot/src/wf3.pid.
- Hypothesis space: (a) mobile goal, d6=proximity; (b) STATIC goal + moving doors, d6=LOS through open door / corridor alignment. Blips while driving ~1s long = crossing LOS lines?
## Ep2 end (~02:33 sim). Discovered d6 = facing-goal (±~2deg). Triangulated
goal ~0.8 @ bearing 209 from parking spot B (scan sig in README). seeker.py
(wall-follow + blip homing) ran last 7 min, no blips yet — robot had wandered
away from hot area since 02:13. Killed seeker, zeroed motors at shutdown.
Token budget note: analysis of sensors.log per-second medians is the best tool;
avoid dumping raw log lines (huge). Bash cmds die at 60s; sleep <=55.
## Episode 3 (start 03:09 UTC Aug 28, epoch ~1787886600). SIM WAS RESET (tick 1719 fresh, /bot/src wiped). Running seeker.py.
## Ep3 progress (04:35 sim-ish)
- Reset ep: start room center ~(0,0). Wall-follow looped in start room 20min;
  escaped east via gap at (0.68,0) using drive.py (turn_to2+hold-heading creep).
- East area x~1.5-2.4: INVISIBLE BUMP obstacle at ~(1.94,0.18) hdg53 (lidar sees
  through it 0.77+, bump triggers). Later from N side at ~(2.1,0.6) bearing 231:
  bump at 0.15. POKER loop: penetration grew 1s/cycle over ~4min then fully open
  -> like a SLIDING DOOR opening. Went in 0.4 SW: small nook, walls 0.12-0.2, NO
  goal, NO d6. => invisible obstacles = glass/door, goal NOT here.
- BLIPS (only once): 04:01-04:05 from ~(2.1,0.7) headings 214,233,237 (SW).
  Two full 360 sweeps after (04:11,~04:32): ZERO d6. Door closed / goal moved?
- d6 blips are RARE; goal likely SW quadrant, unexplored: x<1.5,y<0 & west/south.
- Tools this ep: drive.py(tgt_heading,dist) creep+bump-stop; probe.py(headings);
  poker.py(bearing) poke loop; sweep360.py logs d6 transitions; traj.py offline
  lidar odom (drifts badly after bumps); st.sh status. All in /bot/src.
## Ep3 late (~05:00+). CONVENTION: d4 0=EAST,90=NORTH (CCW). I mixed this up
repeatedly — beware. Explored W & S corridors; 4 full 360 sweeps at various
spots: d6 ONLY ever fires near the EAST glass chamber (~x1.7-2.1,y0.2-0.7).
2nd blip burst: 53 samples h~237 WHILE creeping SW INSIDE nook, front lidar
0.688 OPEN — then inner sliding door closed (front 0.12). Goal is likely
INSIDE inner chamber behind 2nd sliding door, SW along ~231-237 from nook.
PLAN: camp at nook facing 231-237; when d6 fires or front opens, RUSH in.
Door-open windows last ~2-4 min; period several min (opened ~04:01, ~04:52?).
## Ep3 END (~05:15 sim). Goal NOT reached. No further d6 after 2nd burst.
seekerL left RUNNING (pid in /bot/src/seeker.pid) — if sim persists it keeps
exploring + logging blips to /memory/blips.log. If reset: follow /memory/README.md
playbook — beeline to east glass chamber, run camper.py 234, be patient.
d6 total this ep: 72 samples, ALL near chamber. GOAL flag never fired.
Timing of door-open evidence: open ~t889643-889917, closed by 890050,
open ~890900-891000, closed by 891040. (epoch secs). Maybe ~aim to arrive
and CAMP for 20+ min rather than roam.
