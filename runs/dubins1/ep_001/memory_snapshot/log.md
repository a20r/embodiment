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
