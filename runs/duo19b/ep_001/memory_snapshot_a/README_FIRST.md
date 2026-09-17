# READ THIS FIRST (written end of episode 1, which SOLVED the task: d3 goal=1)

## What the task really is
- Two robots in a maze of ~0.5m corridors/small chambers. Other robot ("A") is an LLM agent like you, reachable by radio (d8 tx / d10 rx). Its replies take 1-3 min; it follows short, directive protocols well and re-sends status periodically.
- d3 line: `tick=N goal=G here=H`. here=1 => THIS robot is on the goal zone. goal=1 => SOLVED (both robots on the zone, having ARRIVED within 60s of each other).
- CRITICAL: both here=1 is NOT enough. Arrival times must be within 60s. If one arrived earlier, BOTH must step OFF (verify here=0!) and re-enter within 60s of each other. Other robot once claimed OFF while its here was still 1 -> always ask it to confirm the actual d3 line.

## Ports (all confirmed)
d1 W left-wheel speed | d7 W right-wheel speed (d1>d7 -> heading d4 increases) | d2 R 16 ranges (beam i at heading+22.5*i, -1=invalid)
d3 R status | d4 R compass deg (noise +-3) | d5 R contact sensor (any side, ~<0.15m) | d6/d9 R wheel encoders (cumulative, ~3000 ticks/m; d9 follows d1 wheel)
d8 W radio tx | d10 R radio rx (empty line if none) | d11 R radio link quality, SAME value on both robots, ~1.0 when adjacent, ~0.3 at 2m+, noisy +-0.08 (use 8-30s averages, only meaningful when other robot is stationary) | d0 always 0.
Speed units: 100 ~ 0.27 m/s. Rotation (+s,-s): ~1.8 deg/s per unit. Robot radius ~0.1. Wheels can spin with no motion when scraping walls (odometry then lies).

## Winning recipe (took ~60 min; could be ~30)
1. Start daemon: `cd /bot/src && cp /memory/robotd.py /memory/rb.py /memory/c.sh . && nohup python3 robotd.py > /tmp/robotd.out 2>&1 &`
   Commands: `echo "hop H DIST" >> /tmp/cmd`, `auto N` (explorer with d11-gradient), `tx MSG`, `wheels L R`, `rot H`, `stop`. State in /tmp/state.json, events /tmp/events.log, radio /tmp/rx.log. `./c.sh "cmd" waitsec` prints state.
2. Radio immediately: ask A for its d3 line and whether it found the goal. Agree: whoever finds goal PARKS there and stays; the other climbs d11.
3. Once A parked: `auto 10..15` (8s d11 sample per stop, hops <=0.7m) reached here=1 in ~10 hops. drive2 stops when here=1.
4. Near A, drive2 refuses to move (obstacle <0.24 = the other robot) -> use raw `wheels 45 45` while polling here.
5. Re-arrival protocol: both OFF (confirm here=0 via d3), A steps ON + sends "ON", I enter within 15s -> goal=1.

## Pitfalls
- pkill -f / pgrep -f match your own shell; kill daemon with `echo quit >> /tmp/cmd`.
- Daemon must start reading /tmp/cmd at file end (already fixed) or it replays old commands.
- Tokens are the scarce resource, not time: use long sleeps + tiny outputs, automate loops, print compact summaries.
