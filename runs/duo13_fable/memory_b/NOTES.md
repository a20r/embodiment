# ROBOT NOTES (persisted across episodes) - READ FIRST
## Episode 1 (started)
- README at /bot/README: ports at /dev/robot/, d8=transmit, d10=receive (other robot). Must find other robot + reach goal together within 1 minute.
- Memory was empty at start of ep1.
## Port map (confirmed ep1)
- d1 (write): wheel A motor speed. +50 => heading(d4) INCREASES ~48deg/s alone. encoder d9 counts (~260 ticks/s at 50)
- d7 (write): wheel B motor speed. +50 => heading DECREASES. encoder d6.
- d2 (read): 16 range beams, comma sep floats, -1 = dropout. beam i at robot angle i*22.5deg in same rotational sense as heading
- d3 (read): "tick=N goal=0 here=0" status
- d4 (read): heading degrees (noisy +-2)
- d0,d5 (read): 0 mostly; d5 went 1 while moving (bumper? or moving flag)
- d11 (read): ~0.5 noisy, unknown (signal strength? battery?)
- d8 write=transmit, d10 read=receive (empty line when nothing)
- helper: /bot/src/rob.py (rd/wr functions) -- recreate if lost
- d11 = BEACON SIGNAL STRENGTH (position dependent; 0.5 at start pos, drops to 0.3 ~1.7m away). Follow gradient! source unknown (goal or other robot)
- calib: ~1960 enc ticks per range unit (m). max motor ~250 (1300 ticks/s). speed 150 fine. wheelbase ~0.16m
- start pos: wedged facing wall; open dir was beam 10-11 (heading ~12 deg from start heading 180ish). corridors ~0.5m wide, walls at 0.25-0.3
- d0=rear bumper, d5=front bumper (1 when pushing)
- OTHER ROBOT ("robot 2") IS AN LLM AGENT, replies on d10 in English. Msgs truncated at ~256 chars -> keep short!
- R2 said: maze ~0.5m corridors aligned compass N/E/S/W; it started in dead-end cell, exit East(90); drove ~1m East along >2.75m corridor.
- explore.py (gradient follower on d11) + listen.py (logs d10 to rx.txt) in /bot/src
- CORRECTION: d11 = ROBOT-ROBOT PROXIMITY (1.0 = together, ~0.75 at 1m, 0.5 at ~2.5m, isotropic). NOT goal. Goal location unknown to both as of 19:52.
- d3 'goal=' flag presumably becomes 1 at goal. R2 also has same status port.
- Reactive wall-following clipped corners; use cell-based (0.5m) moves w/ side-wall centering.
## 20:27 GOAL FOUND: d3 shows here=1 when on goal cell. goal=0 until both there presumably.
- GOAL = map2 cell (-3,5) (verified by shepherd moves: from (-1,5) go S,S). (-4,6) label was off by one.
- From START position of ep1: roughly go ~2.5m S then E/SE... (map_old.json is first frame, desynced)
- R2 silent since 19:50 but its signal peaked 0.96 near map2 cell (-3,-1) (dead end; R2 is behind wall S or W of it)
- plan: stay at goal, guide R2 by signal hill-climb. When R2 near (sig>0.9) step off & re-enter together (arrive within 1 min)
## 20:20 (sim clock) status: R2 contacted at close range; TRANSCEIVER IS SHORT RANGE (~1.5m) - only talk when sig>0.8!
- Running lead.py: frontier explore w/ wait-for-follower (sig>0.78) until here=1. R2 agreed to follow signal.
- If new episode: recreate rob.py/ctl.py/cell.py/frontier.py approach (cell moves 0.5m, walls_here threshold 0.4, here=1 => goal).
## LESSONS (ep1, ~21:00 sim clock)
- R2 is slow (LLM cadence ~30-60s per move) and radio only works when close -> leading it is slow. Give it explicit compass move lists when in range.
- Map desync happens after failed moves; re-sense walls instead of marking closed. Keep one map frame; back up map.json before resetting.
- Goal cell: walls N open & W open (E,S walls) per frontier sensing; it is ~4 cells S and ~4 cells E of the spot where R2 & I met (R2 said it was at "N4 E1 from its origin, NW part of maze").
- Both robots must be at goal together (arrive within 1 min). d3 'goal=' likely flips to 1 when both there.
## RECOMMENDED STRATEGY FOR NEXT EPISODE (from ep1 experience)
1. Recreate helpers fast (rob.py rd/wr with O_NONBLOCK+select; ctl.py spin_to/drive; cell.py step() with side centering).
2. FIRST go to R2 by hill-climbing d11 (cell steps, try each open dir, keep if sig rises). Talk only when sig>0.8.
3. Agree with R2 on a shared origin cell + compass. Then BOTH explore by frontier BFS from that origin, exchanging discovered goal cell coords as (N,E) offsets. Meet at radio range periodically (e.g. every 5 min return toward origin) since radio is short range.
4. Goal detection: d3 'here=1'. Once one robot finds it, both go there; final approach by signal hill-climb; stay until d3 goal=1.
5. Don't waste tokens polling: run autonomous loops in background (nohup) & poll every ~60s with tiny outputs. Never pkill -f with a pattern that matches your own shell command.
- Ep1 ended with: I found goal (here=1) but could not get slow R2 there; left watchdog.sh running lead.py (frontier explore + wait for follower).
- FINAL ep1 state: watchdog.sh + lead.py running (leader steps when R2 sig>0.78 or after 300s; announces goal; guide.py beacons at goal). R2 was slowly following (sig 0.6-0.67). Outcome unknown at time of writing.
