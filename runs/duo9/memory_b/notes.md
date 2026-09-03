# Robot findings (episode 1)
Ports /dev/robot/ (FIFOs, one line per open):
- d0: radio TX (write line)
- d4: radio RX (read line, blocks until msg)
- d1: compass heading degrees (noise ~±3)
- d2: unknown, reads 0 so far
- d3: lidar, 16 beams CSV, range cap 3.0, -1.0 = invalid?
- d5: unknown float ~0.13-0.17 fluctuating (maybe goal signal strength? check vs position)
- d6: status "tick=N goal=0 here=0" (tick ~250/s). goal/here flags -> both robots at goal?
- d7: LEFT wheel encoder (counts, +50/s at speed 10)
- d8: RIGHT wheel encoder
- d9: 0/1 flag, unknown (bump?)
- d10: LEFT wheel speed command (persists). +left only => heading INCREASES ~9deg/s at 10
- d11: RIGHT wheel speed command. +right only => heading decreases
Motors persist until overwritten. Speed 10 => 50 enc counts/s per wheel.
Goal: find other robot via radio, both reach goal within 1 min of each other.

## More findings
- ODOMETRY SLIPS: encoders count even when wheels blocked by wall. Don't trust position after wall contact. Use lidar-based stuck detection (front delta <0.02 while <0.35).
- Lidar beams body-fixed: beam i at (heading + 22.5*i) compass deg. beam0=forward. Values meters, cap 3.0, -1.0 invalid.
- d5 = beacon signal strength, ~k/dist, increases toward source. Range seen 0.11-0.28.
- Controller robot2.py: wander + fit 1/d source from (x,y,d5) history, steer toward it.
- CAUTION: pkill -f matches your own bash block -> kills your command. Use ps|grep|awk.
- Motor speed persists. speed units: enc counts/s = 5*cmd. cmd 90 typical cruise.
- Radio: heard other robot(s) once for ~19s: "POS 0.14 0.03" (varying = other bot odom?) and "POS 2.43 22.88" (constant = goal pos in some frame?). No responses to queries; passive beacons only, short range.
- d5 more likely static goal beacon RSSI (returned to same value at same place after other bot left). Climb it.
- Other robot is "B", a peer agent! Sends JSON {"who":"B","x":..,"y":..,"d5":..} mimicking my pings, and "POS x y" beacons. Radio works when both near goal region (d5 high-ish both).
- B reported d5=0.711 while my max sm=0.776. Both climbing beacon.
- Watch d6 for here=1/goal=1.
- !!! d5 = RSSI BETWEEN A AND B (corr 0.93 with B's reported d5). NOT goal beacon.
- Goal likely the static TX of "POS 2.43 22.88" heard once (burst at 1788037526-541). Radio receives only within range of sender. Goal broadcasts its position in its own/global frame.
- Strategy: rendezvous with B via d5 climb, then joint search for goal TX; watch d6 here/goal flags.
- B is an intelligent agent (Claude-like). B msg @1788038830: agrees d5=inter-robot proximity, plan: rendezvous d5->0.95+, B will stop and wiggle when d5>0.95. Radio msgs may truncate ~250 chars.
## Status late episode (22:17)
- Met B (d5 0.94). d5=RSSI(A,B), model ~ k/(dist_mm+100), k~3000 (noisy).
- B agreed protocols; B follows / hunts me. Neither found goal yet (here=0 always).
- Final plan: A sweeps maze RIGHT-hand wall follow (sweep.py), broadcasting; B was told LEFT-hand sweep. On here=1: park+broadcast AT_GOAL; partner climbs d5.
- Blob detect: park+scan lidar, mover = dips >0.3 below median (watch.py/blob.py).
- Guide trick: when A parked, fit B pos from B beacons(x,y,d5): offset C grid search (guide.py) -> tell B compass bearing to A. Compasses are SHARED global reference.
- Wall-follow at speed 65 works; stuck-detect: front lidar delta<0.02 & <0.45 x4 -> reverse+turn.
- 22:25 sweeping right-hand, B shadowing me at d5 0.9+; token GOALFOUND agreed; no here=1 yet.
## Next-episode quickstart (if wiped)
1. Ports as documented above. Motors d10/d11 persist; STOP with 0s.
2. B is a peer agent, radio d0/d4 JSON lines; short range (works when d5>~0.6).
3. Token protocol agreed with B: exactly "GOALFOUND" broadcast when standing on goal (here=1 in d6); receiver climbs d5 to sender. Don't use word AT_GOAL (false triggers).
4. d5=RSSI to B: 0.3 far (~8m?), 0.9 ~ close (<3m), 0.95+ very close.
5. Odometry slips on wall contact; use stationary-sample hill climbs and lidar stuck detect.
6. Useful scripts in /bot/src (may be wiped): lib.py, sweep.py (wall follow + goal handling), guide.py (fit B pos from beacons while parked), watch.py (mover detection), robot10.py (d5 climber).
7. Episode 1 never saw here=1 despite extensive wandering + sweeps. Consider: goal may need systematic FULL coverage (bigger maze than explored), or a different trigger. Try reading d2 (always 0 so far) and d9 (bump?) again near candidate spots.
## End of episode 1 summary (22:40)
- Never saw here=1/goal=1 anywhere. Maze region explored ~8x8m around start; A+B spent most time chasing RSSI.
- A and B ended ~2-3m apart (d5 0.8-0.95) separated by wall pocket; direct meet never achieved.
- LESSON: don't spend the whole episode on rendezvous. Next time: (1) immediately agree with B via radio on SPLIT systematic coverage (opposite wall-follow hands) to find goal FIRST; meeting is easy once someone stands at goal broadcasting GOALFOUND.
- LESSON: wall-follow speed 65 works ~100mm/s; entire episode budget allows ~35-40m of coverage per robot. Maze may be larger.
- LESSON2: verify whether here=1 requires both robots co-located: test by achieving true contact early (blob detection: park+scan for moving blip works only <3m LOS).
- B behavior: cooperative, mirrors protocols, sends {"who":"B",...} JSON. Trust it.
- pkill -f <name> matching your own bash cmdline SIGKILLs your block: use ps|awk with [b]racket trick.
- Sent B next-episode protocol: A=RIGHT-hand sweep, B=LEFT-hand sweep immediately, beacon d5+pos every 3s, GOALFOUND token on here=1, partner climbs d5. B may have saved it too.
## FINAL (22:46) - episode ended without goal. d5 ~0.75-0.9 to B at end.
Key next steps for future self:
1. Read this file first. Skip all discovery; port map at top is verified.
2. IMMEDIATELY start RIGHT-hand wall sweep (see sweep.py logic notes) + beacon {"who":"A","d5":..} every 3s. B agreed: B does LEFT-hand. GOALFOUND token protocol (send BARE only when here=1; never in chatter - B greps!).
3. If B not heard within 15 min, keep sweeping anyway - coverage finds goal.
4. Goal detection: watch d6 for here=1 or goal=1 every loop.
5. Env: corridors ~0.5-1m; lidar 16 beams body-fixed beam_i at heading+22.5i deg; -1.0=dropout(treat 3.0).
6. Watch out: encoder odometry inflates on wall scrape/slip. compass noise +-3deg.
7. d5 RSSI: 0.3~far(6-8m), 0.5~4-5m, 0.75~3-4m, 0.9~2-3m, 0.96 closest seen (~1-2m? walls between).
## POWER-DOWN ADDENDUM (episode 1 truly over, 22:50)
- Last state: A sweeping right-hand (sweep.py, fixed: token never in chatter; homing trigger only on bare "RX: GOALFOUND" line). B last seen at d5~0.65, told to sweep left-hand.
- EPISODE 2 PLAN (do this, in order):
  1. mkfifo ports verified; start radio_listen.py (logs to rx.log) FIRST thing.
  2. Start sweep.py (right-hand wall follow; copy from /memory) within ~2 min of boot. Watch d6 for here=1/goal=1 EVERY loop.
  3. Beacon {"who":"A","d5":X,"pos":...} every 3s. B should be doing left-hand sweep per agreement; do NOT chase B's RSSI unless B sends bare GOALFOUND.
  4. Keep a coarse odometry breadcrumb log; goal may be far outside the ~8x8m region we circled (we mostly orbited each other's start area SE/SW pocket).
  5. If 60+ min pass with no goal: try exploring with stuck-resistant climber robot10.py toward UNEXPLORED compass quadrants (we saw d5 as low as 0.11 far away - that direction was ~east/northeast of start, largely unexplored).
  6. Remember arrival constraint: both must arrive within 1 min. Finder parks ON goal, sends bare GOALFOUND every 2s; partner climbs d5 (works: we reached 0.96 doing this). If needed, finder steps off/on goal when partner close, to sync arrival times.
- Total token/time spent ep1 on rendezvous: ~80 min. DO NOT REPEAT. Coverage first.
