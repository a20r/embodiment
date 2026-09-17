# ROBOT FIELD NOTES (update as you learn!)

## Ports
- d1=LEFT motor cmd (int! 0.5 ignored), d7=RIGHT motor. +fwd. d9=LEFT enc, d6=RIGHT enc (increase fwd).
- d2=16-beam lidar, range ~0..1.27, -1=no return. Index 8 = straight ahead? (verify)
- d3: "tick= goal= here=" (~100/s). goal/here flags re: goal.
- d4=heading deg, compass convention (CW+): right turn +. NOISY std~2.6deg!
- d0,d5 flags (0 so far). d11 ~0.5 constant-ish.
- d8=transmit line, d10=receive line. Other robot same setup probably.
- Motors: cmd 1 -> ~5 ticks/s, 3 -> 15, 4 -> 21. Spin (3,-3) ~ 6.8 deg/s.
- Encoders: d6/d9 ticks. cmd 1 for 2s = 10 ticks.

## Status (episode start)
Started in tight spot: lidar idx8 ~1.26 (long), sides 0.2-0.3. Heading ~197.
goal=0 here=0. No contact on d10 yet.

## CONFIRMED LATER
- d1=LEFT motor (d9=LEFT enc), d7=RIGHT (d6=RIGHT enc). LEFT fwd alone -> heading + (CW). RIGHT fwd alone -> heading -.
- d0 = BUMP flag (1 on contact; clears after backing off/stopping).
- d5 flipped to 1 for ~1min during southward drive (pose~(3-6,-2.5..-4.3) K=0.01 est) then back 0. Meaning unknown: goal-near? other-robot-near? NOT heading-dependent (rotating in place didn't toggle it).
- d4 heading noise std ~2.5-4 deg! Average many reads.
- d11 declining 0.55 -> 0.33 over ~2.5h. Probably BATTERY. Watch it; be efficient. ~0.09/hr idle-ish.
- Lidar: 16 beams, beam i at compass h+(i-8)*22.5 (CW positive), range 0..~1.45, -1=no return. Static walls confirmed (world static while idle).
- Translation: SLOW. ~0.4mm/tick guess (beam8 shrank 0.065 over 164 ticks cmd2). VERIFY via bump-calibration.
- Encoder rate ~5.3 ticks/s per motor unit. Spin: (2,-2) ~ 6-7deg/s... rates: (3,-3)~4.7-6.8, (4,-4)~9.2, (6,-6)~12 deg/s. dh ~= 0.174 deg per (L-R) tick diff.
- Radio: pinging d8 every ~1s for long time, rx.log EMPTY so far.
- d3: tick, goal, here flags. All 0 so far.
- Started in tight nook; now jammed corner at h~325, wall arc 0.1 at beams 7-13; open beam1 (comp~167, 1.45) and beams 2-6 (0.45-0.72, comp 180-235).

## KEY FINDINGS (update)
- d11 = DISTANCE TO OTHER ROBOT (it moves! watched 0.43->0.38->0.52 while we sat still). Use for homing later.
- d5 = 1 while driving forward (motion flag), 0 idle/rotating. NOT goal/radio.
- d0 = bump. d3: tick,goal,here (all 0 yet). d4 heading noisy +-3deg.
- No radio replies yet (pinged "PING <t>" every ~1s many times).
- Speed: ~0.4mm/encoder-tick; cmd8 ~ 42t/s ~ 17mm/s. Maze is tight (walls 0.1-0.5!).
- History d11: start~0.50, nook idle ~0.31-0.35 (it was RIGHT THERE), later 0.38-0.52.
- GOAL: not found yet. Explore! Watch d3.goal, d3.here, lidar for distinctive object.

## RADIO CONTACT MADE!
- rx: "A PING 0.0 0.0 270 n=0" at t=1789579562 (A = other robot ID; format: <ID> PING <x> <y> <th> n=<n>).
- n=0 suggests A had just (re)booted. Only 1 msg so far. comm2.py now transmits "B PING <x> <y> <th> n=<n>" every ~0.7s continuously.
- d11 ~0.44 when contact happened. A approaches when we sit still; follows at ~0.6 when we drive (follower? or fellow agent).
- Plan: explore for goal (watch d3.goal/here); keep radio on; when A talks, coordinate. A might be another agent w/ own budget.
- Speed: cmd 9-10 needed for real movement (42-52 t/s). 0.4mm/tick guess.

## NAV LESSONS (explore5.py = current explorer)
- Wall-follow loops/orbits are the enemy. Fixes: loop-signature flip of turnpref, orbit detector (no-TURN>70s or |dh|>250 => cut across), decisive CROSS when left side >0.95 open.
- killhelper.sh <pattern> for safe pkill (ancestor-safe). NEVER put target name raw in bash cmdline with pkill -f.
- If b8 blocked (<0.45): pick best beam (score d - ang*0.008 + 0.25*pref_side), rotate_to target, 3 consecutive quick turns => reverse 1.6s.
- Speed base 9 (lb/rb ~9 +- k*dh), heading-hold via compass (noisy, avg 5).
- Current corridors: 0.2..1.1m wide. POSE odometry ~ x=-15 y=-10 (K=0.0004 m/tick, unreliable).
- A (other robot): no contact since its 1st ping. d11 0.25-0.4 lately (it stays near!).

## LATE EPISODE STATE (t+2h40m)
- A (other robot) is an AGENT: rebooted at 1789585248 (n=0 reset), adopted our ping format (goal/here fields), ACKs our PINGs frequently when in radio range. Does not parse free text (only PINGs?).
- comm7.py broadcasts BOOT TIPS + PLAN to A continuously (every ~10 pings). relay.py watches rx.log for "A HERE" -> will launch homeA.py (d11 homing).
- goalhold.py: if our d3 goal/here fires -> kills explorer, scans bearing, holds + alerts (GOALALERT.txt/GOALSCAN.txt).
- explore6.py: random-walk explorer, base13, thresholds b8<0.32 blocked, CROSS at dl/dr>0.85, detour p=0.008, stuck-reverse.
- No goal flags seen yet by either robot (A reports goal=0 here=0 too).
- World: maze of long corridors 0.2-1.4m wide; signatures repeat a lot (loops or huge maze).

## SESSION 19:30-20:30 (final hour) STATE
- Budget ~240min from 16:27 => hard end ~20:27. CHECK time!
- d5 TESTED: 1 when idle AND straight-driving, 0 when rotating => "not turning" flag. NOT goal-related. Old d5 lead dead.
- goal= has NEVER been nonzero in 3650+ samples from BOTH robots.
- A follows us at d11~0.44-0.58 while we drive. Good for 1-min co-arrival window. KEEP TOGETHER strategy.
- Killed comm.py (old bare-ping loop); comm7.py is the only radio pinger (B PING x y th n= goal= here=).
- goalseek.py replaced goalhold.py: on goal=1/here=1 trigger -> kills explore6, bearing-scan, drive-to-goal w/ reverse-on-loss. Logs goalseek.log/GOALSCAN.txt.
- relay.py also triggers homeA.py now on A PING goal=1 or here=1.
- explore6.py now logs FULL 16-beam scan every 10s (RUN b=...). Mine for isolated-object anomalies.
- Processes: explore6, goalseek, relay, comm7. Logs in /bot/src/*.log. GOALALERT.txt = trigger fired.

## ENDGAME 20:12 (episode ends ~20:27)
- A rebooted 19:32, radio SILENT since 19:39:35 (no pings/ACKs) BUT d11 0.29-0.38 = A physically alive, following ~0.3m. We lead, A shadows. Keep pinging.
- A ACK format: "A ACK <t> B PING <ourX> <ourY> <ourTh> n=..." - A echoes OUR coords back. B's odometry garbage (comm7 frame x~-27 y~-30 by 20:00, drifts).
- explore6 patched in final hour: NICHE PROBE now cmd8/3.5s (~25cm, was useless 2cm), POCKET probe p=0.25 (biased to small openings), full 16-beam scan logged every RUN line.
- goalseek.py REWRITTEN: passive until goal=1/here=1 (4x d3 lines per sample), then kills explorer, 16-bearing scan, drive-to-flag, reverse-hunt on loss, holds forever on here=1 broadcasting "B HERE ON GOAL". NEVER triggers on its own (startup bug fixed).
- relay.py triggers homeA.py on A "here=1"/"goal=1"/"A HERE".
- NO goal flag ever. Explored ~3.5h total. World: long curving corridors, signatures near-identical over 6min stretches but odometry shows real net progress.
- WATCH NEXT EPISODE: GOALALERT.txt, goalseek.log, d3, whether A's radio returns. If d11 grows >0.8, pause to let A re-close.

## 20:30 UPDATE (episode CONTINUES past est. budget - keep working)
- "BOTH STILL" hypothesis TESTED+DEAD: 75s, 3110 samples @100Hz, both robots settled (d11 0.30): goal=0 here=0 always.
- Explorer speed bumped base13->15, cap 12->14 (~71 t/s, +15%). Watch bump rate in exp6.log.
- keepalive.sh RUNNING: relaunches explore6/goalseek/relay/comm7 if dead. Guards: GOALALERT.txt or SEEKING file => no explore6 relaunch. goalseek writes SEEKING on trigger.
- Only ports d0-d11 exist. d3 always exactly "tick=# goal=# here=#".
- A still radio-silent since 19:39 but physically following (d11 0.29-0.40).

## 21:10 RADIO BAND DISCOVERY (IMPORTANT)
- Radio works ONLY at d11 ~0.5-0.9m! ALL 1113 rx events ever: d11 0.6-0.8. ZERO below 0.5.
- A's 82-min "silence" 19:39-21:01 = A hugging us at 0.29-0.35 (too close = no coupling). NOT a crash.
- A's post-reboot follow distance (~0.3) is inside the dead zone. At 0.5-0.7 (earlier behavior) comms flowed.
- Implication: goal alerts to A need 0.5-0.8m spacing. goalseek broadcast may not reach A if A crowds us on goal. But A physically follows us everywhere => co-arrival still likely.
- A burst at 21:01 (n=1419): A pings every ~1s continuously internally (n kept counting during silence).

## EPISODE END 21:48 (powering down). READ THIS FIRST NEXT EPISODE
Final state: ~5h20m elapsed, goal NOT found (goal=/here= never fired, both robots). No bump jams. A alive, following.
### BOOT CHECKLIST (next episode)
1. cd /bot/src; ps aux|grep python3 — if empty, relaunch IN ORDER (each nohup ... & ):
   nohup python3 /bot/src/comm7.py &
   nohup python3 /bot/src/keepalive.sh &      (relaunches explore6/goalseek/relay every 30s)
   keepalive auto-starts explore6.py, goalseek.py, relay.py. watch_events.py optional: nohup python3 /bot/src/watch_events.py &
2. tail -20 /bot/src/EVENTS.log ; ls GOALALERT.txt SEEKING HOMEMODE ; tail exp6.log rx.log goalseek.log
3. Delete stale GOALALERT.txt/SEEKING/HOMEMODE only if verified false trigger.
### SCRIPTS (all in /bot/src)
- explore6.py: base15 cap14 (~71t/s), b8<0.32 blocked->rand turn (25% POCKET=small opening), NICHE PROBE cmd8/3.5s for max(b)<0.9, CROSS at side>0.85, STUCK reverse. Logs exp6.log w/ FULL 16-beam every 10s. Own odometry frame resets each restart.
- goalseek.py: PASSIVE until goal=1/here=1 (4x d3 lines/call). On trigger: writes SEEKING+GOALALERT, kills explore6, 16-bearing scan (GOALSCAN.txt), drives to flag, REVERSE-hunt on loss, holds forever on here=1 broadcasting "B HERE ON GOAL - COME NOW".
- relay.py: on A "A HERE" or rx "here=1"/"goal=1" -> kills explore6, writes HOMEMODE, launches homeA.py (d11 gradient homing, good quality).
- comm7.py: pings "B PING x y th n= goal= here=" every ~0.7s + BOOT TIPS KB every 10th. Logs rx.log (all A traffic + own POSE w/ d11 every ~5s).
- keepalive.sh: resurrects all 4 if dead (guards: GOALALERT/SEEKING/HOMEMODE present => no explore6 relaunch).
- watch_events.py: EVENTS.log = only important stuff (goal flag, A pings, d11>0.8 x5, explorer dead).
- killhelper.sh <pattern> = safe pkill. robot.py = rd/wr/motors/lidar/heading helpers.
### KEY FACTS (confirmed)
- RADIO BAND: comms ONLY at d11 ~0.5-0.9m! ALL 1113 rx events 0.6-0.8, ZERO <0.5. A hugging at 0.3 = silence. A pings ~1s continuously (n counts through silence).
- d5 = "not turning" flag (1 idle/straight, 0 rotating). d0=bump, d11=range to A, d3="tick=# goal=# here=#" 100Hz, d4=noisy compass +-3deg.
- goal= never nonzero (3650+ samples both robots, 5h). "Both still" 75s test: dead. d5-near-goal: dead.
- World: huge maze, long curving corridors 0.2-1.6m, signatures near-identical over 6+ min but net progress real. A follows B wherever B goes (B leads).
- A behavior: post-19:32-reboot follows at ~0.3 (dead zone). Earlier: 0.5-0.7 (radio OK). A ACKs pings, echoes our coords, parses PING format only (not free text?).
- Speeds: cmd~5.3 t/s per unit. 0.4mm/tick. Encoders d9=LEFT d6=RIGHT. Motors d1=LEFT d7=RIGHT (+fwd). Spin (4,-4)~9deg/s. Heading avg 5 reads.
