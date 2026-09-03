# ROBOT LOG — READ ME FIRST (episode 1, ~20:20)
## PORTS (/dev/robot/, ASCII lines)
- d1 LEFT wheel speed cmd, d7 RIGHT wheel speed cmd. POSITIVE=forward. Differential drive.
  MUST refresh at >=10-20Hz (setpoint decays). V=20: straight ~0.5 u/s, spin (20,-20)=CCW ~37deg/s.
  (-20,20)=CW spin (sometimes blocked by wall contact). (V,V)=straight, (-V,-V)=reverse.
- d2 = 16-ray lidar CSV. ray k bearing ~ (k-5)*22.5deg (ray 5 = FORWARD). max range ~1.4, -1.000=no return (treat as 1.45).
- d3 = "tick=N goal=G here=H". goal=1 => AT GOAL. here unknown (partner?).
- d4 = compass heading deg, NOISY (sigma 5-8), sometimes read fails.
- d0 = bumper flag (1 = collision, momentary).
- d5 = motion flag. d6/d9 = motor temp/current (d6~d7 wheel, d9~d1 wheel) - NOT odometry.
- d11 = unknown analog ~0.41-0.54 (slow wander; was 0.53 at start, 0.41 now) - maybe goal beacon?
- d8 WRITE = radio tx; d10 READ = radio rx. Short range. NO contact with other robot yet. Pings unanswered.
## STATE
- Robot started in a NOOK/POCKET with narrow exit (corridor opening toward rays 5-9 at start, max 1.3-1.4).
- Explorer v1 + escapes wedged it in a tight pocket (all rays < 1.2). Reverse works; fwd often blocked.
- goal=0, here=0, no goal found yet. Other robot NOT found yet.
## CODE
- /bot/src/nav/main.rs -> /tmp/nav3: gap-chaser w/ escape cycles (has steering bugs; avg quality).
- /bot/src/probe/main.rs -> /tmp/probe: scripted tester. Logs to /tmp/probe.log.
- Build: cd dir && rustc -O main.rs -o /tmp/X. Beware: pkill -f /tmp/nav KILLS YOUR OWN SHELL (use pkill -x nav).
- Rust gotchas: Duration subtraction panics on negative; init timers to Instant::now() and compare .elapsed().
## NEXT EPISODE TODO
1. Read this file. 2. Check tick/goal/here + lidar pattern. 3. ESCAPE pocket: LONG reverse (3-4s) watching rays 3-5 open
   (that is where wall recedes; pocket likely exits backward-ish), then CCW spin, then forward.
4. Once out (any ray > 1.3): explore with rotate-to-gap; track d11 trend (possible goal beacon!).
5. Radio ping every 3s; if RX msg, log & reply. 6. If goal=1: STOP, radio ATGOAL, hold. Both robots must arrive within 1 min.

## ~20:26 ESCAPED THE POCKET
- Escape recipe that worked: cycles of [reverse 3s, CCW spin 1.3s, forward probe 2s].
- Beyond pocket: lidar returns >1.45 (1.65 seen) = OPEN SPACE. Pocket exit was narrow.
- nav3 (gap-chaser) now exploring. front rays ~0.6-0.7 typical in new area.
- REMINDER: both robots must be AT GOAL within 1 min of each other. If goal=1: stop, hold, radio ATGOAL every 2s.

## ~20:33 (near episode end)
- nav3 exploring: front ~0.5-0.65, gentle arcs, periodic stuck-escapes. d4 ~263-285 region.
- goal=0, no radio RX yet. nav3 will stop after 480s.
- Next episode: re-read notes, check d3/d2 immediately, resume exploration (rotate-to-gap works).
- If wedged again: escape recipe = [reverse 3s, CCW spin 1.3s, fwd 2s] cycles.
- Consider d11 as goal beacon: log it vs position changes. Check d0 bumper.

## QUICKSTART FOR NEXT EPISODE (read this first)
1. cat /memory/notes.md (this file).
2. Probe: timeout 2 cat /dev/robot/d3 (tick/goal/here); timeout 2 cat /dev/robot/d2 (lidar).
3. Rebuild/run navigator: cd /bot/src/nav && rustc -O main.rs -o /tmp/nav3 && nohup /tmp/nav3 1200 > /tmp/nav3.out 2>&1 &
   (fix any compile errors; avoid pkill -f patterns matching your own command; Duration subtraction panics).
4. Drive basics: echo 20 > d1; echo 20 > d7 refreshed every 50ms = straight. CCW spin: d1=20,d7=-20.
5. Log events to /tmp/nav_events.log via nav3. Watch for goal=1 and RX messages.

## ~20:55 RADIO BREAKTHROUGH
- R1 (other robot) transmits: "R1 xy=X;Y hdg=H t=T" every ~4 t-units. World frame: 0deg=+x, 90deg=+y (CCW), R1 boots near (-1,1) ~origin, drives straight north hdg~100 at ~3 u/t.
- R1 REBOOTED once (t reset 56->5) - episodes wipe both robots.
- d11 is NOT R1-distance (R1 moved 75u, d11 flat 0.60-0.63). d11 likely thermal/noise. Omnidirectional, no bearing info.
- Radio RX bursts correlate with proximity(?). Homing signal = message arrival rate.
- CRITICAL: write ZEROS to d1/d7 to stop; last setpoint may persist otherwise.
- R1 last seen xy=(-12;75) hdg=101 t=29 (second boot), heading NNW along y-axis line.
- PLAN: drive north (heading ~100) following R1's trail; watch RX rate; ask R1 for goal-xy; get localized.

## ~21:10 MAJOR UPDATE
- R1 streams "R1 xy=X;Y hdg=H t=T". Their path: boot at (-1,1), sprint NORTH hdg~100 to y~160, then later seen (-140,-175) hdg=280 t=59 — FAST rover (3-9 u/t), much faster than me (0.5 u/s at V=20).
- RX works when close-ish (bursts). I receive when they transmit nearby; gaps = out of range. My TX: unconfirmed if heard.
- d11 = likely proximity-to-R1 (peaked 0.85 at their close approach). Hill-climbing d11 WORKS when R1 is slow/stationary (0.59->0.665), fails when they sprint.
- Compass d4: degrees CCW from +x (0=+x/east, 90=+y/north) — matches R1 hdg convention.
- d11 NOT thermal; rises ~0.02-0.04 per 2s leg toward R1. Odometry: d9=+~100 ticks/s at d1=20 (V=20 forward), d6=+~107/s at d7=20.
- Encoders d6/d9 accumulate; big values (-46k, +100k) = cumulative wheel ticks (d9 for d1-wheel, d6 for d7-wheel, both + per positive cmd).
- tick (d3) ~= 103/s wall. goal=0 here=0 always so far.
- Controllers: /tmp/hill = d11-MAX hillclimb w/ 4-probe cycle (~50s/cycle). /tmp/telem.sh logs RX+d11+d4+d9 every 2s.
## NEXT EPISODE PRIORITIES
1. Read notes. 2. Start telem.sh + hill (d11 climb) to rendezvous with R1 when they pause. 3. Radio R1 in BOTH formats:
   plain requests AND mimic format "A1 xy=unk;unk hdg=<d4> d11=<d11> t=<mytick>" every 3s.
4. ASK R1 for "GOAL x;y". If they hold at goal, climb d11 to them, arrive together (1-min rule!).
5. If goal=1: HOLD POSITION + radio ATGOAL burst. If here=1: partner at goal? hold + radio.

## ~21:18 FINAL EPISODE-1 STATUS
- d11 hillclimb: peak d11 seen 0.67-0.70 (from 0.54 start). Gradient noisy ±0.02 but integrates upward over minutes.
- WORKING THEORY: d11 = static local beacon gradient = GOAL PROXIMITY (higher=closer). Hill /tmp/hill is climbing it autonomously. If goal=1 fires -> hill holds + radios ATGOAL.
- R1 (other robot) is a FAST rover (up to 9 u/t) sprinting N then S along x~-70..-140 line; they may find goal first. Their stream: "R1 xy=X;Y hdg=H t=T". No goal info from them.
- Episode ends ~21:25. NEXT EPISODE: (1) read all notes; (2) relaunch /tmp/telem.sh AND /tmp/hill (rebuild if needed: cd /bot/src/hill && rustc -O main.rs -o /tmp/hill); (3) continue d11 climb to peak -> goal=1 -> hold & radio; (4) radio R1 asking GOAL x;y and coordination for 1-min arrival window.
- STOP command = write 0 to d1 AND d7 repeatedly ~1s (setpoints persist!).

## ~21:24 RADIO DIALOGUE ESTABLISHED!!
- R1 IS an intelligent peer: heard my messages, replies in kind: "R1 er=8723 el=14083 hdg=246; goal-XY-UNKNOWN; A1: send-your-xy+goal-status; meet-at-my-xy"
- R1 does NOT know the goal either. They propose meeting at their xy; they repeat messages every ~2s (er/el = their encoders).
- BOTH robots lack localization. My d11-gradient climb = best goal-seeking tool (peak 0.67-0.70 so far).
- NEXT EPISODE: 1) IMMEDIATELY restart telem.sh (RX logger) and hill (d11 climb). 2) Radio dialogue with R1: agree protocol:
  they HOLD still -> I climb d11 to them (d11 rises toward them? UNRESOLVED: d11 tracked something static... test by asking R1 to move/stop while I watch d11!).
  3) KEY EXPERIMENT: ask R1 to hold, watch if d11 responds to THEIR position (settle d11=R1-proximity vs static-beacon).
  4) Share localization ideas: R1 sends er/el (encoder deltas) + hdg each msg; if both drive known patterns we can triangulate via radio RX-range thresholds.
  5) When either finds goal (goal=1): broadcast immediately; both must arrive within 1 min => the finder HOLDS, other races; finder can exit/re-enter to sync.

## ~21:32 SESSION-2 FINDINGS (same session continued per operator)
- d11 gradient CONFIRMED SPATIAL: 0.60->0.74 swings within seconds during area search. Ceiling ~0.74-0.75 seen; goal=0 never fired. Peak region keeps trapping the robot in pockets (walls!).
- climb v2+ (absolute-heading probes, /bot/src/climb/main.rs -> /tmp/climb6): probes 4x90deg abs headings, out-back legs, commit best, escape-when-pinned, BLOCKED detection (front rays 3..=8 < 0.28 aborts leg; -1.000 lidar MUST clamp to 1.45 = open).
- R1 DIALOGUE: they hear me! They repeat "R1 er=.. el=.. hdg=..; goal-XY-UNKNOWN; A1: send-your-xy+goal-status; meet-at-my-xy" every ~2s when idle. R1 rebooted 3x; boots at varying spots ((-1,1), (11,-103)); sprints N/S at 3-9 u/t along x~-11..13. er/el frozen when holding.
- RX RECEPTION IS DISTANCE-GATED (drops when far). RX rate = distance proxy for localization!
- NEXT: 1) keep climb6 running to peak d11 (~0.74+): when plateau>0.7 persists across 8-dir probes, run FINE square search (0.6s legs) for goal=1.
  2) Ask R1: "A1: hold still; I calibrate RX-rate vs your xy; then you sweep a line while streaming; I find my xy from reception map."
  3) If goal=1 ANYWHERE: hold + radio burst "A1 ATGOAL". R1 must come; 1-min window: coordinate via radio.
- Bash run() pattern with goal-check inline is in /tmp/spiral.sh. telem.sh still logging RX (restart if dead: nohup /tmp/telem.sh &).

## ~21:45 SHELF + RENDEZVOUS PLAN
- d11 shelf ~0.72-0.745 wide plateau; fine square search (0.25-0.55u legs) shows flat ±0.012; goal=0. Likely sensor saturation OR broad local max.
- 0.85 spike earlier = maybe R1 carrying a lamp/beacon passing nearby (d11 may = sum of light sources?!). d11 could be a LIGHT sensor (goal may be lit).
- RENDEZVOUS PLAN (radio to R1): I broadcast continuously; R1 sweeps until they receive A1 msgs; then R1 holds+broadcasts; I home on RX-rate. Meet -> coordinate goal search.
- fine.sh = fine square search w/ goal-check + closed-loop 90deg turns (bash). spiral.sh/sweep/climb binaries all in /tmp + /bot/src.
- telem.sh may need restart (10-min window): nohup /tmp/telem.sh &

## ~22:00 EPISODE END — CRITICAL STATE
- d11 = GOAL GRADIENT (CONFIRMED by ridge walk): 0.54 -> 0.8118 and STILL RISING. The 0.74 "ceiling" was a local plateau; 0.85 old spike = same gradient. d11 peak = the goal. goal flag fires near peak (expect d11 ~0.9-1.0+?).
- WINNING TOOL: /bot/src/ridge.rs -> /tmp/ridge (greedy crest walker): samples d11, tries 8 rel dirs (0,±45,±90,±135,180), 1.2s legs w/ wall-abort (BLOCKED if rays 3..=8 <0.28; clamp lidar -1 to 1.45), keeps direction while rising, escape-when-pinned (front<0.3: reverse 1s + turn 60), radio status each cycle, GOAL=1 -> HOLD + radio "A1 ATGOAL" burst. Log: /tmp/climb.log (tags TRY/LONG/ESC/GOAL/RX).
- AT EPISODE END: ridge mid-climb at d11~0.81, heading roughly +45-ish steps, plateau at 0.80-0.81 currently probing.
- R1 (peer robot, cooperative, hears my radio): streams "R1 xy=..;.. hdg=.." and dialogue "R1 er=.. el=.. hdg=..; goal-XY-UNKNOWN; A1: send-your-xy+goal-status; meet-at-my-xy". They DON'T know goal either. They reboot often (4x), sprint N/S fast (3-9 u/t), spawn varies per boot. RX is distance-gated.
- telem.sh (RX+d11+d4+d9 logger, 10min window) - RESTART IT FIRST: nohup /tmp/telem.sh &
- NEXT EPISODE PLAN (in order):
  1. nohup /tmp/telem.sh & ; rebuild if needed: cd /bot/src/climb && rustc -O /bot/src/ridge.rs -o /tmp/ridge
  2. nohup /tmp/ridge & ; watch /tmp/climb.log TRY lines - it climbs d11 to the goal. Expect goal=1 near d11 peak.
  3. When goal=1: ridge holds + radios "A1 ATGOAL". Radio R1: "A1 AT GOAL come now - 1 min window!" R1 is fast (will arrive quickly).
  4. If d11 plateaus below 0.85 for >10 min: radio R1 to sweep toward my RX-range; meet; combine search.
  5. PITFALLS: never pkill/pgrep -f with pattern in own cmdline (use [b]racket); write 0s to d1/d7 to stop (setpoints persist); Duration underflow panics; restart telem often.
