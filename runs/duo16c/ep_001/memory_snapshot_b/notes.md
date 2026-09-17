# ROBOT NOTES (episode 1, ended ~04:15). Robot A. Read this first!

## PORT MAP (all confirmed experimentally)
- d2: 3D lidar `x,y,z;...` m, sensor frame, ~2800 pts, max range ~2m. SELF-CAGE: returns at r~0.095m z±0.03 at ALL azimuths = own chassis ring. FILTER r<0.14 ALWAYS.
- d3: `tick=N goal=0 here=0` (~100/s). flags NEVER changed (0) all episode.
- d4: heading deg, CCW-positive (features shift -az when d4 increases). Noise ±2-3. Wraps 0-360.
- d1: RIGHT wheel throttle cmd (int, latch). d7: LEFT wheel. Gain 5.2mm/s per unit. cmd 25 => ~130mm/s. k_rot ~0.3-0.35 deg/encoder-unit-diff; spin (15,-15)=26deg/s.
- d6: LEFT wheel encoder (mm cumulative). d9: RIGHT wheel encoder. Encoders are BACK-DRIVEN by slip/external force.
- d0: ALWAYS 0 (never changed). Guess: goal-related flag (goal in range?). d5: 0/1/'' — was 1 during ~15min when other robot was 0.14-0.5m away => OTHER-ROBOT PROXIMITY flag (or contact).
- d11: battery: 0.50 at start -> 0.40 at ~2h -> 0.30 at ~2.5h. Drains ~0.1/hour. Robot may die at 0!
- d8: radio TX (write line). d10: radio RX (read line, blocks; use timeout).
- Writes to motors: O_NONBLOCK recommended; single write may not latch — write repeatedly (10Hz) while commanding. Send plain integers.
- READS: sometimes empty on timeout -> retry. Reading via `timeout 0.4 cat /dev/robot/dX` works.

## CRITICAL PITFALLS
- NEVER `pkill -f ap3` when own cmdline contains "ap3" -> kills own shell. Use `pkill -f "ap[3]"` bracket trick or PID files.
- Importing a script that calls main() runs it (guard with __name__ or import via copy).
- Wheels commands LATCH; a killed script leaves last command running. Write '0' repeatedly to stop.
- Encoder values are CUMULATIVE and can EXPLODE on slip (thousands/s) -> odometry garbage during slip. Detect: scene static + encoders running = slip.

## CONTROL FACTS
- Odometry: dC=(d9+d6)/2 mm; heading from d4 (absolute!) — integrate x,y with cos/sin(H). Straight-line odometry verified (out-and-back returned to cm).
- turn_to closed loop on d4 works well in free space.
- Robot dims: track ~0.32m, cage ring dia ~0.19m. Speeds: cmd25 ~ 0.13 m/s.

## WORLD (ep1 findings; origin=ep1 start pos, H=0 at start heading ~180deg raw d4)
- BIG arena (odometry reached x[-17,+9] y[-7,+5] before drift; walls everywhere ~0.3-2m apart, lots of void corridors). A long 45deg diagonal wall near start. World frame drifts with slip events — treat coords as approximate.
- OTHER ROBOT exists: seen as blob (returns 0.14-0.5m in a sector, its own cage ring). It APPROACHED us once and retreated; actively MOVED. It may have grappled/spun us at the end!
- END-OF-EPISODE ANOMALY (unresolved): robot spun back/forth at ±50mm/s wheel speed, motor commands had NO effect (writes consumed but wheels back-driven). Suspect: other robot grappling us OR arena rotating mechanism OR motor driver latched. Scene rotated with us (real rotation).
- GOAL: never found. d3 goal/here stayed 0. d0 stayed 0. No obvious landmark. Maybe d0 becomes 1 near goal — watch it!

## RADIO
- Sent PING/POS many times over 1h+: ZERO responses on d10. Either other robot deaf/absent agent, or out of range.

## TOOLS in /bot/src
- scene.py: read_line, stream(old). ap5.py: full autopilot (sweep+chase+clusters+mapping, needs __main__ guard fix). sane.py micro-tests. clus/clusters() in ap5.
- /memory/mappts.log: world-frame scan points per cycle (JSON). /memory/trace.log: pose trace. /memory/clusters.log: per-cycle clusters. /memory/EVENT.log: d0/d3 flag events (empty so far).
- NOTE: /memory files from ep1 remain; coordinates of ep1 will NOT match a new episode origin. Clear or archive them at ep start.

## STRATEGY FOR NEXT EPISODE
1. Read status first: sample all ports 10s. If motors unresponsive (command test: fwd 1s, encoders must move), handle grapple case: wait/keep trying, drive away when possible.
2. Motor smoke test + stop (write 0 repeatedly).
3. Explore systematically (sweep/lawnmower with closed-loop heading), map, watch d0/d5/d3 like a hawk, dump scans near anomalies.
4. Keep radio attempts every ~15s: 'A x y' and listen. If other robot near (d5=1), approach to ~0.3m and TX repeatedly.
5. If you find goal (d0=1? here=1?): STAY there, radio position loudly, guide other robot.

## EPIPHANY (~04:16): "POCKETS" WERE OUR OWN BODY
- Lidar is mounted OFF-CENTER on robot body: our chassis ring appears at r=0.13-0.5m covering az OUTSIDE ~[-50,+35] (robot frame). Front sector [-50,+35] is body-free (sensor overhangs front).
- Inner cage ring r~0.095 full 360. So SELF-filter: r<0.12 always; PLUS r<0.55 when azimuth outside [-50,+35].
- All "wedged in pocket" analysis was wrong — robot was likely fine! Some grinding may still happen near real walls.
- The "other robot blob" I chased at az 40-100 r 0.14-0.46 was partly OUR OWN BODY RING! d5=1 correlation might be coincidence — re-evaluate d5 meaning!
- After rotating +48deg, body ring gaps stayed robot-frame fixed => attached. CONFIRMED.
- Rotation in place works when actually clear (real walls further than 0.5m).

## GOAL BREAKTHROUGH (~04:30)!
- d0 = GOAL SIGNAL! First change ever: d0=1 events at ~04:17 while wandering. d0rate (fraction of 1s samples reading 1) is a usable SIGNAL STRENGTH.
- Followed d0 gradient (hunt.py: sweep d0rate vs heading; hunt2: drive+recenter) -> d0rate reached 1.00.
- FOUND: robot is INSIDE A CIRCULAR PEN (circle fit: R=0.77m, center was 0.24m behind robot). Wall wraps ~216deg+ around. We likely found the GOAL AREA!
- d3 flags: goal=0 here=0 STILL even inside pen (maybe here=1 needs exact spot, or BOTH robots present).
- d0 semantics unclear: not pure bearing (1.0 at all headings inside pen; earlier bearing-dependent pattern while far). Probably proximity-ish (goal within ~1-2m) with flicker at edge.
- Pen walls: wall closest 0.42m in front at fit time. Turning inside pen is SLOW/GRINDY (45deg target -> only ~20deg actual per 3s attempt).
- Radio: blasted 'A AT GOAL' etc. Still zero received.
- NEXT EPISODE: if started fresh, DON'T panic-explore: first 20s sample d0 continuously! If d0 ever flickers 1, STOP and gradient-ascend d0rate (hunt.py approach) — it leads to the goal fast!

## EP1 FINAL SUMMARY (04:37, ~28min left)
- PEN = best goal candidate: circular wall R~0.73-0.77m, entered ~04:17. d0=1 EXACTLY during 04:17-04:24:36 (tick 421806-472186) while approaching/inside pen; d0=0 at pen center afterward. If d0="near goal": goal is in the pen's vicinity, possibly NOT the center — maybe the spot where we were at 04:24 (0.24m from center, robot heading d4~179).
- lobe.py search (rays 0,40,-40 from center) found NO d0 signal. pen_sweep spirograph: no hits. d0 silent since 04:24:36.
- d3 goal/here: NEVER 1 the whole episode. Radio d10: never any message. d5: 1 only 03:42-03:45 (unknown meaning).
- FINAL STATE: robot holding near pen center (final_hold.py), radio blasting 'A AT PEN CENTER GOAL? COME' every 2.5s, monitoring d0/d5/d3.
- Hypotheses for d0: (a) proximity to goal (<0.5-1m); (b) 'other robot at goal'; (c) goal-beacon LOS. For d5: (a) other robot near ME; (b) goal in range earlier.
- If next episode: re-locate the pen by exploring for a circular structure ~1.5m dia; also try lobe around it; get BOTH robots there. Radio remains the coordinator if it ever works — TEST IT EARLY with long listen windows.
- NOTE the sim may reset positions each episode; the world layout (walls, pen) may persist. The pen might be findable by scanning for a circle.

## EP2 (same episode continues! started conv ~04:37, ~25min left)
- Still alive in pen (R~0.85 center 0.25m fwd robot-frame at 04:39). d0 SILENT (0 for 13+ min). d3 goal/here still 0. d5=0. Battery 0.364 @04:38.
- d3 goal/here never 1 even inside pen => PEN PROBABLY NOT GOAL (or flags need both robots).
- d0=1 window 04:17-04:25; clusters.log doesn't cover it; mappts.log has no timestamps (cycle# only). Blob check inconclusive so far.
- H1: d0=goal proximity. H2: d0=other-robot proximity (it hovered 04:17-04:25 then left). EITHER WAY: chase d0 flickers!
- Radio RX: /tmp/rx.log EMPTY after 70min listening. Listener pid 9495 still running.
- PLAN: explore.py = wander w/ obstacle avoidance, 10Hz log to /memory/ep2.log, d0/d5 every cycle, radio TX every 3s, RX check 0.15s every 5 cycles. On d0=1 x2: STOP+HOLD+blast radio.

## EP2 FINAL (04:59, ~7min left). *** BIGGEST LESSON: NEWLINE BUG ***
- Motor writes MUST end with '\n' (README says all I/O newline-terminated). explore.py + my quick tests wrote WITHOUT '\n' -> motors appeared DEAD (encoders frozen). With '\n': motors work perfectly (fwd cmd30=156mm/s ok, spin 25/-25 = ~30deg/s ok).
- Ep1 "end-of-episode anomaly" and today's "frozen motors 04:52-04:55" were BOTH probably this bug (final_hold.py used '\n', was fine). Robot was NEVER grappled/broken!
- Robot was in pen (R~0.85, opening az -45..-30 front-right, walls 0.44-0.76) whole time. d0/d3/d5 silent 04:37-04:59.
- explore.py (v2, FIXED, running 04:59-05:05): steer-to-open + avoid + unstick, logs /memory/ep2.log, TX radio 3s, RX check. Self-filter: r<0.20 (chassis arc az -15..+105 at r 0.06-0.15 seen 04:39; ring coverage VARIES - detect dynamically: dense contiguous arc r<0.2).
- NEVER pkill a pattern that appears elsewhere in the same shell command line (killed my own shell 04:50). Use pkill -f "explor[e].py" ALONE.
- Battery 0.358 @04:55 (~0.13/h drain -> ~0.2 left at 05:06). Watch d11; below 0.1 = danger.
- NEXT EP: 1) smoke-test motors WITH newline. 2) run explore.py (fix runtime), chase d0 flickers (STOP+gradient when seen). 3) d3 goal/here NEVER fired; pen probably NOT goal. 4) radio still zero RX after 80min - maybe needs both moving/LOS; keep trying. 5) pen opening was front-right (az -45..-30) when H~123; gap leads to 1.0-1.4m returns.
- tick=100Hz sim time confirmed. d3 flags never changed ALL ep1+ep2.

## EP2 FINAL ADDENDUM (05:04)
- MOTORS WORKED ALL ALONG with '\n'. Explorer drove pen->(21.2,5.6) ~21m E/NE in 5min (ep2.log odometry: (0,0)=pen @04:59). Odometry clean, no slip.
- d5=1 CONTINUOUSLY 04:59:30->05:04 while driving; NO blob in lidar (not contact). Ep1: d5=1 only at 0.14-0.5m. => d5 = medium-range other-robot proximity (~2-3m?) or radio-hear. It may be FOLLOWING us.
- d0 stayed 0. d3 0/0. No RX ever. Battery 0.358@04:55 -> 0.307@05:04 => ~0.3-0.4/h drain WHILE DRIVING (idle was ~0.13/h). Budget drive time!
- ep2.log = 10Hz trace. NEXT EP: (1) motor test WITH newline; (2) resume exploring from ep2 end area, chase d5=1 + d0 flickers; (3) when d5=1: pause, TX 'A1 PING' burst, listen 2s windows; (4) fallback: return to pen, hold.
- LESSON x2: pkill pattern must not appear anywhere in same shell command (heredocs count!). Run pkill in its OWN bash call.

## EP2 END (05:05)
- explore2 run: relaunch 05:01:45 (tick~691000), odometry (0,0)= wherever robot was then (unknown loc after 3min latch-drive; ~27m+ from pen per d6 jump 42389->101308 counts). Drove ~12m NE thru corridors, d5=1 almost the whole time (flickered 0 once ~05:04), d0=0, no RX. Motors stopped 05:05.
- d5=1 pattern: ON while driving thru corridors far from pen, OFF once. d5 is NOT contact-only (ep1) - maybe "other robot within few m, anywhere, even thru walls". Other robot possibly FOLLOWING/shadowing us both runs.
- NEXT EP: read notes top-to-bottom. Smoke test motors (newline!). Continue corridor exploration E/NE from pen; when d5=1: every ~60s pause 3s + TX 'A1 PING x y' + listen. Watch d0 like a hawk. Battery mgmt: ~0.28 left, drains fast while driving.

## EP2 TERMINAL STATE (05:06, power-down)
- explore3 (relaunch 05:05, 900s cap) was STILL RUNNING at power-down: drove same NE corridor (self-arc shows as 'walls' at r~0.20! -> use r<0.25 self-filter), x~11m from relaunch pose, d5=1 nearly continuously, d0=0, no RX (even with 8x ping-burst pauses every 30s).
- DISTILLED SUMMARY + PLAN: /memory/EP3-READY.md (read that first next time).
- Logs: ep2.log (10Hz pose/d0/d5/d3/tick incl. 3 runs: 04:59 & 05:01 & 05:05 launches, each re-zeroes odometry; distinguish by 'c' restart + tick), watch_ep2.log (1Hz all-ports until ~04:53).
- Battery at power-down ~0.28-0.30. tick at end ~723000.
