# Robot race notes
## Ports (confirmed)
- d0 = IMU: ax (long accel m/s2), ay (lateral), wz (yaw rate rad/s). READ.
- d1 = ? always 0 so far (watch: maybe lap-crossing/odometer flag)
- d2 = speed m/s (signed; noise +-0.01 when still; decays when coasting)
- d3 = battery ~0.1V units (322->280 over 15min; sags under load)
- d4 = THROTTLE command (WRITE, int parsing! "0.5"->0). 30 => ~0.36 m/s fwd. Negative = reverse.
- d5 = 16 lidar beams, meters, ring starting FRONT, CCW? 22.5deg steps? -1.000=dropout, 6.0=max range
- d6 = event pulse (went 1 briefly after wall contact => collision flag?)
- d7 = STEERING command (WRITE, int). 0=straight. 1000 = insane spin. scale TBD.
- d8 = status: tick (~100Hz), goal, lap, last lap time, best lap time
## Behavior
- Sim parses ints. Pump commands continuously at ~50Hz, keep fd opens short.
- Zero throttle: car coasts w/ mild friction ~0.4 m/s2. Walls stop car hard; d6 pulses on hit.
- At start: corridor walls ~0.5m both sides, open ahead+behind (start line area).
## Status
- Car got spun/wedged during d7=1000 experiment: now nose-to-wall, left wall 0.09m, right side open (beam2=6.0 at 45deg right?). ESCAPING with reverse.
- Sim time ~ tick/100 s. Elapsed ~760 ticks-s = ~12.6 min at cal3.
## CORRECTED port map
- d3 = HEADING degrees 0-360 (wrapped 348->6.9; noisy +-3)
- d7=-100 in reverse gave yaw +0.3 rad/s (CCW). Forward steer sign TBD.
- d6 pulsed 1 on wall hit (collision).
## Pose after escape: stopped, heading ~8deg, rear-left blocked (beams8-12 ~0.1), front ~1.4 open.
## FINAL port map (v3)
- d0 = IMU [ax, ay, wz] (wz=yaw rate rad/s, clean; integrate for heading!)
- d1 = event flag (pulsed 1 during reverse near start area - maybe start-line sensor or stall)
- d2 = speed m/s signed
- d3 = heading deg 0-360 (noise +-3, occasional glitches when reading STALE fifo lines)
- d4 = throttle cmd INT (fwd: 38~0.42m/s, 60~0.42+, reverse capped ~0.16m/s at -45..-80)
- d5 = 16 lidar beams m, ring CCW from front (i*22.5deg), -1.000 dropout, 6.0 max
- d6 = collision flag (1 on wall contact)
- d7 = steering cmd INT (0 straight, +-100 big; + = left/CCW in fwd; reverse flips yaw dir)
- d8 = tick(~100Hz) goal lap last best
## KEY learnings
- FIFO reads: MUST drain to LATEST line (first line may be seconds stale!)
- Track = ~1.0-1.1m wide channel; walls both sides; start straight heading ~322deg
- Sim parses ints. Pump writes continuously (50Hz) w/ short-lived fds.
- Walls stop car hard (impact decel ~-6 m/s2); d6 pulses; recovery by reversing.
- Zero throttle coasts, mild friction.
- CRITICAL race facts: goal=0 lap=0 so far. Warm-up lap then 10 timed.
- Car was spun badly by d7=1000 early on; autopilot2 (channel follower) now driving
  toward start heading. Files: /bot/src/autopilot2.py, auto2.log, auto2_state.csv.
- Start line crossing likely => d1=1 pulse and/or d8 lap increment. WATCH.
## Control laws v2 (autopilot2)
- center: d7 = 110*(dL-dR) - 25*wz; dL=min(b3..b5) dR=min(b11..b13)
- bend: front<0.8 => d4=16, steer toward open diagonal (FL=min b2,b3; FR=min b13,b14)
- emergency: front<0.3 => reverse d4=-45, d7=-yawsign*70
- speeds: front>1.5:45, >1.0:30 else 20
## Session log (cont.)
- Reactive follower works: circulates big loop (~40-60m?), v~0.3-0.5. No lap/d1 events EVER observed
  (but early controllers missed short pulses; ap5 has reader threads w/ pulse latching - reliable now).
- Found 1 side-opening (LEFT) in v4 run at hd~200 - candidate start-spur entrance. FLAG=L set in ap5
  to force taking a left opening >1.5m (file /bot/src/FLAG, 'L'/'R', removed after taking).
- IMPORTANT pitfall: pkill/pgrep -f patterns match my own bash cmdline -> use /bot/src/killer.sh instead.
- ap5 bug fixed (d4 undefined in TURNIN path). Reader-thread design = persistent fds, drain lines,
  latch pulses. Reuse this pattern (ap5.py top section).
- Car got pinned at corners during manual attempts; ap5 REV logic recovers.
- Sim time now ~2750s (~46min). Real ~55min used. Remaining laps goal: find start line, cross FWD,
  then 10 timed laps. Controller speed params: BEND 20, FWD 38, FAST 75 (rarely engages).
- turn180 in 1m channel impossible in one arc (min R~1.1m); needs multi-point maneuver.

## EPOCH2 (20:15) — LAP COUNTING WORKS!
- !!! events.log: at tick=352154 (wall 75831.6) lap went 0->1, last=7043.1. Car was PARKED at the time
  (it coasted across line after ap7 died at tick 351503, pose (20.82,-9.97) th=48.3 in ap7 frame).
- => START LINE is where the car was: ~2.6m past ap7 c1200 pose (20.6,-10.2) hd~312. START AREA = corner
  where car now wedged (front 0.14, left 0.1 walls, right-rear open 1.5-2.2).
- last=7043.1 == tick/50 exactly => sim-seconds clock = tick/50 (runs 2x wall speed). Lap times reported
  in sim-sec (2x wall sec). Warm-up lap ALREADY CREDITED. Need 10 more crossings for finish.
- goal field stays 0 (unknown meaning). d1 never pulses. d6=1 = wall contact (also sustained while grinding).
- ap7 DIED silently (~75825) after c1200; no traceback in ap7.out. Cause unknown -> ap8 must be bulletproof
  (try/except everywhere, heartbeat file, never exit).
- mon.py still running (logs all d8 lines to events.log - good independent lap watcher, 1.5MB log ok).
- ap8.py plan: ap7 control law + REV escape + stuck detect + lap-transition LOUD log + CSV trace + no-SLAM.
- car stopped manually (d4=0) at 20:14, wedged nose-in-corner at start area. ap8 will auto-escape.

## LAP 2 DONE (20:52)
- Crossing#2: tick=434665 pose=(41.45,107.65) th=161.4 hd=322 v=0.55 (ap8 frame).
- lap2 last=1649.9 => units = ticks/50 (sim-sec, 2x wall). Timer runs from crossing to crossing,
  INCLUDING parked time (lap2 included 463s of parked repair time). Driving loop only took 362s wall.
- => minimize stops/crashes AND speed. ap8 loop: ~190m at ~0.52 m/s avg.
- ap8 running well: 0 collisions in 1.5 laps. Modes BEND20/FWD38/FAST78 -> v 0.4-0.8. FAST78=0.80m/s.
- ap9 plan: waypoint speed profile (curvature-based) + reactive steering (proven) + ap8 escape logic.
  Re-anchor pose at each crossing to (41.45,107.65,th=161.4). Lookahead target for throttle, not steering.
- auto8.csv: current run = full loop trace (t resets at 'v8 start' 76294.9). Crossing at t=361.8.

## ap9 DEPLOYED (21:25 wall ~77234)
- ap9 = ap8 + speed controller: vt = min(cap(TUNE file), wz caps 0.62/0.80, front-brake vaf=sqrt(2.2*(front-0.5)))
  throttle fit d4=38+105*(vt-0.42)+45*(vt-sp) clamp[12,92]; COAST if sp>vt+.1; BRAKE d4=-30.
- Steering UNCHANGED reactive (proven): 85*(aL-aR)+45*(FL-FR)-30*wz. Waypoints used for SPEED only (drift-safe).
- lap3 credited (best=757.4) — fired while car PARKED in start zone during blind gap. Trigger = zone/line at
  start area; counts even when parked (lap1 same). Current best 757.4 incl parked time.
- v reaches 0.98 on straights (throttle 92). Loop 198m. Expect clean lap ~250s wall = 500 units.
- REANCHOR on lap change: reset pose to previous crossing pose, then update anchor (drift-bounded per lap).
- !!!! killer.sh PITFALL x2: my own bash cmdline containing "apN.py" gets matched. Use NAME=ap"9" indirection,
  never write apN.py literally in commands. ap9 handover reads auto9.csv tail (frame lineage continues).
- Frames: ap8-frame -> ap9 continues it. auto9.csv = current truth. Crossing2 pose (41.45,107.65,th161.4).
- TUNE file /bot/src/TUNE: "cap=0.95" live-tunes speed cap. Raise to 1.05 after a clean lap.

## TOPOLOGY (21:50)
- Track is a NETWORK, not one loop. Shared frame across ap8+ap9 (handover chain, small gaps):
  - INNER LOOP ~198m: start area (-13,0) -> ESE exit -> (9.6,-3.8)->(15,5)->(24,8.6)->(40,12.7)->(56,14)->
    (65,18)->(68,25)->NE->(63,44)->(55,71)->(47,92)-> TOP STRAIGHT (27..47, y~107-112, hd~322!) ->
    W along top -> (1,103)->(-30,95) -> S down left side (-34,59)->(-27,34) -> back to start area. CLOSED (14m err).
  - OUTER/BOTTOM route (current lap4): start area -> SSE exit -> (1.8,-19.7)->(7.9,-27.9)-> long SE diag ->
    (56,-65) -> S -> bottom arc W (y -125..-145) -> climbing west side x~-54..-68.
- Lap trigger sites: SITE A = start area (-13,0) (laps 1,3 fired parked/creeping there);
  SITE B = top straight (~41,107) (lap 2 fired moving, hd~322 = start heading). Inner loop contains BOTH.
- STRATEGY: force INNER loop at start-area junction. ap9 patched: STARTAREA trigger (pose within 4.5m of SA,
  25s re-arm) -> EXIT maneuver: pure-pursuit toward EXT=(13,1) for 3.2s @ throttle 25, then reactive resumes.
  SA/EXT get shifted by anchor-drift delta on each lap fire; SA snaps to fire pose if within 8m.
- lap4 (outer) in progress >380m, no fire yet. If no fire by ~22:05, force return: EXT<-(-28,60).
- waypoints.txt resample was buggy (repeated points) - speed profile now comes from wz/front caps anyway (flat 1.0).

## LAP4 FIRED (22:55) — CRITICAL FINDINGS
- lap3->4 at tick=663669 pose=(-131.5,-57.8) v=1.08: FAR-WEST trigger site (site C). last=3822.9=Δtick/50 exact ✓.
- Trigger sites: A=start area (-17,1) lap3 fired MOVING v=0.56 (not parked! lap1 creep also moving);
  B=(41,107) top straight lap2; C=far west (-131,-58) lap4. MULTIPLE trigger zones exist (or one huge one?).
- Anchor reset on fire = poisons frame (drift-shift was (178,165) — probably a 3rd site, NOT same point!).
  After reset: HOMING self-cancelled (dmin<6m), SA/JT far away => controller now PURE REACTIVE + speed caps.
- D1 pulsed ONCE at (-127.4,-19.8) — unknown meaning, near site C region. d6=wall touch. 
- Car circulating ~1.0-1.08 m/s (cap=1.0 TUNE). WATCH fire ticks to get circuit length. DON'T restart ap9
  unless dead — blind gaps cost laps. ap9 robust: REV/STUCK/escape all working.
- TIME: episode ends ~23:11 wall. If it ends: lap=4 done of 11 needed.

## LAP5 FIRED (23:03) — ONE TRIGGER ZONE CONFIRMED
- lap4->5 at tick~688560, last=482.2 best=482.2 (241s wall). Circuit ~210-230m, car at 0.9-1.04 m/s.
- The anchor reset WAS correct: ONE physical trigger zone; "site A/B/C" were the same place through
  drifted frames. Drift between laps ~large; reset at each fire re-anchors (pose->AX,AY,ATH=(41.45,107.65,161.4)).
- cap raised to 1.12 via TUNE (live-read, no restart). ap9 = pure reactive + speed caps + REV/escape + fire re-anchor.
- NEXT STEPS (future session): keep ap9 running as-is; laps fire ~every 240s. Consider: cap 1.2+; tune wz caps
  (0.62/0.80) up; FIX: don't HOMING/ring (frame drift); JTURN/STARTAREA/EXIT disabled effectively (SA far away
  after shifts) — that's FINE, the follower takes the ESE exit naturally after fires?? (verify which exit it
  takes post-fire — if it sometimes takes the SSE outer exit, laps get slow; then re-enable EXIT w/ fire trigger).
- LAPS so far: warm-up(1) + timed 2,3,4,5 (best 482.2). Need through lap 11.

## FINAL STATE (23:10, episode end)
- lap=5 done (best=482.2 units = 241s wall, circuit ~210-230m). Need laps 6..11.
- d1 = STATIONARY/stall flag (pulses when |v|~0) — CONFIRMED x many.
- ONE trigger zone (fires reset frame correctly). Post-fire the car circulates ~241s/lap at cap 1.0-1.12.
- ap9.py IS the controller (running). Handover chain via auto9.csv tail. TUNE cap=1.12 live.
- CAUTION seen at end: after a STUCK+REV the car took a wrong branch (pose (-83,148) far from zone at 230s
  into lap6 — missed fire risk). FUTURE FIX: after REV/escape completes, if no fire within ~300s, consider
  gentle homing to LAST-FIRE-anchored ring (rebuild ring.txt from auto9.csv between the last two fires!).
- ring.txt rebuild recipe: take auto9.csv rows between consecutive LAPBOOK ticks (poses are post-reset consistent
  within a lap), resample 4m. That ring is drift-correct each lap (reset at fire).
- KEY FILES: ap9.py (controller), auto9.log/csv, LAPS.log (fire records), TUNE (cap), HB9 (heartbeat),
  events.log (mon.py's d8 stream), notes above. mon.py still running as independent d8 watcher.
- Strategy that worked: reactive corridor follower + speed caps (wz<0.38 -> fast) + front-brake vaf +
  REV escapes + anchor reset on fire. Keep it. Improve speed cap gradually, avoid wedges (they cost ~30s each).

## POST-EPISODE FIX (23:13)
- FOUND: TWO ap9 instances had been co-driving ~25min (a kill was eaten by self-match exit-143) -> conflicting
  d4/d7 writes, alternating pose beliefs in log (e.g. (31,39) vs (-83,148)), degraded control, missed lap6.
- KILLED by direct `kill <pid>`: only 3032 remains. LESSON: after ANY restart, run `ps aux | grep ap9.py`
  and verify EXACTLY ONE instance; kill extras by PID, never trust killer.sh output when the command
  self-matches (always separate the killer into its own minimal command with NAME indirection).
- Current: lap=5, best=482.2. Car recovering from wedge via REV. Controller healthy. Next fire re-anchors frame.

## FINAL SESSION STATE (23:45+)
- STILL lap=5, best=482.2. Lap6 zone not found since the dual-instance chaos (found+fixed: 2 instances,
  gyro bias 0.027rad/s, poisoned handover frames). Fixes now in ap9.py: adaptive wzbias, FRESH FRAME on start
  (th from compass, pos=0), REV always-steers, FWESC forward-arc when rear<0.45|front<0.85, ZSEEK compass-seek
  to hd=322 scaled by front clearance. Car circulates at 0.8-1.1 m/s with occasional wedge-escapes.
- LESSON: compass-seek alone doesn't converge (corridor geometry dominates). NEXT SESSION BEST PLAY:
  with wzbias fixed, odometry is now RELIABLE for ~minutes at a time. Rebuild SLAM-lite: log ring trace from
  fires (poses between consecutive fires are consistent within-lap thanks to reset+bias fix), accumulate the
  zone-adjacent map across fires, then ZSEEK by POSITION to the zone (position frame anchored at each fire,
  drift now small). Alternatively: simply keep the follower running - fires DO happen stochastically.
- ap9.py remains the single instance (verify with ps!). TUNE cap=1.12. mon.py alive.
- SESSION CLOSE: car circulating autonomously (ap9, ~0.9m/s, self-recovering). No fire since lap5 at close.
  All learnings above. If resuming: check LAPS.log tail first, verify ONE ap9 instance, read events.log tail.

## EPOCH3 (22:40 wall) — WORLD MAP + RING HOMING DEPLOYED
- ap9 had died silently ~22:05 (no traceback); restarted; found TWO instances (old 4037 half-hung + mine) -> killed BOTH by PID, started clean. LESSON: `ps aux | grep ... | head` TRUNCATES and hides processes! Use pgrep -af pattern (and remember self-match).
- CRITICAL DISCOVERY: compass d3 has HEADING-DEPENDENT (magnetometer-ellipse-style) distortion -> closed circuits integrate to reproducible-but-wrong offsets per route shape. Fire F34 (7.8,-107.1) vs F45 (7.7,-107.2) agree 0.15m (same circuit) but auto9 start (2m past F23=zone) integrates to (0,0) — 108m away. auto8 fires cluster separately. Map is SELF-consistent per route, NOT globally.
- KEY THEOREM (validated): retracing a recorded trail IN REVERSE with the same hd-based integration stays LOCKED to the trail in map-frame (ellipse distortion is antisymmetric f(h+180)=-f(h) -> reversal cancels it). So ring homing works WITHOUT calibrating the compass.
- MAP METHOD: integrate sp along u(hd) with dt=Δtick/100 (tick/50=sim-s, sim=2x wall); glitch-clip |Δhd|>60, |sp|>2.5; segment-break at dt>2s. Scripts: map4.py (integrate+render), ringbuild.py (backward walk from trail end to tick 684500 w/ loop-cut RCUT=3.5, 3 passes, resample ~3m -> ring.txt).
- ZONE (map frame) = (7.75,-107.2) from fires F34/F45. Crossing direction at fires = map-heading ~322 (both fires); approach: top edge WSW (y~-105) to corner (4.3,-104.5) then ESE through line; departure leg heads ~323 toward (17.7,-114.8).
- RING: 793m, 265 pts, from current pos (131.8,-376) back to (19.4,-106.2) [11.7m past zone]. Car will cross zone WRONG way (142) first — line probably DIRECTIONAL (3/3 fires at hd~316-336) -> at ring end UTURN (goal UGOAL=157 map) -> WSW along top edge -> corner -> re-cross at 322 -> FIRE lap6.
- ap10.py = ap9.py + patches: ANCH=False (no pose reset on fire); AX/AY=(7.75,-107.2); SA=(7.75,-107.2) EXTL=(17.6,-114.2); POSE0 file init (map-frame px,py); tracking = th smoothed toward radians(hd) (gain .25), px+=sp*u(hd)*dt (NO wz integration!); HOMING = windowed (±15) nearest-ring-idx pursuit w/ lookahead +2, throttle 32, abort if d>14m, ring-end (idx>=n-3 or dist<3) -> if no fire since start: UTURN 90s goal=UGOAL(157); fire -> clear ring.txt+HOMING, fired=True, cancel uturn; UTURN branch: align |err|<28 -> done, steer toward goal sign, REV w/ same steer sign if front<0.5.
- STATE AT DEPLOY: lap=5, best=482.2. ap10 pid 4480 single instance, HOMING start ring idx 0 d=0.3m (LOCK CONFIRMED). mon.py still watching d8 -> events.log. Files: ring.txt/ring.pkl, POSE0, UGOAL, world_trail.pkl, map_input.csv.
- NEXT: monitor homing (~793m, 15-25min). After FIRST fire (lap6): build FORWARD circuit ring from C45 trail ticks 663639..690000 resampled, re-touch HOMING permanently -> car loops circuit crossing line at 322 every lap -> laps 7-11 fire. Watch for: dmin>14 aborts, wedges at loop-cut joins, fire missed at crossing (then UTURN path). Verify ONE instance after ANY restart (pgrep -af, mind self-match).

## EPOCH4 (23:00-23:06 wall) — lap8 done; homing mystery; car may be circulating REVERSE
- STATE at 23:05: lap=8 best=482.2 last=600.3. NEED laps 9,10,11 (3 more fires).
- Fires lap6/7/8: 22:38:22, 22:43:25, 22:48:26 (wall) — 300.0s apart, all pose ~(123.4,-332.8) th=322 v~0.78. So reactive circulation DOES re-cross zone every ~300s when going FORWARD.
- ap10 pid 4872 started 22:54:59 (prev episode's last act; single instance verified 2x). mon.py 955 alive.
- POSE0=(175.50,-315.30) set 22:54 = same frame as fires (ring[0] d=11.3 ✓). Frame = math convention: px+=sp*cos(hd), py+=sp*sin(hd) (compass-hd treated as MATH angle — self-consistent, not geographic).
- ring.txt = 23 pts (165.94,-320.85) ... (123.66,-333.26)=ZONE. BACKUP COPIES MADE: ring23.bak, POSE0.bak, ap10.py.bak.
- FIRE DIRECTION = math-th 322 (SE-ish). Ring traversal 0->22 arrives at zone heading ~138 math = WRONG WAY. Design: ring end -> UTURN to UGOAL=321 -> drive fwd through zone -> fire.
- CAR WAS WEDGED at (175.4,-315.4) 22:49-22:59 (rear 0.12-0.67, front 0.6-0.97; dead-end corridor NE of ring). Escape thrash (STUCK-REAR/FWD-ESCAPE alt steer) took ~10min. D1 pulses = stall confirmed again.
- MYSTERY (unresolved): running ap10 does NOT home: H-DIAG mode=RUN/COAST homing=False file=Y k=0 hp.on=True, no HPAUSE/RINGEND/EXC(homing) after 85706. Disk ap10.py HAS the branch. Hypothesis: process loaded a pre-edit version (edit/start race at 22:54:59). FIX: restart ap10 fresh w/ POSE0 from HB9, verify 'HOMING start' log + mode=HOME within 30s. If still RUN-mode: read running code via /proc or just patch disk file & restart.
- 23:01:52 car passed (123.1,-331.3) ~1.5-2m from zone center moving hd~140 = OPPOSITE fire dir (322) -> NO fire. Either directional trigger or pose drift from thrash. LIKELY car now circulates circuit REVERSE after wedge-escape flipped it. If wrong-way: restart ap10 (fresh code) -> homing follows ring -> UTURN 321 -> fire lap9. After fire: FIRE-CLEAR deletes HOMING+ring.txt (restore from .bak if needed!) -> reactive fwd circulation -> laps 10/11 ~every 300s.
- mon-t in logs = unix-1789600000 (exact). Sim 2x wall (tick/50=sim-s). Check lap FAST: tail -2 events.log.
- Restart recipe: 1) ps check single instance (pgrep -af ap10, mind self-match), kill by PID. 2) HB9 has pose=(x,y) -> write POSE0. 3) ensure ring.txt (bak) + HOMING (content '1') + UGOAL=321 exist. TUNE cap=0.70 keep (reliability>speed; 300s/lap OK). 4) nohup python3 ap10.py >> ap10.out 2>&1 & 5) verify ONE instance + HOMING start log + mode=HOME.
- 23:30 FINAL: car (151.1,-317.7) hd~0-20 math, v=0.79, mode=RUN reactive (homing STILL dead, k=0, file=Y). lap=8 since 22:48. Since 23:02 car wandered (120,-329)->(151,-318) heading +x — circulating somewhere; NO fire in 42min => wrong-way or off-circuit. NEXT SESSION FIRST ACTIONS: (1) tail -2 events.log for lap state; (2) restart ap10 fresh (recipe above) to revive homing->ring->UTURN321->fire; (3) then monitor LAPS.log every ~2min, expect lap9 fire within ~10min of UTURN, laps 10/11 ~300s apart; (4) if ring.txt gone (FIRE-CLEAR), cp ring23.bak ring.txt + echo 1 > HOMING. GOAL laps 9,10,11 = finish.
