# Robot port map (confirmed by experiment)
- d1: LEFT motor, write "v\n" (newline REQUIRED). + = forward. ~5 ticks/s per unit.
- d7: RIGHT motor, write "v\n". + = forward.
- d6: RIGHT wheel encoder (signed, cumulative)
- d9: LEFT wheel encoder (signed, cumulative)
- d4: compass heading, degrees, CCW-positive (heading INCREASES turning left/CCW). noise +-4 deg.
- d2: LIDAR raster scan, rolling window (~2800 pts): "range,elev,azimuth;" triplets.
  elev -1.12..+0.4 rad (mostly downward => floor scanner), az +-0.155 rad (~+-9deg).
  range can be NEGATIVE (no return, ~-2.5). Tiny values ~0.005 = junk; filter >0.02.
  Range units UNKNOWN (suspect meters; floor returns dominate mean).
- d3: "tick=N goal=G here=H". tick ~100/s. goal=1 when at goal? here=1 when? (both 0 so far)
- d0: flag, seen 0 always. Maybe bumper/rear contact.
- d5: flag, =1 twice while driving forward pressed near obstacle. Maybe front contact/bumper.
- d11: float ~0.40-0.58, NOISY +-0.02, roughly constant regardless of motion. Unknown (battery? RSSI to other robot?)
- d8: radio TX (write line). d10: radio RX (read line, '' if nothing).
- tick rate ~100/s; 40 ticks per 0.4s.
- Motor writes IGNORED without trailing newline!
- Rotation: d1=+25,d7=-25 => CCW ~22 deg/s. d1=30/d7=-30 => ~55deg/s (probe9 log suggests 16deg/0.35s)
  fwd60 for 1s: right enc +314, left +~290 (drift).
- Earlier "forward" failures were missing newline. 
- pinger.py (TX PING every 2s) + listener.py (logs RX to /memory/radio_log.txt) may be running in bg.
  START: nohup python3 /bot/src/listener.py & ; nohup python3 /bot/src/pinger.py &
- No RX heard in first ~40 min. d10 silent.

# EPISODE 1 FINDINGS (later findings, ~70min in)
- d11 = DISTANCE TO THE OTHER ROBOT (companion). Confirmed by gradient: dropped 0.52->0.25 while
  driving toward wb~90 (descend run); rises to ~0.6 plateau (maybe max range) when apart.
  NOISY +-0.02. When we wander it reads 0.44-0.61. The companion MOVES (follows/roams).
- We bumped/pushed the companion: real STALLs + d5 latch (d5 = contact/obstacle latch, ~front).
- d0: still never fired (maybe rear contact).
- d3 goal/here NEVER fired despite driving ~300k ticks of path.
- Lidar far returns (2.4-2.9 units) at wb~90 and wb~272: steep-DOWN elevations (-1.2..-2.3 rad)
  => elevated terrain / drop-offs those directions; range shrank when driving toward wb90 then
  grew again => passed edge. Local floor med ~0.09-0.10 units.
- Scale estimate: 1 lidar-unit ~ 2000 encoder ticks (rough, from closing rates). Speed 100 cmd ~ 700 ticks/s.
- Radio: PING every 2s for 60+ min, NO RX ever. Maybe range < 0.3 units! Try radio only when d11<0.3.
- HOT TIP: chase companion via d11 gradient descent worked (0.52->0.25 driving straight wb90).
  When both robots chase each other -> plateau 0.6. STOP-AND-WAIT so the other can approach.
- Scripts in /bot/src: hotcold.py (d11 gradient), closein.py, sprint.py, descend.py, slowcrawl.py.
  Kill with: ps aux | grep NAME | grep -v grep | awk '{print $2}' | xargs -r kill
  (NEVER pkill -f "NAME" — it kills your own shell if pattern is in cmdline!)
- Odometry: encoders cumulative; X+=fwd*cos(h), Y+=fwd*sin(h), h=d4 compass deg CCW+.
- CAUTION bug: first poll() after fresh r0=0 reads whole encoder as delta (offset only).

# EPISODE 1 FINAL STATE (~85 min in)
- Robot position: drove compass-272 for several min from corridor x=-540 y=-3681 (odometry frame
  arbitrary across scripts; encoders d6/d9 are cumulative-absolute: tick~329000 at this point).
- Lidar elev convention HYPOTHESIS (from-vertical, 0=up, pi=nadir): steep-down returns at
  elev -1.2..-2.3 rad = ground BEHIND-BELOW 2.4-2.9 units => WE ARE ON A ~2.4 unit HIGH EDGE/MESA.
  Local plateau floor ~0.05-0.1 below sensor. Cliff bearings compass ~90-110 and ~264-272.
- d11 = companion distance: follows our motion, 0.45 when slow/near, up to 0.6+ when fast,
  0.25 when we charge straight ahead for a while. NOT a goal sensor (probably).
- mission status: goal NOT found. Flags never fired. Radio silent (maybe needs <0.3 range).
- NEXT EPISODE PLAN:
  1. Locate cliff precisely: rotate 360 sampling deep-return (elev<-0.9) max vs bearing.
  2. Drive OFF/THROUGH the cliff bearing; watch floor_med & stalls; if descending, sweep lower level E-W.
  3. Keep pinger + listener running always.
  4. When d11<0.3, do radio burst tests (maybe short radio range!).
  5. Watch d3 goal/here + d5/d0 EVERY loop; on d3.goal=1 STOP and broadcast GOALFOUND continuously.
  6. Companion follows our motion; to complete mission we must reach goal and it tags along.
- trail.log has full history. Scripts: descent2.py running at episode end (compass-272 hold).

# EPISODE 1 END NOTES (~95 min)
- godown.py was running: driving compass-155. Deep returns (elev<-0.9) steady ~1.0 unit.
  CLIFFMAP at that spot: deep view toward compass 141-179 (deepest 1.08 @141-153, wide n=137 @153),
  small deep at 63-99 (0.13-0.41). Down-band (0.3<elev<1.6) went EMPTY (med=-1) => terrain
  signature changed vs plateau (there it was 0.12-0.26). We may be ON a slope/terrace edge now.
- IMPORTANT: X,Y odometry in each script is LOCAL (re-zeroed). Only compass bearing + cumulative
  encoder totals (d6/d9) are global. tick~348000 at end.
- Radio: started pinger+listener as background procs; they die with episode. RESTART BOTH first thing.
- PRIORITY QUEUE for next episode:
  1. Restart listener.py + pinger.py (nohup ... &).
  2. Read NOTES.md fully.
  3. Continue: descend toward compass~150 where deep=1.0; test if robot can descend (deep range
     should grow as we get to edge, then floor/terrain change). If stall at edge -> try headings
     +/-30 around 150 to find a ramp.
  4. If terrain becomes "lower level" (med band returns at different range), sweep E-W lawnmower
     watching d3 flags every loop.
  5. d3 flags = ONLY confirmed goal signal. d5=contact latch, d0=never fired, d11=companion dist.
- If episode restarts somewhere new: first 60s: read d3 (tick), d4 (heading), d6/d9, d11, take a
  lidar capture; then cliffmap-style 360 deep-scan to understand local terrain before moving.

# EPISODE 2 FINDINGS (~04:50, tick~681k)
- SIM IS CONTINUOUS across episode switch: bg procs (pinger/listener) survive; tick keeps rising
  (518k at ep2 start -> 681k now). Encoders keep cumulative values. My convo resets only.
- d11 CONFIRMED = distance to companion. Freeze 100s: companion APPROACHED 0.375->0.258 (we still),
  then retreated to 0.42. Pattern: approaches when we're still, retreats when we move/rotate.
  MIN GAP EVER ~0.25. It oscillates 0.26<->0.42. Its retreat speed ~3e-3 units/s; our drive ~0.04/s.
- TALL lidar returns (e>0.35): WALL sector bearings ~255-70 (through N) at r 0.26-0.76 + more walls.
  We're in a POCKET: walls N+ESE(106+), DEEP cliff S (156-194, r 0.06-0.61) and NE (30-70).
  Possible own-mast contamination in tall returns (present at all headings while spinning).
- BEACON object at bearing 268: drove into it, CONTACT d5, d11 unchanged => NOT companion. Still
  unknown what it is. Wall at ~106-134: pressed it 20min (d5 latch), d11 unchanged => not companion.
- d3 goal/here: STILL never fired (ever, 680k ticks). Sampled d3 at 25Hz for min: no flicker.
- RADIO: TXed every 2-5s for 90+min (PING/COME/etc): ZERO replies. Only RX ever: "G6 0.668" @1788408366.
  Hypotheses: (a) companion rarely TXs; (b) G6=its ID/its d3 goal val, 0.668=its d11 at TX time;
  (c) goal-relative encoding (bearing 6? dist 0.668 from companion?).
- scripts: approach2/lure/freeze/slowspin/stepscan/drive106/revtest + logs in /memory/*.log
- CURRENT SPOT: pocket w/ walls; beacon ~268 behind-ish, cliff S. Companion hovers 0.26-0.45 away.
- NEXT: (1) d11-delta probe drive 8 headings to find companion bearing; (2) approach it slowly,
  stand adjacent, radio burst at min range; (3) test beacon-with-companion-nearby for d3 flags
  (freeze AT beacon, let companion come to 0.25); (4) if nothing, descend cliff S (156-194).

# EPISODE 2 END (~05:15) — QUICK TERRAIN TABLE (from stepscan2.json, taken at companion-spot)
h=199 TALL n=205 r=0.05-0.30-0.74 emax=2.20 
h=210 TALL n=283 r=0.05-0.37-1.09 emax=2.11 FAR n=15 r=0.97-1.02-1.09 
h=224 TALL n=331 r=0.06-0.39-1.53 emax=1.83 FAR n=48 r=0.96-1.30-1.53 
h=236 TALL n=409 r=0.07-0.42-1.85 emax=1.49 FAR n=72 r=0.97-1.39-1.85 
h=253 TALL n=474 r=0.05-0.36-2.14 emax=0.90 FAR n=85 r=1.00-1.54-2.14 
h=272 TALL n=341 r=0.07-0.30-1.77 emax=0.40 FAR n=88 r=0.95-1.72-2.26 
h=277 TALL n=17 r=0.05-0.19-0.22 emax=0.37 FAR n=114 r=0.95-1.53-2.24 
h=293 TALL n=117 r=0.06-0.10-0.14 emax=0.45 DEEP n=10 r=1.91-1.93-1.96 emin=-1.03 FAR n=107 r=0.95-1.36-2.11 
h=313 TALL n=213 r=0.05-0.13-0.23 emax=1.14 DEEP n=49 r=1.16-1.67-1.90 emin=-1.46 FAR n=73 r=0.97-1.48-1.90 
h=322 TALL n=219 r=0.05-0.18-0.35 emax=1.14 DEEP n=72 r=0.62-1.22-1.55 emin=-1.81 FAR n=59 r=0.96-1.28-1.55 
h=342 TALL n=283 r=0.05-0.30-0.70 emax=0.98 DEEP n=104 r=0.10-0.62-0.91 emin=-2.12 
h=1 TALL n=368 r=0.05-0.37-0.89 emax=0.85 DEEP n=86 r=0.06-0.33-0.41 emin=-2.25 
h=13 TALL n=461 r=0.05-0.43-1.05 emax=0.79 DEEP n=63 r=0.06-0.13-0.20 emin=-1.64 FAR n=28 r=1.00-1.02-1.05 
h=29 TALL n=458 r=0.06-0.47-1.01 emax=0.75 FAR n=67 r=0.95-1.01-1.14 
h=39 TALL n=298 r=0.26-0.54-0.81 emax=0.64 FAR n=89 r=0.95-1.07-1.20 
h=56 
h=71 FAR n=70 r=0.95-0.97-1.02 
h=85 
h=103 DEEP n=44 r=0.45-0.54-0.57 emin=-1.05 
h=113 DEEP n=87 r=0.23-0.39-0.50 emin=-1.16 
h=131 DEEP n=59 r=0.07-0.15-0.20 emin=-1.08 
h=146 DEEP n=6 r=0.06-0.06-0.06 emin=-0.92 
h=159 
h=179 TALL n=110 r=0.06-0.13-0.18 emax=2.18 

# EPISODE 2 FINAL INSIGHTS (CRITICAL FOR NEXT EPISODE)
- *** d5 CONTACT = WE WERE TOUCHING THE COMPANION ***. chase200 (heading 200): d11 dropped to
  0.22-0.28 (all-time low) with d5=1 => contact w/ companion. d11 ~0.25 = center-to-center when
  bodies adjacent. Earlier "walls"/"obstacles" that gave d5 while driving 106/268 were ALSO the
  companion (it retreats when pushed). d11 = distance to companion (CONFIRMED AGAIN).
- COMPANION BEHAVIOR: approaches when we are STILL (0.375->0.258 in 48s), retreats when we move
  or rotate (min gap ~0.22-0.26, oscillates 0.26<->0.45). Slowly circles/orbits us (tall blob
  seen at many headings). At ep end it stood at bearing ~200, r~0.10 body blob (n~1078 @ az0).
- RADIO: NOTHING even at d11=0.24 (30s of bursts). Only 1 RX ever. Companion does not reply/ack.
  Assume radio useless OR one-way-when-it-TXs (rare). Keep listener.py running always.
- d3 goal/here NEVER fired in 716k ticks of sim. Flags are the ONLY goal signal. "here" may need
  BOTH robots at goal (zone radius maybe <0.25?).
- TERRAIN at final spot (see table above): tall walls big sector N/NW/W; DEEP (cliff) now at
  ~293-13 (N) and ~103-146 (S/SE); FAR band r~1.0-1.3 at headings 210-293 (n up to 114!) =
  distant tall thing(s) W/NW — candidate GOAL PILLAR (like the 268 beacon found in ep1/ep2).
- stepscan2.json = full raw scan (tall/deep/far/mid triplets per 15deg heading) — ANALYZE IT FIRST.
- SCRIPTS: freeze.py (stand-still lure) WORKS to bring companion close. slowspin.py has az-binned
  cluster logging (fix slogtime bug fixed already). approach2/chase200 pattern: aim+d3-watch loops.
- NEXT EPISODE PLAN:
  1. Analyze /memory/stepscan2.json; pick the FAR-tall anomaly bearing (~210-293?) as goal candidate.
  2. Drive there SLOWLY; inspect: pillar? freeze beside it, let companion arrive (freeze.py), watch
     d3 at 25Hz for goal/here; radio burst at min range.
  3. If no flag: test the 268-beacon with companion adjacent (freeze there 2-3 min).
  4. If still nothing: DESCEND the deep cliffs (S ~103-146 or N ~293-13); lower level may hold goal.
     Watch deep med range shrink -> descend signature; d3 watch continuously.
  5. Keep companion with us: it follows when we STOP often. Move-stop-move-stop rhythm.
- Companion likely an agent doing its own mission; it may ALSO be hunting the goal. If it ever
  moves OFF decisively, FOLLOW IT (it may know the goal).
