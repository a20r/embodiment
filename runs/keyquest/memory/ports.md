# Robot port map (learned by experiment)
- /dev/robot/d0: read, compass heading degrees (noisy +-3)
- /dev/robot/d1: read, bump sensor (1 when hitting wall)
- /dev/robot/d2: read, LEFT wheel encoder (counts up when d3 driven)
- /dev/robot/d3: write, LEFT wheel speed (e.g. -10..10; 5 => ~25 counts/s)
- /dev/robot/d4: read, ~0.02-0.04 constant even when moving; unknown (battery? speed?)
- /dev/robot/d5: write, RIGHT wheel speed
- /dev/robot/d6: read, RIGHT wheel encoder
- /dev/robot/d7: read, lidar 16 beams CSV, range ~0..1.0 (units?), -1.000 = dropout/noise
- /dev/robot/d8: read, 0 so far (another bump? goal sensor?)
- /dev/robot/d9: read, "tick=N goal=0" - goal flag presumably ->1 at goal
- d3=+,d5=- turns heading DOWN (clockwise?); d3=-,d5=+ heading UP
- Driving fwd (both +5): beams 0-3,14,15 shrank => front is around beam ~1
- Pipes: reads block if no writer; use timeouts. Writes persist (motor keeps speed).
## Corrections
- Encoder swap: d6 counts wheel driven by d3(left cmd); d2 counts wheel driven by d5(right cmd).
- Reads sometimes return empty string -> always retry.
- Beam i world bearing = heading + i*22.5 deg. drive(+,-) decreases heading, drive(-,+) increases.
- Speed cmd 5-6 => ~25-30 encoder counts/s. Rotation (5,-5) ~ -9 deg/s.
- Lidar max ~1.0 (units unknown, corridors ~0.2-0.9 wide).
- Controller: /bot/src/wander.py (reactive: forward if front>0.22 else turn to widest beam). Trace -> /memory/trace.csv (t,x,y,hdg,front) in encoder counts.
## Strategy notes (ep1)
- d4 = scalar signal, hypothesized goal proximity. Spawn ~0.02-0.04, elsewhere 0.10-0.16. NOISY (+-0.01).
- seek2.py: obstacle avoid + gradient-fit chemotaxis + return-to-best. Points logged /memory/points.csv (x,y,s in encoder counts, frame resets each process start!).
- BEWARE: pgrep/pkill -f patterns match your own bash -c command -> self-kill. Kill in separate tool call with pattern not in start command.
## Goal localization (ep1, follow/goto frame = pose.json frame)
- d4 signal fits s = k*r^-1.6 around a point source, r2=0.99
- Estimated goal position in pose.json frame: see below (refined)
- Scale: ~760 encoder counts per lidar unit
## EPISODE 1 SUMMARY / NEXT STEPS (IMPORTANT)
- GOAL NOT REACHED this episode. Best signal s(d4) max ~0.78.
- WINNING ROUTE (fast): run right-hand wall follower (/bot copy in /memory/src_backup) from spawn.
  It reaches s~0.42 in ~2min, keep following ~25+ min total: s dips to 0.06 then rises:
  0.10 -> 0.31 -> 0.78 as it rounds NE. Peak s=0.78. Goal compound walls near there.
- At peak area: NE directions blocked by wall ~150-250 counts. Increases seen bearing ~38deg.
  Entrance to goal compound NOT FOUND yet. Try: continue right-follow PAST the 0.78 peak
  (we aborted at 0.65 dropping) - the follower may round the compound and find entrance.
  Or explore north wall gap: at peak room, north walls at 0.28-0.34 units.
- s field fits s = k*r^-1.5 roughly (k~26000 counts-units). s=0.78 => r~1200 counts (crude).
- hill.py (run-and-tumble) works in open areas but walls defeat it; followL.py spins in open areas (buggy arc-left).
- Controllers in /memory/src_backup/: lib.py (robust IO), follow.py (right wall follower + pose.json + points.csv logging), goto.py (target from /bot/target.json + wall-follow escape), hill.py, east.py, nav.py (mapper, unfinished).
- Episode used all 120min mostly on gradient flailing. NEXT TIME: immediately start follow.py,
  poll d9 for goal=1, and let it run 30+ min before interfering. Log s; near peak switch to
  tracing the compound wall CLOSELY (right-hand) until goal flag trips.
## FINAL ep1 observations (frame = last pose.json, drifted!)
- Right-follow near end: s=0.70 at (1502,4426), dropped to 0.38 at (1673,2989) going south.
- s=0.78 peak recorded near (2034,3693); room at (2060,3600) s=0.70-0.77 walled NE.
- The compound wall seems to enclose goal roughly NE of that room; follower circles it without
  entering => entrance may be narrow (<0.2 lidar units?) or requires different wall (island).
- IDEA for next ep: at peak room, do a FINE perimeter scan: hug the NE wall at ~0.15 dist,
  full circuit of the compound (both hands), checking d9 goal flag continuously; also consider
  the follower's min-front threshold 0.22 may be too conservative for a narrow entrance -
  try threshold 0.15 and slower speed near the compound.
- Also: check d8 (always 0 so far) again near goal - may flip near entrance.
- Spawn area s~0.02-0.04. Signal d4 is THE beacon; median-filter 8+ samples (noise +-0.01).
## ep1 FINAL RESULT: goal NOT reached. smax=0.874 (d4 median).
## Fastest reproduction of best approach (do this immediately next episode):
1) Start follow.py (right-hand follower, thresholds: front<0.22 turn left; right band 0.16-0.32).
   Watch s=d4. It goes: spawn 0.03 -> 0.42 (2min) -> drops to 0.06 -> rises again as it
   circles a big loop; after ~20-25min it reaches an ISLAND-like block; s cycles 0.38..0.78
   circling it (counterclockwise loop: S face low 0.38, N/NE faces high 0.7-0.78, period ~6-7min).
2) When s>0.7: stop follower. Hill-climb (hill.py, leg 160) to plateau ~0.76.
3) Tight LEFT-hug (hug.py: front<0.14, left band 0.10-0.22, speed 4-6) heading EAST along the
   north wall: s climbs to 0.874 then wall bends away (s crashes). ENTRANCE NOT FOUND on that face.
## UNTRIED IDEAS (next ep):
- At smax point (0.874), stop and precisely scan: the goal may need approach from NORTH side of
  that wall: the region north of the island (y larger) was reached earlier via north-wall corridor
  (follower passed (1359,4433)-(2279,4045) with s up to 0.78). Try hugging SOUTH side of THAT
  north region, i.e., the island's north face, at 0.10-0.15 dist, both directions, front<0.12.
- Maybe entrance is on island NE/E corner between the two 0.78 peaks - do a full tight circuit
  of the island (pick one hand, keep island side consistently on that hand, 10+ min).
- Check d8 near smax (was 0 everywhere so far). Also try writing to d8? (never tried WRITING d8/d1...).
- Consider lowering front threshold to 0.10 + speed 3 during circuits: entrance may be very narrow.
- Verify goal flag: d9 'goal=' parsing OK. Maybe goal requires stopping ON it, flag then =1.
- d1/d8 are read-only (writes block/timeout). Confirmed ep1 end.
## ep1 TRUE FINAL (post-episode notes)
- smax ever = 0.874 (hug left, east along corridor N wall); peak.py trace hit 0.838.
- CRITICAL FINDING: tracing the LEFT wall at the peak (tried TWICE) always leads AWAY (s crashes
  0.84 -> 0.2 within ~90s). So the goal block is likely the OTHER (south/right) wall at the peak,
  which we NEVER traced. peakR.py (right-hand tight trace) was just written+started but episode ended.
## EXACT RECIPE FOR NEXT EPISODE (do in order, no experimenting):
1) cp -r /memory/src_backup /bot/src (contains lib,follow,peak,peakR,hill,goto,hug...).
2) Run follow.py (right-hand follower). Poll d4: wait until s>0.70 (takes 10-25 min; s dips
   to 0.06 mid-route - DO NOT INTERVENE). Keep checking d9 for goal=1 the whole time.
3) When s>0.70: kill follow, run peakR.py = tight RIGHT-hand contour trace (front<0.13,
   right band 0.09-0.20, arc right when right>0.30) - this traces the SOUTH/right wall
   at the peak corridor, the one never explored. Watch /bot/peak.log smax.
4) If s crashes <0.3 for >2min: kill, rerun follow.py till s>0.7, then try peak.py (left trace)
   but at the FIRST s-drop turn 180 and trace back with the other hand.
5) Entrance may be <0.3 units wide; if traces keep failing, try trace with front<0.10, speed 3-4,
   and arc threshold 0.25.
- Signal ref: spawn 0.03; corridor peak 0.87; goal flag d9 still never tripped (max seen s=0.874).
- Loop period circling the island ~6-7min at speed 6. Odometry drifts ~300+ counts per loop; use s only.
