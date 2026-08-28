# Robot notes (persist across episodes)
Ports /dev/robot/ (FIFOs, one line per open):
- d0: READ 16-beam lidar, comma floats, -1.0 = no return(?). beam spacing likely 22.5deg
- d1: WRITE left motor speed (e.g. 0..1?)
- d2: READ "tick=N goal=0"  <- goal flag!
- d3: WRITE right motor speed
- d4: READ 0 (unknown)
- d5: READ empty line (maybe RX of transceiver)
- d6: WRITE (maybe TX of transceiver)
- d7: READ odometer counter (increases while driving, ~5.5/s at speed 1)
- d8: READ 0/-1/1 jitter (unknown)
- d9: READ 0, WRITE blocks (maybe RSSI of beacon?)
- d10: READ compass heading deg (noisy +-3)
Driving d1=1,d3=1 -> forward. d1=1 only -> heading increases (CW). d3=1 only -> heading decreases.
Beam 0 = FRONT. Beam i world bearing = (heading + 22.5*i) mod 360.
motors(l>r) => heading DEcreases. turn rate ~0.5 deg/s per unit diff.
speed units: fwd speed s -> ~0.033 lidar-units/s per s (speed 20 = 0.67 u/s). No clamp <=100.
odo (d7): ~156 counts per lidar-unit.
ctl.py: turn_to(hdg), face_beam(i). Corridors ~0.35-0.5 u wide. Maze axis ~aligned to compass (h~88 down a corridor).
goal flag in d2. d4,d8 ~0, d9=0, d5 empty so far.
*** d10 is NOT compass: it is BEARING TO GOAL relative to robot front (deg, noisy +-3).
Turning changes it 1:1; translation changes it per geometry. Use Bug algorithm: face d10=0, drive; wall-follow around obstacles.
Triangulation at one point suggested goal ~1.8 lidar-units away from corridor end area.
d9=0,d4=0,d8 jitter -1/0/1: still unknown (d8 maybe sign of turn?).
Findings mid-episode:
- Env seems to be rings/spiral around goal at center. Corridor width ~0.35u. Pockets scanned: only opening tangential (+90 from goal dir), 2.4u long.
- d7 odo may reset/wrap sometimes; approx 156 counts/lidar-unit.
- Right-wall follow (side=4) got stuck orbiting an island (~1.6u circumference). Left-wall follow (side=12) travels far but likely loops the boundary.
- Transceiver: sent hello/PING/open/etc on d6, no reply on d5/d9 anywhere yet. Keep background RX logger running.
- pkill NOTE: use pattern like "wf.p[y]" so you don't kill your own shell.
- Strategy in progress: follow wall on GOAL side; when goal-direction beam opens >0.5, dive toward goal; repeat.
More findings:
- Physics: driving into wall SLIDES along it; odo counts commanded wheel travel regardless.
- Pushing into goal-side wall does NOT trigger goal.
- d10 works as exact gyro during in-place turns (bearing changes 1:1 with rotation).
- Hypothesis: braided grid maze ~7x7, cell~0.5u, wall-follow loops (~13u perimeter circuit ~22s @spd18).
- Goal dist from edge ~1.8u (triangulated). Goal flag d2 still 0 everywhere incl pushing.
- Plan: grid DFS: 90-deg turns via d10 delta, cell steps ~0.5u with lidar centering, visited set, prefer dirs reducing |bearing|.
Session ~90min findings:
- Inner block around goal is SEALED from the ring: 5 laps continuous scanning, goalward beams never >0.41 within +-40deg.
- Radio d6 TX: no response on d5 (d5 emits instant EMPTY lines always). d9 always '0' (write blocks). d4 always 0.
- Goal-beam relation: facing beam i = turn(+22.5i) where turn(+x) increases d10 reading by x. Goal beam gb=(16-round(b/22.5))%16.
- Pushing slid us within ~0.3 of goal (bearing swept fast) - no goal flag. Trigger radius small.
- Next: build global map via dead-reckoning EKF (d10 gyro for turns, odo/156 for dist, bearing innovation to fix heading).
- Global dead-reckoning map failed (odo slides -> huge drift). D measured 2.6 at one spot: maze bigger than initially thought.
- d8 = random noise -1/0/1. Ignore.
- Best coverage tool: src/walker.py (random junction walk w/ goal bias) + background goal watcher writing /memory/goalhits.log.
- Code: /bot/src/{rob,ctl,walker,orbit2,mapper,seek2}.py. Watchers: d2 goal watcher, d4 watcher (bash loops).
- If starting fresh: check /memory/goalhits.log first!
LATE-EPISODE STATUS:
- Pressure-crawl around block: multiple bearing-laps, no slit, no goal. Block likely sealed OR d10 beacon is NOT the goal (decoy/infrastructure!). Goal flag d2 never fired anywhere.
- Running final.py: fast unbiased random walker + radio sniff (TX hello on d6 every ~8 junctions, check d5/d9).
- IDEAS NOT YET TRIED: negative motor values for reverse mechanisms; TX numeric/structured messages; exploring outward regions systematically (boundary may have exits); pushing specific wall segments; d5 may emit blank lines = in-range-but-empty vs out-of-range?
- Perf notes: walker ~4-5 junctions/min with align+look overhead.
RADIO BREAKTHROUGH (partial): TX 'hello' on d6 then rapid-reading d5 returned chars 'p' (twice) at one spot while roaming (near ring, b~72 shortly after the b~25/R~0.40 ring corner). Could NOT reproduce standing on ring or corridor. Responder is SHORT range, responds to TX. hunt2.py = random walk + TX probe each junction + interrogation on hit. KEEP RUNNING IT.

=== EPISODE 1 FINAL SUMMARY / PLAYBOOK FOR NEXT EPISODE ===
ENVIRONMENT: 2D maze, corridors ~0.35-0.45u wide (lidar units). Central-ish "block" region sealed to lidar.
PORTS: d0 lidar16(beam0=front, beam i at +22.5i in reading-space); d1 Lmotor W; d2 "tick,goal" R; d3 Rmotor W;
 d4 always0; d5 RX line (usually blank, ONCE gave 'p' chars); d6 TX W; d7 odo (~156/u, counts commanded, slides!);
 d8 noise; d9 always'0' (W blocks); d10 = BEARING TO BEACON (relative to front, noisy ±3).
MOTION: motors(l>r) DEcreases d10 reading. turn rate ~0.5deg/s/unit. speed ~0.033u/s/unit, max tested 100.
 Walls SLIDE (no stall): pushing angled into wall translates you along it.
KEY GEOMETRY: beacon (d10 target) sits inside sealed block. Ring circuit around it: corridors 2.4u & 2.0u,
 corner signature b~25 F~0.3 R~0.4. Pressure-crawl laps found NO opening; goalward beams never >0.41 in 5 laps.
 => beacon may be DECOY or door is hidden/opens by other means. Goal flag (d2) NEVER fired anywhere.
RADIO: once, TX 'hello' + immediate single read of d5 gave 'p' (2 sniffs in a row) while roaming near ring after
 that corner. Never reproduced by stationary probing along ring/corridors. Suspect short-range peer somewhere
 in maze; respond window small; maybe need exact spot, or it broadcasts rarely.
TOOLS in /memory/*.py (copy to /bot/src): rob.py (I/O), walker.py (align/turn/step/look randwalk),
 orbit2.py (ring orbit + enter attempts), crawl.py (pressure crawl), hunt2.py (walk + TX probe + interrogate).
NEXT STEPS (ranked):
1. Run hunt2.py LONG (probe every junction, also probe MID-STEP continuously; add d5 read every 0.1s during motion).
2. Try TX messages: numbers, 'p', 'pong', single chars; read d5 fast WITHOUT sleep after TX.
3. Explore OUTWARD: boundary may have exits; unbiased walk with d5 sniffing everywhere.
4. Try negative motor speeds (reverse) & weird values; maybe a hidden actuator behavior.
5. Consider that goal may be a specific CELL with small trigger radius anywhere in maze; systematic Tremaux with
 (D,phi) localization: measure D by bearing-rate trick at junctions (D = ds*sin(b)/dtheta_bearing).
GOTCHAS: pkill -f PATTERN kills your own shell if pattern text appears in it — run pkill alone with [x] trick.
 Commands killed at 60s; poll background jobs. Keep goal-watcher: while true; read d2; log goal=1 to /memory/goalhits.log.
END-OF-EPISODE-1 STATE: hunt2.py roaming+probing (no radio hits in ~12min of probing; the single 'p' event remains unexplained
- possibly noise or ultra-short-range peer). Background: goal watcher -> /memory/goalhits.log, d5 sniffer -> /memory/rxhits.log.
Coverage worry: walker may still be confined to the ring region near beacon; consider forcing OUTWARD exploration
(e.g., keep |b|~180, i.e., travel with beacon behind) to find unexplored maze regions. ALSO: try driving with
negative speeds (reverse) and speeds >100. And try holding still at MANY spots TXing various strings while reading d5 with zero delay.
FINAL: away.py (outward-biased walk + TX probe) also no radio/goal hits by episode end; bearing still ~76 at last check,
suggesting exploration remains stuck near beacon ring — outward bias may not overcome geometry, or maze really is just
ring+block+pockets and the trick is unknown. STRONG candidates for next episode (in order):
1. Precise systematic Tremaux with (D,phi) beacon-polar localization (measure D at each junction via bearing-rate while
   stepping: D = ds*sin(b)/delta_b_rad; phi += ds*sin(b)/D). Map every cell, find unexplored edges.
2. Radio protocol fuzzing from MANY standing spots: for each of ~30 distinct cells: send each of
   ['hello','ping','open','p','pong','marco','0','1','enter','exit','key','unlock'] with rapid zero-delay d5 reads.
3. Reverse driving (negative speeds) through suspicious walls (start-slot bottom; b~25 corner), and very slow creep
   (spd 4) hugging walls to catch sub-beam-width slits.
4. Check d9 again in new places (was '0' everywhere) - maybe it's a keycount/RSSI that changes after radio success.
Scripts all in /memory. Start by: cp /memory/*.py /bot/src/; start goal-watcher and d5-sniffer (see notes above).
=== RADIO SOLVED-ISH ===
/tmp/rx.log (ep1) shows d5 delivered 'ping' lines ~1:1 with my 2s 'hello' TX pinger, in 32 windows over a 1000s era,
whenever robot was on the seek2 pocket circuit. Responses CEASED exactly when my pinger process died (pkill suicide).
The single 'p' was a SPLIT line (two concurrent d5 readers each stealing chars of 'ping').
NOW: exact reproduction (seek2 + 2s hello pinger + d5 logger) gets NOTHING => responder deactivated/moved/duty-cycled.
STRATEGY: keep hello-pinger + d5 logger running at ALL times (/tmp/rx2.log); if 'ping' reappears, STOP and converse
(try: ping->?, marco, who, help, open, goal). Do not run competing d5 readers (they split lines!).

=== EPISODE 2 FINAL SUMMARY (read this first next time) ===
GOAL NOT REACHED. d2 goal=0 always. Maze: annulus corridors around a SEALED central block containing the d10 beacon.
Coverage: ~50 cells via beacon-polar Tremaux (explore.py); D ranged 1.3-2.4; no opening into block ever seen
(lidar -1 = ~1% dropout noise, NOT over-range). Long pushes slide around block, never catch a door. Both-handed
pressure sweeps (sweep.py REF=30/330) found nothing. Reverse speeds work (negative values OK).
RADIO FACTS (key mystery):
- d5=RX stream: sim writes a line per tick, blank when nothing. d6=TX (sim consumes writes). NEVER run two d5
  readers at once (lines get split; that caused the 'p' fragments of 'ping').
- Episode-1 era ~tick 100-200k: d5 delivered 'ping' ~every 2s (1:1 with my 2s 'hello' pinger) in 32 windows over
  1000s while robot looped the pocket circuit next to the block (seek2.py path). Log preserved: /memory/rx_ep1.log.
- Never reproduced since (same setup, same circuit, hours later): responder seems deactivated/duty-cycled/exhausted,
  OR it responds only in an early tick window (test EARLY next episode!).
TOP PRIORITY NEXT EPISODE (fresh start, tick resets?):
1. IMMEDIATELY start: 2s 'hello' pinger on d6 + SINGLE d5 logger. If 'ping' arrives, STOP, converse: send
   ping/pong/marco/who/help/open/goal/follow and log replies. Maybe response window exists only early.
2. Keep goal-watcher on d2 (log goal=1 to /memory/goalhits.log).
3. If radio silent, run explore.py (Tremaux) but push D<1.2 attempts: every block-adjacent cell, verify with
   slow creep + reverse probing for sub-beam slits.
4. Consider timing/tick gates: watch block walls (F readings) over minutes from a pocket for periodic opening.
All tools in /memory/*.py; calibration + port map + gotchas in earlier sections of this file.
