# Robot notes (episode 1 findings)
Ports /dev/robot/ (FIFOs, ASCII lines):
- d0  READ: 16 lidar rays, CSV floats, max range 3.0, -1.0=dropout noise. 22.5 deg/ray.
- d2  READ: "tick=N goal=0" -> goal flag! ticks ~1.3/s?? (ticks advance fast: ~1777 at start)
- d4  READ: single int, 0 so far
- d5  READ: blank lines so far - likely radio RX (transceiver pair)
- d7  READ: 0 (was blocked until first write happened somewhere)
- d8  READ: int, was 1 initially then 0 - maybe bump/contact?
- d9  READ: 0
- d10 READ: heading deg, noisy +-2
- d1  WRITE: left wheel?? positive -> heading DECREASES ~1 deg/s per unit. persists.
- d6  WRITE: right wheel: positive -> heading INCREASES. d1=d6 -> ~straight
- d3  WRITE: no motion effect - likely radio TX
Reads: each open() of fifo yields ONE line (snapshot). Poll by reopening.
Robot started facing wall (ray0 ~0.09 = front). Blocked when driving fwd into wall.
More findings:
- d7 = left encoder counts, d8 = right encoder. ~5 counts per unit speed per sec.
- ~700 encoder counts per lidar-distance-unit. speed cmd 20 -> ~0.14 units/s.
- Forward = ray0 direction. ray i at heading + 22.5*i deg (CCW). ray4=left, ray12=right.
- wheels: d1=left, d6=right. equal positive -> forward, slight rightward drift (left counts more).
- d3 TX tried hello/ping/etc: no reply on d5 yet (short-range - maybe only near beacon).
- Explorer: /bot/src/explore.py right-wall follower, logs to /memory/trail.csv
  (t,x,y,hdg,16 rays,tickgoal,d4,d9,radio)
RADIO findings:
- d3 TX any text -> d5 RX numeric reply (~per ping, buffered in FIFO).
- Replies only when "in range". Value time-varies at fixed pos => SOURCE IS MOBILE.
- Values: ~55 max (seen when very close), down to -50 (far). Quantized ints 8/-8/0/55
  interleaved with decimals. Higher = closer (probably ~55 - k*dist, k~19?).
- Robot max speed: wheel cmd scales linearly. cmd200 = 1.48 units/s. cmd60=0.44.
- Goal flag d2 still 0 even at v=55 pass. Maybe must touch/stay near mobile source?
- Map: corridors, explored bbox x[-12,1.4] y[-5.9,4.4]. Hot zone seen near x=-11.6,y=0.5
  (odom frame of trail2).
- chase.py = chemotaxis chaser. sweep.py, local.py, goto.py, runner2.py in /bot/src.
=== EPISODE 1 SUMMARY (most important) ===
HW protocol summary:
- FIFO read = one snapshot line per open(). d0 lidar16(max3,-1=noise), d2 "tick=N goal=0/1",
  d7/d8 = L/R wheel encoders (700 counts/unit), d10 heading deg (noisy +-2, CCW positive).
- d1/d6 = L/R wheel speed cmds, persist until changed. dist/s = cmd*0.0074 (cmd200=1.48u/s).
  positive d1 turns CW (heading down), positive d6 CCW. no cap seen up to 200.
- ray i points at (heading + 22.5*i) deg. ray0=front, ray4=left, ray12=right.
- d3 TX -> d5 RX: reply arrives ~0.1-0.3s after ANY tx line, ONLY if "in range" of a MOBILE
  source. value: higher=closer, ~55 max seen, -50s far, exact ints 8/-8/0/55 common,
  time-varying at fixed spot => source moves ~0.5-1 u/s. Contact windows are minutes apart.
- d4,d9: always 0. write-blocked. unknown.
- Goal NEVER triggered in ep1 despite v=55 contact and lots of coverage. bbox explored
  x[-12..1.4] y[-5.9..4.4] (trail2 frame, start pos = (0,0) mid-east vertical corridor).
Maze: corridors ~0.6 wide, bounded region approx 13.5 x 10.5.
Tools in /bot/src: lib.py (read/write/lidar/heading/goal/wheels/turn_by), runner2.py
 (novelty explorer), goto.py X Y (waypoint), hunt.py (explore+chase radio, stuck recovery,
 logs /memory/hot.csv + hunt_trail.csv), plot2.py (ascii map from trail2.csv).
IDEAS NOT YET TRIED: bump into source deliberately; send commands to source when in range
 (patched hunt sends stop/halt/freeze/come here - unverified effect); listen on d5 passively
 for long periods w/o TX (does it broadcast unprompted?); check if goal flag needs STAYING
 still somewhere; visit exact map center; unexplored NW & SW corners of bbox.
=== END OF EPISODE 1 STATUS & NEXT STEPS ===
- Goal never found. goal flag in d2 stayed 0 for entire ~2h despite covering most corridors
  and passing the mobile radio source at point-blank (v=55).
- Leaving west.py running (west-side N-S patrol + chase on radio contact; writes
  /memory/GOAL.txt if d2 goal flips).
NEXT EPISODE PLAN (do in this order, skip rediscovery):
1. Read this file. All port semantics are proven; trust them.
2. FIRST try long PASSIVE listen on d5 (no TX) ~60s while driving: does source broadcast
   unprompted? (never tested cleanly: every test TXed first)
3. Try TX message variations while IN CONTACT and compare reply patterns:
   maybe replies ARE structured (e.g. "ping"->RSSI, other cmds -> other fields).
   In ep1, msg content seemed ignored, but only tested casually. Try "position", "goal",
   "x", "y", "bearing", numbers.
4. Chase source aggressively at cmd 150-200 (robot does 1.5 u/s, source ~1 u/s max obs).
   Goal may = touching it. Watch d4/d9 for change during near contact (v>=50).
5. If not, do EXHAUSTIVE coverage: grid-visit every 0.5-cell (runner2-style visits work;
   add coverage completion check). Map bbox ~x[-12,1.4] y[-5.9,4.4] rel. start-corridor.
   Start position each episode may differ!
6. Radio contact zones (trail2 frame ep1): all around (-11.5..-12.2, -1.5..+1.5).
   Contact windows lasted 1-3 min, gaps of 10+ min. Patience or ambush there.
CAVEATS: pkill -f patterns must not match your own bash cmdline (use [b]racket trick).
Two readers on same FIFO race - stop background scripts before manual port reads.
scripts saved to /memory/src (lib.py + explorers). west.py left running.
FINAL ep1 note: ended with west.py patrolling, still goal=0, no further radio contact after
~40min (source active only in first ~hour?). Consider that the source's schedule may be
time-based (active early in episode!) - next episode, chase it IMMEDIATELY at max speed
(cmd 150+) from the start, while goal-checking; that is the strongest untested lead.
LATE ep1 BREAKTHROUGH: world is MUCH bigger than first bbox! From NE vertical corridor
(~12 units long, found by pushing compass-North) reached a ~26-unit DIAGONAL highway
going SW, then more corridors/rooms around cover-frame (-15..-21,-16..-20). No goal yet,
radio silent for >1.5h (hot.csv stuck at 70 entries). Strategy that worked: pure-novelty
explorer (cover.py, patched, visits-grid penalty) at spd 120/70. KEEP EXPLORING OUTWARD -
prior 'bounded maze' assumption was WRONG (3.0 lidar range hid long corridors).
=== FINAL STATE (end of my context) ===
cover.py (patched pure-novelty explorer) left RUNNING; it writes /memory/GOAL.txt and
prints GOAL!! if d2 goal flips. It resets odom frame each restart; cover_trail.csv mixes
frames (split by lines where x,y jump to 0,0).
World structure known so far: original corridor maze (~13x10) in SE; 12-unit N corridor
from its NE area leads to northern zone; from there a ~26-unit diagonal highway runs SW to
a big room complex (~8x8) with an E-W corridor at its south edge extending further W.
World is >30 units across. Goal not found after ~3.5h. Radio source silent since early.
NEXT SELF: 1) restart cover.py-style explorer IMMEDIATELY (it's in /memory/src/cover.py -
copy to /bot/src with lib.py); restart it every ~10 min to reset visits grid (drift makes
old visits stale). 2) chase radio source at episode start (it was active early). 3) watch
d2 goal flag every loop - that IS the success signal. 4) consider goal needs: maybe a
specific FAR corner of the big world; push frontiers systematically (compass-extremes).
=== EPISODE 1 FINAL (powered down, goal NEVER found) ===
Total ~4h. Coverage: SE corridor maze (13x10), N corridor (12u), NW zone, 26u diagonal
highway SW, SW room complex + E-W corridor, world >30u across, STILL had open frontiers.
Radio source only active in first ~1h (values -50..55, higher=closer, mobile ~1u/s).
d2 goal flag stayed 0 the whole time.
EPISODE 2 PRIORITY LIST (do immediately, in order):
1. cp /memory/src/lib.py /memory/src/cover.py /bot/src/; start cover.py FIRST THING
   (nohup python3 cover.py > log 2>&1 &). It handles everything incl. GOAL detection
   (writes /memory/GOAL.txt). Restart it every ~10min (fresh visits grid; also recovers
   from any hang). NOTE: after killing it, ALWAYS write 0 to d1 and d6 (cmds persist!).
2. While it runs, ping radio hard EARLY (source active early!): if contact (d5 replies),
   chase v uphill at cmd 150-200; try to physically TOUCH source; watch d4/d9/d2.
3. Log bbox of cover_trail.csv; when bbox stops growing 5+ min, plot map (plotd.py style,
   last ~900 lines), identify frontier gaps, drive there via compass-target bias.
4. Untested ideas: touch/bump source; TX structured cmds to it while in contact;
   goal may need standing STILL at a spot for N seconds - try pausing at distinctive spots
   (room centers, dead ends); check d4/d9 semantics (never nonzero in ep1).
PITFALLS: pkill -f self-match (bracket trick [c]); >60s cmds die (exit 124 wastes a round);
FIFO reads = 1 line/open; 2 readers race; sleep<=57 in polls.
