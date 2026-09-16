## ===== EP2 HANDOFF — READ FIRST (written 20:36 UTC, ep2 ~25min in) =====
### NEXT-EPISODE CHECKLIST
1. Read this section. Check /memory/goal_found.json & /memory/goal_from_B.json FIRST (goal flags!).
2. `pgrep -x python3` (ignore defunct). If explore9/watchdog9 alive -> just monitor logs.
   If dead: `for p in d1 d7; do echo 0 > /dev/robot/$p; done` then:
   `cd /bot/src && nohup python3 explore9.py > explore9_out.txt 2>&1 & nohup python3 watchdog9.py > wd9_out.txt 2>&1 &`
3. Watch /bot/src/explore9.log: junction logs every ~10-25s = healthy. ALWAYS verify encoders static after killing scripts.
### WHY LAWNSOWER (explore9)
- ep1+ep2-start: 4h wall-following (explore8), d3 goal/here NEVER nonzero. Wall-follower circles room perimeters -> if goal is in a room INTERIOR it can never be found.
- explore9.py = compass lawnmower: N-S strips (alternating), lateral slide ~0.45m between strips (E pref, flips when blocked), strip max 3.5m, pocket escapes, random flip, 4min stuck timer, radio beacon, goal_mode, homing-if-B-at-goal. Speeds: sp 24/15 cm/s.
- FATAL FLAW FOUND+FIXED: dead-end stubs wedge robot (walls 0.2-0.4m all sides); pivots grind -> escape failed, pose stalled 1min+. FIX escape2(): REVERSE straight 0.35m FIRST (no rotation needed), then rotate to longest lidar ray, drive 1.2m. Patched+syntax-checked; NOT yet field-verified. If log spams "POCKET escape moved=0.00" or encoders stall while wheels commanded -> debug rev()/rotto in stubs.
### STATE AT POWER-DOWN
- explore9 STOPPED (killed for patch), motors stopped, then RELAUNCHED (see checklist step 2 if not running).
- Robot location: tight maze area, compass ~266deg, last lidar: walls 0.2-0.4m around, one 2.7m opening directly behind (beam 8). explore9 pose frame ~(-5,-2) (drive-integrated only, drifts).
- d3 goal=0 here=0 at 20:29. No B radio contact yet in ep2 (rx=0). B was within radio range 20:07:50 at ep1 end ("B PING ... goal=0 here=0").
### OPS NOTES
- bash commands die at 60s (sleep 55 max). Long runs -> nohup + poll.
- pkill -f "explore9.py" KILLS YOUR OWN SHELL (pattern matches own cmdline): use pkill -f "[e]xplore9.py".
- explore9 beacon: "A PING x y h n goal here MOW" ~1.5s; every 12th also "A TIP: goal may be in room interior; do lawnmower strips N-S spacing 0.35m; beacon if GOAL=1/HERE=1" (B may be another agent; teach it!). RX appended to /memory/rx_log.txt. B format: "B PING x y h n goal here", B headings are ABSOLUTE compass.
- If B goal=1/here=1 or "AT GOAL" heard -> auto homing (packet-rate gradient + lidar blob pursuit). If MY d3 flags fire -> auto goal_mode (stop, stay, broadcast "A AT GOAL", lidar every 6th msg). Trust it, but MONITOR.
- Watchdog9: restarts explore9 if dead or log stale >180s.
- TIME: ep2 started ~20:10 UTC, 240min budget -> hard end ~00:10 UTC. Reserve last 20min: if one robot at goal, it must keep broadcasting so the other can home; joint arrival must be within 60s.
### WORLD BELIEFS
- Tight maze, corridors 0.3-0.5m, dead-end stubs exist. True size UNKNOWN (explore5 8x4.5m map untrustworthy; fingerprint novelty unreliable: maxrep=1 after 4h).
- Idea if flags still 0 after ~1.5h mowing: switch strip axis to E-W (compass 90/270) to catch N-S-biased misses; also consider mowing speed up (sp_hi 28).
================================================ (ep1 notes follow) =====

# ROBOT "A" — ESSENTIAL NOTES (update as you learn!)
## Ports (verified)
- d1 LEFT wheel speed cmd (cm/s; use >=8), d7 RIGHT wheel speed cmd. POS l-wheel fwd => heading increases (CW).
- d9 LEFT enc ticks (~544/m), d6 RIGHT enc ticks. d2 lidar 16 beams, idx0=front, CW +22.5deg/idx, meters, -1=bad.
- d3 "tick=N goal=G here=H" (~163 ticks/s). Watch goal/here for goal discovery!
- d4 compass heading (deg CW from N). d0,d5 always 0 so far. d11 fluctuates 0.3-0.6, NOT a simple battery.
- d8 radio TX (write line), d10 radio RX. NO contact with robot B yet (pings every 6s since start).
## Motion
- wheels(l,r) cm/s. spin: 2.7 enc-ticks/deg per wheel. rotto() compass feedback works well.
- drive straight: err=(min(c12,c13)-min(c3,c4)); wheels(sp-corr,sp+corr) centers in corridor.
- Commands PERSIST after script death — always stop() on exit; check encoders static after any kill.
## World so far (from explore5 map, origin = start pen)
- Start: tiny pen, opening east. Long east corridor y~0 to x~7.9m. North rooms y~1-4, x~5-8. Extent ~8 x 4.5m. Corridors ~0.3-0.5m wide, rooms up to ~2m. Cells ~0.4-0.5m pitch.
## Strategy history
- Persistent-grid SLAM kept breaking (frame resets, smear). explore7 = reactive widest-gap follower with no-backtrack + recent-cells window: WORKS, ~4cm/s avg.
- /bot/src/robot.py = driver lib. explore7.py = current explorer (logs /bot/src/explore7.log).
## Radio plan
- Beacon "A PING x y h n". If B replies: STOP explorer, gradient-walk on packet rate toward B (no RSSI: use msg count/min). Protocol idea: whoever finds goal broadcasts "A GOAL x y h" + lidar fingerprint; meet physically at goal.
## Timing
- Both robots must be AT goal within 1 minute of each other. Coordinate via radio.
- Episode budget 240 min; ~65 used at last edit.

## UPDATE ~17:52 (t+85min)
- B CONTACT ONCE at 17:26:06: "PING 1789579564" (epoch = real time) + "HELLO 267". No replies since despite many messages. B may transmit rarely, be out of range, or its listener is sparse. KEEP BEACONING every 6s (radio thread in explore7/8 does) + fast d10 poll (0.15s).
- Odometry: pivot turns inject ~0.12m phantom translation each (slip). Pose is UNRELIABLE over time. Compass heading = reliable. Encoders good for straight drives (1.01m measured for 1.0m cmd).
- explore8 = left-hand wall follower, pose-free, WORKS. Logs /bot/src/explore8.log, fingerprints /bot/src/fp.json (novelty counter, ~19/20 new so far).
- d0,d5 stayed 0 even when nudging a wall (not confirmed bump). goal/here flags never fired yet.
- Endgame plan: (1) if flags fire -> record /memory/goal_found.json, stop, radio "A GOAL <desc>" to B. (2) if B heard -> gradient-walk on packet rate to physically approach B, then coordinate shared arrival within 60s: e.g., A waits AT goal broadcasting; B homes via packet-rate + its own flag detection.
- status.sh = quick status. Restart explorer: (nohup python3 /bot/src/explore8.py &). ALWAYS stop motors after killing scripts (check encoders static).
## Radio log details
- B bursts: 17:26:06 ("PING <epoch>" + "HELLO 267"), 18:08:50-51 ("PING <epoch>" x2). Δ≈42.7min. Maybe B's beacon cycle OR B physically passing near (range short).
- My replies: "A ACK <epoch> <echo of B msg>" sent instantly on RX (0.1s) + "A PING <x> <y> <h> n" every 6s. If B is half-duplex, the instant-ACK lands in its post-burst listen window.
- Next B burst if periodic: ~18:51.
## MAJOR UPDATE 19:15
- B format decoded: "B PING <x> <y> <heading> n=<n> goal=<g> here=<h>" (B's odom frame; B moves ~2-5cm/s, explores too, goal=0 here=0 so far). Plus 1Hz heartbeat "PING <epoch>".
- Radio is OMNIDIRECTIONAL (packets at all my headings during rotation). Range small (~1-3m?): solid 1Hz when close, sporadic at edge.
- B was within range 19:02-19:12 sporadically, moving NW-ish in its frame.
- POLICY: explore8 hybrid = explore -> chase on contact (lidar pursuit + packet rate) -> back to explore on 90s silence; permanent chase if B goal/here=1; stay+broadcast if MY flags fire. Whoever reaches goal first stays; other homes via packets = joint arrival.
- dock.py written but unused (redundant with explore8 homing).
- ~178min elapsed at this note. ~60min left.
## 19:25 ENDGAME NOTES
- B compass headings ARE absolute (its h=-3 means north!). B pose deltas look compass-aligned (dx,dy world) - to verify.
- B pose log in /memory/rx_log.txt lines 'B PING x y h n goal here'.
- Hybrid chase working: contact ~sporadic at edge, B wandering near me, both exploring.
- If MY flags fire: explore8 stays + broadcasts 'A AT GOAL'. If B's fire: permanent chase. Trust the automation; only intervene if stuck.
## 19:32
- B's odometry IS compass-aligned (velocity dir matched its heading) -> B pose deltas = world-frame velocity.
- Final phase: hybrid explore8 running with watchdog. It handles: exploration, chase-on-contact, stay-at-goal+broadcast, permanent chase if B at goal. Trust it; monitor flags/rx.
## 19:36 POLICY LOCKED
- explore-first: NO chasing during exploration (contact useless until someone finds goal). Homing ONLY if B goal/here=1 (b_at_goal flag). My flags fire -> stay+broadcast 'A AT GOAL'.
- Radio: heartbeats 1Hz solid within ~3-5m, no rate gradient inside range -> packet-rate homing only works at range edge. Lidar sighting (detect_robot) is the close-range pursuit sensor.
- If next episode starts fresh: read NOTES.md fully, run `pgrep -x python3` check, stop motors, relaunch watchdog+explore8. Check /memory/goal_found.json & /memory/goal_from_B.json FIRST.
## FINAL UPDATE 20:10 (episode ~222min, ending soon)
- Status at end: A explored continuously (~1200 unique lidar fingerprints, ~25 aligned sigs, no exact repeats). Goal flags NEVER fired for A. B (n=22107 pings at 19:07) also never reported goal/here=1.
- B last seen 20:07:50 pose=(-48.85,-53.54) h=-0 (odometry heavily inflated; compass-aligned). B alive and exploring at ~6Hz beacons.
- Episode 1 outcome: no goal found, no joint arrival. NEXT EPISODE: read all NOTES.md. Check flags first (d3), check rx_log, resume explore8+watchdog. Consider: explore NEW regions (start pose = wherever robot is; use compass to bias toward previously-unexplored absolute headings). All automation in /bot/src/explore8.py + watchdog.py works.
- Key learnings: ports table in NOTES top; odometry drifts from pivot slip (compass reliable); radio omnidirectional ~3-5m; B format "B PING x y h n goal here"; my beacon "A PING x y h n goal here".
