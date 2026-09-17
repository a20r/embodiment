# ROBOT B2 — EPISODE 1 FINAL NOTES (read me first next time!)

## MISSION: find other robot (B1/R1) + reach THE GOAL together (both within 1 min).

## PORTS (/dev/robot/)
- d1/d7: motors (velocity, float ok). d1->d9 enc, d7->d6 enc. speed100≈500 ticks/s.
  d1 fwd => heading(d4) INCREASES; d7 fwd => decreases. Both fwd = straight.
- d4: compass (absolute, 0-360). d6/d9: cumulative encoders. d3: "tick= goal= here="
  here=1 => ON GOAL. goal=1 seen once? (unconfirmed semantics).
- d2: 16 range beams, beam k = heading + k*22.5, beam0=fwd, meters, -1=invalid.
- d8 TX / d10 RX radio. d11 = RSSI of other bot's signal (~0.5 far, ~1.0 adjacent)!
- d0/d5: fired (d5=1) near OTHER BOT (probably proximity, NOT goal beacon).

## CALIBRATION
- 0.000503 m/tick (wall-approach fit, 2 runs). Track ~320-340 ticks.

## WORLD: tight maze, cells ~1m, walls 0.15-0.7m. Start chamber exit ~abs 90deg.

## B1 PROTOCOL (SEEN LIVE)
- B1 TX: "R1 t= h= st=ROAM/TURN" ; "B1 PING d11=" ; "B1->B2: ..." ; "B1 homing/FREEZE/MOVE"
- B1 runs d11-RSSI trilateration of MY pings; asked: PING every 1-2s, HOLD STILL.
- B1 game: I send "B2 WARMER <d11>"/"B2 COLDER <d11>" after each of its 0.4m stops.
- B1 said: "GOAL x~0.4 y~-0.8" (in my instance-3 frame, see frames below) and
  "After meet: you walk to goal, I follow." B1 FOLLOWS me via my d11 reports.
- B1 knows/computes positions. ASK it: "B2 REQUEST: send GOAL x= y= in MY frame".

## FRAME HISTORY (all compass-aligned, only origins differ)
- inst3 origin O3. B1's goal: (0.4,-0.8) in O3 frame.
- O3 coords of freeze-spot F: pose.json said (-1.06,-0.53) => goal in F frame = (1.46,-0.27)
  (bearing 349.6deg, ~1.48m from F). F frame = origin at F.
- After F I wandered (mower+hunt+rush) — current pos unknown (~1-2m from F).

## CODE (/bot/src/) — lessons
- robot.py: solid port lib. drive.py: turnto ok.
- dfs.py: maze DFS explorer (works, visited-memory). BUG FIXED: bound BFS!! (it
  flooded RAM 3GB when goal enclosed; cap 15000 cells). goto() must motors(0,0) on 'ok'.
- pinger.py/oracle.py: radio helpers (oracle replies WARMER/COLDER w/ median d11).
- sweep2/hunt/rush: goal-seeking scripts (rush: drive heading + sidestep walls).
- ALWAYS: touch /memory/STOP then pkill "^python3 /bot/src/X.py" then motors(0,0).
  (pkill alone leaves motors RUNNING; regex must not match own bash.)
- Reads can return None transiently — ALWAYS guard.

## NEXT EPISODE PLAN
1. Read this. Probe ports. Check tick counter (episode reset?).
2. Start pinger.py immediately (1s pings). Listen d10 for B1.
3. Ask B1 for goal coords in MY new frame; confirm by driving there; here=1 verifies.
4. Or DFS-explore (dfs.py) until here=1/goal=1; keep PINGing so B1 can trilaterate me.
5. When here=1: STOP, broadcast "B2 ATGOAL x y" forever, let B1 arrive.

=== EPISODE N (20:29+) STATE ===
- rush4.py KILLED (was stuck oscillating d~16.1m; odometry drift suspected). Motors stopped.
- pose (-14.86,-4.55) in posetrack frame = DRIFTED, do not trust for long-range.
- oracle.py v2 RUNNING: logs ALL RX to /memory/rx_all.log; every 15s asks B1:
  "B1: reply 'B2VEC dx dy' = (GOAL minus B2pos) meters, compass frame x=cos(h)*d y=sin(h)*d. Or 'B2BRG brg dist' brg=deg from B2 nose. I hold still."
  + auto-replies WARMER/COLDER to B1 PINGs (B1 homing game) + ACK otherwise.
- pinger.py + posetrack.py RUNNING.
- PLAN: when B2VEC arrives, drive in 1-2m chunks toward pose+vec, RE-ASK B1 for fresh vec every ~30s (drift reset), check d3 here=1 continuously. B1 follows me.
- B1 was spamming 'B1 closing! FREEZE!' -> I am now frozen so B1 can catch up.
- LESSON: pkill -f "X.py" kills own bash if pattern in cmdline. Use python killer or PID.

=== EPISODE N+1 END (21:39 wallclock; ep ended at ~20:39) ===
RESULTS THIS EP:
- Killed flailing rush4. Froze. B1 SILENT since 20:18 (no RX at all; oracle v2 REQ spam every 15s got NO answer).
- d11 was 0.44 (B1 far). Launched home2.py (d11-gradient homing, undo-on-fail). IT WORKS GREAT:
  d11 0.567 -> 0.851 in ~60s. Killed by timeout at 55s; motors stopped OK. pose=(-16.22,-5.99) odo frame.
- B1 proximity: d11>=0.93 = adjacent (B1 read 0.95-0.98 when 0.7m). d0/d5 fire when near bot.

NEXT EPISODE DO IMMEDIATELY:
1. ps aux: if pinger/oracle/posetrack dead, restart:
   cd /bot/src; nohup python3 pinger.py >> /memory/pinger.out 2>&1 &
   nohup python3 oracle.py >> /memory/oracle.out 2>&1 &   (v2: logs ALL RX to /memory/rx_all.log, REQ B2VEC every 15s)
   nohup python3 posetrack.py >> /memory/posetrack.out 2>&1 &  (seeds from pose.json)
2. nohup python3 /bot/src/home2.py >> /memory/home2.out 2>&1 &   then tail it every ~2min.
   home2: climbs d11 (proven), stops+ping-spams at d11>=0.93 waiting for B1 to talk.
   *** BUG: home2 'VECTOR MODE' branch just breaks/stops - vector DRIVING NOT IMPLEMENTED. ***
3. When B1 talks (rx_all.log): ask "B1: send B2VEC dx dy = GOAL minus B2pos (compass frame, x=cos(h)*d,y=sin(h)*d)".
   B1 trilaterates my pings => it knows my pos w/o odometry drift. Re-ask every 30s while driving; reseed target=pose+vec each chunk.
   Then implement/patch vector driver (base on rush4.py goto() + wall sidestep), check d3 here=1 continuously.
4. If B1 stays silent even adjacent: try plain asks; maybe its episode starts later - keep pinging 1/s FOREVER (pinger.py).
5. here=1 on d3 = ON GOAL (verified semantics). goal= seen 0 so far.
LESSONS: (a) pkill -f "X.py" kills own bash (pattern in own cmdline) - kill by PID or python; (b) d11 median of ~9 samples, guard None everywhere; (c) run long jobs with nohup ... & and poll, 60s limit kills fg.
