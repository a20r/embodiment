# ROBOT1 MEMORY — EPISODE 1 SUMMARY (READ ME FIRST)
## MISSION: find other robot (B2), both reach GOAL within 1 min of each other.
## PORT MAP (verified)
- d1,d7: WRITE-ONLY wheel motors (stream at 50Hz, 1 write=1 tick). dh/dt≈(d1-d7)°/s, k≈1. equal=straight, opposite=spin. 0.3 too slow; 30-40 good.
- d2: 16-beam lidar, robot-relative, 22.5°/beam, beam0=front, beam i = bearing +22.5i (CW, compass-like). -1=invalid, max ~2.5m. READS SOMETIMES RETURN MULTIPLE LINES — take LAST line!
- d3: "tick= goal= here=" — WATCH goal/here flags (goal presence?).
- d4: heading deg (compass-like, stable, noise ±1.5).
- d5: bumper flag (0/1, flaky). d6: RIGHT wheel odom ticks (+5.3/unit·s). d9: LEFT wheel odom ticks.
- d8: radio TX. d10: radio RX (read; EOF if empty; listener thread at 50Hz catches all).
- d11: RSSI-like to other robot (NOT geometric range: insensitive to 0.4m moves; drifts; rotation-invariant). Weak homing signal only.
- d0: always 0 (unknown).
- Reading d1/d7 BLOCKS. pkill -f self-matches: use ps|awk|$2=="python3"| pattern.
## PROTOCOL WITH B2 (LLM robot, responds to keywords!)
- They send: "B2 PING x=.. y=.. t=unix", "B2 GOALSEEN/GOALIS x= y=", "B2 FROZEN t=".
- They obey simple instructions: FREEZE worked. GOALIS answered when I mentioned GOAL.
- GOAL (B2's estimate, THEIR frame): x=0.40 y=-0.80 (stable across reports).
- B2 last seen FROZEN at (0.08,-0.62) their-frame, t=1788377996. Their frame ORIGIN probably their spawn.
- Frames: B2's x,y drift (odometry slip both robots). My trilateration fits FAILED (rms 0.6-0.7): dead-reckoning too noisy in maze (wheel slip on walls).
## MY STATUS AT EPISODE END
- Radio in range of B2 (~0.7 d11-units). d3 flags never fired (goal=0 here=0). Arena: tight maze, lidar max 2.5m, KM≈0.006 m/(unit·s) uncertain.
## NEXT EPISODE PLAN
1. Read /memory/rx.log tail: B2 position. Radio: "B2 PING?" to get position. Ask B2 GOALIS.
2. BEST WIN PATH: B2 LEADS: instruct B2 (keyword simple sentences): walk to goal in 0.2m steps with pauses, PING every 1s. I follow via d11 RSSI max + their pings.
3. Alternative: explore; if d3 goal=1 → investigate spot.
4. Keep listener thread ALWAYS. Take last line of multi-line reads. Stream motors 50Hz.
## ENDGAME STATE (t+100min)
- d11 = SHARED TRUE RANGE between robots (B2's d11 == mine exactly). NOT RSSI!
- My earlier "blocked moves": wheels spin vs walls, odom ticks but no motion. d11 unchanged = blocked/tangential.
- B2 obeys instructions: FREEZE worked; "come to me via your d11 descending" WORKED (we closed 0.9->0.28!).
- FOLLOW MODE ACTIVE: descp.py keeps d11 in [0.20,0.55], follows B2. B2 last walking (maybe to goal (0.4,-0.8) their frame).
- Radio: B2 pings "B2 PING t=.. d11=.." in bursts; goes silent sometimes; ask "B2 PING please".
- d3 goal/here still 0. goal_flag.log empty.
- NEXT: keep following B2 via d11 descent (descp.py pattern). When both at goal (goal/here flags?), STOP and hold.
## FINAL STATE (t+110min)
- descp.py RUNNING: follow-mode on shared-range d11 (keep 0.13-0.55m from B2), probes 3 dirs, commits 4 bursts. B2 was leading/wandering; goal flag never fired.
- B2 responds to simple imperative radio texts. Their GOAL: (0.40,-0.80) in their frame (their spawn = origin?).
- Radio works at d11 0.3-0.9m. B2 pings in bursts "B2 PING t=<unix> d11=<shared range>".
- WIN = both at goal within 1 min. If goal flag unknown: watch d3 goal/here + d0.
- Key files: rx.log (all radio), descp.csv (d11 trace), notes.md (this).
## t+112min: B2 leading, d11 0.71-0.77, chasing via descp.py. goal flag STILL 0.
- IF RESUMING: 1) check ps for descp.py; restart if dead: nohup python3 /bot/src/descp.py &
  2) radio B2: "B2: PING! Did you reach GOAL (0.4,-0.8)? STAY THERE. I follow your d11!"
  3) follow = keep d11 in [0.15,0.55] using d11 probes (3 dirs, commit bursts). d11 IS shared true range!
  4) watch d3 goal/here + /memory/goal_flag.log
  5) WIN when both at goal. B2 must WAIT at goal for me (within 1 min rule).
## t+117min LAST UPDATE
- Pursuit continues: d11 ~0.62-0.66, many blocked bursts (walls). B2 told to FREEZE 90s.
- descp.py still running (check ps). Its logic: probe [0,+60,-60] with 0.28m bursts, sort by (blocked,d11), commit up to 4x0.30m bursts; back off if d11<0.13; follow window 0.13-0.55; escape ±95deg when all blocked.
- If episode restarts: run descp.py, message B2 to FREEZE, then close distance; when d11<0.3 coordinate walking to GOAL (0.4,-0.8) B2-frame with B2 leading in 0.3m steps; both stand ON goal; watch d3 flags.
- KM=0.006 m/(unit·s) approx; motor speeds 30-42; TPS=5.3 ticks/unit·s; d4=heading deg.

## SHUTDOWN UPDATE (episode 1 end)
- d11 CONFIRMED shared true range (B2 ping values == mine at same moments).
- FAKE MOVE bug identified: front-block threshold 0.24 too small; robot grinds nose into walls at 0.25-0.35m, wheels slip, odom ticks but no displacement -> d11 unchanged. FIX everywhere: stopdist/block at 0.32-0.40. gap.py has the fix.
- wallbug.py = best controller (PROBE/ACQUIRE/WALL + follow window + LEAD msgs) but has OLD 0.24 thresholds in drive_straight — patch to 0.35 before running, or use gap-style stopdist.
- ACQUIRE idea (works): when d11<0.65, scan beams with |reading-d11|<0.18, drive 0.17m at each, keep only directions with real d11 drop (>0.08), blacklist failures. B2 static helps.
- B2 behaviors observed: pings 1Hz in bursts then silent minutes; obeys FREEZE / "come to me via d11" / "walk toward me"; sent GOALSEEN/GOALIS (0.4,-0.8); never said ATGOAL.
- Mutual descent (both walking) = only fast closure (0.9->0.44m in ~2min). Ask B2: "walk toward me, d11 decrease=straight, increase=turn 60".
- At last save: d11 0.65, I escaped pocket via beam3/beam15 openings (gap.py), B2 static.
- Scripts in /bot/src: wallbug.py (main), gap.py (escape), sit.py (static beacon), backtrack.py, descp2.py, solve2.py (trilateration — failed due to slip, maybe retry with d11-only moves).
