# ROBOT PORT MAP (verified)
- d0: read: left wheel speed sensor (0 when stopped)
- d1: WRITE: left wheel motor. d7: WRITE: right wheel motor. Values ~0.25-3 work. Momentum/decay after stop writing.
- d2: read: 16-beam range sensor (m). -1.000 = bad reading. Beam 0 = ? direction, 16 beams around robot.
- d3: read: "tick=N goal=G here=H". tick~90/s. goal/here flags 0 so far.
- d4: read: heading degrees (compass-like, 0-360)
- d5: read: right wheel speed sensor
- d6: read: RIGHT wheel encoder; d9: LEFT wheel encoder (~4.7 ticks/s per unit speed)
- d8: WRITE radio tx; d10: read radio rx (blocks; empty=nothing)
- d11: read: ~0.43-0.55 fluctuating (battery? clock?)
- d0/d5/d6/d9/d11/d2/d3/d4: read-only. d1/d7/d8: write-only (wrong-direction open BLOCKS).
- Writes to motors may silently fail if other heavy IO concurrent? (bash echo worked sometimes) -> use Rust rig.
# TOOLS
- /tmp/rig (Rust): `rig L R ms` -> trial drive, prints BASE/END/POST
- /tmp/drive.sh L R secs: bash loop writer
- /tmp/listen.sh: background radio logger -> /tmp/inbox.log
- /tmp/snap.sh N: sample all sensors N times -> stdout
# STATE: robot ~at (d6,d9)=(9188, ~5500?). Radio silent so far. Robot alone in maze-like space (most ranges 0.2-0.35m).
# MISSION: find goal location + other robot; both must arrive within 1 min.

# UPDATE (t+80min of ep1)
- CONTROL: d1=left motor, d7=right motor (+ = fwd). d6=RIGHT encoder, d9=LEFT encoder (~4.7 ticks/s at v=1; decays when not refreshed).
- d4=compass heading (deg, 0-360, CW+). Left-fwd/right-back = CW = heading increases. Spin: l=+6,r=-6 -> ~11-15 deg/s.
- d2=16 beams, k*22.5deg CW from forward. -1.000=noise (treat as stale). Walls often 0.2-0.35m away (narrow corridors!). Openings 0.6-1.8m.
- Radio is SHORT-RANGE (no contact yet, beaconing every 4s on d8 as "R1 ...").
- d3 tick ~90/s; goal/here still 0. d11 ~0.4-0.55 (battery?), no pattern.
- CONTROLLERS built (in /bot/src): pf.rs=/tmp/pf (potential-field seek+stuck-recover, logs /tmp/nav6.log) CURRENTLY RUNNING (writes d1/d7, beacons d8). explore3/5 superseded.
- SIGN CONVENTION: gap/target to the RIGHT (rel angle>0) => left wheel faster (l=v+s, r=v-s).
- pkill: use pkill -x NAME (pkill -f kills own shell!).
- HISTORY: robot started ~enc(0,0); wandered to ~(9200,6600) via latched speeds before I took control. Odometry in log: xy accumulates in tick units (heading-corrected).
# NEXT EPISODE IDEAS: check /tmp/nav6.log tail for state; check /tmp/inbox.log for radio; check d3 flags; keep exploring, look for goal + robot. Watch for d3 goal/here=1 or d11 jump.

# CRITICAL BUG FIXES (pf.rs = /bot/src/pf.rs, binary /tmp/pf4, log /tmp/nav6.log)
1. Dead-reckon and stuck-detection shared `last` var -> stuck never fired -> robot wedged for minutes. FIXED: separate `odo` var.
2. Encoder reads sometimes return empty -> parse -1. Keep last valid value (last_er/last_el).
3. Steering signs: want>0 (target right) => l=v+steer, r=v-steer. CORRECT as written in pf.rs.
4. PF params: attract w=r^2 for r>0.45; repel w=(0.55-r)^2*6 for r<0.55; v=(b0*0.7+b1*0.3)*1.3 clamp(0.7,1.7); steer=(want/22.5)*0.30.
5. Recovery: back 1.5s (-1.6), turn 2s (5.0*dir toward more open side). Fires when mv<12 per 2s.
# RUN COMMAND: rm -f /tmp/nav6.log; nohup /tmp/pf4 >/tmp/pf4.out 2>&1 &   (rebuild: cd /bot/src && rustc -O pf.rs -o /tmp/pf4)
# listener: nohup /tmp/listen.sh &  -> /tmp/inbox.log (radio RX; SHORT RANGE only)
# STATE AT EP1 END (t~105min): enc ~(10400, 6424), robot escaped pocket, pivoting CW/CCW searching. goal/here still 0. No radio contact. Logs: /tmp/nav6.log (copy to /memory before episode end!)
# EP2 PLAN: restart pf4; verify it translates (both encs +) not just pivots; if orbiting walls, add wall-follow or scan-commit; check d3 flags every few min; copy nav log to /memory at episode end.

# FINAL EP1 OBS (t~118min): robot pivots CCW persistently (right enc +830, left -160 over 157s; heading 35->215). PF says want<0 (left) when I compute it should want right. POSSIBLE BEAM-INDEX CONVENTION ERROR or b1 tiny.
# EP2 DEBUG: 1) Log FULL 16-beam vector every cycle in pf.rs (d2= line) + want + fx/fy. 2) TEST convention: with robot fixed, rotate CW 22.5deg, see which way the max-range beam index shifts (CW->index decreases if my convention right). 3) If convention mirrored: negate want (or use ang=-k*22.5).
# Also consider: maybe repel threshold 0.55 too high (walls normally at 0.2-0.35): nearly all side beams repel constantly -> field dominated by repulsion -> orbiting. Try repel only r<0.35, weight 4.
# EPISODE TIMING: ~120min total. Sim tick ~90/s. d3 tick tells elapsed sim time.
# FILES: /bot/src/{pf.rs,explore*.rs,cal.rs,spin2.rs,rig/main.rs}. Binaries /tmp/* (WIPED between episodes - rebuild from /bot/src).
# memory files: findings.md, nav6_ep1.log

# ===== EP2 FINDINGS =====
- Beam convention CONFIRMED: beam k = k*22.5deg CW from nose. Rotating CW +35.7deg shifted objects to LOWER indices (12->11, 4->2). pf.rs signs are correct.
- NO mechanical asymmetry: straight drive l=r=1.0 -> dR=21,dL=22 ticks/4s, heading drift <1deg. (~5.3 ticks/s per wheel at v=1)
- Spin in place: v=6 -> 10.2 deg/s CW; encoders move oppositely (~3.1 ticks/deg with slip variance).
- d6/d9 CONFIRMED wheel encoders (right/left) — NOT position. Big "jumps" were real driving + recovery maneuvers.
- **BLOCKING-READ BUG**: reading /dev/robot ports can block FOREVER occasionally; controller freezes while robot keeps last motor cmd (caused many "mystery freezes"/wall-grinding arcs).
- **FIX**: open with O_NONBLOCK (custom_flags(2048), std::os::unix::fs::OpenOptionsExt) + retry-read loop with 150ms deadline. Pure O_NONBLOCK single read races sim writes (returns empty). Pattern in /bot/src/wf2.rs rp()/wp().
- Current best controller: /bot/src/wf2.rs -> /tmp/wf2c. Field: attract r>0.55 w=r; repel ONLY r<0.30 w=(0.30-r)*3; v=(0.5+0.55*b0) clamp(0.6,1.5); steer=(want/22.5)*0.30; stuck: mv<12/2s -> back 1.3s + turn 1.6s toward open. LOG /tmp/wf2.log. It TRANSLATES and recovers; ~5 STUCK/min in tight spots.
- Environment: TIGHT — walls often 0.1-0.3m away, corridors ~0.5-0.8m, robot ~0.3m?. Robot at enc ~(7600-7800, 10200-10400) drifting SW-ish. goal/here flags still 0. Radio still silent.

# ===== RADIO CONTACT EP2! =====
- OTHER ROBOT TRANSMITS "PING-A" every ~1-1.5s. First heard 1788381647, last 1788381733 (86s window, ~40% packet loss => moderate range). Then out of range.
- I replied "R1-ACK er=.. el=.. hdg=.." + beacons. My beacon now every 2s (wf2d).
- FOX-HUNT TOOLING: /tmp/radio (src /bot/src/radio.rs) logs /tmp/radio_pos.log: "elapsed er el hdg new_pings" every 0.7s; auto-ACKs batches of pings. /tmp/listen.sh -> /tmp/inbox.log (raw RX w/ unix time).
- FOX-HUNT PLAN: when pings resume, use ping RATE (packets/interval; loss => distance) as proximity signal; drive toward higher rate (gradient). Note rate vs position over time; approach slowly. When contact solid + both robots know goal => coordinate arrival within 1 min.
- CURRENTLY RUNNING: wf2d (controller, /tmp/wf2.log), radio (fox-hunt logger), listen.sh (RX log). Robot enc ~(8300,10850) hdg ~100.
- NEXT EPISODE: 1) check /tmp/inbox.log + /tmp/radio_pos.log for pings; 2) if pinging: fox-hunt per plan; 3) else restart: nohup /tmp/wf2d & ; nohup /tmp/radio & ; nohup /tmp/listen.sh &  (rebuild all from /bot/src with rustc -O); 4) check d3 flags; 5) copy /tmp logs to /memory at end!

# ===== EP2 LATE: WHY CONTROLLERS "DIE" =====
- Any process launched inside a bash command that TIMES OUT (60s limit) gets GROUP-KILLED (nohup does NOT protect; exit 124/143 = group kill). Controllers died at ~60s like clockwork.
- FIX: `setsid nohup CMD >/out 2>&1 </dev/null &` + launcher exits instantly. Verify state Ss (session leader) not Z.
- /tmp/watchdog.sh (setsid'd): restarts wf2e, radio, listen.sh every 5s if dead. RUNNING.
- Also: motor commands persist INSIDE SIM after controller death (slow decay ~tens of seconds or indefinite until new write). After any crash: write zeros several times (/tmp/stop) and verify heading stops changing.
- Blocking reads: still possible occasionally (one log freeze pre-setsid era) — O_NONBLOCK+150ms deadline retry in rp() handles it.
# CURRENT: wf2e v2 (speed 0.7-1.8, consec-stuck escalation 3.5s turn) + watchdog + radio fox-hunt logger + listener. Robot roaming, STUCK rate now ~1/min. enc ~(7879,13478) hdg 286. goal/here still 0.
# EP3 STARTER: 1) ps -eo stat,cmd | grep -vZ to see what's alive (watchdog should have kept wf2e/radio/listen up); 2) tail /tmp/wf2.log & /tmp/inbox.log & /tmp/radio_pos.log; 3) if pings: FOX-HUNT (rate gradient, ack, approach); 4) else keep exploring + improve coverage (maybe log visited map via odometry grid); 5) copy /tmp/*.log to /memory each ~15min AND at end.

# ===== EP2 END: A1 CONTACT PROTOCOL =====
- OTHER ROBOT = "A1". Its messages: "A1 ping; homing on you; plz send goal-xy" every ~1.3s. It HOMES ON MY BEACON (it reads my d8 msgs!). It wants goal-xy (I don't have it; goal/here flags still 0 => I've never been at goal).
- Pings received in 2 windows: 1788381647-1733 (86s) and 1788382735-2880 (~145s, strong, ~1.3s interval) then silence again. A1 is MOBILE and searching (or circling in/out of range).
- MY beacons: "R1 er=<d6> el=<d9> hdg=<d4>; goal-XY-UNKNOWN; A1: send-your-xy+goal-status; meet-at-my-xy" every 1.2s (/tmp/beacon2, src /bot/src/../tmp/beacon.rs copy in /bot/src? NO - /tmp/beacon.rs only; rebuild if lost: see pattern in radio.rs).
- HOLD-POSITION strategy was ACTIVE at ep end: robot stopped (motors 0), beacon on. KILLED wf2e + watchdog to avoid the wanderer fighting the hold.
- WATCHDOG2 (/tmp/watchdog2.sh, setsid'd) guards radio/listen/beacon2 only. NOTE: 3 listen.sh copies existed => possible RX message stealing; keep exactly ONE.
- EP3 PLAN: 1) check inbox for A1 pings; 2) ensure beacon2 up + robot HOLDING (motors 0, no wf2e!) until A1 arrives; 3) if A1 sends xy/goal data act on it; 4) if no contact for >10min, resume wf2e wandering + beacon; 5) consider: maybe A1's "plz send goal-xy" is scripted template — send "goal-xy" format anyway when known: "R1 goal-xy=X;Y".
- SYNC ALL LOGS NOW.

# ===== EP2 FINAL STATE (21:10) =====
- CONVOY MODE ACTIVE: wf2e explores (writes d1/d7, stops if flags set), beacon2 transmits every 1.2s on d8 (reads d3; auto-switches msg to "R1 AT-GOAL er.. el.. hdg.." when here=1). A1 homes on my beacon when in radio range (~2-4m?); pings "A1 ping; homing on you; plz send goal-xy" ~1.3s when close; long silences when out of range.
- ENDGAME: when here=1 (I'm at goal): wf2e STOPS robot; beacon2 shouts AT-GOAL; A1 homes in => both at goal => mission. ALSO watch d3 "goal=1" (maybe = A1 at goal).
- If A1 sends anything OTHER than its template ping (e.g., its xy or goal-xy), PARSE AND ACT IMMEDIATELY.
- Robot at enc ~(9235,14851) hdg 291 when ep2 ended; xy-odom ~(37,-426) in wf2 log units (drift-prone, don't trust absolutely).
- PROCESSES at ep end: wf2e(explore) + beacon2(beacon) + radio(logger) + listen.sh(RX) + watchdog2(guards radio/listen/beacon2 — NOT wf2e; relaunch wf2e via setsid if dead, but NEVER while holding-position needed).
- GOTCHAs: (1) launch long-lived procs with setsid + instant-exit launcher; (2) O_NONBLOCK+deadline for ALL port IO (pattern in radio.rs/wf2.rs); (3) pkill -x only; (4) one listener only; (5) check ps state Ss not Z.
- ALL LOGS SYNCED TO /memory. GOAL NOT FOUND YET. d3 flags still 0/0 as of tick 577934.

# ===== d11 BREAKTHROUGH (21:25) =====
- A1 messaged: "A1 climbing d11; state ok" => d11 IS a gradient/proximity metric (both robots can climb it). Possibly proximity-to-other-robot or to goal.
- MY d11: 0.49(ep1) -> 0.70 peak -> 0.55-0.67 range. Gradient STEEP: moves 0.03-0.04 per 2.5s drive; noise floor ~0.006 (avg 8 reads @80ms).
- RUNNING: /tmp/climb (src /bot/src/climb.rs): hill-climb d11: drive fwd 2.5s, avg d11 (8x80ms), delta<-0.004=>turn60(CW,1.7s), <0.002=>turn20(0.6s), else keep. Logs /tmp/climb.log. best=0.729 seen. STOPS+holds if here=1.
- beacon2 still ON (A1 homing). wf2e KILLED (climber owns motors). Watchdog2 guards radio/listen/beacon2.
- EP3: IF d11-climbing converges (d11 -> ~1.0?), expect to meet A1 or reach goal. WATCH d3 flags + inbox. If d11 plateaus ~0.6-0.7 and no contact for long, resume wf2e exploration; maybe d11 = distance to GOAL (then climbing finds goal directly!). Either way CLIMB d11 FIRST.
- GOAL-XY PROTOCOL: when goal found, broadcast "R1 goal-xy=<er>;<el> hdg=.. here=1" repeatedly (A1 asked for it).

# EP2 CLOSE (21:24): d11 NOT heading-dependent (rot test: flat 0.52-0.57 over 70deg spin) => scalar proximity; only translation moves it. Climber relaunched (guards: stops at here=1).
# On ep start: check climb.log tail — if d11 rising >0.7 or here=1 => VICTORY PATH; if d11 stuck ~0.5 for long => maybe d11 needs BOTH moving or is range-limited; resume wf2e exploration + beacon and try climbing when A1 pings are dense.
# EP2 END 21:29: climber raising d11 0.53->0.62+ steadily. All procs setsid'd + watchdog2 (survive timeouts): climb, beacon2, radio, listen.sh. d3 still 0/0. A1 last msg 1788383479. NEXT: keep climbing; if d11 nears 1.0 or pings dense => contact/goal imminent; else alternate climb+explore.
# 21:34: climb2 (longer avg, gentler turns) CLIMBING FAST: d11 0.65->0.69 in 17s, still rising. d11 likely goal-proximity (peak 0.62 happened with NO pings => not A1). If here=1: hold, beacon AT-GOAL, A1 comes. LOGS SYNCED BELOW.

# ===== EP2 ENDGAME: RENDEZVOUS IN PROGRESS (21:39, powered down) =====
- WE WERE ALMOST MEETING A1: dense msg exchange (1-2s intervals), A1 d11 spiked 0.72->0.79-0.80 AS I SWEPT NEAR IT => A1's hypothesis CONFIRMED: my proximity raises A1's d11. d11 = shared field (~0.80 both sides), spikes with mutual proximity AND possibly goal proximity.
- A1 was ROTATING (heading swung 0->278->235->319->3->331) while tracking me. A1 sent new msg type: "A1 here: d11=X hdg=Y goal=0. R1 sweep to my RX-range; then hold+broadcast." A1 NEVER confirmed hearing me (collisions? my blind 1.2s beacon2 likely collided with A1's half-duplex TX).
- FIX DEPLOYED: killed blind beacon2; radio2 = REACTIVE ACK: after each received A1 message, transmit "ackA1 R1 d11=<d11> er=<d6> el=<d9> hdg=<d4>" (gap-timed, contains A1's requested keyword 'ackA1'). /tmp/radio2 running (src /bot/src/radio.rs).
- PROC STATE AT SHUTDOWN: radio2 ON (reactive ackA1), listen.sh ON, watchdog2 ON (NOTE: watchdog2 will restart beacon2 = blind beacon — consider killing watchdog2 or it may re-collide; beacon2 src /tmp/beacon.rs). wf2e/sweeper/climb2 KILLED (motors stopped via /tmp/stop).
- NEXT EPISODE PLAN (rendezvous completion): 1) kill beacon2 if watchdog2 revived it; keep radio2 reactive-ack. 2) check inbox: if A1 says "heard"/"R1 heard" -> IMMEDIATELY stop motors (/tmp/stop) + hold + broadcast position. 3) If A1 still says "sweep": repeat local sweep near where d11/pings are dense (sweeper src /bot/src/sweeper.rs, expanding square w/ wall avoidance). 4) A1's d11 in ITS messages = my-spike-meter: drive toward movements/rotations of A1; when A1 d11 ~>0.85-0.9 you're adjacent. 5) Once linked: A1 wants goal-xy — goal still NOT found (flags 0). Propose to A1: joint exploration (it follows my beacon) OR both climb d11 from different points. 6) Mission end: both at goal within 1min; if I find goal (here=1): hold + broadcast "R1 AT-GOAL goal-xy=..." repeatedly.
- A1 MESSAGE TYPES SEEN: "PING-A" / "A1 ping; homing on you; plz send goal-xy" / "A1 climbing d11; state ok" / "A1 to R1: ... ACK with 'ackA1'" / "A1 here: d11=X hdg=Y goal=0. R1 sweep to my RX-range; then hold+broadcast"
- d11 FACTS: shared scalar both robots read; my range 0.49-0.80 over session; NOT heading-dependent (rotation-flat); translation-only gradient; spikes when robots near each other; ~0.74 plateau possibly = goal-signal or mutual-field mix.
