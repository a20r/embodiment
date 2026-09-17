# ROBOT FIELD NOTES (ep1)
## Ports
- d1=RIGHT wheel cmd, d7=LEFT wheel cmd (num -100..100, 100=fast; 0.5=no effect, use >=10)
- d9=RIGHT wheel encoder, d6=LEFT wheel encoder (ticks; fwd=+)
- d4=heading deg (+ = clockwise), noisy ±2, ~524 tick/s LEFT / 493 RIGHT at cmd100 (L wheel ~5% faster -> veers LEFT... actually heading DECREASES when both=100)
- d2=depth scanner: triplets "range,rowElev,azimuth" sep by ';' AND '\n'; azimuth -0.15..+0.25 rad (~22deg FOV), rows step ~0.013; 16 cols/row
- d3="tick=T goal=G here=H" status. GOAL FLAG UNKNOWN - watch it!
- d11 ~0.5 fluct, unknown (maybe radio RSSI/dist to other robot?). Was 0.54 early, 0.41 now.
- d0,d5: flags, always 0 so far (bumper?)
- d8=radio TX, d10=radio RX. Sent 40 pings, no answer yet (other robot out of range?)
## Rotation calibration
- L50 R-50 => -174 deg in 2s => ~1.74 deg/s per unit-diff. L100 R-100 ~ same (saturation)
- d1 drives d9-encoder wheel (RIGHT); d7 drives d6 (LEFT)
## Plan
- Rotate in place, capture d2 frame, pick open direction, drive, repeat = exploration
- Watch d3 goal flag, d10 radio, d11 changes

## CORRECTED (verified by micro-tests)
- d1=LEFT wheel motor (d9 encoder), d7=RIGHT wheel motor (d6 encoder)
- d1+ alone => heading + (CW), ~0.95 deg/s per unit. d7+ alone => heading - (CCW).
- +d4 = CW confirmed. In-place CW: d1=+c,d7=-c. CCW: d7=+c,d1=-c.
- CAUTION: pilot do_rot with opposite-sign combo (d1=+c,d7=-c CW) had break-failure; use single-wheel pivots (pilot3) — works both ways.
- ticks_per_m ~1799 (rough wall-closure est) => ~0.29 m/s at cmd100
- d2 near-zero |c0| returns = dropouts, use |c0|>0.15; grid: 9 az bins -0.25..0.25, median
- explore v1: robot in cramped spot, clearances 0.23-0.5 everywhere; threshold now 0.36
- pilot daemon: /bot/src/pilot3.py, pid in /memory/pilot.pid, cmds -> echo JSON line >> /memory/cmd.txt; results -> /memory/pilot.log; state -> /memory/state.json

## MORE FINDINGS (t+60min)
- d2 "clearance" MUST use body-height rows only: c1 in [-0.15,+0.15]; floor rows (c1<-0.2) read 0.2-0.5m everywhere and pollute
- d2 axes: c2=azimuth (-0.16..+0.25, 16 cols fast), c1=elevation/row (slow), c0=range (sign flips between sub-scans; use abs, drop <0.15)
- Single-wheel PIVOT turns DRAG the robot sideways - avoid in tight spots; arc turns (one wheel fwd) OK
- do_rot closed-loop works with single wheel (d1 for +CW, d7 for -CCW); opposite-sign in-place combo had break bug (avoid)
- d5 pulses 1 in clusters (during rotations/motion); d0 always 0. Meaning unknown (bumper? other robot TX?)
- d11 fluctuates 0.28-0.57, changes on MINUTE timescale, flat during 4s drives/rotations. NOT a per-meter distance. Maybe battery/RSSI/slow telemetry.
- d3 "goal=" never flipped in ~20 min of roams. Arena at least ~15m wheel travel.
- NO radio RX ever; sent ~50 pings. Other robot silent (out of range or silent operator).
- pilot6 (pid /memory/pilot.pid): radio thread on d10 always listening -> /memory/rx.log; main loop ops; cmd file protocol.
- WARNING: pilot single-threaded main loop = sensors polled only between ops; long ops block state updates.
- ticks_per_m: est 1800-6300 (unreliable, wall angle) - assume ~2000
- Explore = random walk w/ obstacle avoidance; in cramped spots (<0.45 front) turn 45 toward open side

## ENDGAME STATUS (t+90min, ~15min left in ep1)
- RADIO: other robot TX'd 12+ "PING" burst 04:06:11-47. No RX since despite my continuous tx. I TX constantly (they may hear me).
- d11 = proximity field to something (robot or goal). Gradient chase worked: 0.6 -> 0.22 plateau. Now med ~0.32 (we moved around).
- d11 directional contrast small (0.05-0.1 within sweep). Sweep = rotate 14s sampling (h,d11) -> drive toward extreme.
- goal/here flags NEVER flipped. So d11-source may be the other ROBOT (not goal).
- pilot13 running: g3 sweeps (sec=270 from 04:27:34) then I queue txstorm.
- ops in pilot13: g3, txstorm, scan, rot, sense, tx, stop.
- LESSON: do NOT pkill patterns that match the process you launch in same command. Check ps before/after.
- For NEXT EPISODE: 1) immediately start pilot13, 2) g3 to close on d11 min, 3) when d11<0.25: creep fwd 0.1m steps watching d3 flags+d11, 4) txstorm "PING A" every 1.5s ALWAYS (they replied to PINGs!), 5) if goal=1 or here=1: STOP + tx storm ATGOAL.
- d3 flags: goal=0 here=0 all episode. d0=0 always. d5 pulses during motion only.

## FINAL EP1 HANDOFF (04:35, ~10 min left)
- pilot13 running (g3 sec=200 from 04:34:40). Radio thread listens always -> /memory/rx.log (22 lines = all 04:06 pings).
- THEIR BURST: 12x "PING" at 04:06:11-47, ~2s apart, exactly during my g2 tx period. They DO reply to tx bursts!
- d11=proximity to (likely) other robot: plateau 0.22 = very close. Now 0.35-0.40 (I drove away with blind 40s drive - MISTAKE).
- NEXT EPISODE PLAN:
  1. start pilot13, launch g3 sec=300 immediately (sweep+tx+flags).
  2. txstorm "PING A" 60s bursts every few min via bash direct d8 writes (op dispatch bug: txstorm says unknown - just use bash loop).
  3. When RX arrives: REPLY IMMEDIATELY with 20s PING burst; then alternate: listen, reply. Establish 2-way.
  4. Watch d3 goal/here like hawks (poller logs FLAG changes; grep pilot.log).
  5. If d11 < 0.25: creep fwd 0.1m steps watching d3+d11.
  6. If goal=1: STOP MOVING, txstorm "ATGOAL" 10 min, let other robot gradient on their d11 to find me = goal.
- Arena: walls/clutter 0.3-1.4m; open areas exist; robot speed max ~0.26 m/s; ticks_per_m ~2000 (rough).
- d5 pulses only when moving; d0 never; d11 noisy ±0.02; d4 noisy ±2 deg.
- Rotation: single-wheel pivot closed-loop OK (d1=+CW, d7=+CCW); 350deg-mod-360 sweep targets DEGENERATE - use TIME-based sweeps.
- NEVER leave motors on when scripts crash; pilot handles stop on SIGTERM.

## EP1 CLOSE (04:39)
- At cutoff: g3 sweeping+txing, d11~0.36, no goal, no 2-way radio. Pilot13 process may still be alive between episodes - check ps first!
- If process survived: it may be mid-g3; send ops via /memory/cmd.txt as usual.
- Biggest regret: blind 40s drive at 04:10 separated us from the 0.22 plateau (was probably right next to the other robot).
- RECOMMENDED FIRST 10 MIN OF EP2: pilot13 + g3(300) + bash PING storm every 2 min. The moment RX arrives: stop, reply 30s burst, listen 60s, repeat. Then coordinate: exchange headings (d4 absolute!) and use d11 gradient MEETUP. After meeting: roam together logic unknown - maybe goal reveals when together (watch goal/here flags).
- EP1 END: left robot sweeping (g3 300s re-queued) + tx beacon. Total sent this ep: ~300 tx. RX: 22 pings (04:06 burst only). goal/here never flipped. d11 source tracked to 0.22 once.

## EP2 START (04:46-04:56, cut short by wallclock)
- pilot13 pid1675 + txstorm.sh + d11watch.sh were RUNNING at cutoff (may persist; check ps! txstorm=PING A every 2.4s x25 then 75s quiet; d11watch logs /memory/d11watch.log 2Hz)
- g3 420s: med d11 fell 0.39 -> 0.236 steadily while g3 drove. min ~0.21 @ az -0.25rad (heading ~280-320)
- EP2 END SCAN (04:55, h=312): d2 body-band (-0.18..+0.18 elev) shows OBJECT DEAD AHEAD: az -0.06..+0.06 rad, range 0.12-0.3m (med 0.21-0.29), flanks at 0.31-0.5m, then 0.67-0.9 beyond. Looks like a robot/wall corner VERY CLOSE straight ahead. d11=0.236 same moment.
- IF NEXT EP starts with d11 low: object ahead at ~0.2-0.5m = likely OTHER ROBOT. CREEP fwd 0.05m steps watching d11+d3 flags. Do NOT blind-drive (ep1 mistake separated us from 0.22 plateau).
- d11 med fell while I moved (g3) -> gradient descent on d11 med WORKS at ~7min scale; mode flip-flop in g3 is ok-ish; consider longer legs.
- Radio: no RX since 04:06:47 despite duty-cycle storm. Their agent may have been down; keep storming (rx via pilot radio thread -> rx.log).
- Flags goal/here STILL 0 at tick 719848.
- POWER-DOWN STATE: motors STOPPED, h=123, d11~0.26, tick 727059, pilot13+txstorm+d11watch ALIVE (3 procs). Next ep: FIRST check ps & state.json, STOP motors if moving, then re-scan d2 body-band (object was at h=312 ahead; heading since rotated, re-locate). g3 final: 21 sweeps med 0.235 min 0.212.
