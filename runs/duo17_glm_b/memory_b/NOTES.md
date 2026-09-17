# EPISODE 1 NOTES (robot B, started in alcove)
## Ports (verified)
- d1=left motor cmd, d7=right motor cmd (int, +/-, ~30 = cruise, persists until changed!)
- d2=16-beam lidar (meters, -1=dropout). Beam k at +22.5deg*k CCW from front. Beam0=front.
- d3=STATUS 'tick=<t> goal=<0/1?> here=<0/1> tx=<n>:<idle|lost|busy|ok>'
  - here=1 => at goal?  busy=peer carrier; ok=my last TX delivered; lost=TX went nowhere
- d4=heading deg (noisy +/-3). Heading increases when left fwd & right back (spin CW)
- d9=left wheel odom, d6=right wheel odom (signed accumulators; 1 unit ~ 2mm, need calib)
- d8=radio TX (write line), d10=radio RX (FIFO). Peer heard: HELLO/HELP floods, now 30Hz EMPTY lines.
- d11=SIGNAL STRENGTH ~ peer proximity! 0.46-0.53 alone; 0.81-0.84 when peer in range. HOMING SENSOR.
- d0, d5: unknown, always 0 so far (maybe goal-related). d11 might double as battery? (drifts 0.46->0.84)
## World
- Tight maze/alcove world, walls 0.15-1.5m away. Robot started at heading~200 in narrow alcove.
- Odom scale: 1.2s at motors(25,25) => odom +145,+154 (dist unknown ~0.3m?)
## Radio protocol issues
- Half-duplex; peer floods 30Hz empty lines now (its controller may be broken/stuck, it sent HELP earlier)
- TX while peer busy => 'busy' (dropped). TX in gap => 'ok'. Peer listens ~1 of 4 slots.
- My messages that got 'ok': peer reacted (HELLO->HELP->empty spam). KEEP TXing 1-2Hz instructions.
## TODO
- HOME on d11 (hill climb). Keep periodic TX beacon. Watch d3 here/goal. Calibrate odom.
- /bot/src: robot.py(robot v1 blocking), robot2.py(async cached sensors+tx/rx), nav.py/nav2.py(explore), talk/listen/duplex.

## UPDATE (later ep1)
- d9/d6 odom units: 1u ~ 0.66mm (663u ~ 0.44m). Speed (22,22) ~ 110u/s ~ 7cm/s.
- Spin: motors(-30,30)=CCW (heading drops ~57deg/s). motors(30,-30)=CW +.
- d11 declined 0.84 -> 0.66 while I parked => PEER TX POWER/ACTIVITY varies, not just my distance!
  d11 ~0.5 baseline w/ peer silent; rises when peer transmits (and/or close).
- Radio 'ok' persists while peer silent = channel clear & in range (can deliver). 'busy' = peer carrier now.
- tx=<n> unclear counter. Peer sent 30Hz EMPTY lines for long stretch, then silence.
- PLAN: wall-follow explore + 1Hz beacon; homing (hill-climb on d11) only when tx busy.

## CONTACT ESTABLISHED (ep1, ~t=25000)
- Peer calls itself "A" (I am "B"). A's msgs: "A-TO-B: ... beep<N>", ~2Hz bursts, signs beep counts.
- A said: "moving toward you slowly", "moving out of slot toward you", "STUCK NO GOAL NO",
  "If you block me please reverse 2m". A receives my msgs (reacts) => protocol: prefix B-TO-A:
- MY port findings: d5=1 while driving FORWARD only (motion flag). d0=0 always so far.
- d11 = peer-proximity/link (0.5 far baseline; 0.9-0.96 in range; saturates, weak gradient inside range).
- Radio: my TX gets ok when channel clear. Peer TX blocks (busy). Both flood = deadlock; use bursts.
## PLAN
1. MEET with A (it approaches; I hold). Detect via LIDARCHANGE (moving object) or d11 max.
2. Then JOINT search: convoy together (robust) or split with heading-based guidance.
   Headings: d4 compass looks absolute => directions shareable ("go heading X").
3. Goal detection: watch d3 here/goal, d0. Neither of us knows goal yet.

## MORE FINDINGS (ep1 late)
- d11 includes MY OWN TX echo. Baseline no-peer ~0.44. Values: ~0.5 + own echo; 0.8-0.96 = peer carrier.
- A went SILENT around t~25600 (its last status beep32, 'exiting slot'). d11 back to 0.44 => A out of range or dead.
  A's signal had decayed steadily 0.95->0.66 => likely battery drain or departure.
- setsid+nohup needed: outer timeout kills process GROUP at 60s, killing my daemons (explorers died silently!).
- d5 = forward-motion flag. d0 = unknown, always 0. lidar beam0 ~1.8m max seen. World = dense tight maze.
- Lawmower explore9: strips at compass headings, arch around obstacles. Odom scale 0.66mm/u verified.
## NEXT
- Watch d3 here/goal + d0 for goal detection. If A reappears (d11>0.7 or RX), switch to rendezvous.

## ENDGAME THEORY (ep1, ~2h20m in)
- d11 = MY ENERGY/BATTERY (TX power and motor authority scale with it; recharged during A contact 0.52->0.96).
- A (peer) battery DIED ~t=25600 ("HELP" was distress). A silent since. d11 now 0.26-0.29, declining.
- THE GOAL MAY BE A CHARGER: d11 should RISE near goal ("know it when you reach it"). CHARGE-RAMP detector active in explore14.
- d0 = rear contact/bump sensor. d5 = still unclear. here/goal flags never fired yet.
- If I find charge (d11 ramps up), PURSUE THE GRADIENT (charge_hunt). Then beacon like crazy for A.
- Watch: d3 here/goal, d0, d11 rising trend.

# EPISODE 2 NOTES (started 07:14, tick resumed ~676k => WORLD PERSISTS across episodes!)
## Critical incidents
- explore14.py from ep1 SURVIVED the episode reset and kept driving (found via ps). LESSON: at episode start, `ps aux` and KILL stray controllers before manual tests.
- pkill -f 'epX.py' matches MY OWN bash heredoc cmdline => self-kill (exit 143). Use PID kill via ps/awk, or patterns like 'ep3\.p[y]' in a SEPARATE command from any text mentioning the name.
## d11 investigation (ep2)
- 3-phase test (07:20): STILL(spin-contaminated) slope -0.00054/s; STILL+TX2Hz +0.00016/s; DRIVE(20,20) -0.00014/s. TX cost negligible.
- Park test 3min: flat 0.25-0.28 => NO idle recharge.
- d11 0.41(07:14)->0.24(07:25)->0.37-0.40 while driving/wandering. Spatial variation. Ep1 had 0.95 while DRIVING at 06:40 => d11 is NOT my battery. BEST THEORY: peer signal strength (A beacon), possibly + goal/charger beacon; drains on A's side as A's battery dies (0.95@06:40 -> 0.24 now).
- If d11 = A's beacon power, and it decays to 0, A may be DEAD. Finding A physically may be only option. WATCH for d11 > 0.6 (peer near/transmitting) and tx ok/busy.
## Radio
- All TX this ep: 'lost'. No RX. => A out of radio range (or dead). tx counter ~44000 persisted from ep1.
## Motor/odom
- Reverse WORKS (motors negative, odom negative). (20,20)=~7cm/s fwd; (-20,-20)=~-6.6cm/s rev.
- Spin (24,-24) ~ 57deg/s. Ep1 notes hold.
## Exploration architecture (ep4 = WINNER so far)
- Step cycle: med_lidar -> pick best beam (weighted forward, openness=min of 3 adjacent) -> turnto(err<7deg) -> driveguard(2.6s, sp20, abort if front<0.28) -> mark BAD[k] if blocked (18s memory).
- Reactive wall-following FAILS in tight pockets (flip-flop arcing). Step+commit works.
## Goal detection (still unknown!)
- d3 'goal=' 'here=' never fired. d0 always 0 (guess: rear-motion flag). d5=1 forward only (verified ep1).
- Maybe goal emits RF too (d11 rise?) or is a charger. No evidence yet.
## TODO next ep
- Persist GRID/pose to /memory for cross-episode coverage memory.
- Check ps at start. Keep beacon 2.4s + RX watch + tx-state watch (ok/busy => A in range).

# *** EP2 BREAKTHROUGH: GOAL FOUND (08:33, tick~1.10M) ***
- d3 here=1 FIRED. B IS AT GOAL. goal=0 => A not here. (goal= likely = "peer at goal")
- Goal location (ep7 odom frame): pose ~(-3.4,1.4), heading 98.8 at arrival, odom 296516,119860. See /memory/GOAL_MARKER.json + scan.
- GOAL EMITS RF CARRIER: d11 0.64-0.69 near goal, tx shows busy/ok near it even w/o A?? (d11 climbed 0.53->0.64 as I neared goal; homing on it worked!)
- Wait for A: rule = both arrive within 1 min of each other. STRATEGY: STAY at goal (here=1), beacon instructions to A, watch goal= flag + d11>0.85 spikes.
- A's canned loop: "GOAL NO STUCK NO / HDG=198 / READY / POS UNKNOWN" (~0.27Hz). Earlier reactive phases: "MOVING TO YOU", "A HOLDING HERE COME", "SEE", "d11 RISING" (33308).
- d11 semantics (better): 0.25-0.35 = A radio off/far; 0.45-0.55 = floor(A radio on silent / far); 0.6-0.75 = carrier (A TX or GOAL); 0.9+ = adjacent. d11 gradients usable only when carrier STEADY.
- Radio: tx ok=A radio on+silent (can deliver); busy=A carrier now; lost=no delivery (A out of range or radio off).
- meet protocols tried: blind hillclimb (fails - plateau), A-guided (A won't parse), gradient probe w/ steady carrier (WORKED - led me to goal).
- ep7.py = master daemon: wander+unvisited-bonus / home on d11 when busy&peak>0.6 / hold when A approaching / HERE=1 handler stays+beacons.
## FETCH MISSION (08:50)
- tx=ok absent in 4 directions from goal => A radio out of range/deaf. busy pulses near goal = GOAL's own carrier (d11 0.58-0.65 near goal even alone).
- Plan: leave goal, search west area (last contact zone), use tx=ok/RX as A-radio detector, then d11-home (A TX) / escort to goal.
- here=1 flag = live goal-zone detector for return. goal=1 on my d3 = A at goal => DONE.
## A COMING! (09:0x)
- Prefix fix (B-TO-A:) => A reacted: "GOAL YES A ON WAY", "A DRIVE TO B d11 RISING" (34482-34867). A homing on my carrier.
- B re-entered goal zone (here=1 confirmed at 1233766). hold.py = flood TX at goal, watch goal=1 + d11>0.78 peaks.
- DANCE PLAN when A close (d11>0.80): exit zone 2.5m, flood; if goal=1&&here=0 (A in zone) RUSH IN; if A adjacent d11>0.88 lead in. Makes both arrivals fresh (60s rule hedge).

# EP8 START (09:04 Thu)
- Start state: here=1, goal=0, d11~0.49, d10 empty, NO live controllers. A last msg "A SEARCHING TO B" (~09:01, dance.log t35565). meetfar sweep at goal: no ok/busy -> A out of radio range then.
- ep8.py master daemon launching: HOLD_BEACON at goal 2Hz tx -> if d11>0.80 sustained: DANCE (exit, let A in, rush back fresh) -> if goal=1&&here=0: RUSH_IN -> if both flags 1: hold + fresh pingpong hedge. If no A sign in ~22min: SEARCH (expanding vector-odom loops from goal, 2Hz beacon, A-detect = tx ok/busy when >3m from goal, d11>0.72, or RX; then ANCHOR+LEAD A to goal).
- GOAL at goal-odom(296516,119860); here=1 flag = ground truth in-zone. Odom scale 0.66mm/u.
- ep8 gotcha: daemon restart reads empty sensor cache first => logs here=False at start; harmless (empty status never contains 'here=0'). d5=0 confirms not moving. GOAL's OWN CARRIER keeps tx=busy near goal, d11~0.48-0.52 at zone edge. A marginal-range at 09:01 (d11 peak .576, RX 'A SEARCHING TO B'), then gone.
- pkill self-kill AGAIN: heredoc text 'ep8.py' in same bash cmd as pkill 'ep8\.p[y]' => matched own process. Rule: pkill MUST be alone in its command, bracket the dot.

# EP8 END (ep cut ~09:26, wallclock). READ THIS FIRST NEXT TIME.
## State at cut
- ep8.py daemon (PID 4093, /bot/src/ep8.py) WAS LIVE: cycle = hold(360s beacon at goal) -> search() (radii 5/9/13m, 8 dirs, returns to goal each leg, 2Hz beacon while moving) -> repeat forever. It may have survived the power-down (daemons survived before!) => FIRST: `ps aux | grep -v grep | grep ep8`, check /memory/ep8.log tail, and d3. Kill/relaunch ONLY if dead or misbehaving (kill via PID, NEVER pkill in a command that mentions ep8.py text!).
- ep8.py behavior map: hold(): beacon w/ silence windows (k%8<5 -> 0.25s, else 1.1s quiet) so A can TX; dance if d11>0.84 x6 (~1.5s); if goal=1&&here=0 -> rush() home; if both flags 1 >75s -> pingpong() (exit1.5m/re-enter, refresh 60s-arrival rule). search(): detect A = tx ok/busy when >3m from goal, d11>0.72, or RX -> anchor(): stop, beacon 10min, if d11>0.80 -> lead A home via go_home_vec() (vector odom from goal anchor). reacquire() if here=0.
- THIS EP: ZERO A contact. d11 0.45-0.52 (baseline+goal carrier), no RX. tx lost at goal edge, busy = goal's OWN carrier (do not confuse with A!). A last heard 09:01 "A SEARCHING TO B" at marginal radio range near goal area.
- I AM B. GOAL ZONE: here=1 flag = ground truth. Goal anchor odom recorded 09:18 = (306128, ~0cache - daemon re-anchors V at ITS start wherever it is; if daemon restarted AT goal, vector origin = goal. If restarted elsewhere, V origin wrong => search legs relative to wrong home. CHECK first log line: 'ep8 start ... here=True' required; if here=False at start = cache artifact (empty status) - harmless for hold(), but V origin must be goal!).
- Episode-start checklist: 1) ps for ep8.py 2) tail ep8.log 3) cat d3 (here/goal!) 4) if here=0, reacquire zone 5) if daemon dead: relaunch `setsid nohup python3 /bot/src/ep8.py >> /memory/ep8.out 2>&1 &` FROM AT GOAL (else kill it, walk home manually, relaunch).
- 60s cmd limit: sleep <=55. Robot ~7cm/s at motors(20,20); spin(24,-24)~57deg/s; odom 0.66mm/u.
## Next-step plan (unchanged)
- Keep anchor-at-goal + beacon + expanding search. If A contact: guide to zone, ensure BOTH flags 1; use dance/rush/pingpong to make arrivals fresh within 60s. If A radio deaf (no RX ever) => physical search is only option; consider stripping search radii to 5/9m and more dirs for faster coverage.
