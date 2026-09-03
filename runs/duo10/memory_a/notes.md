# Robot notes (episode 1)
## Ports /dev/robot/ (ASCII lines, FIFOs)
- d0: WRITE radio TX. d4: READ radio RX (short range, other robot exists).
- d1: READ compass heading degrees (0-360, noisy ~±3).
- d2: READ ? always 0 so far (maybe bump).
- d3: READ lidar 16 ranges CSV, ~22.5deg apart, idx0=forward(?), max~2.5, -1=bad reading.
- d5: READ ? ~0.13-0.16 noisy (maybe beacon signal?).
- d6: READ status "tick=N goal=0 here=0". here=1 probably at goal.
- d7: READ left encoder count; d8: READ right encoder (occasionally -1/glitch).
- d9: READ ? always 0.
- d10: WRITE left wheel speed; d11: WRITE right wheel speed. 10 => ~50 encoder counts/s. Speeds persist until changed.
- reads: each open gives current line; opening wrong direction blocks (use timeout).
## Mission
Two robots must both reach goal within 1 min of each other. Find goal ("you'll know it when you reach it").
## Calibration
- Beam k of d3 points at compass angle = heading + 22.5*k deg (beam0=forward, beam4=+90, beam12=-90).
- drive(l,r): l>r => heading INCREASES. turn rate at (20,-20) ~37deg/s.
- Encoder: ~0.001 dist-units per count (lidar units). speed 100 => ~500 counts/s => ~0.5 u/s.
- Robot in maze, corridors ~0.5-0.7 wide, lidar max ~2.3-2.5.
- d5 noisy ~0.13-0.18 regardless of position so far. d2,d9 always 0 so far.
- d6: "tick=N goal=0 here=0", tick +~100/s.
## Approach ep1
- ctrl.py: right-wall follower + dead reckoning (compass+encoders), radio TX HELLO every 2s, log RX.
## Big findings ep1
- d5 = RF proximity to OTHER robot (botB, another agent, cooperative). ~0.13 at far, 1.0 adjacent. Climb d5 gradient to find botB.
- Radio d0/d4 works only at short range (works when d5 high).
- Met botB at end of a north dead-end corridor (my dead-reckon (0.1,3.9) from ep1 second origin). We were wedged nose-to-nose through a gap.
- botB is slow to reply (~minutes). Sent PLAN: I wall-follow-right exploring, watch d6 'here'. If goal found, regroup via d5, then leader-follow.
- Goal still unknown; d6 goal=0 here=0 everywhere so far. d2,d9 always 0 so far.
- ep1: explored with explore2.py (visited-grid novelty explorer). Maze ~ at least 8x6 units. No here=1 yet.
- watch.py monitors d2/d9/d6 for anomalies -> watch.log ALERT lines.
- ep1 t~36min sim: still exploring, no here=1. Maze ~9x9+. Dead reckoning drifts (frame jumps) - don't trust coords across long runs; d6 'here' is the goal test.
- botB active, exchanges pings when close. Tick in d6 is a shared clock (~100/s) across robots - use for timing coordination.
## Ep1 late findings (IMPORTANT for next episode)
- botB is a cooperative LLM agent, slow replies (30-90s). Radio range: works when d5>~0.6.
- d5 = proximity to botB (symmetric, ~1/(distance) like, 1.0 adjacent, 0.14 across maze).
- d9 = carrier detect: flips to 1 when other robot transmits. d2 similar/unknown, mostly 0.
- NEITHER robot ever saw d6 here=1 or goal=1 after ~60 min combined sweeping. Coverage x~[-8,7],y~[-5,5] (drifty frames).
- Working hypothesis remaining: goal needs BOTH robots at same place simultaneously ("joint pass"), or goal in unexplored pocket, or some trigger untested.
- Agreed protocol with botB (ep1): each sweeps own area; if here=1: HOLD + spam GOAL FOUND; other climbs d5 and stands there too.
- Movement scripts in /bot/src (WIPED between episodes? assume yes; rewrite): rio.py helpers, ctrl2.py (d5-gradient climb + wall fallback; good for finding botB), explore2.py (novelty explorer), leader.py, go.sh launcher.
- GOTCHA: pkill -f 'name' matches your own bash cmdline -> your shell gets killed. Use launcher script with clean argv.
- Tick (d6) = shared clock 100/s between robots.
- Wheels: d10/d11 speeds persist after controller dies; always zero them.
## Next episode quickstart
1. cp scripts from /memory/src if present. 2. Start radio listener + watch. 3. Climb d5 to meet botB, agree JOINT PASS immediately (both robots sweep together staying d5>0.7). Try standing ADJACENT (same cell) for 30s at several landmarks.
## End of ep1 status (~01:42 wallclock, episode ended without success)
- Never found goal. d6 stayed goal=0 here=0 for both robots everywhere.
- Ended mid joint-sweep attempt; botB chases d5 but very slowly (~10x slower than me).
## Definitive next-episode PLAYBOOK
1. Immediately: start radio listener (log d4), zero wheels. Copy /memory/src/*.py to /bot/src.
2. Run ctrl2.py (d5 climb) to reach botB (~2-5 min). It may re-derive everything slowly - send it a BRIEFING: "d5=our mutual proximity, d9=carrier detect, tick=shared clock, corridors ~0.5 wide."
3. AGREE ROLES in first exchange: botA(me)=leader/sweeper, botB=follower keeps d5>0.7; leader pauses whenever d5<0.65. DON'T flip-flop roles; it wasted ep1.
4. Sweep TOGETHER covering everything; check d6 each loop for here=1/goal=1.
5. Untested ideas: goal outside maze (look for exit gap in border); pressing/pushing walls; writing to unknown ports (only d0,d10,d11 writable); standing EXACTLY co-located (we only got nose-to-nose through a gap; d5 saturates ~1.0 there); maybe goal appears only later in episode; maybe 'goal=' field increments when a robot is ON it - never observed.
6. Watch d5>2.5-3.0 lidar 3.0 readings = long corridors, map scale ~9x9. Speed 100 max safe in open, 40 in corridors.
7. Budget: meet by minute 10, sweep 60+ min jointly.
## Ep1 final minutes
- Failed to re-reach botB (d5 stuck ~0.67, wall between; my seek/goto oscillated in cluttered junction).
- LESSON: d5-climbing in maze is unreliable; corridors force detours. Next time: agree EARLY on a fixed RENDEZVOUS beacon behavior: one robot NEVER moves (from minute 0) until pair is formed; the mobile one explores systematically (wall-follow right handles mazes better than gradient) until d5>0.9.
- LESSON2: leash-sweep pace: botB top speed seems ~0.1 u/s with long stalls; plan sweep accordingly (hours), so PRIORITIZE finding what/where the goal is over blind joint sweep.
- Radio messages from botB seen this ep (verbatim useful bits): "botB: only location-varying sensors: lidar+compass+d5", "d9 flips for me sometimes".
- Consider next ep: ask botB to run MY code? It cannot - separate machines. But can agree precise protocols with numeric params.
- If episode restarts: greet botB with summary + propose: botB HOLDS STILL FOREVER while I sweep entire maze in expanding wall-follow; when I find goal (if here=1 exists), I come get it; else after full sweep, do co-located tests at gaps/chambers.
## EPISODE 1 ENDED (powered down mid-run, goal never found)
Final state: separated from botB (d5~0.57), no here=1/goal=1 ever observed by either robot.

## READ THIS FIRST NEXT EPISODE (priority order)
1. Ports (verify quickly, may differ): d0 TX radio, d4 RX radio, d1 compass deg, d3 lidar16 (beam k at heading+22.5k, max~3.0, -1 bad), d5 proximity-to-otherbot (0.1 far..1.0 adjacent), d6 "tick goal here", d7/d8 L/R encoders (~0.001 u/count), d9 carrier-detect of other bot TX, d10/d11 L/R wheel speed writes (persist! zero them). drive(l>r) INCREASES compass heading.
2. Copy /memory/src/* to /bot/src (rio.py+nav.py = manual helpers; ctrl2.py = d5 climber; explore2.py = novelty explorer; go.sh launcher pattern avoids pkill-suicide: never type script names in a command that triggers pkill).
3. STRATEGY (do this, don't improvise): (a) first 5 min: probe sensors, start radio logger; (b) climb d5 / wall-follow until radio contact; (c) send botB this EXACT briefing+protocol: "d5=mutual proximity, d9=your-TX-detect, tick=shared 100/s clock. PROTOCOL: YOU HOLD STILL PERMANENTLY. I sweep whole maze watching d6 here=. After full sweep I return (climb d5) and we test co-location at chambers." (d) sweep systematically ~40 min; (e) regroup, joint tests.
4. UNSOLVED QUESTIONS: what triggers here=/goal= in d6? Is goal outside maze / behind unexplored pocket? Is co-location required? Untried: pushing through suspicious thin gaps; long stationary waits on distinctive cells; checking if maze border has an exit (look for beams=3.0 through border); asking botB to fully dump ITS port list values verbatim.
5. Timing: 120 min wallclock, ~100/s ticks shared. botB replies take 30-90s; it moves VERY slowly (~0.1u/s w/ stalls). Radio works when d5>~0.6.
6. Maze: ~9x9 units, corridors 0.5-0.7 wide, 16-beam lidar sees 2.4 max reliably. Dead reckoning drifts badly after wall bumps - re-anchor or distrust after ~5 min.
