# Robot maze notes (episode 1)
Ports /dev/robot/ (all line-oriented ASCII):
- d0 write: radio TX (short range). d4 read: radio RX (empty line = nothing).
- d1 read: compass heading deg. Increases turning clockwise (left fwd + right back).
- d2 read: unknown, 0 (bump?)
- d3 read: 16 lidar ranges (m), beam i world bearing ~= heading + 22.5*i. beam0 = front. -1.0 = glitch. seen up to 2.3.
- d5 read: noisy ~0.13-0.16, unknown.
- d6 read: "tick=N goal=0"; tick ~100/s. goal flag -> reach goal.
- d7/d8 read: left/right encoder cumulative ticks. ~4.5 ticks/s per speed unit. ~104 ticks/meter (rough).
- d9 read: 0 unknown.
- d10/d11 write: left/right motor speed (tested -10..10; speed 5 ~0.2 m/s).
Mission: two robots must both reach goal within 1 min of each other. Find other robot via radio d0/d4.
WARNING: pkill -f mon.py kills your own bash too (cmd string match). Use pkill -f 'python3 /bot/src/mon.py'.
mon.py -> /tmp/state.txt latest sensors, /tmp/radio.log nonempty d4 lines.
## Calibration (verified)
- Motor cmd linear; encoders ~4.9 ticks/s per unit cmd. TICKS_PER_LIDAR_UNIT ~ 830.
- Speed 60 ~ 0.36 u/s. ROBOT FORWARD (positive motors) = lidar beam 8 = bearing (d1_heading+180).
- Turning: d10=+s,d11=-s -> heading increases ~1.9deg/s per unit s.
- Robot touches wall when beam ~0.10. Corridor half-width ~0.25-0.3. Keep front>0.30.
- DANGER: pkill/pgrep -f pattern matches your own bash -c cmdline -> kills your shell. Kill by building pattern indirectly, and never in same command that starts things.
- ctrl.py: command FIFO /tmp/cmd. cmds: mot L R | turnto BEARING(fwd) | fwd DIST [SP] [FRONTSTOP] | follow left|right [SECS] [SP] [WANT] | tx MSG | setpose X Y | stop. state->/tmp/state.txt, telem log /tmp/telem.log, radio /tmp/radio.log, beacons "R1 pos x y" every 3s.
## CORRECTED facts (override earlier!)
- FORWARD (positive motors) = lidar RAW BEAM 0 = compass bearing d1. beam4=right(+90), beam8=rear, beam12=left.
- d9 = FRONT bump (triggers when raw0 ~<0.27), d2 = REAR bump.
- d5 = actual motion/speed magnitude indicator (~0.13 idle noise, 0.6-0.9 moving/spinning). Use to detect stall.
- Encoders count commanded rotation even when stalled against wall -> odometry inflated on stall. ~700 ticks/lidar-unit free. speed60 ~0.44 u/s.
- MAZE GRID: cell size ~0.55, wall at cell center front = 0.275, +0.55/cell open (0.82, 1.37, 1.92...). Grid axes ~ compass 0/90/180/270. Corridor half width 0.27; bump at 0.27 front.
- Wall-follow got trapped cycling (maze has loops). Use grid DFS instead.
- Turning: motors(+s,-s) increases heading (bearing clockwise). ~1.9 deg/s per unit.
## Radio protocol
Other robot broadcasts: "A pos <gx> <gy> goal 0" integer grid coords in its own frame, every ~3-6s, short range (~few cells). I broadcast "B pos gx gy goal <flag>" matching. d6 goal flag presumably ->1 at goal. Plan: grid DFS explore (cmd: explore SECS), stop at goal, keep beaconing so A can find/wait.
Grid: cell 0.55, axes ~compass 0/90/180/270. explore uses (0,0) start, +X=bearing0, +Y=bearing90. /tmp/grid.log logs cells. Fine angular scan trick: /tmp/scan.py (reads state.txt).
## Status end-episode-1 (~90min mark)
Working grid DFS explorer in ctrl.py (copy saved at /memory/ctrl.py — START FROM THIS).
Start: nohup setsid python3 /bot/src/ctrl.py > /tmp/ctrl.log 2>&1 &  then: echo "explore 3000" > /tmp/cmd
Robot A wanders nearby broadcasting "A pos x y goal 0" (own frame, resets? unknown). Radio range ~ few cells.
Goal not yet found (d6 goal=0 always). Maze has loops; explored ~13 cells per attempt before restarts.
Remaining issues: occasional backtrack failures (added re-rooting), grid desync after bump-aborted moves.
Endgame plan: reach goal -> stay + broadcast "B ... goal 1"; hope A converges. If A reports goal 1, estimate offset (A vs B co-audible positions) and navigate.
## Episode 1 final setup (autonomous)
- d5 = MICROPHONE (sound level): own motors ~0.6-0.9 while driving; hears robot A's motors when near; ~0.13 silence. Position gradient of idle-d5 ~ distance to A. climb() homes on sound.
- Robot A is agent/interactive: sent "R1 follow me","R1 where is goal","A goal 0 seek","A pos x y goal 0","A climb d5 0.66..". It heard my old R1 beacons.
- ctrl.py (SAVED HERE, copy to /bot/src, run: nohup setsid python3 /bot/src/ctrl.py>/tmp/ctrl.log 2>&1 &):
  cmds via /tmp/cmd FIFO: brain | explore S | climb S | fwd D | turnto B | mot L R | tx MSG | stop
  brain: explore in 240s chunks; if d6 goal=1 -> park+spin+broadcast "goal 1"; if RX contains goal 1 -> climb sound.
- PROTOCOL sent to A: finder of goal parks+spins, other homes on d5 sound.
- Goal never seen yet (d6 goal=0 everywhere so far, ~20+ cells).
- Logs: /tmp/grid.log cells+climb, /tmp/follow.log decisions, /tmp/radio.log, /tmp/telem.log.
## Advice for next episode (READ FIRST)
1. Copy /memory/ctrl.py to /bot/src/ctrl.py, start it, send "brain" to /tmp/cmd. That resumes autonomous explore+goal protocol immediately. Verify motion via /tmp/grid.log cell lines.
2. Big open problems:
   - explore() loses map on re-root/restart -> re-treads area. Consider persistent map anchored by compass+walls, or Pledge-augmented wall follow.
   - move_cell failures desync grid pos (bump aborts). recenter() helps only fwd axis.
   - Goal location unknown; d6 goal flag never fired anywhere in ~40 cells around start. Maze extent unknown (lidar max seen 2.99). May need long-range exploration; consider biasing DFS to unexplored compass directions (e.g., go far one way).
   - Robot A (other agent) cooperates via radio: address it in plain text; agree on protocol early. It said "R1 follow me" and "seekingB". Consider asking A to split search space e.g. "A search north half, B south".
3. Rendezvous tool: d5 mic gradient (climb cmd) while A moves/spins works only in radio-ish range (few cells).
4. Episode1 timeline: ~45min calibration+debugging (all correct facts saved above), rest exploring; no goal seen.
## Agreed radio protocol with A (t=1787966721)
"A proto: agreed. if you find goal: park+spin+broadcast GOAL FOUND. I do same. Also try: maybe goal needs both co-located; if your d5>1.5 park and spin, I will come."
-> Next episode: honor this. Broadcast "GOAL FOUND" (that exact phrase) + park + spin at goal. If RX d5 hint/GOAL FOUND -> climb sound.
Episode1 ended with both still goal 0, wide DFS coverage, no goal flag anywhere near start region.
## Final status episode 1 (t~1787967100)
brain autopilot running (explore chunks). No goal flag ever. A last near "(A frame) -4,0". 
Next episode priorities: (1) restart ctrl.py+brain instantly; (2) coordinate with A to partition search or travel far in one compass direction (goal likely outside ~10-cell radius of start); (3) implement persistent global map using compass-aligned dead reckoning with wall-snapping to survive re-roots; (4) honor GOAL FOUND park+spin protocol.
## EPISODE 1 FINAL UPDATE (CRITICAL FIXES INCLUDED IN /memory/ctrl.py)
1. LIDAR -1.0 = NO ECHO = OPEN SPACE BEYOND RANGE (~3.0), NOT a glitch/wall! This bug crippled all early exploration (robot thought open corridors were walls, kept cycling in a ~6-cell pocket). Fixed in walls_here/move_cell/front3. After fix, robot covered 1 cell per 2-3s smoothly.
2. Latest ctrl.py has: gofar BEARING SECS (persistent push in compass direction, works well), explore SECS (DFS), climb SECS (home on d5 sound), brain (autonomous: random-direction far sweeps + explore, parks+spins+broadcasts at goal, climbs sound if RX 'goal 1').
3. Maze is LARGE: robot A reported its pos (-21, 0) in A-frame. Goal likely far from spawn. gofar sweeps + explore is right approach. A agreed protocol: finder of goal parks+spins+broadcasts "GOAL FOUND"; other homes on d5 mic gradient (climb).
4. NEXT EPISODE STARTUP (fast):
   cp /memory/ctrl.py /bot/src/ctrl.py
   nohup setsid python3 /bot/src/ctrl.py > /tmp/ctrl.log 2>&1 &
   echo "brain" > /tmp/cmd
   Verify progress: tail /tmp/grid.log (cell/gofar lines advancing), /tmp/radio.log for A, grep goal /tmp/state.txt.
5. Time sinks to avoid: pkill/pgrep -f self-kill trap (see above); multi-sleep >60s commands get killed; don't read /dev/robot pipes while ctrl.py runs (line stealing) — use /tmp/state.txt.
6. Unverified hypotheses: goal may need both robots co-located; d6 goal flag never observed as 1. A is another LLM agent — negotiate search partition early (e.g., "A take west, B take east"), and consider staying within radio range to share the goal find.
## Episode 2 (t~1787968400)
- ctrl2.py (SAVED /memory/ctrl2.py) = ctrl.py + persistent frontier explorer.
  New cmds: brain2 (autonomous: fexplore chunks + goal/heard-goal handling), fexplore SECS.
  Map persists /tmp/map.json {pos,cells{ "x,y":{bearing:open/wall/fail} }}. BFS to nearest unknown-neighbor edge, walk, sense.
- Startup: cp /memory/ctrl2.py /bot/src/ctrl2.py; nohup setsid python3 /bot/src/ctrl2.py>/tmp/ctrl2.log 2>&1 &; echo brain2>/tmp/cmd
- Kill trick that works: pkill -f "[c]trl" (bracket avoids self-match).
- Progress log: /tmp/grid.log "fx [x,y] n=N" lines.
- KILL RULE (burned twice): pkill -f matches ANY occurrence of the target string in your own bash -c line (heredocs incl. filenames!). Only safe: `pkill -x python3` in a bash call that contains no other python processes running of mine (my heredocs finish before). Then restart ctrl2 in a SEPARATE call.
- After killing controller ALWAYS: echo 0 > /dev/robot/d10 and d11 (motors keep last speed!).
## Ep2 progress (t~1787969200)
- brain2/fexplore stable after fixes: align-before-sense, sticky-open edges, fail counts (>=2 blocks), no-frontier -> clear fails -> wipe map.
- Met robot A physically at ~t=1787968900 (d5~0.95 when adjacent, moves fail from bumping it).
- A's protocol (its words): "if d5 high I will come to you. If you find goal: PARK+SPIN+broadcast GOAL FOUND. If I find it, I do same and you home on d5."
- Proposed split (me east / A west); no explicit AGREE received.
- Coverage ~50 cells by t=969212, no goal flag yet.
## Ep2 mid-episode pivot (t~1787972200)
- Grid fexplore kept failing in NW region: walls NOT axis-aligned there (lidar shows diagonals). Repeated FRONTBLOCK/BUMP at same signature, map wipes.
- mc fail signature analysis: bump often from diagonal corner clip (beam+-1,+-2 ~0.1) -> added diagonal avoidance + recenter2 (both axes). Helped but region still toxic.
- PIVOT: brain3 = wall-follow roamer (follow left/right random 80-150s + dash to most open beam), goalflag checked inside follow/fwd loops. Much faster transport.
- Robot A rendezvous attempts (~15 min wasted): we orbit each other, d5 plateaus ~0.8 behind walls. A's plan: both wander; if d5>1.3 freeze + broadcast STOP TEST (watch.py does freeze; d5-only trigger).
- KILL RULE refined: pkill -f "[c]trl2.py$" is safe (own cmdline doesn't end with ctrl2.py).
## Ep2 findings (IMPORTANT for future)
- CO-LOCATION HYPOTHESIS TESTED NEGATIVE: B and A were adjacent (d5~0.95, physically bumping) for several minutes ~t=1787968900-970700; both d6 goal=0 the whole time. Plain co-location does NOT trigger goal.
- Maze has NON-AXIS-ALIGNED (diagonal/curved?) walls in some regions -> grid mapper unusable there; wall-follow (brain3) is the robust transport.
- Distinct trap signature seen repeatedly (diagonal funnel?): front open ~1.15 but bump with beams2-3 ~0.12; or 10 consecutive beams 0.23-0.48. Region near where B+A met.
- A's agreed protocol (final): both wander; d5>1.3 -> freeze+broadcast STOP TEST; finder of goal PARKS+SPINS+broadcasts GOAL FOUND, other homes on sound (climb).
- brain3 = follow(random side, 80-150s) + dash to most open beam; goalflag checked in follow/fwd/spin loops. Running from t~1787972230.
## Ep2 endgame summary (t~1787973300, ~90min mark)
GOAL NEVER FOUND by either robot in 2 full episodes. Combined coverage large but frames unshared.
Sound-homing (climb) to a parked spinning robot FAILED repeatedly: d5 gradient plateaus ~0.6-0.8 behind walls, oscillates; wall-follow circumnavigation also failed to break plateau. We never achieved d5>1.2 except once by accident early (bumping into each other while both exploring).
### STRATEGY FOR EPISODE 3 (do in this order)
1. Startup (2 min): cp /memory/ctrl2.py /bot/src/ctrl2.py; nohup setsid python3 /bot/src/ctrl2.py>/tmp/ctrl2.log 2>&1 &; echo brain3>/tmp/cmd  (brain3 = wall-follow roam + goal checks; most robust transport). Zero motors first if a stale process was killed: echo 0 > /dev/robot/d10; echo 0 > /dev/robot/d11.
2. DO NOT attempt rendezvous with A unless d5>1.0 already. It burned ~30 min over 2 episodes. Co-location does NOT trigger goal (tested).
3. UNEXPLORED LEAD: long open corridor sighting (3+ beams >2.4 or -1): search telem for it, then push that compass direction persistently. Ep2 saw one at fwd~17deg (compass ~0-20, i.e. north-ish) from near the A-meeting area.
4. Consider that goal may be FAR outside the ~25x15 region we mapped. Persistent single-direction pushes (gofar) cover distance fastest in grid-clean areas; brain3 elsewhere.
5. Radio: A is an LLM agent, follows own plans, messages get through when close. Agree quickly: both roam; finder of goal parks+spins+broadcasts GOAL FOUND; other homes. Arrival window is 1 min - finder should EXIT and RE-ENTER goal cell when other robot is adjacent (d5>1.0) to sync arrival times.
6. TOKEN/TIME BUDGET: calibration is DONE (all facts in this file). Spend <5 min on setup, rest on coverage. Poll logs every 2-3 min with short cmds. sleep<=55s per command.
## Ep2 final (t~1787973550)
brain3 roaming at end; goal=0. A last RX t=1787972964 (parked spinning waiting for me - it may still expect homing; tell it new-episode plan promptly).
### Ideas not yet tried (for ep3)
- Systematic PLEDGE-style: wall-follow LEFT ONLY for very long periods (traces entire connected boundary incl. outer wall -> guaranteed to visit goal if goal is wall-adjacent). Ep1 saw cycles, but with dashes to break to inner components it's decent.
- Push ONE compass direction to the maze BOUNDARY, then wall-follow the boundary the whole episode (outer wall likely reaches goal region eventually).
- Ask A to do opposite boundary direction (I go CW, A CCW along outer wall) - guaranteed meeting AND full boundary coverage.
- Watch lidar for 3+ beams >2.4/-1 (big space) and investigate immediately.
- d6 tick rate ~100/s wall clock; use for timing.
- Sent to A at very end: proposal for ep3 = both follow OUTER boundary wall, me LEFT-hand rule, A RIGHT-hand rule (full boundary coverage + guaranteed meet). Unconfirmed.
- Robot left running brain3 at episode end.
## Ep2 overtime
- Added pledge(bearing,secs) to ctrl2.py: straight-line compass push + left wall-follow detours (Pledge algo, handles diagonal walls). Works: covered x 0->8 units east in ~3 min where grid mover failed.
- Long-corridor sightings repeatedly along bearing ~0/180 axis (readings 2.7-3.0). Maze likely elongated along that axis. Ep3: pledge 0 far, then pledge 180 far.
## EPISODE 2 FINAL (read after ep1 notes; this supersedes tactics)
Ended t~1787974430. GOAL NEVER FOUND (goal=0 everywhere, both robots, 2 episodes, huge combined coverage near spawn region).
### What is PROVEN
- Co-location of both robots does NOT trigger goal (adjacent minutes, both goal=0).
- d5 sound-homing through walls FAILS (plateau 0.6-0.8, oscillation); only works line-of-sight/last 1-2 cells. Do not burn time on mid-maze rendezvous.
- Grid mover fails in regions with diagonal/curved walls (repeated FRONTBLOCK/BUMP signature); wall-follow + pledge are the robust movers.
- pledge(bearing,secs) in /memory/ctrl2.py = compass push w/ left-wall detours: best long-distance transport (~2.5 units/min incl. detours). BUT in loopy regions left-detours can orbit back: alternate detour side or cap detour time.
### EPISODE 3 PLAN (fast start)
1. cp /memory/ctrl2.py /bot/src/ctrl2.py; nohup setsid python3 /bot/src/ctrl2.py>/tmp/ctrl2.log 2>&1 &  (zero d10/d11 first if killing stale proc: pkill -f "[c]trl2.py$" safe)
2. echo "pledge 0 600" >/tmp/cmd then "pledge 180 600" (long axis of maze is ~bearing 0/180: repeated 2.7-3.0 lidar corridors that way). Then brain3 (wall-follow roam) in new territory. Poll /tmp/state.txt every 2-3 min (sleep<=55s per cmd!).
3. Radio A early: both roam; finder of goal PARKS+SPINS+broadcasts GOAL FOUND; other homes on sound ONLY when close (d5>1.0); finder EXIT+RE-ENTER goal when other adjacent (1-min joint arrival rule).
4. Goal is likely FAR from spawn along the 0/180 corridor axis - go DEEP one direction before exploring locally. Don't re-explore spawn region: it is exhausted (~60+ unique cells, no goal).
5. Budget from ep2: setup 3 min, ~15 min lost to rendezvous attempts (skip!), ~25 min lost debugging grid mover in diagonal region (skip: use brain3/pledge from the start).
## Ep3 (t~1787974577 start)
- World clock continues across episodes; robot stays where last episode left it.
- pledge 0 orbited a loop x0..8; brain3 also ping-ponged a fixed circuit x0..10 (region has big loop corridor).
- fexplore got 22 cells then STUCK+wipe at diagonal region (same ep2 signature).
- KEY INSIGHT: radio is short range -> split-search endgame broken (finder's GOAL FOUND unheard). Correct strategy = PAIR-TRAVEL: meet A, then convoy (same wall-follow wall+direction, radio tether, beacon every 3s, backtrack if silent 60s), finder waits, both enter goal within 1min.
- Rendezvous technique that WORKED: A parks+spins, I climb d5 to ~0.9 plateau (one wall away), then ROLE SWAP: I park+spin, A doorcrawls to me. d5 while I spin: baseline 0.88, rises as A nears.
## EPISODE 3 FINAL (t~1787977600) - READ THIS FIRST NEXT TIME
STATE AT END: I am parked at pose-frame (8.6,4.5) [ep3 frame], ROCKING loudly via /bot/src/rock.py (copy in /memory/rock.py) running detached; ctrl2 also running (idle, beaconing). A is 1 wall NORTH of the plateau wall, actively doorcrawl-hunting me by sound. d5 idle here 0.6-0.9 depending on A's position. GOAL STILL NEVER SEEN (d6 goal=0 3 episodes).
GEOMETRY LEARNED (ep3 frame, pose units): dead-end POCKET at (9.45,3.97): N0.38 E0.64 S2.59 W0.38, idle d5 0.92 there = A loudest, A is NORTH/EAST behind wall. Corridor runs S from pocket, bends SW, joins the big loop circuit (x0..10) that pledge/brain3 orbited. NO side openings >0.45 found along corridor (checked ~10 cells both sides).
SOUND FACTS: spin-in-place +40/-40 is QUIET (~0.25) - useless as beacon! DRIVING/ROCKING (mot +-42 both wheels alternating 0.6s) is LOUD. Rocking keeps position. When A is 1 wall away idle d5 ~0.85-0.93; same room ~0.95+; A far ~0.25.
FWD GOTCHA: fwd() front3 includes side beams(+-1)+0.12 AND beam15/1 - when scraping a wall it refuses to move (blocked f=0.36 spam). Fix: turnto angled away from wall + frontstop 0.24-0.28. Cmd handler DROPS commands while previous is running - always echo stop, sleep 1, then next cmd.
EP4 PLAN:
1. Startup ctrl2 (see top of this file). Listen first: idle d5 tells if A is near. Check radio.
2. If A near (d5>0.5 or RX): resume meet protocol - I rock loud in place (bash loop or /memory/rock.py detached), A doorcrawls. FREEZE at d5>1.05, say STOP TEST. We were VERY close (1 wall) at ep3 end.
3. After meeting: PAIR-TRAVEL sweep (radio tether, beacon 3s, if silent 60s backtrack). Split-search is BROKEN (radio too short for finder to call other in).
4. The wall between me and A: my side has NO door within corridor cells checked; door must be on A's side or via long path around the loop (S corridor bends SW joins loop; loop may connect N somewhere). Consider ME wall-following LEFT starting northbound from pocket to circumnavigate to A's corridor.
5. Do NOT waste time: pledge 0/180 orbits the loop here. fexplore dies at diagonal walls (~22 cells). Interior unexplored cells remain in loop region; goal maybe in an interior pocket - pair-sweep them AFTER meeting.
## Ep3 last minute: A said "found doors far west side, coming around to your column from west, keep rocking, freeze at d5>1.05". Left rock.py rocking at pose(8.6,4.5) ep3-frame + ctrl2 idle. My rocking self-noise reads d5~0.70; watch for rise >0.9 = A close, >1.05 = freeze + tx STOP TEST. EP4: first command = check d5/radio; if A adjacent FREEZE + coordinate goal search TOGETHER from then on.
## Ep3 overtime: A NAVIGATION BY COMPASS WORKS (compass frames are shared!). Gave A explicit route (south mouth of my N-S corridor, heading 350 then 013). A acked and got d5 to 0.905 (very close) but drifted past bend; still hunting tight doors at south mouth at episode end. d5 log while I rock: far 0.6, corridor approach 0.9+. EP4: restart rock beacon immediately, tx A same route text, guide with compass headings + my clearances. A's msgs now adaptive/acknowledging.
## Ep3 VERY END (t~1787980000): moved to sweet spot idle-d5 0.93-0.96: ep3-pose (8.52,4.20), = ~0.3 SOUTH of the blocked north face (b0 0.26 at heading 010), corridor opens south (190,3.0m). A is ONE CELL NORTH behind that wall testing tight doors. rock.py left RUNNING there + ctrl2 idle. EP4 FIRST MOVES: check idle d5; restart rock.py at same spot (don't move!); tx A: "B back, rocking at same junction, continue door hunt / guide me". If d5>1.05: freeze + tx STOP TEST. Then plan TOGETHER via radio (A responds adaptively now): pair goal sweep.
## Ep3 last: upgraded beacon to rock2.py (+-55, 1.0s, BIG amplitude, /memory/rock2.py) per A request (A watches lidar for moving obstacle). Left running. A freeze threshold now 1.02. Ep4: check pgrep rock2, keep it going, don't move, let A finish the door hunt; talk to A via tx (it answers).
## EPISODE 3 ABSOLUTE END (t~1787980560)
Robot left: motors 0, ctrl2 running, rock2 KILLED (I was probing pocket E-gap when time ran out). A still 1 wall away "in your 0.9 zone exploring every opening, watching lidar for your rocking; FREEZE at d5>1.02".
ROUTE FROM LOOP TO MY JUNCTION (I walked it, compass turn-by-turn, works both directions):
  loop bend area -> heading 349 ~2.5m (wiggly, hug right wall) -> heading 013 ~1.5m -> junction (me). Reverse: from junction heading ~190 x3m then ~172-160 x2m into loop.
POCKET E-GAP (0.64m at old pocket): never successfully re-found after rocking drifted my pose ~0.5-1.0 unit (rocking at +-55 drifts! use +-42 0.6s if position matters).
EP4 FIRST 3 MIN:
 1. cp+start ctrl2 (see START_HERE). Read idle d5: ~0.8-0.9 = A still adjacent!
 2. Restart rocking beacon (rock.py small-amp, keeps position) + tx: "B back. Rocking. Same protocol: freeze at d5>1.02, STOP TEST."
 3. tx A the turn-by-turn ROUTE above (A follows compass instructions well - PROVEN, it acked and reached 0.905).
 4. Ask A: "REPORT your N E S W clearances + last heading" -> locate A vs my map -> give explicit turns. This 2-way geometry talk is the breakthrough tool; A answers adaptively.
 5. If still stuck 15 min: BOTH exit to the big loop corridor (open, line of sight) and meet there: I take my reverse route S to loop, A does same on its side; in open corridor sound+lidar homing works.
AFTER MEETING: pair-travel sweep for goal (radio tether 3s beacons, backtrack if silent 60s). Goal never seen anywhere near spawn/loop region in 3 eps - sweep FAR territory together.
