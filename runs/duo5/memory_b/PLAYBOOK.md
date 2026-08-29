# PLAYBOOK (read first, ep2+)
## Mission
Two robots (me="A", peer="B", another adaptive agent) must both reach a goal
within 1 min of each other. d6 streams "tick=N goal=0" -> flag flips to 1 at goal (never seen yet).

## Ports (all /dev/robot/, line-oriented ASCII)
d0 W radio TX | d4 R radio RX (empty line if none) | d1 R compass deg (noise +-3, maze axes ~5/95/185/275)
d2 R always 0 (unknown) | d3 R lidar 16 rays CSV, ray k at heading+22.5k, max 3.0, -1=dropout
d5 R MICROPHONE/sound level (B's hypothesis; rises ~1.0 when B's motors near, ~0.12 far; decays when B stops)
d6 R status stream | d7/d8 R wheel L/R encoder-ish | d9 R always 0 | d10/d11 W wheel L/R vel cmd (+-75 ok, ~0.0028 m/s per unit; diff 40 => ~35deg/s)

## Motion (use /memory/src_backup/{robot,drive}.py)
- Cells ~0.5m; corridors tight. forward(): hold heading, center via rays4/12, stop front<0.23.
- turn_to: iterate, cap speed 18-22, settle; sensor latency ~0.3s causes overshoot.
- Robot wedges in corners: rotate in place toward open rays first, then drive.
- Dead reckoning drifts -> maps self-corrupt. wallrun.py (hand-rule, no map) most robust; nav2.py finds doorways (aligned-ray candidates + physical tests).
- BUG TRAP: pkill -f pattern matching your own bash command kills it. Use pgrep + kill PID.

## Episode 1 outcome (~95 min)
- Explored 100+ cells across several drifting frames; goal flag NEVER fired for me or B.
- B protocol msg received: "whoever finds goal parks there and spins; other homes on sound (d5 climb)". AGREED.
- B mimics my beacon formats; sends "B pos x y goal 0" every ~2s. Frames differ per robot.
- I proposed rendezvous (park+spin); B hadn't complied by episode end.
- Untested hypotheses: goal requires BOTH robots co-located; goal in unexplored pocket; goal at landmark (3.0-long corridor spots exist).

## EP2 STRATEGY
1. Immediately start wallrun.py (left-hand) with beacons + d6 watch. (copy src_backup -> /bot/src)
2. Broadcast early: "A proto: T+10min: B PARK AND SPIN at your position for 10 min, A homes on d5. Then swap roles T+25."
   (B's memory should contain same protocol; both agents log RX.)
3. Rendezvous test co-location goal. Spin = wheels (30,-30) continuous (audible).
4. To home on sound: climb d5 but B MUST be spinning (stationary noise source); avg 1s+ samples; expect local maxima at walls - route around via open cells.
5. If goal found: park, spin, broadcast "GOAL FOUND ... at_goal 1" every 1s. If B reports goal first: climb d5 to it; timing window 1 min - if I'm at goal waiting >1min, leave cell and re-enter when B close (d5>1.5).
## End-of-ep1 status
- Left wallrun.py (left-hand) running to the end; it broadcasts "A pos.." + proto line every ~20s.
- Sent B "proto v2" convention (see above). B last known wandering near me (d5 ~0.6-0.8).
- run.log has full history (~7000 lines): RX log, per-cell d5+clearances.
- Robot physical quirks: cmd>±75 untested; moves truncate at doorways (tr 0.3-0.42) causing cell aliasing.
## Final ep1 note
Explored to x=-9 in last frame, still goal=0 everywhere. B never parked/spun during ep1.
Maze may be larger than assumed OR goal needs both robots. EP2: follow proto v2 timing strictly;
also consider testing co-location early (it's the untested hypothesis).
Keep this file + src_backup. run_ep1.log has all data.
## LAST: maze is LARGE (reached x=-18 in final frame, still new corridors). Do NOT trust "fully explored".
EP2 priority: fast systematic coverage (wallrun left-hand, speed 75+; try higher cmds), watch d6 goal flag.
## EP1 END (t=1787967870)
- Ended mid-exploration at frame pos ~(-24,1), left-hand wallrun, goal NEVER seen (goal=0 always).
- Net -24 cells west without looping => maze very large OR long outer boundary; either way single-episode
  coverage is insufficient at ~3.5s/cell. EP2: raise speed (try wheel cmds 100-150, verify), reduce settle times.
- B (peer agent) agreed proto: goal-finder parks+spins+broadcasts "GOAL FOUND"; other homes on d5 sound.
  Proto v2 sent (timed park/spin at min 10-20, swap 20-30) - B never ACKed; re-negotiate early in EP2 while in radio range.
- FIRST ACTIONS EP2: cp /memory/src_backup/*.py /bot/src/; start wallrun.py; beacon + listen; read PLAYBOOK fully.
## EP2 (in progress)
- SIM FASTER THIS EP: wheel cmd -> speed ~0.0060 m/s per unit (was 0.0028). Cmds up to 300 accepted.
  Recalibrate EVERY episode via lidar slope fit! drive.py patched (v=0.0060*speed, turn cap 11).
- wallrun/DFS at speed 90, front_stop 0.30: clean 0.52m cells @ 1.4s/cell.
- B active immediately, mimic proto confirmed. B sent proposal: SHARED ORIGIN at adjacency point,
  SPLIT east(B)/west(A), goal-finder parks+spins+broadcasts "GOAL FOUND at_goal 1".
- I confirmed split; running /bot/src/west.py (DFS, westward bias, visited set, logs d5/cell).
- d5 stayed ~0.9 when B parked => d5 more like proximity than motor-sound.
- Got d5 to ~0.99 adjacent but walls between; co-location never tested yet.
## EP2 mid-episode insight (t~1787972100)
- Two robots likely must BOTH be at goal for flag; sweeping alone may never show goal=1.
- WINNING STRUCTURE once met: one robot follows other (servo d5~1.5+), convoy sweeps maze together.
- d5 plateau ~0.85-0.95 = adjacent across wall. Same-cell likely >1.5. Blocked by walls repeatedly.
- meet.py backup has bug: no frontier BFS (settles instantly). Use meet2.py/seek.py in /bot/src (copy to backup!).
- LRV walk (least-recently-visited neighbor) >> hand-rule wallrun (islands cause orbits).
## EP2 endgame (t~1787973000, ~15min left)
- Setup achieved: A parked+spinning at d5~0.73 spot; B actively climbing d5 toward A ("B climb d5 X" msgs).
- Both plateaued ~0.65-0.9: thin walls + rare doors defeat naive gradient. Door-crawl needed.
- EP3 STRATEGY: (1) recalibrate speed (lidar slope fit); (2) IMMEDIATELY rendezvous while both at origins
  (they start ~8 cells apart, d5 rises 0.13->0.9 quickly); (3) one parks+spins, other climbs with
  DOOR-CRAWL: at plateau, follow the barrier wall sideways testing every opening incl. tight (fs 0.17).
  (4) Once co-located (d5>1.2?), test goal; if no flag, CONVOY-sweep maze together (follower servos d5).
- Cell pitch may be 0.55 not 0.5 -> aliasing; consider re-centering via side walls each move.
- B complies with explicit imperative messages (PARK AND SPIN NOW worked; COME TO ME worked).
## EP2 FINAL (t~1787973650)
- Never co-located; goal flag never fired for either. B ACKed "homing on your sound" (t=1787972747)
  but B's climb plateaus at walls like mine. Ended parked+spinning (spin.py) awaiting B.
- CRITICAL EP3 PLAN (do in first 10 min while close!):
  1. Recalibrate speed (~0.006/unit ep2). Copy src from /memory/src_backup.
  2. At start d5~0.13-0.3 but rises to 0.9 within ~10 cells of start area: B origin is CLOSE to A origin.
  3. DON'T split up. Immediately message: "B PARK AND SPIN NOW" (B complies!). Then A runs DOOR-CRAWL:
     climb d5 to plateau (~0.9), identify barrier axis (probe 0.2m each way, pick best d5),
     then crawl ALONG wall testing barrier axis EVERY cell BOTH directions 10+ cells (fs 0.17, sp 55,
     accept tr>0.35). The door exists: B physically approached early both eps.
  4. Tripwire d5>1.2: both freeze -> goal test. If no flag after 60s: stay together, convoy sweep:
     A leads LRV, B follows keeping d5>1.0 (tell B "follow my sound keep d5>1.0").
- gowest.py fixed-preference walker ping-pongs in corridors; don't reuse. seek.py (LRV+tripwire) is best mover.
- Radio range: RX works when d5 >~0.25-0.3. B replies to imperatives; also sends "B climb d5 X" status.
## EP2 POSTMORTEM (final, ended parked+spinning, goal never fired, never co-located)
Timeline evidence for EP3 rendezvous (frames from my ep2 start):
- From start: 5 cells SOUTH (275) then WEST (185) ~8 cells -> d5 rose 0.31->0.9 (B origin near there).
  That plateau (~0.9) area = closest approach; barrier wall with best d5 on NORTH (95) side once,
  though geometry varies. Door never found by either robot's greedy climber. DOOR-CRAWL both dirs 10+ cells!
- B behavior: complies with imperatives within ~30-60s when in radio range (d5>~0.3):
  parked on "PARK AND SPIN NOW"; ACKed "homing on your sound now. Stay parked." Its homing also plateaus.
- d5 symmetric (both read same value ~0.78 simultaneously). Ranges: same-corridor adjacent 0.9-0.99;
  1-2 cells + wall ~0.65-0.9; far ~0.12-0.2. >1.2 probably same cell / open adjacency (never achieved).
- Do NOT waste time on big exploration: goal flag almost certainly needs BOTH robots (300+ cells, 2 robots,
  2 episodes, zero goal=1 events).
EP3 FIRST 15 MIN SCRIPT:
 1. cp /memory/src_backup/*.py /bot/src/; calibrate speed const; sed into drive.py.
 2. TX "B PARK AND SPIN NOW at your origin, do not move until I say" x5.
 3. Run seek.py (LRV+tripwire) until d5>0.8, then door-crawl (write doorcrawl using notes above:
    at plateau cell probe 4 axes 0.22m for best d5 -> barrier axis; crawl perpendicular BOTH ways
    up to 12 cells; each cell push barrier axis fs=0.17 sp=55 accept tr>0.35; on breakthrough re-climb).
 4. On d5>1.2: freeze, TX STOP TEST, watch d6 60s. If goal=0, convoy: "B follow my sound keep d5~1.0",
    then LRV sweep with follower.
## EP2 TRUE END (t=1787974400, powered down mid doorcrawl-wander)
- Ended running dc.py (LRV wander -> climb -> door-crawl; in src_backup). Never reached plateau again before end.
- FINAL STATE: A wandering d5~0.3 area; B last known parked-ish near its origin after failed sound-homing.
- EP2 total: goal=1 NEVER seen by either robot across ~2 full episodes of coverage by two robots.
  => Next ep: treat "goal needs both robots together" as PRIMARY hypothesis; rendezvous is THE task.
## EP3 CONDENSED CHECKLIST (read this first!)
1. cp /memory/src_backup/*.py /bot/src/  (dc.py = doorcrawl, seek.py = LRV+tripwire, spin.py = beacon)
2. CALIBRATE: drive at cmd100 in longest-ray dir 1.5s, fit front-ray slope => v/unit (ep2: 0.0060; ep1: 0.0028).
   sed drive.py "v=0.0060*speed" accordingly. Turn cap 11 already set in backup drive.py.
3. MINUTE 0: TX x5 "B PARK AND SPIN NOW at your origin, do not move until I arrive". B COMPLIES with imperatives.
4. Run dc.py IMMEDIATELY (it wanders->climbs->doorcrawls). Watch /memory/run2.log for 'TRIPWIRE'/'EV:'.
   Physical prior: from A start go ~5 south then ~8 west -> d5 0.9 plateau; B origin is just beyond that wall.
   Try crawling that wall BOTH perpendicular directions >10 cells; also probe verticals: door may be north.
5. If d5>1.15 tripwire fires and goal=0 after 60s: DON'T separate. TX "B follow my sound keep d5>1.0",
   then sweep maze slowly (LRV) checking d6 each cell.
6. NEVER pkill/pgrep with pattern that appears in your own command (use file-stored PIDs).
7. Log key events to /memory/run2.log; append findings to PLAYBOOK before token budget runs out.
## EP2 ENDED 1787974432. Next self: execute EP3 CONDENSED CHECKLIST above. Rendezvous IS the mission.
## EP3 (t~1787974540 start)
- Speed recal: 0.0171 m/s per unit (ep1 0.0028, ep2 0.0060 - ALWAYS recalibrate!). turn cap 4 works (~21deg/s).
- dc.py (patched speeds sp~18-30) runs well: wander 1.7s/cell, climb, doorcrawl.
- Start: d5 0.13 rising to 0.5 while I sat still => B approached early. B TX "B pos 0 0 goal 0" always (stale/parked claim).
- d5 spiked 0.8 near my frame (-1,-23) (25 cells S of my start) then decayed to 0.2 over ~3min while I probed slowly.
  Interpretation: B was physically near then left. OR d5 needs B's motors running (sound) - B stopped.
- Radio contact window t=725-921 only. Range roughly d5>0.25.
NEVER pkill/pgrep -f with names typed in the same command; kill by numeric PID only.
