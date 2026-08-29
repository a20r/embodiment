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
