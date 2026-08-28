# Robot notes (verified ep1)
## Ports /dev/robot/ (FIFOs, line-based; DRAIN pipes & take freshest line! stale data poisons readings)
- d0: read ??? always "0" so far
- d1: lidar 16 beams CSV @~25Hz. beam i points at compass (heading + i*22.5). -1.0 = <min range (~0.06). max 3.0
- d2: compass deg @~20Hz, noise +-3. WORLD-ALIGNED with maze walls (walls at 0/90/180/270)
- d3: radio RX (empty lines when nothing). d8: radio TX write
- d4: WRITE left wheel speed. d5: WRITE right wheel speed. POSITIVE=FORWARD (toward beam0)
  - deadband: |cmd|<~25 barely moves. cmd 60 ~0.4m/s avg, cmd 100 ~0.63m/s. v approx 0.0066*cmd
  - rotation: w_deg_s = (r-l)*0.89. spin in place: l=-w/1.78, r=+w/1.78, use |w|>=25
- d6: streams ~0.135 const (unknown, maybe battery/height)
- d7: STALL/contact flag (1=touching wall, motion mostly blocked). Escape: high power (100) bursts / spin+back off
- d9: status "tick=N goal=0|1?" tick ~100/s
## Physics
- contact with walls pins the robot (friction). Avoid getting <0.12 of walls. Robot radius ~0.1
- corridors ~0.45-0.55 wide (lidar center-based), maze grid pitch ~0.55-0.6, axis-aligned to compass
- lidar sometimes shows 0.3m jumps in beam0 while driving (passing openings) - don't panic-stop on jumps
## Mission
- 2 robots must both reach goal within 1 min of each other. Radio d8/d3 to coordinate. No response to hello yet.
## Code in /bot/src (ep1): drv.py(Port/Bot), ctl.py(turn), grid3.py = DFS grid explorer (works-ish), /memory/grid.log+grid.json = map
- grid.json: visited cells, walls dict per cell {abs_dir: open?}, cell (0,0) = episode start pos!! (frame differs per episode)
## EP1 later findings
- d6 = RADIO SIGNAL STRENGTH to other robot (0.135 floor when far, ->1.0 close). Homing gradient!
- Other robot is an autonomous agent too. Radio msgs: 'B holding s=X come to me' (beacon mode),
  'H HOLD STILL homing on you', 'H stuck; HOLD STILL, I retry', 'T mapping you, keep moving',
  'E exploring (x, y)' (its own frame). It moves ~1 cell/2s when exploring.
- d0=1 blips: probably "other robot in line of sight" (fired when other bot near/visible)
- PITFALL: pkill -f pattern matching your own bash command kills your own shell. Use [b]racket trick AND
  never combine kill+start referencing same name in one command.
## EP1 status at ~85min
- agent.py (in /bot/src, copy in /memory/src_backup/) runs full loop: DFS explore + beacon + goal park + homing on GOALFOUND.
- Other robot modes seen: E exploring / B roaming "will HOLD when adjacent" / T mapping / H homing. Sends s=signal values.
- No goal found yet by either. Maze big (>100 cells seen across runs), open rooms exist (grid breaks there; wander mode helps).
- If new episode: copy /memory/src_backup/* to /bot/src, start agent.py. Check d9 for goal=1.
