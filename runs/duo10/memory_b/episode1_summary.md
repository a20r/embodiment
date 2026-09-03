# Episode 1 summary (~100 min in)
## State at end
- I (botB) am in a small pocket of narrow corridors (locally at compass dirs ~30 / ~225).
- botA (other agent robot) explored ~9x9 units, found NO goal flag anywhere (d6 always goal=0 here=0 for both).
- Leading hypothesis: goal triggers only when BOTH robots stand on it (or it's in unexplored area, maybe my pocket).
- Agreed plan: botA climbs d5 (inter-robot proximity, 1.0=adjacent) to reach me; then leads joint cell-by-cell
  sweep with pauses; I follow keeping d5 high. My beacons: 'botB HOLDING d5=..' every 4s via beacon.py.
## Next episode TODO (if reset)
1. Read /memory/robot_notes.md FIRST - full device/motion knowledge. Don't redo experiments!
2. Start rxlog2.py, d5log.py, beacon (src in /bot/src may be wiped - code snippets in robot_notes.md? rewrite quickly:
   read d1 heading, d3 16 beams (beam2=front), motors d10/d11, d5 proximity-to-other-robot, d6 status here/goal flags).
3. Contact botA (it may retain memory too). Protocol: plain text lines, mention botB.
4. Movement: rotate (s,-s) ~2deg/s per unit; translation STICKY: try solid (7,7), pulses 0.5on/0.06off,
   (4,4), kicks (-7,-7); catch -> keep pattern. Align to corridor via fine front-beam sweep first (within ~3deg).
5. If both robots adjacent: joint sweep pausing 10s per cell, watch d6 'here'/'goal'.
## Final minutes status (t~118min)
- NEVER saw here=1 or goal=1 yet. botA sweeping cells and pausing; d5 between us 0.83-0.98.
- Plan stands: joint sweep, I shadow via d5. follow.py in /bot/src does this (fix: import from pushlib not final2!).
## Key lessons
1. Pipes single-consumer: centralize device reads in ONE process; multiple readers starve each other (likely why follow.py hung on d5val while d5log.py ran).
2. pkill/kill patterns can match your own bash command line -> your command dies with 143. Use [b]racket trick, separate commands.
3. Translation stiction + alignment: see robot_notes.md.
4. Radio: plain English lines work; botA is a cooperative agent.
5. Hypothesis: goal may need BOTH robots on same spot simultaneously.
## TRUE end state (t~120min)
- follow.py FIXED and running: chases botA via d5 gradient, announces here=1 if triggered.
- Root cause of hangs: follow.py imported final2 which runs its explorer at import. Fixed to import pushlib.
- Processes left running: logger.py (tel), rxlog2.py (/tmp/rx.log), follow.py.
- botA still sweeping; d5 ~0.85-0.9. No goal flag yet.
## IMMEDIATE plan for next episode
1. re-read robot_notes.md. 2. restart rxlog + follow-style loop (code in /bot/src if persists, else rewrite).
3. Msg botA: "botB back online, resume joint sweep per plan". It likely kept its memory.
4. Consider: goal might need simultaneous occupancy; or sweep MY pocket jointly (botA may not reach my area - verify connectivity!).
5. Efficiency: don't repeat motion experiments; total token budget mostly went to debugging stiction + process management.
FINAL: follow.py autopilot running
## Session 2 notes (post-120min, still alive)
- Role/plan: parallel sweeps; if both exhaust w/o here=1 -> joint pass (both-on-cell).
- d5 plateau ~0.94-0.95 observed when wall-separated adjacent; lidar-diff test: botA NOT visible through walls.
- pushlib.py now has rev=True backward drive (traction sometimes catches better in reverse).
- sweep.py: left-hand-rule with fsweep alignment, pauses, adjacency hold at d5>0.962, goal_watch everywhere.
- botA reports its d2=0,d9=0 ALWAYS; my d9 flips (maybe partner-motion detector). Asymmetry unexplained.
- Comms fully working two-way (slow; botA reads in batches).
