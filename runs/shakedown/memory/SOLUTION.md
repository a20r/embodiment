# Episode 1 SOLVED via left-hand wall follow (135 steps, ~9 min)
Start cell (0,0) facing heading 0 (+x). Goal cell: (5, 2)
Grid: +x=heading0, +y=heading90(CCW/left). Cell=0.5m.

Shortest known path (17 moves) through mapped passages:
[(0, 0), (1, 0), (1, -1), (1, -2), (1, -3), (1, -4), (1, -5), (2, -5), (2, -4), (2, -3), (2, -2), (2, -1), (2, 0), (3, 0), (4, 0), (5, 0), (5, 1), (5, 2)]

Next episode: verify start heading ~0 and initial lidar F~0.75,L~0.24,R~0.24; then just follow this cell path with drive.py primitives (turn_to snapped heading, forward 0.5m). If early lidar readings mismatch run.log episode-1 step 0-3 pattern, maze/start may differ -> fall back to left-hand follower (/memory/drive.py, WORKS).

## IMPORTANT CAVEAT on coordinates
The (0,0) origin above is NOT the episode start! It is where run 2 of the
controller began: mid-maze, after run 1 crashed (encoder parse bug) ~8 steps
in, hit a wall, and the fixed script restarted facing 180. True episode start
was ~8 cells away (run 1 trace = first 9 'step' lines in run.log, starting
snap-heading 90, but note probes had already moved robot ~0.13m fwd and +47deg
before run 1).
=> Do NOT blindly replay the 17-move path from the episode start.

## Recommended plan for next episode (15 min budget)
1. Just run the proven left-hand follower: python3 -u /memory/drive.py
   (copy to src/ first; it logs to /memory/run.log, exits on goal=1).
   It solved from crash-position in ~9 min / 135 steps. From true start it
   should be similar or less; budget fits.
2. Optional speedup: raise base PWM 130->170 in forward(), turn speeds
   40..140 -> 60..170. Test 2-3 steps before committing.
3. While it runs, reconstruct map (see /memory/replay.py) and if you can match
   the new trace's early F/L/R pattern to episode-1 run.log, you can localize
   and cut over to the known shortest path to goal cell (5,2 in run-2 frame).
