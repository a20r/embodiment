# BRIEFING FOR FUTURE ME (read first). Written end of episode 1 (2026-09-16 21:23).
Raw ep1 log: NOTES_ep1_raw.md. Code: /memory/src (cp /memory/src/* /bot/src/ FIRST). Logs: explore_log.txt, autopilot_log.txt.

## RESULT EP1: I FOUND THE GOAL but partner (robot 1) froze/went silent at 20:30 and never came. Not completed.

## WORLD (verified)
- Grid maze, 0.5m cells, corridors aligned to compass N=0,E=90. Robots are tiny (can sit 0.09m from wall).
- Ports: d1=left wheel speed (write), d7=right wheel (write); persistent until "0". cmd 40 => 0.13m/s, cmd<15 = no motion (deadband). d1 + => turns CW.
  d2=16 range beams (beam i at heading+i*22.5 CW; beam0=forward; sat ~2.75m; -1 dropout). d4=compass (noise +-3). d3="tick goal here". d6/d9=fake encoders (ignore).
  d11=sig: proximity to partner (0.998 @0.5m, 0.87 @1m, 0.69 @1.5m, ~0.3 @3m+); passes through walls. d8 write/d10 read = text link (<=250 chars, works at any distance seen).
- Status flag 'here=1' = I am inside goal zone (radius ~0.2m around goal cell center). 'goal' stayed 0 (probably = partner at goal / both done).
- Cell coords: x east, y north. MY START = (-3,0): dead-end cell, only exit East, corridor 2.7m+. (Assume same start next time; verify with first scan.)
- GOAL = (7,0): dead-end cell east of (6,0). (6,0) reached from north: (6,3)->(6,2)->(6,1)->(6,0).
- PATH START->GOAL (bearings per 0.5m step), 42 steps ~15min:
  (90,90,90,90,90,90,90, 0,270,0,270,0,270,180,180,270,270, 0,0,90,0,270,0,0, 90,90,90,180,90,90,0,90,180,180,270,180,90,90,180,180,180,90)
  cells: (-3,0)..(4,0), (4,1),(3,1),(3,2),(2,2),(2,3),(1,3),(1,2),(1,1),(0,1),(-1,1),(-1,2),(-1,3),(0,3),(0,4),(-1,4),(-1,5),(-1,6),(0,6),(1,6),(2,6),(2,5),(3,5),(4,5),(4,6),(5,6),(5,5),(5,4),(4,4),(4,3),(5,3),(6,3),(6,2),(6,1),(6,0),(7,0)
  Run: python3 src/explore.py 0 "(-3,0)" "<tuple above>" "[]"   (route mode; check 'routed to' lines in /memory/explore_log.txt; explore.py needs rx not required)
- Unexplored: (3,3) south of (3,4); (3,6) west of (4,6); region around (5,2)/(4,2) where robot 1 sat.

## PARTNER (robot 1) - an LLM agent too, calls itself R1, defined ORIGIN = spot where we were 0.5m apart (= my cell (-1,0), it was N or S of me through a wall).
- It found the goal on its own by ~20:16 (its here=1). It wanted to "lead" me by signal and waited in a loop for sig>0.85; then went silent from 20:30 on (static at ~(5,2), behind wall W of (6,2)). Probably its agent died/budget.
- It has its own memory; next time it should know the goal too. MESSAGE IT EARLY AND OFTEN, short (<250 chars):
  "R2: goal = dead-end cell where here=1. Go there; wait 1 cell before it until our sig>0.9, then both enter. I arrive in ~15 min."
- Two robots fit in goal cell: I park NORTH side (face 0, drive to N wall 0.12m), partner SOUTH side.

## LESSONS
- Don't pkill with a pattern that matches your own shell (use var: L=logger; pkill -f "python3 src/$L.py").
- Failed steps must not increment position. Snap along-track to 0.25+0.5k using front wall. Recenter laterally.
- Partner wandering nearby looks like moving walls in range data.
- Budget: context tokens are the binding limit, not wall-clock. Use background scripts + cheap polls (src/p.sh).

## FINAL STATE EP1 (21:40): I am parked inside goal (7,0) with here=1 since 20:42. Robot 1 silent since 20:30, static ~1.5m away (sig 0.69).
autopilot.py kept messaging it every 45s until episode end. Check autopilot_log.txt for any late RX from robot 1.
NEXT TIME: be fast (goal path known, ~15 min), message robot 1 the goal immediately, coordinate simultaneous entry (me NORTH side, it SOUTH side).
