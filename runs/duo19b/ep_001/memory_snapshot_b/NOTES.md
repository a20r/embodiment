# READ THIS FIRST — playbook from episode 1 (SOLVED, goal=1 reached ~65 min in)
Working code saved in /memory/src (copy to /bot/src: `cp /memory/src/*.py /bot/src/`).
  rio.py = pipe I/O w/ timeouts; ctl.py = drive/turn_to/forward/scan/status; nav.py = goto(x,y);
  explore.py = frontier-ish explorer (env: BIAS,GX,GY, STOP_ON_HERE=1); seek.py = hill-climb d11 toward other robot;
  lead.py = lead other robot in hops; daemon.py = logs radio RX to /bot/rx.log + telemetry; beacon.py = periodic TX;
  showmap.py = ASCII map from /bot/map.log. Kill bg procs with: pkill -f "^python3 src/NAME.py" (anchored! else kills your shell)

## Ports (/dev/robot, named pipes; one line per open; use nonblocking+select)
READ:  d2 = 16 range beams (360deg, beam0=FRONT, indices clockwise, 22.5deg, beam4=right, beam8=rear; -1 = invalid)
       d3 = "tick=N goal=G here=H" (100 ticks/s). here=1 <=> I AM ON THE GOAL ZONE. goal=1 <=> task solved.
       d4 = compass heading deg, clockwise positive (0=N, 90=E). d9 = left encoder, d6 = right encoder (cumulative)
       d11 = radio signal strength to other robot: ~1/(1+(dist/2)^2) but ALSO attenuated by walls; 1.0 = adjacent, 0.5 ~ 2 units, 0.25 ~ 3.5
       d10 = radio RX. d0, d5 = always 0 (unknown)
WRITE: d1 = LEFT wheel speed, d7 = RIGHT wheel speed (floats, -100..100 ok; ~5 enc ticks/s per unit); d8 = radio TX
Calib: ~1850 enc ticks per range unit; rotation ~2.86 (L-R)/2 ticks per degree. Odometry drifts ~1.8 units per 20 min: re-anchor at landmarks.

## World (ep1; may be same next time)
Maze of corridors ~0.5 units wide. Start pose faced ~180 with wall 0.22 ahead. Goal zone (odom, x=E,y=N) ~ (0.4,-1.0):
from start go SOUTH along the x~0 corridor ~1.0 then EAST ~0.4 (explorer w/ SW bias found it in ~1.5 min). Zone is ~0.5 wide,
has wall on its S/SE side, open W/NW and N/E. The other robot (calls itself B) started ~1.5 S of my start area.

## Radio / other robot ("B" = another LLM agent, slow: replies every 1-3 min, sometimes 10 min)
- Lines TRUNCATED at 256 chars. Range limited: messages only arrive when d11 > ~0.7 (dist < ~1.3 units).
- B has SAME port layout, same units, has a compass. B can climb d11 to reach me (took ~4 min from 4 units away).
- RULE that worked: ONLY ONE ROBOT MOVES AT A TIME. Both moving => endless chasing.

## WINNING SEQUENCE (do this again, faster):
1. Immediately start daemon.py (radio log) and explore.py biased SW (BIAS=0.4 GX=-0.5 GY=-0.87) with STOP_ON_HERE=1 -> parks on goal (here=1).
2. Radio contact only when near. B initially came near my start. Get B adjacent (seek.py hill-climbs d11 in ~2 min if B stays still),
   then LEAD B to goal in 0.6-unit hops (lead.py): B follows by climbing d11; wait each hop until d11>0.88. Tell B compass dirs of hops.
3. CRITICAL: goal=1 requires BOTH robots' ARRIVAL times (here 0->1) within 60 s. Standing together for 6 min with both here=1 did NOT work.
   Re-arrival protocol that worked: A steps OFF (verify here=0, use an open side away from B), B steps ON and messages; A re-enters
   within 10 s (watch.py auto-triggers on B's message; make the regex strict e.g. 'B ON'). goal flipped to 1 within seconds.
   Zone is small: don't block the entry spot; enter from a different side than B.

## Pitfalls seen in ep1 (avoid)
- Don't send messages >256 chars (truncated). Don't assume radio reaches B when d11<0.7.
- pkill -f "explore.py" killed my own shell (pattern matched the bash -c cmdline). Always anchor: pkill -f "^python3 src/x.py".
- Assuming d11 = pure distance misled me; walls attenuate it (sudden 0.9->0.45 drops when a wall gets between).
- nav.goto in narrow corridors misses entrances due to odometry drift; explore.py w/ bias + STOP_ON_HERE was more robust.
- Don't send OFF/ON messages before verifying the here flag actually changed (I sent false ones; B got blocked by me at the entry spot).
- Budget: ep1 used ~65 min wall-clock and ~150k tokens. Token cost is the binding constraint; poll with sleep 55 + tiny outputs.

## Suggested opening radio messages to B (short lines):
"A: same ports? d3 'here=1' = standing on goal; goal=1 = done. d11 = signal strength to you (1.0 adjacent). Radio only works <1.3 units apart, msgs cut at 256 chars."
"A: PLAN: I find the goal (fast code), then come to you; STAY STILL until I am adjacent, then follow me by climbing d11 in hops. Finally we re-enter the zone within 60s of each other."
