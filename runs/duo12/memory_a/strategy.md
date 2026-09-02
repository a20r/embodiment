# Strategy notes (end of ep1) — READ FIRST
World: maze ~6x8 units. Devices: see hardware.md. Speed 100 ≈ 0.3 u/s. SCALE=0.0006 u/enc-count.
Odometry DRIFTS badly after wall hits; compass absolute & reliable.

## What happened ep1
- botA (other robot) wanders, broadcasts pings. It ADOPTED my message format
  "botA PING x=.. y=.. d11=.. here=1 seq=N" -> its agent reads my radio msgs.
- botA found goal (here=1), parked at its-frame (-3.13,4.14). Radio seems long-range (rx at v=0.25).
- d11: when botA parked, static field: HIGH (~0.88 plateau) near north wall area of my frame
  (my frame ~(-2.3..-2.7, 5.9..6.2) at that time), low (~0.25) south. Field ~ exp(-dist).
  Never found entrance to goal chamber; here never fired for me. d11 0.98 when botA passed adjacent.
- d11 fluctuates ONLY because botA moves. Not directional (early "directional" reading was temporal).
- Gradient climb (run&tumble) works in open areas; fails in maze corridors + plateaus at walls.
- Waypoint/planner failed due to odometry drift ghost walls.

## Next episode plan
1. Start ALL logging + a background pinger immediately (announce "botB").
2. Talk to botA via radio ASAP: agree protocol: botA parks AT GOAL and pings loudly;
   ask botA to REPORT its lidar 16 ranges - the goal chamber signature; ask it to "pulse":
   drive small circles so I can confirm proximity by d11 swings.
3. Use v=d11 as beacon: climb to plateau (v~0.85+), then WALL-FOLLOW (right-hand rule)
   the ENTIRE boundary until here=1 (bug algorithm). Do not give up: full perimeter may take minutes.
   bug.py exists in ep1 /bot/src but /bot is wiped; code snapshots in /memory/src_snapshot/.
4. here flag = d3 "here=1". Both robots must be at goal within 1 min. When here=1, STOP, tx to botA.
5. AVOID: pkill with pattern matching your own bash cmdline (kills your shell). Use ps+awk+kill.
6. sleep >60s in one command = killed at 60s. Chain short sleeps.
## Ep1 late notes (~05:15)
- botA parked at goal since ~04:55 (its pings froze at x=-3.131 y=4.141 here=1; seq RESET happens -
  botA agent restarts scripts; payload may be stale template, but here=1 + parked seems real).
- v(d11) field with botA parked: ~0.88 plateau near goal chamber wall, 0.25-0.4 elsewhere south.
- I never found chamber entrance. Untried: full perimeter wall-follow (bug.py wall mode) for 10+ min;
  east side of map (x>1.5) at north (y>5); also check if "goal" chamber opens from x>0 side.
- Suggest next ep: IMMEDIATELY climb v to plateau, then wall-follow whole boundary patiently (20 min),
  monitoring here flag every loop. That covers all reachable walls incl. entrance.
- Consider asking botA to step 0.5 out of goal & back periodically ("pulse") to detect proximity live.
- 05:21: botA went SILENT (agent episode ended?). Its pings resume when its agent restarts.
- v swings steeply sometimes -> botA likely moving when its agent runs; only trust v-gradient when
  botA confirms parked via pings.
- PROTOCOL TO PROPOSE NEXT CONTACT: "whoever is at goal PARKS FOREVER and pings 'AT GOAL' every 5s;
  the other climbs v-gradient + wall-follows. When both here=1 within 60s -> win."
- Ep1 ends ~05:30. bug.py left running (climb->wall-follow at v>0.75, unstick, here-flag watch).
- FINAL ep1 state: bug.py running, climbing v around (-2.5..-3.8, 4.2..5.0) frame-ep1, v 0.3-0.6,
  botA silent, goal never reached by me. here flag NEVER fired for me all episode.
- Consider next ep: verify 'here' semantics early (maybe d3 'goal' flag = goal known, 'here' = at goal).
- REMEMBER: read /memory/hardware.md + /memory/strategy.md first; code in /memory/src_snapshot/
  (robot.py Bot class + explore.py Nav/turn_to/drive are solid; bug.py = climb+wallfollow).
## Ep1 final (~05:15 wall clock 04:57+)
- Wall-follow+climb kept looping a pocket around frame-ep1 (-2.4..-3.8, 4.1..5.2), v peak ~0.6.
  Never fired here=1. Pocket exits are narrow; follower's front-threshold 0.28 may refuse gaps.
- NEXT EP IDEAS (priority):
  1) Reduce robot's collision thresholds: try front<0.22, and add gap-shooting: if any beam>1.0
     while neighbors<0.3, aim precisely down that beam slowly (speed 40).
  2) The maze changes little; goal chamber likely near ep1-frame (-2.6,6.6)±drift; approach from
     NORTH/WEST side may exist (unexplored x<-4 or y>6.5).
  3) Coordinate with botA when its agent is awake (it reads radio; adopts protocols).
  4) Consider measuring drift: pose.json persists but frame differs after robot repositioned between eps.
     Re-anchor by compass+lidar features, or just restart pose at 0,0 (delete pose.json).
- Token budget died before time budget; prefer fewer, larger analysis steps; poll rarely.
- LAST FIX ep1: goal-chamber v-peak zone in final frame: (-3.7..-4.2, 5.4..6.2), v up to 0.85 outside wall.
  Entrance NOT at south/east of it (walked). TRY NORTH & WEST of (-4,6.2) next ep FIRST.
- bug.py final version: climb<->wall thresholds 0.75/0.62, island-detach. Works but slow; add gap-shooting.
- BUG ep1: after several process kills, encoder Reader gave bad deltas -> pose ran away (13,-16).
  Nav pose unreliable after restarts; next ep: reset pose.json to 0,0 at start and sanity-check
  enc deltas (cap |delta|<100/tick). v stayed ~0.77 at end; still no here=1.
