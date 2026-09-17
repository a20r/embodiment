# Episode log (started Thu Sep 17 03:06:15 UTC 2026)
- Episode 1 started. README: ports at /dev/robot/, d8=TX, d10=RX transceiver to other robot. Must find other robot + reach goal together within 1 min.
## Port map (episode 1 findings)
- READ ports stream ~40Hz: d0(0 const), d3(status "tick=N goal=0 here=0"), d4(compass deg, noisy ±3),
  d5(0 const), d6 & d9 (noisy -1/0/1), d11(~0.5 float), d2(16 comma-sep ranges = lidar scan), d10=RX text
- WRITE ports: d1, d7 (actuators?), d8=TX
- src/sensord.py = bg daemon -> /tmp/state.json, /tmp/rx.log ; src/io.py write PORT TEXT / tx MSG / state
## Motion model (calibrated ep1)
- d1=left wheel cmd, d7=right wheel cmd (value = speed; 1->5 ticks/s, 10->50, 100->500 ticks/s, linear). Persist until written 0.
- d9=left encoder (d1), d6=right encoder (d7). Translation ~0.0004 units/tick; rotation ~0.163 deg per tick of wheel difference.
- Both +: forward. d1 + alone / (d1+,d7-) => compass d4 increases (clockwise). Robot veers; use compass feedback.
- Lidar d2: 16 beams, beam i points at compass = d4 + 22.5*i. -1.000 = dropout. Range ~3 max seen.
- Start position: dead-end of corridor ~0.44 wide, corridor axis at compass ~96 deg (open) / 276 (wall).
- d11 ~0.5 slowly falling? (0.52->0.47 in 5 min) maybe battery. d0,d5 always 0 so far (bump?).
- src/mv.py L R SECS ; src/snap.py (median scan) ; src/rio.py tx MSG
## Key facts (ep1, ~30min in)
- d5 = bump sensor (1 on contact). d0 probably bump too.
- d11 = likely RSSI to other robot: rose to 0.8+ right when radio beacons started arriving. Also varies in time when static (other robot moving).
- OTHER ROBOT is an LLM agent, calls itself "robot A", beacons every 20s with its own odometry pose (different origin). I call myself B.
- Radio reception intermittent (~50% of beacons) at d11~0.8.
- Start = dead-end corridor; went E 2.0, then N/NW into a chamber around (0.5-1.1, 1.1-1.7) [my frame: x=east,y=north, start=(0,0)]. Other robot heard from there.
- Tools: src/wallfollow.py SECS left|right (stops on RX or goal=1/here=1; log /tmp/wf.log); src/explore.py; src/ctl.py turn/fwd/report; src/restart_sensord.sh
- LESSON: never put 'sensord.py'/'explore.py' literal in pkill -f pattern (kills own shell). Use pkill -f "explor[e].py".
## Ep1 progress (~55 min)
- Radio truncates lines at 256 chars. Other robot = LLM agent; agreed plan: both explore; whoever gets goal=1 STOPS there and beacons 'GOAL FOUND'.
- d11 = RSSI to other robot (heading-independent, stable when they park; rose 0.78->0.89 homing until they moved). Homing works only if they park.
- src/mapper.py SECS = frontier exploration w/ occupancy grid (/tmp/grid.json, /tmp/map.txt, log /tmp/map.log). Works. INFL=1 (0.08) needed for 0.45 corridors.
- src/k.sh NAME kills src/NAME.py safely. Never write 'nohup python3 src/X.py' and k.sh X in the same command.
- Robot radius ~0.1; bump when min beam ~0.09.
- ~100min: MET other robot near my pose (3.8,3.2) (they parked; I homed on d11; d11 0.99 when adjacent; 'here' flag did NOT flip). Radio range ~1.5-2 units (rx works at d11>=0.66).
- Convoy plan: I lead+explore (CONVOY=1 python3 src/mapper.py SECS), park when d11<0.72, resume >0.85; goal=1 -> park + beacon GOAL FOUND; RX 'GOAL FOUND' -> home mode.
## Ep1 late (~120min): autonomous supervisor running (src/supervisor.py): alternates mapper(240s, fresh grid)/wallfollow(150s); on goal=1 parks+beacons; on partner 'goal=1' msg runs mapper HOME_MODE=1.
- Mapped region so far ~4.3x4.4 units (x 0.6..4.9, y -0.1..4.3 in my frame), narrow corridors ~0.45, no goal found yet by either robot after ~2h.
- Partner refuses to follow; they wall-follow independently. Their odom frame offset: mine - theirs = (+1.95,+2.99).
- IDEAS for next episode: start exploring immediately with supervisor; goal not in start area; try far regions (west of x=0.6? north of y=4.3?).

## HOW TO BOOTSTRAP NEXT EPISODE (copy tools): 
  mkdir -p /bot/src && cp /memory/src/* /bot/src/ && chmod +x /bot/src/*.sh && /bot/src/restart_sensord.sh && /bot/src/start_beacon.sh
  then: python3 /bot/src/ctl.py setpose 0 0 ; nohup setsid python3 /bot/src/supervisor.py >/tmp/sup.out 2>&1 </dev/null &
  Logs: /tmp/sup.log /tmp/map.log /tmp/wf.log /tmp/rx.log ; beacon text in /tmp/beacon_extra.txt (<250 chars total msg!)
- mapper.py: frontier explorer (RES=0.12 grid, INFL=1 else 0 fallback); marks blocking cells; convoy mode via CONVOY=1; HOME_MODE=1 = d11 gradient homing.
- climb.py N: discrete d11 hill-climb (works when partner parked). wallfollow.py SECS left|right.
- ctl.py: turn_to(hd), forward(dist,speed,front_stop) w/ heading hold; pose in /tmp/pose.json (x=east,y=north, from compass).
## RESULT ep1: did NOT find goal in ~2.3h. Explored ~4.3x4.4 region east/north of start thoroughly (both robots). Goal likely elsewhere:
  unexplored candidates: west of start (x<0), region around (-1.2,0.7..3.0) partially seen, far east x>4.9, north y>4.3.
  Next time: skip manual probing (all calibrated above), go straight to systematic exploration far from start; agree protocol with partner in 1 msg:
  "I am ROBOT-1, you ROBOT-2. Explore; whoever gets goal=1 parks & beacons 'goal=1'; other homes in on d11 (rises toward partner). Msgs<250ch."
- ep1 end-phase: mapper's grid sees the 4x4 region as a closed box -> exits are narrow diagonal gaps (e.g. from (2.0,0.45) heading 313deg into chamber (0.7,1.5); chamber -> west (-1.0,0.7) -> north (-1.2,3.0)).
  Next time consider: RES=0.10 grid + ray thinning, or explicit "gap detection" from scan (beam >1.0 with neighbors <0.5 = doorway) and drive through doorways not yet used.
- Partner never adopted follow/convoy; keep protocol minimal. Their beacon includes literal 'GOAL FOUND' text -> match on 'goal=1' only.
## EP1 FINAL (~150 min, out of context tokens): goal NOT found by either robot. Supervisor left running autonomously.
- Partner last plan: they go far EAST then SOUTH edges; I go WEST/NORTH. Neither confirmed goal.
- Biggest time sinks: manual sensor probing (30 min), pkill self-kills, mapper bugs. Tools in /memory/src are debugged - REUSE THEM.
- Token budget is the real limit: keep tool outputs tiny (cut -c, tail -1), run long autonomous jobs, poll rarely.
- ep1 last obs: mapper broke out to NEW area around (4.65,4.87) (north-east, y>4.3) - explore there first next time. Supervisor still running.
- ep1: reached (6.8,7.1) far NE, partner nearby (d11 .9). Maze extends well beyond 5x5. Still goal=0.
- ep1: explored out to x~11.5 (far east). Still goal=0 at ~170min.
