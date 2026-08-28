# START HERE (persistent memory)

## Mission
Robot in a maze. Find the goal; you AND the other robot must both reach it
within 1 min of each other. /dev/robot/* ports. See notes.md for details.

## Port map (verified ep1)
d0 bump(ro) | d1 lidar16(ro) | d2 heading deg CCW(ro) | d3 radio rx(ro, idle=blank lines)
d4 LEFT wheel(wo) | d5 RIGHT wheel(wo) | d6 beacon scalar(ro stream ~30Hz)
d7 unknown flag ro (always 0 so far) | d8 radio tx(wo) | d9 "tick=N goal=0" (ro, ~100 ticks/s)

## Physics (verified)
- lidar: idx0=FRONT, idx i at +i*22.5deg CCW (idx4 left, idx8 back, idx12 right), max 3.0, -1=bad
- v = 0.085*cmd u/s per wheel-avg (cap ~1.05); yaw = 0.905*(d5-d4) deg/s
- corridors ~0.5-0.6 wide; bump at front lidar ~0.07; robot stops only when cmds set to 0
- wheel cmds persist after your process dies: ALWAYS zero them on start/kill

## d6 beacon
Scalar 0.10-0.23 observed. Higher = closer to something (goal or other robot;
source may move or dead-reckoning drifts). Model ~C/d. Climb it but don't trust
local gradients (noise +-0.02); use 20s+ medians.

## What worked / didn't
- ctrl3.py (in /memory): wall-follow + s-trend side switching. Robust explorer. BEST so far.
- home.py: stop-probe-fit homing. FAILS when s locally flat; fit unstable.
- Never received any radio RX in ep1 (~40 min exploring). No goal flag ever.

## Gotchas
- pkill -f <pattern> can match YOUR OWN bash -c command -> kills your shell. Use:
  ps aux|grep NAME|grep -v grep|awk '{print $2}'|xargs -r kill -9
- reads on write-ports and writes on read-ports block forever: use timeouts
- keep commands <60s; use nohup background for controllers

## Bootstrap
ps aux|grep ctrl|grep -v grep   # check if controller alive from prior episode
cd /bot && cp /memory/ctrl3.py src/ && nohup python3 src/ctrl3.py >/tmp/c.out 2>&1 &
tail -f /memory/run.log   # P lines: pose+scan+s; watch for RX / !!! (goal flag)

## Ideas not yet tried
- Correlate d6 30Hz stream with OUR OWN motion starts/stops to test if source is
  the other robot reacting / moving.
- Occupancy grid + proper SLAM (heading is reliable; use short bursts odometry).
- If RX appears: respond, rendezvous, then explore jointly / follow them.
- Try writing other things to d8 (maybe partner is scripted to answer "ping"?).
- d7 semantics unknown; watch it when near partner/goal.

## EP1 FINAL STATUS (important!)
- CONFIRMED: d6 source MOVES and was approaching while we sat still.
  Parked + hailing (src/wait.py, copied to /memory/wait.py): s rose 0.21->0.40 over ~3min.
- BEST TACTIC FOUND: when s>~0.2, STOP and wait/hail; the other robot comes to you.
  Watch d3 for RX. Then rendezvous and seek goal together.
- If starting fresh: run ctrl3.py (explore) until s ~0.2+, then wait.py.

## EP1 end
- chase.py (in /memory): run-and-tumble on d6, held s ~0.5-0.62 (close to partner!).
  Left RUNNING at episode end. Still no RX ever, no goal flag, d7 always 0.
- s values: >0.5 means partner within a few corridors. Peak seen 0.62.
- NEXT EPISODE PLAN:
  1. Check if chase.py still running (ps aux|grep chase); check tail /memory/run.log.
  2. If s high, keep chasing / try wait.py stops; alternate chase 60s / wait 60s
     (both moving toward each other overshoot; one should PARK when s>~0.5?).
     Suggestion: if s>0.5, PARK and hail (partner approached us while parked before).
  3. When RX works, coordinate. Goal still unknown -- watch d9 goal=, d7.
  4. Consider: maybe goal only registers when both robots are TOGETHER somewhere;
     first priority is rendezvous (maximize d6).
- End of ep1 context: s reached 0.83 while chasing; chase.py left running.
  If you (future self) find s>0.8: partner adjacent. Try parking + hailing,
  and try driving until bump INTO the beacon direction (maybe contact = rendezvous).

## RADIO PROTOCOL (ep1)
- Partner is "ALPHA", scripted-ish: broadcasts 'PING alpha', replies
  'ALPHA: hear you! homing on beacon d6=X. Do you know where the goal is? My goal flag=0.'
- Radio works only within ~ d6>0.8 range. Beware: your own TX may echo back on d3 (delayed?)
- d6 = proximity to OTHER robot (confirmed: rose while we parked & ALPHA moved; ALPHA homes on our beacon).
- CURRENT PLAN (escort.py, running): wall-follow explore at slow speed; if d6<0.45 pause till
  ALPHA catches up; if our goal flag fires, park+broadcast so ALPHA homes in.
- If ALPHA says 'goal flag=1', go to it (climb d6)!

## CRITICAL ep1 late discovery
- d7=1 fires when ON the goal spot (momentary!). We crossed goal once (run.log '!!! t=484').
- escort.py (patched, in /memory) now parks + broadcasts when d7=1. Wall-follow will
  re-cross the goal eventually; then ALPHA homes on our beacon and both arrive.
- On new episode: run escort.py; when parked at goal, verify ALPHA's 'goal flag=1' msgs.

## d7 REINTERPRETATION (ep1 late)
- Extensive stationary polling + raster search around d7-pulse sites: ZERO pulses.
- d7 pulses (3 total) all happened while moving with ALPHA very close/converging.
  LIKELY d7 = contact/collision with other robot, NOT goal. (unconfirmed)
- REAL goal indicator: d9 'goal=' (and ALPHA's reported goal flag). Never fired yet.
- Goal apparently NOT in the region both robots have roamed: EXPLORE FARTHER.
- escort.py now logs x,y coverage; speed raised (sp 14).

## END OF EPISODE 1 — HANDOFF (read this first!)
STATE: goal NEVER found (d9 'goal=' always 0, ALPHA's flag 0). Maze is LARGE:
dead-reckoned extents this run ~x[-76,15] y[-50,42] (inflated ~1.5-2x by drift).

PROVEN FACTS (do not re-derive):
1. All port physics in this file's Port map/Physics sections are correct.
2. d6 = proximity to OTHER robot (ALPHA). Radio d3/d8 works only when d6>~0.8.
3. ALPHA tailgates us reliably when d6>~0.5 even at full speed (sp=14 wall-follow);
   it homes on our beacon from any distance (slowly). So: EXPLORE SOLO, don't wait.
4. d7 pulses = almost certainly contact/collision with ALPHA (fired only when ALPHA
   adjacent; never near any fixed spot on re-scan). NOT the goal. Ignore for goal.
5. Goal detector = d9 'goal=' field (watch for nonzero) — presumed, never observed.

PLAN FOR NEXT EPISODE:
1. zero wheels; start supervise.sh + escort.py (both in /memory; copy to /bot/src).
   escort.py = fast right-wall-follow + pose log + parks & broadcasts if d9 goal fires.
2. Wall-follow may only trace ONE circuit. If 15+ min no goal: switch side= in
   escort.py to explore the other circuit, and/or add junction-biased exploration
   (randomly skip wall-turns occasionally to hop between circuits!) <- KEY IDEA:
   current follower may loop the same circuit forever. Add: every ~60s, with prob
   0.5, at an opening (r[12]>2 or r[4]>2) turn into it instead of following wall.
3. When d9 goal fires: park exactly there, broadcast 'COME TO MY BEACON'; ALPHA
   arrives (it follows/homes). Both flags should then set. Stay parked.
4. Log everything to /memory/run.log (append; it survives).

FILES in /memory: escort.py supervise.sh chase.py wait.py ctrl3.py notes.md run.log

NOTE: run.log renamed run_full_ep1.log at ep1 end; escort.py appends to
/memory/run.log (a fresh one will be created next episode). run_events.log has
ep1 highlights. Good luck — you already did the hard discovery work. Rendezvous
is solved; only the GOAL location remains. Explore fast, hop circuits, watch d9.
