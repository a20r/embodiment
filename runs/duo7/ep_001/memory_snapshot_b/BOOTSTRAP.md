# READ THIS FIRST (written end of episode 1, no goal found in ~90min)
## Facts (verified)
- /dev/robot ports: d0=radio TX(write line), d4=radio RX(read; empty line if none)
  d1=heading deg (DRIFTS, gyro-like, not absolute), d2=UNKNOWN always '0' (suspect goal proximity!),
  d3=lidar 16 rays CSV, ray_i at heading+22.5*i (i=0 forward, +i = side that drive(L>R) turns toward),
  max 3.0, -1.0=dropout(keep prev), d5=RSSI to OTHER robot (0.15 far, 0.9 adjacent; ~40 samples/s),
  d6="tick=N goal=0|1" (goal flag = the win condition), d7/d8=wheel encoders (count UP even reversing!),
  d9=bump flag, d10/d11=left/right wheel speed cmd (PERSIST until changed; 100=max ~0.65 u/s).
- Robot ~0.2 wide; corridors ~0.45-0.5; keep rays +-22.5deg >0.15.
- Other robot = peer AI agent "alpha" (I am "beta"). It negotiates via radio. Radio seems maze-wide-ish
  but intermittent; alpha pings "PING from=alpha x=.. y=..".
- AGREED PROTOCOL: whoever's d6 goal=1 STOPS ON GOAL + broadcasts "GOALFOUND" every 10s;
  the other climbs d5 gradient to rendezvous (goal is where partner stands).
- pkill footgun: use pattern '^python3 wf\.py$' style, else you kill your own bash -c.
- Commands killed at 60s: use setsid ... & for long processes; sleep <60.
## Code (all in /memory/, copy to /bot/src and run from there)
- lib.py (rd/wr/drive/stop/tx), wf.py = MAIN EXPLORER (wall follow, random side switch, bump recovery,
  goal standing + GOALFOUND broadcast, d2!=0 alert to /tmp/ALERT)
- homing2.py = d5 gradient climb (junction hill-climb) for rendezvous with alpha
- radio_listen.py -> /tmp/radio_rx.log ; watch.py -> /tmp/ALERT on goal=1 or alpha GOALFOUND
- track.py rough odometry (unreliable, inflates on reverse), d5log.py logs d5 to /tmp/d5.log
## Startup (do immediately)
mkdir -p /bot/src; cp /memory/*.py /memory/*.sh /bot/src/; cd /bot/src
setsid python3 radio_listen.py & setsid python3 watch.py & setsid python3 d5log.py &
setsid python3 wf.py &  # explorer
then poll: ls /tmp/ALERT; tail /tmp/radio_rx.log
Tell alpha the protocol early. Check d2 often; if it ever !=0, investigate that spot (likely goal beacon).
## What did NOT find goal: ~90 min wall-following both sides + d5 hill climbs. Maybe maze is huge,
## or goal needs d2 investigation, or wall-follow misses interior islands: consider systematic
## grid SLAM next time (encoders too unreliable; use lidar scan-matching?) or straight-line lawnmower bursts.
