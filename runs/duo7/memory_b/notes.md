# Robot findings (episode logs)
Ports /dev/robot/ (all line-based ASCII, FIFOs):
- d0: WRITE radio transmit; d4: READ radio receive
- d1: READ compass heading deg (float)
- d2: READ ??? (always 0 so far; maybe goal proximity flag)
- d3: READ lidar 16 rays CSV, max ~3.0, -1.000 = dropout/noise
- d5: READ occasional lines (float ~0.13-0.15) meaning ???
- d6: READ status "tick=N goal=0"  (goal flag!)
- d7: READ counter (maybe left encoder; saw 2123)
- d8: READ maybe right encoder
- d9: READ bump flag (1 when pressed against wall)
- d10: WRITE left wheel speed, d11: WRITE right wheel speed
  Commands PERSIST until changed! speed 100 saturates; moves fast.
Robot: differential drive. tick rate ~ hundreds/sec?
Goal: both robots must reach goal within 1 min of each other. Other robot in maze, radio d0/d4.

## Working nav approach (episode 1)
- d1 heading DRIFTS (gyro-like). Don't trust as absolute; corridors not axis-aligned in compass frame.
- Robot body wide; must avoid rays +-22.5/+-45 (<0.12 -> blocked). d9 bump=1 when scraping.
- Corridors ~0.45-0.5 wide, some >4 long. Speed: cmd 65 -> ~0.42/s. K=0.0065 per cmd unit.
- /bot/src/nav2.py: relative right-hand-rule explorer w/ verify+bump-backup. start/stopnav.sh.
- pkill footgun: pattern must be "^python3 nav2\.py$" else kills own bash -c.
- d5: spams floats ~0.15 while moving (unknown meaning).
- Radio quiet so far; other robot not heard (tx d0/rx d4).

## Radio protocol with other robot (it's an AI agent too, name=alpha; I am beta)
- alpha sends: "PING from=alpha x=.. y=.." every ~3s (own frame).
- AGREED PLAN: explore independently; whoever finds goal STOPS ON IT and
  broadcasts "GOALFOUND" every ~10s; other keeps exploring until own d6 goal=1.
- Switched to continuous right-wall-follow /bot/src/wf.py (better than nav2 discrete).
  wf.py: rays rel index; right=+i side; front<0.26 -> spin left; hug right wall 0.25; bump recover.
- IMPORTANT: d5 = proximity/RSSI to OTHER ROBOT (moves when I'm parked). ~0.15 far, 0.9 very close.
  Use d5 gradient climb (run&tumble / junction hill climb, see /bot/src/homing2.py) for RENDEZVOUS.
- Goal still unknown after ~1h. d6 goal=0 everywhere so far. d2 always 0.
## Episode 1 late status (~80min)
- Explored heavily via wf.py (right/left wall follow + random side switch). No goal found yet.
- d2 stays 0 everywhere: suspect d2 = goal proximity sensor (0 = far). Watch for d2 != 0!
- alpha (peer agent) explores too; proposed+agreed protocol: goal finder stands on goal,
  broadcasts GOALFOUND every 10s; other homes in using d5 RSSI gradient (homing2.py).
- Tracker (track.py) inflates distance (counts reverse as fwd); positions unreliable long-term.
- Alpha's dead-reckoned coords stay small (+-6), mine blew up to -50: maze likely ~15x15 max.
## Recommended bootstrap next episode
1. start radio_listen.py, watch.py, d5log.py, track.py, monitor.py (in /bot/src if persisted - THEY ARE NOT; /memory only!) 
## End of episode 1 (~93min): goal NEVER found by either robot. goal=0 to the end.
- Wall-follow both sides + speed 85 covered a lot; d2 never nonzero (sampled ~15Hz late).
- NEXT EPISODE PRIORITIES:
  1. Bootstrap per BOOTSTRAP.md immediately (saves ~30 min of rediscovery).
  2. Coordinate WITH alpha to split exploration (e.g., alpha follows LEFT wall, beta RIGHT wall).
  3. Consider that goal may need something unusual: try standing still in distinctive places
     (large rooms, dead-end centers), try d2 harder, try writing to d2? (untested!), 
     try messages like "goal" on d0? (untested).
  4. Untested ideas: writing to d1/d2/d6 ports; maybe a port ENABLES a goal sensor.
