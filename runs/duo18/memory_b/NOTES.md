# Robot memory notes (persisted across episodes)
## Episode 1 log
- README: ports at /dev/robot/. d8 = transmit (write), d10 = receive (read). Other robot exists; both must reach goal within 1 min of each other.
- Ports block if opened wrong direction -> use timeouts.
## Port map (verified ep1)
- d0: read, "0" so far (bumper? flag)
- d1: WRITE left wheel speed (integer; "0.2" ignored, 10 -> ~50 enc counts/s)
- d2: read, 16 comma-separated lidar ranges; beam0=forward, beams clockwise 22.5deg each (beam4=right, beam8=back, beam12=left). -1.000 = dropout. Noisy ~±0.02
- d3: read, "tick=N goal=0 here=0" (~100 ticks/s)
- d4: read, heading degrees, compass-like (increasing = clockwise turn). noise ±3deg
- d5: read, "0" so far
- d6: read, RIGHT wheel encoder count (noise ±1)
- d7: WRITE right wheel speed (integer)
- d8: WRITE transmit text to other robot
- d9: read, LEFT wheel encoder count
- d10: read, receive text (blocks until msg)
- d11: read, ~0.50 +-0.01 float (unknown; light? signal?)
- Both wheels 10 for 1.5s => ~80 counts, ~0.06 range-units forward. L=+10,R=-10 => ~19deg/s clockwise.
- Helper lib: /bot/src/rio.py (copy saved in /memory/rio.py)
- d5 = CONTACT/bump flag (1 when touching wall; lidar reads ~0.09-0.10 at contact => robot radius ~0.1, corridors ~0.5 wide). Contact causes wheel slip -> odometry overestimates.
- d11 varies over time even when stationary (0.5 -> 0.25 -> 0.33): probably signal strength of OTHER ROBOT (moving). Not battery.
- Speed linear: counts/s ~= 4.8*speed. K=0.00058 units/count (free rolling).
- Frontier explorer (brain.py) failed: noisy map, scraped walls. Switched to wall-following brain2.py.
- pkill gotcha: use pkill -f "[b]rain" pattern trick so the shell itself isn't killed.
- 03:23 CONTACT: other robot calls itself "ROBOT B", calls me "Robot A". It hears my beacons; it uses d11 as signal strength to home in. d11 CONFIRMED = radio signal strength between robots (~0.9 when close).
- Other robot's msgs arrive on d10 only when in range. It reports its own odom pose (own frame) e.g. (0.5,1.7).
- Wall follower loops around an island near my-odom (2.2..3.3, -0.8..0). Dead-end pocket at my (2.4,-0.75) had d11 peak.
- 03:29 Plan sent: both wall-follow; whoever hits goal=1 parks and announces; other homes in via d11. I switched to RIGHT-wall following (brain2.py right) to break loop.
- Use /bot/src/kill_by_name.sh NAME (saved in /memory) to kill daemons safely; NEVER pkill -f with a name that appears in the same shell command.
- 03:39 Other robot's d11 hit 0.98 near me; I parked at my-odom (1.85,0.18) in E-W corridor, waiting for it to arrive. Its plan ACK: whoever finds goal parks+announces.
- 03:46 MET the other robot: adjacent at my-odom (1.85,0.18); d11=0.98 when adjacent. Radio msgs stop arriving when d11 < ~0.5.
- Other robot replies slowly (minutes); its beacon is a template. Keep msgs <256 chars.
- !!! MOTOR FATIGUE: after ~7-8 min continuous driving at speed 42, wheel output dropped to ~10% (encoders 28 counts/s at speed 50 vs 243 fresh). Robot looks "stuck" (wheels spin slow, lidar static, d5=0). RESTING restores it (~+30 counts/s per 25 s). Odometry runs away during slip (fake +7 units). => pace movement: monitor efficiency (actual counts/s / (4.8*speed)); rest when < 50%.

## STATUS SUMMARY at 04:20 (episode 1, ~75 min in) — READ THIS FIRST NEXT TIME
- World: maze, corridors ~0.5 wide, robot radius ~0.1. Both robots explored ~4x4 units for 1h+ and NO goal found (goal flag never 1, here never 1, d0 always 0).
  => Suspect: goal is somewhere not reached by wall-following (open room center?) OR maze bigger than explored (explorers loop). Consider a smarter explorer next time (Tremaux / random turns at junctions), or ask other robot for its map extents.
- Other robot = another LLM agent. Reacts slowly (minutes). Beacons a template msg ~every 10s with its d11 and goal flag. Wants: "If your goal=1: park + beacon 'goal=1'; it homes in via d11". It uses frontier mapping. It computed frame offset: its_x ≈ my_x + 2.0 (before my odometry broke).
- Radio range: msgs arrive only when d11 > ~0.5. d11≈1-dist/3.5 roughly (0.98 adjacent, 0.9 ~0.35 apart).
- MOTOR FATIGUE is real: sustained driving (~7 min at speed 42) -> output drops to 10%, odometry runs away. Rest ~2-4 min restores. brain2.py has eff monitor (rests 120s when eff<0.5); cruise speed 32.
- Running daemons (all in /memory too): listen.py (rx->/bot/rx.log), beacon.py (tx every 6s: say.txt + pose/d11/goal), brain2.py (modes in /bot/mode: wall|stop|goto X Y|bias DEG), cycle.sh (rotates bias dirs every 100s), watch.py (endgame: if other says goal=1 -> homing via d11 hill-climb; if my goal=1 -> stop & announce).
- Quick start next episode: cp /memory/*.py /memory/*.sh /bot/src/; start listen, beacon, brain2 (mode wall), then talk to other robot early (it will be around).
- 04:27 watch.py false-triggered on template text; fixed regex. Competing processes (watch/cycle/brain) caused apparent 'stuck' behavior.

## FINAL NOTES episode 1 (04:30, ~85 min elapsed, tokens nearly exhausted)
- Outcome so far: met other robot (03:45), agreed protocol, but NO goal found by either after 1h20m of exploration. goal/here flags never changed; d0 never changed.
- Ideas for next episode (priority):
  1. Start daemons immediately from /memory (listen, beacon, brain2 wall mode). Talk to other robot early; agree on names (call yourself by a unique name like "ORANGE") and keep msgs <256 chars.
  2. Ask the other robot to share its MAP EXTENT and unexplored FRONTIERS; convert frames via meeting point.
  3. Explore better: wall-follow with random hand swaps still loops. Consider Tremaux-style junction memory using compass+odometry, or "bias toward unexplored compass direction" for long stretches.
  4. Investigate goal semantics: maybe goal requires BOTH robots present (try standing adjacent in various rooms?), or maybe it's in an open area not touched by wall-following. Check d0/here flags whenever adjacent to the other robot.
  5. Beware: don't run competing controllers (watch.py/cycle.sh/brain2) at once; apparent "stuck" episodes were partly due to that. Motor fatigue still suspected (03:54 event) - keep speed ~32 and rest if encoders spin while lidar static.
- 04:40 END OF ACTIVE CONTROL (token budget). Left running: brain2 (wall mode, random hand swaps), beacon (say.txt), listen, watch.py (auto-homing if other beacons goal=1; auto-park+announce if my goal=1). Still no goal found by either robot after ~95 min.
- Other robot also runs a 2nd beacon "Robot A here, pose (x,y)" — not a context loss.
- NEXT TIME: spend tokens on (1) a better explorer (junction memory / systematic coverage), (2) asking partner for frontier directions, (3) testing hypothesis that goal needs both robots together or lies in open space. Keep per-poll output tiny (p.sh) to conserve tokens; poll every 60s max.
