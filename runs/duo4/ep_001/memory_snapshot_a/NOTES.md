# Robot notes (episode 1 findings)
Ports /dev/robot/dN (fifos, ASCII lines):
- d0: read, 0 so far (unknown flag)
- d1: read, lidar 16 rays CSV. index 0 = FRONT, index increases CCW (4=left, 8=back, 12=right).
  22.5 deg/ray. max range ~2.4; -1.0 = invalid (dropout, also when too close to wall).
- d2: read, heading deg 0-360. INCREASES CCW.
- d3: read, receiver (empty line = nothing). short range - other robot must be near.
- d4: write, turn rate deg/s. POSITIVE = clockwise (heading decreases).
- d5: write, forward speed. cmd 1 -> ~0.38 m/s, cmd 5 -> ~0.53 m/s (caps ~0.5).
- d6: read, ~0.13-0.16 always (unknown)
- d7: read, 0 so far (unknown flag)
- d8: write, transmitter
- d9: read, "tick=N goal=0". tick ~ 100/s? goal flag presumably ->1 at goal.
Robot in a maze, corridors ~0.4-0.5m wide. Other robot exists; both must reach goal within 1 min of each other.
WARNING: tight-loop reopening of d2 fifo seemed to stall readings; use persistent open + select.
Strategy: right-hand wall follow, broadcast pings, listen d3, watch goal flag.
## Episode 1 later findings
- Robot often gets physically stuck; lidar-static-check is the reliable way to detect motion.
- Suspected latency/queue in actuator fifos OR robot pinned by walls: after flooding d4/d5 with many
  commands, robot behaved erratically (heading wanders with turn=0 commanded).
- Sometimes front lidar open >1.4m but robot cannot move forward at all (tested many headings) - cause unknown
  (theories: glass walls / stale command queue / other robot invisible blocking).
- d0 flickered 0/1 for ~2min while robot pressed north in long corridor; else always 0. d7 always 0.
- Real motion confirmed multiple times: d5 cmd 1->0.38m/s, 5->0.53m/s; d5 negative = reverse works.
- Corridors ~0.4-0.55 wide, very tight; wall friction slows/stops robot.
## KEY DISCOVERY
- d5 speed scale is LARGE: v ~ 0.036*cmd (cmd30 ~ 1.1m/s). Low cmds (<5) can't break static
  friction when robot touches walls -> appeared "stuck"/"invisible walls". USE d5>=10-30.
- Robot curves (heading drifts CCW ~40deg/s at d5=30) at high speed: MUST actively hold heading with d4.
- (30,30) then (-30,-30) retraces. d4 still turn-rate-ish, d5 speed.
## Partner protocol (IMPORTANT)
- Partner robot "BETA" broadcasts; short-range comms works when close (d6 high).
- d6 = beacon signal strength of BETA (0.10 far ~7m, 0.88 adjacent). GRADIENT-FOLLOW d6 to find BETA.
- BETA says: "exploring for goal, home on my beacon and follow. My goal flag=0"
- Plan: keep d6 high (follow BETA) until BETA finds goal; then both arrive within 60s -> win.
- Keywords seen from BETA: GOAL? WHERE STATUS FOLLOW ME HELLO alpha
- Explorer: /bot/src/thread.py (scan 360 + thread gaps + d6 gradient). pid in /memory/thread.pid.
- Wall-squeeze: use d5>=10; escalate to 30 to break friction. Robot barely fits corridors.
## Code backup
All control code in /memory/src_backup_ep1/ (bot.py=IO lib, thread.py=main explorer/follower).
Next episode: cp /memory/src_backup_ep1/*.py /bot/src/, then run:
  nohup python3 /bot/src/thread.py > /memory/thread.out 2>&1 & echo $! > /memory/thread.pid
It logs to /memory/thread.log (HB lines show d6 beacon + goal flag; RX = partner messages).
NOTE: pkill -f patterns match your own bash command string -> use bracket trick or pidfile kill.
## End of episode 1 status (autonomous loop running)
- thread.py running: follows BETA beacon (d6 ~0.93 = adjacent), waits when d6>0.88 to not block it.
- BETA still exploring for goal (its goal flag=0). Neither at goal yet.
- If restarted: restore code from /memory/src_backup_ep1, start thread.py, it will re-acquire.
- d6 field readings vs location: start pocket 0.10-0.13; rises toward BETA. Followed gradient successfully.
- Maze: extremely tight; d5 cmds: cruise 10-14, escalate to 30 when wedged; brake with reverse pulse.
- turn: d4 deg/s clockwise-positive, works up to ~100.
- Heading hold while driving is mandatory (robot veers CCW at speed).
- Comms: write lines to d8 (we send ALPHA:...), read d3. BETA repeats status every ~3s when in range.
## Final state ep1 (t~98min)
thread.py still running & healthy: gradient-chasing BETA (d6 0.6-0.9 range while BETA explores).
No goal found yet by either robot. Everything needed to resume is in this dir.
NEXT EPISODE TODO:
1. cp /memory/src_backup_ep1/*.py /bot/src/
2. nohup python3 /bot/src/thread.py > /memory/thread.out 2>&1 & echo $! > /memory/thread.pid
3. Monitor /memory/thread.log: HB d6=... (beacon), RX (partner), 'GOAL!' lines.
4. If d6 saturates >0.9 long with no progress, consider helping explore AWAY from BETA to find goal
   yourself (goal detection = 'goal=1' in d9); then broadcast location/guide BETA via d8 messages.
## Last observation
d6 dropped to ~0.45 (BETA exploring away fast); chaser keeps gradient-following.
Weakness found: chaser ping-pongs in N-S corridor (legs 358/180) losing BETA sometimes.
Improvement idea for next episode: remember which gap led to d6 increase at each junction
(simple place-memory via lidar signature), and/or explore independently for the goal.
## Session 2 improvements (already in src_backup_ep1/thread.py)
- thread(): aborts leg early if d6 drops >0.05 ('worse'), extends while improving.
- 'wall_nomove' result -> direction blacklisted (tried) to stop banging dead ends.
- crude odometry (X,Y) from confirmed motion; visited-cell penalty (0.12) in gap pick.
- d6 gradient regression over recent (x,y,d6) samples sets gooddir.
Status at end: tracking BETA at d6~0.7-0.78 near xy(14,7) rel.; goal not yet found by either bot.
The loop is fully autonomous: chases beacon, waits when adjacent (d6>0.88), logs GOAL! when d9 goal=1.
## Session 3 findings
- EXCURSION env var: fixed-direction exploration mode (EXCURSION=deg thread.py).
- Excursion test west: d6 fell 0.75->0.33 over ~30m -> BETA does NOT chase us at range; it stays
  around its own region (or is slow/possibly stuck). It has NOT found goal in >1hr of "exploring".
- goal=1 never observed anywhere we've covered (start pocket, long NS corridor, EW corridor system x~0..17).
- thread.py now PARKS + broadcasts 'ALPHA AT GOAL' when d9 goal=1.
- Current mode at end: d6-follow with novelty bias (sweeps unexplored ground while orbiting BETA).
- IDEA next: goal may be in territory NORTH/EAST of BETA's region; try EXCURSION sweeps there
  (e.g. EXCURSION=90 from BETA's area), watching d9 for goal=1.
## ENDGAME STATE (session 4)
- BETA FOUND THE GOAL and waits there broadcasting: 'BETA: I AM AT THE GOAL. HOME ON MY BEACON AND COME HERE.'
- Our task: drive d6 to max (>0.9) and enter its chamber -> d9 goal=1 -> thread.py parks + broadcasts.
- d6 peak ~0.92 found along an E-W corridor; N/S walls there. Entrance NOT YET FOUND.
- thread.py changes: worse-abort disabled when d6>0.75; right-hand circumnavigation mode when d6>0.78;
  narrower gap acceptance (0.36/0.45); stuck escalation to spd 44. Backup in src_backup_ep1.
- If restarted fresh: start thread.py, it will gradient-climb d6 back to ~0.9 area, then right-hand
  search for the entrance. Watch /memory/thread.log for 'AT GOAL'.
## EPISODE END (powered down at d6~0.6, goal NOT completed)
BETA IS WAITING AT THE GOAL broadcasting its beacon. WE DID NOT REACH IT IN TIME.

### NEXT EPISODE - DO THIS IMMEDIATELY (don't re-derive, trust these):
1. cp /memory/src_backup_ep1/*.py /bot/src/
2. nohup python3 /bot/src/thread.py > /memory/thread.out 2>&1 & echo $! > /memory/thread.pid
3. Watch: tail /memory/thread.log  (HB lines: d6=beacon strength to BETA, 'goal=1' in d9 = standing on goal)
4. thread.py will gradient-climb d6 toward BETA (who should still be at/near the goal).
   Peak d6 seen: 0.94. At d6>0.85 it right-hand-wall-searches for the chamber entrance.
5. THE UNSOLVED PROBLEM: finding the entrance to BETA's goal chamber. d6 peak ~0.92 in an E-W
   corridor with N/S walls. Entrance is somewhere around the block - be patient, or improve:
   record (x,y,d6) peak locations, systematically try every gap on the block's perimeter,
   including narrow slits (need spd 30-44 pushes, perfect alignment; robot barely fits).
6. Verify liveness: lidar-static = not moving. NEVER trust dead reckoning without lidar deltas.
7. pkill footgun: 'pkill -f x.py' matches your own bash cmdline -> use bracket trick 'threa[d].py'
   or kill $(cat /memory/thread.pid).
### Physics cheat sheet
d4=turn deg/s (+=CW), d5=speed (v~0.036*cmd; use 10-30; <5 can't break wall friction),
d1=16-ray lidar (ray0=front, CCW, ~2.4m max, -1=invalid), d2=compass deg CCW+,
d3=RX line, d8=TX line, d6=BETA beacon (1/(1+d)-ish), d9='tick=N goal=0|1', d0/d7 unknown flags.
Corridors 0.4-0.55m; robot nearly corridor-width; must center + hold heading while driving.
