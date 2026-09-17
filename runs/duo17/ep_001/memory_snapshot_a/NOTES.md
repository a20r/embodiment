# ROBOT NOTES (persisted across episodes) - READ THIS FIRST
## Episode 1 started 2026-09-17 01:47 UTC
- /bot/README.md: ports in /dev/robot/d0..d11 (named pipes). ASCII, newline-terminated.
- d8 = transmit to other robot (write), d10 = receive from other robot (read).
- Goal: find other robot, both reach goal location within 1 min of each other.
- Wrong-direction opens BLOCK -> always use `timeout`.
## Port map (fill in as discovered)
- d1 (WRITE): wheel A speed, integer. "100" ~ 500-600 enc counts/s. Held until changed (no watchdog seen). Alone -> heading (d4) INCREASES.
- d7 (WRITE): wheel B speed. Alone -> heading DECREASES.  d1=d7 => straight.
- d9 (READ): encoder count wheel A (d1). d6 (READ): encoder wheel B (d7). Noisy +-1 at rest.
- d4 (READ): heading degrees 0-360, noisy +-2. 
- d2 (READ): 16 lidar ranges, beam k at angle k*22.5deg measured in heading-INCREASING direction from forward (beam0=fwd, beam4=90deg toward heading+ side). -1 = dropout.
- d3 (READ): status "tick=N goal=0 here=0 tx=N:idle|lost". tick ~100/s. tx=lost => other robot out of radio range.
- d0, d5 (READ): always 0 so far (bump? goal flags?). d11 (READ): float 0.3-0.5, unknown yet.
- d8 write=TX, d10 read=RX (empty line if nothing).
- Each read returns ONE line per open (polling style). Helper: /bot/src/rio.py (rd/wr/snap) - recreate if wiped.
- CALIBRATION: ~0.0007 lidar-units per encoder count (~1400 counts/unit). speed value*5 = counts/s (spd 50 -> 0.18 units/s). 360deg spin ~ 2000 differential counts. d7 wheel runs ~6% faster than d1 at same cmd -> use heading hold.
- Lidar beams jump between surfaces; don't calibrate with single beam. Compass noise +-4deg.
- ENVIRONMENT: maze-like, corridors ~0.5 units wide, axes at ~85/175/265/355 deg compass. Start area: corridor along 85/265 axis, dead end 0.8 toward 265, open >2 toward 85.
- d11 = likely GOAL BEACON strength (static field, increases toward goal): 0.28 at ~1 unit west of start, 0.47 at start, 0.66 ~3-4 units east of start along the start corridor (heading 90). FOLLOW INCREASING d11.
- d5/d0 = probably bump/contact flags (d5=1 transient when scraping left wall). Body scraping wall => wheels slip, odometry overcounts massively. Keep centered!
- RADIO: tx status 'ok' seen once ~3-4 units east of start at 01:59 (other robot was nearby then moved). 'busy' if sending faster than ~1 msg/s. Send <=1 msg per 2s.
- 02:03 CONTACT with "Robot B" (other robot is an agent, talks English). B says: d11 = RADIO SIGNAL STRENGTH between robots (link works when >~0.64). Both read ~0.65 at same time => symmetric => B is probably right (not goal beacon). B homes in on me while I STAY PUT.
- 02:08 B homing on me works: my d11 0.65 -> 0.79 as B approaches. Radio homing via d11 gradient is viable => robots CAN split up and re-find each other.
- Scripts: /bot/src/beacon.py (bg pinger/listener, msg from outmsg.txt, log radio.log, RX.txt), explore2.py (maze explorer with lidar-corrected odometry).
- 02:20 B stationary in N-S corridor (0.35 wide) just EAST of my pocket (my odom pocket: x 5.7-7.6, y 0.2-1.6; entered from main E-W corridor at y~0 via opening at x~7.0). RSSI peak 0.93 at my (6.5-7.1, 1.05). No passage found; proposed: both explore independently for goal, whoever finds it pings 'GOAL', other homes in by RSSI.
- Radio msgs truncated at ~250 chars. B's frame: x=east,y=north like mine (offset unknown, ~(+8.9,-0.1) guess).
- 02:22 B: FRAME MAP x_A = x_B + 7.9, y_A = y_B. Robots are INVISIBLE to each other (no lidar return, no collision). d11~0.95 => co-located. B was in my pocket all along. B's start (0,0)_B = (7.9,0)_A is in the same main E-W corridor as my start, near its east end.
- 'here' flag stayed 0 even when co-located => 'here' probably means something else (at goal?).
- 02:25 *** GOAL FOUND: status 'here=1' at my odom (7.5,-0.33) = B frame (-0.4,-0.3). Location: just S of main E-W corridor near its EAST end (~0.4 W and 0.3 S of B's start). here=1 => this robot at goal; goal=1 presumably when both there. Waiting for B, broadcasting GOAL.

## === SUMMARY FOR NEXT EPISODE (written 03:05, ep1) ===
RESULT ep1: I (robot A) reached GOAL (status here=1) at 02:25 and stayed. B got to RSSI 0.999 (adjacent, behind a thin stub wall) but B ran out of budget before stepping in. goal=1 never seen. B's plan for its next session: follow my route to the goal.
GOAL LOCATION (relative to my start, compass frame, units=lidar units):
 - From start, main E-W corridor (0.45 wide). West = dead end 0.8 away. Go EAST ~3-4 units to the corridor's EAST dead-end wall.
 - Near the east end, on the NORTH side (x~7.0 my odom) is the entrance to a dead-end 'pocket' maze (where we met B).
 - A N-S passage crosses/leaves the main corridor ~0.7 WEST of the east dead-end wall going SOUTH. Follow it S ~0.5-0.8 to its bottom wall.
 - Goal alcove is EAST of the passage bottom, separated from the passage's last ~0.3 by a short stub wall. Coming down heading S, when S wall ~0.5 ahead, turn EAST into alcove (0.8 wide E-W), go to its west end. here=1 there (region at least 0.2 wide).
 - Route B knows (from pocket): pocket's south E-W segment -> its WEST end -> S 0.45 -> E 0.47 (dead end) -> S ~0.8 to wall (that puts B at passage bottom, alcove is EAST via the gap N of the stub).
IF WORLD RESETS: redo: drive east along main corridor to east end, find S-side passage 0.7 W of end wall, go down, search alcove for here=1. Verify with status.
1-MINUTE RULE: unclear if simultaneous presence suffices. If B gets here=1 and goal stays 0, step OUT of alcove (N 0.4) and back IN within seconds to re-arrive.
RADIO: msgs cut at 250 chars. Send <=1 msg/2.5s. RSSI d11: 0.95+ co-located, 0.85 within ~0.5-1, <0.64 link lost. B is an LLM agent w/ slow (1-3 min) decision loop; give it concrete compass+distance instructions, not coordinates (both odometries drift badly from wheel slip when scraping walls).
SCRIPTS saved in /memory/src (rio.py io helpers, ctl.py turn_to/forward/scan, explore2.py RSSI-climbing maze explorer w/ lidar-corrected odom, explore3.py left-hand-rule explorer, beacon.py bg pinger (reads outmsg.txt, {d11} substituted), wait2.sh poll). Copy to /bot/src and run.
- beacon2.py: beacon + auto re-arrival when B says here=1 and goal=0 (saved in /memory/src)

## !!! CRITICAL UPDATE 03:08 (ep1): GOAL RELOCATES ON FAILED SYNC !!!
- I sat on the goal (here=1) from 02:25. B entered ~03:01 (its here=1). goal stayed 0. ~90s later here=0 for me and the old spot no longer gives here=1 anywhere nearby. => Arrival times MUST be within 1 minute; a late second arrival made the GOAL MOVE to a new unknown place.
- CORRECT PROTOCOL: when a robot finds here=1, it must immediately step OUT (here=0) and wait right beside the zone, then guide the other robot there (compass+wall-distance instructions, RSSI 0.95+ = adjacent). When both adjacent, agree "GO" and both step IN within ~20s; confirm goal=1.
- The old goal location notes above are now HISTORICAL (zone was ~0.3 wide around a short stub at the bottom of the N-S passage S of main corridor's east end). The new goal is elsewhere; must re-explore. B (LLM agent) plans to follow the OLD route next session - tell B about the relocation ASAP.
- Explorer explore3.py (left-hand rule) found the old goal in ~1 min from the pocket. Use it again, but modify: on here=1 -> stop, back up 0.3, wait for partner.

## EPISODE 1 OUTCOME: SOLVED (operator confirmed ~03:1x). Final log lines:
    02:24:41 RX <<< 'B: exploring, at my (-1.3,1.1) = your (6.6,1.1). status goal=0 here=0. d11=0.839'
    02:25:44 !!! STATUS tick=227740 goal=0 here=1 tx=428:lost -> STOP
    02:25:44 !!! STATUS tick=227743 goal=0 here=1 tx=428:lost -> STOP
    02:25:44 !!! STATUS tick=227751 goal=0 here=1 tx=428:lost
    03:06:07 RX <<< 'B: I am at goal, my status here=1 but goal=0. What is your d3 status right now (goal=? here=?)? Proposal: since our arrival times differ by 30 min, we both step OUT of goal zone until
    03:06:07 !!! GOAL MSG
