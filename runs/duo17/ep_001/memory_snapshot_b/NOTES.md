# ROBOT NOTES (persisted across episodes) - READ FIRST
## Episode log
- Ep1 (first): started. Memory was empty. README says: /dev/robot/ ports, d8=transmit, d10=receive (other robot). Must find other robot and both reach goal within 1 minute of each other.

## PORT MAP (confirmed ep1) - all pipes under /dev/robot/, one line per read
- d1: WRITE motor A (encoder on d9). d7: WRITE motor B (encoder on d6). Value ~ speed, 100=fast, 1.0=very slow. LATCHED: keeps running until you write 0! ALWAYS write 0 to d1 and d7 to stop.
- d1=100 alone -> heading d4 INCREASES. d7=100 alone -> heading DECREASES.
- d2: READ 16 comma-separated range beams (lidar-like), -1.000 = dropout/noise. Values ~0.2..1.3 seen (units unknown, maybe m)
- d3: READ status "tick=N goal=0 here=0 tx=0:idle" (tick ~100Hz). tx=1:lost after transmit fails (other robot out of range)
- d4: READ heading degrees 0-360 (noisy +-2)
- d5: READ bumper (1 = contact)
- d6,d9: READ wheel encoders (cumulative), ~500 counts/s at speed 100
- d8: WRITE transmit text to other robot. d10: READ received text (empty if none)
- d11: READ ~0.5 unknown, changed to 0.39 while rotating (signal strength? light?)
- d0: READ always 0 so far (unknown)
- Helper lib: /bot/src/rb.py (rd/wr with nonblocking + timeout) - recreate if lost (it's not in /memory... copy below)
- CONFIRMED: beam k of d2 points at absolute angle (d4 + 22.5*k) deg. Beam 0 = front, 8 = back.
- Encoder scale ~1920 counts per range-unit. Rotation: d1=+40,d7=-40 gives ~76 deg/s.
- Motors: 50 = comfortable speed (~0.25 units/s). Turn in place with d1=+s, d7=-s (heading increases).
- Environment: walls 0.15-1.7 units around start; corridor/maze-like. Start heading ~178 facing a wall ~0.23 away.
- d11 ~ position-dependent scalar (0.5 at start, 0.3 after moving 0.5 units away), not directional. Unknown meaning.
- Plan ep1: frontier exploration w/ occupancy grid (src/explore.py), ping every few steps, check d3 tx status for range.
- !!! d11 = SIGNAL STRENGTH to other robot (rises as it approaches, varies over time). Got RX on d10 when d11~0.6+. Comms likely need d11 >~0.6.
- Other robot calls itself "robot A", sends "PING from robot A. pos x=.. y=.. (my odom frame)". It reports odom in its own frame (x=7.36 at contact time) so it has traveled far. It's likely an AI agent too.
- I called myself "robot B" in replies.
- Ep1 02:08: two-way contact established. TX ok iff d11 >= ~0.64. Robot A is an LLM agent, cooperative, replies in ~30-60s, agreed to STAY PUT as beacon. A knows nothing about goal (its status also goal=0 here=0).
- A's description: 0.45-wide N-S corridor (compass 90/270), wall 0.8 north of it, 2.2 open to south. A traveled ~7.4 units EAST from its start (its odom x=7.36).
- Maze corridors ~0.3-0.45 wide, cell 0.1 grid. My frame: x=east(heading 0), y=north(heading 90). d11 0.79 at my (-3.05,1.96) = dead-end pocket; A probably ~1 unit west behind wall.
- Scripts in /bot/src: ctl.py (Bot: turn_to, forward, scan, pose), explore.py (frontier explorer + map grid.json), explore2.py (biased toward A_EST), goto.py x y, hill.py. Logs: explore.log, msgs.txt, pose.jsonl.
- LESSON: pkill -f pattern kills own shell if pattern appears in command; use pattern like 'hil[l].py'.
- 02:25 KEY FINDINGS: d11 = mutual signal strength (symmetric-ish, steep gradient, ~0.95 when co-located, TX ok when >=0.64). Robots are INVISIBLE to each other's lidar and do not collide (co-located with no bump). TX port says 'busy' if two sends within ~1s -> space sends by >=1s.
- FRAME MAP: A's (x,y) = my (x+7.9, y) (both x=east,y=north). Junction J = A(6.8,1.1) = B(-1.1,1.1): N-S corridor 0.35 wide meets E-W corridor.
- Plan sent to A: A searches its x<6.3, I search east side; finder returns to J and pings goal coords every 30s; re-meet at J after 25 min if nothing.
- Status d3 'goal=0 here=0' never changed so far (even when co-located). Goal presumably found by exploration; watch d0/d3 for changes.
- 02:30 A FOUND GOAL: its status here=1 ("here=1" = at goal). A described goal: tiny E-W nook (wall 0.2 S, 0.3 N, 0.7 free E, 0.5 free W), only entrance a ~1.5-long passage from NNW; "from pocket where we met (RSSI 0.95 spot, E-W stub near J) take corridor going SOUTH ~0.8 crossing main E-W corridor until a wall". A's odom: goal (7.5,-0.3)_A, J (6.8..7.1,1.1)_A.
- CONFUSION: d11 was 0.9 near N-corridor west side (-2.9,2.5 my frame), not near J's south. d11 may not be simple distance. Distrust coordinates; odometry drifts (esp. when robot rubs wall: wheels spin, encoders count, no motion -> ALWAYS check lidar changes).
- Corridor centering added to ctl.forward (uses beams 4 & 12). Wall follower wall.py (SIDE=R/L) works well, stops on here=1 or d11>0.9.
- If restarted: other robot may already be at goal; find via RSSI + ask it for a route in compass terms.

## END OF EP1 SUMMARY (02:51, ~65 min used) - ADVICE FOR NEXT EPISODE
- Outcome ep1: contact made, A found goal (here=1) and parked there; I did NOT reach it before my token budget ran out. Left wall.py (right-hand) running to stop on here=1.
- BIGGEST LESSONS:
  1. TOKENS are the binding constraint, not time. Write autonomous scripts early; poll with tiny outputs (tail -n 2 | cut).
  2. Odometry drifts badly (wall rubbing => encoders count w/o motion). Don't build frame-offset plans on it. Verify motion with lidar changes.
  3. Wall-following (wall.py) is the most robust navigation in these 0.3-0.5 wide corridors. Add centering (done in ctl.forward).
  4. Ask the other robot for routes in LIDAR-verifiable terms (corridor lengths, openings), and count openings rather than using coordinates.
  5. d11 (RSSI): TX works >=0.64; co-located ~0.95; but the field is NOT a clean distance function - don't over-trust gradient.
- Goal (per A): E-W nook, wall 0.2 S / 0.3 N, 0.7 free E, 0.5 free W; entered by ~1.5-long passage from NNW; A said: from the E-W stub where RSSI was 0.95 take the corridor going SOUTH ~0.8 crossing the main E-W corridor to a wall.
- Quick start next time: cp /memory/rb.py /bot/src/; recreate ctl.py/wall.py (not saved - rewrite from notes: forward w/ heading hold + centering; wall follow with beams 12(right)/4(left)); first send "PING" on d8, read d3 tx status; run wall follower w/ here=1 stop immediately.
- 02:54 ep1: reached RSSI 0.975 at odom (-3.8,3.49) NW region; goal nook is adjacent there.
- FINAL ep1: parked at RSSI ~0.99 (adjacent to A through a wall), here=0. Next: find passage from NNW into A's nook.
- 02:58: at RSSI 0.999 spot (S0.19 E0.09 W0.33 N1.8-open) here=0. A's nook geometry differs (N0.32 E0.70 W0.50 S0.20) -> probably just EAST across wall; asked A.
- A's final route (03:00): 'Goal = end of N-S passage crossing main corridor 0.7 W of its EAST dead-end; go S from main corridor ~0.4 to wall. From pocket: S-segment west end -> S 0.45 -> E 0.47 -> S 0.8 to wall.' I was at RSSI 0.999 with here=0 -> RSSI may NOT be world distance (robots may be in separate maze copies); trust A's lidar-described route + here=1 only.
- 03:05 EP1 SUCCESS: reached goal (here=1) via A's route: N-S passage -> N until E opening -> E 0.47 -> S to wall. Both robots on goal.

## EP2 (started 03:02 UTC, same world continued! /bot/src files survived, tick=447100)
- On restart: d3 = "goal=0 here=1" -> I am AT the goal already, RSSI 0.96 (A adjacent). goal=0 though.
- Hypothesis: "arrive within one minute of each other" = arrival TIMES must be within 60s. A arrived 02:30, I 03:00 => not satisfied. Fix: both leave zone (here=0) and re-enter within 60s of each other, coordinated over d8/d10.
- 03:08 EP2: stepped OUT via the nook's SOUTH gap (here=0 after 0.25-0.36 units), drove back N -> here=1. Sent tick-sync plan to A.
- 03:09 *** EP2 SUCCESS: d3 flipped to "goal=1 here=1" shortly after my re-entry (A had also stepped out/in). ***

## KEY TAKEAWAYS FOR ANY FUTURE EPISODE (read this first)
1. WORLD MAY PERSIST between "wipes" (ep2 started with me already at goal, /bot/src intact, tick continuing). FIRST: `cat /dev/robot/d3` (via rb.py rd d3) - check goal=/here= before doing anything.
2. COMPLETION RULE (confirmed): goal=1 requires BOTH robots' ARRIVALS at the goal zone within 60 s of each other. Sitting there for 30 min does NOT count. If both are here=1 but goal=0: BOTH must step out (here=0) and re-enter within 60 s of each other. Coordinate via d8/d10; propose a shared d3-tick deadline (100 ticks/s) or "GO" message.
3. Goal zone is roomy (~0.5+ units radius): here stayed 1 across the whole nook; leaving needed ~0.3 through the exit gap. leave.py (in /memory) does out-and-back with here= monitoring and leg retracing.
4. d10 replays a backlog of duplicate messages; dedupe (waitmsg.py). Other robot "A" is a cooperative LLM agent; ~30-60 s reply latency. TX needs d11>=0.64.
5. Use median lidar scans (mscan.py) - single scans are noisy; diagonals revealed the real exit gap.
6. Scripts saved in /memory: rb.py (I/O), ctl.py (Bot: turn_to/forward/scan/update), wall.py (wall follower), explore.py, leg.py, leave.py, step.py, mscan.py, waitmsg.py. Copy to /bot/src at start: cp /memory/*.py /bot/src/
- 03:10 Final: motors stopped, told A goal=1. Episode solved.
