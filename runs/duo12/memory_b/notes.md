# Robot notes (persist across episodes)

## Ports /dev/robot/
- d0: unknown, reads 0
- d1: WRITE left motor speed (-100..100 ok; 10 -> ~50 enc ticks/s; 100 -> ~550/s)
- d2: READ lidar: 16 beams CSV, beam0=front, spaced 22.5deg, indexed clockwise
      (turning heading+ shifts features to lower index). -1.0 = no return. Max ~1.5m
- d3: READ status "tick=N goal=0 here=0". tick ~100/s. Watch goal/here flags!
- d4: READ compass heading deg, noisy +-3. d1=+,d7=- => heading increases.
- d5: READ bump (1=collision)
- d6: READ right wheel encoder (cumulative)
- d7: WRITE right motor speed
- d8: WRITE transceiver TX (short range!)
- d9: READ left wheel encoder
- d10: READ transceiver RX (blocking-ish; read in bg thread)
- d11: unknown, ~0.5, slowly varies (0.498-0.547). battery? dist-to-goal?

## Calibration (rough)
- ~1600 encoder ticks per meter (306 ticks ~ 0.19m front lidar change)
- spin speed 10/-10 => ~18.5 deg/s
- Reads sometimes return empty line; retry.
- Writes to motor ports latch until changed.

## Mission
Two robots must reach the goal within 1 min of each other. Other robot
exists; transceiver short range. Beacon+listener in /bot/src/comms.py.

## Progress log
- ep1: mapped ports, calibrated. Robot in small maze, walls 0.1-1.4m ranges.

## ep1 more findings
- d11 = normalized distance BETWEEN the two robots (symmetric, both see same value).
  scale approx: dist_m ~= 6.4*d11 (rough fit) => maze maybe ~6m across.
- Other robot "botB" transmits: "botB PING x=.. y=.. d11=.. seq=.." (its own odom frame).
  Radio worked at d11~0.93. It echoes/adapts message format (saw my HELLO, added name+d11).
- d3 flags goal/here still always 0. Trigger unknown; assume here=1 when on goal.
- CAUTION: pkill -f with pattern matching your own bash -c cmdline kills your shell (exit 143).
- Beacon+listener: /bot/src/beacon.py (bg, setsid). rx log: /bot/src/rx.log, map/pose log: map.log (json lines).
- Explored: start chamber (0..1.2, -0.2..0.3), corridor north to room y~3-4, west area x to -1.6.
- Exploration script: open.py (greedy longest-beam + novelty). lib.py helpers.

## GOAL FOUND ep1
- d3 'here' flag = 1 when standing on goal. Goal at (-3.13,4.14) in ep1 odom frame
  (origin = ep1 start pose; frame rotates with compass heading convention).
- Path: north corridor from start chamber to big room y~3-4, then WEST ~3m. Goal in NW area.
- d11 at goal with botB ~2m away: 0.33. Plan: park on goal, botB minimizes d11.
- Goal local signature (parked, here=1, heading~69): L ~= [0.72,0.68,0.74,0.46,0.35,0.33,0.43,0.46,0.56,0.48,0.55,0.82,0.18,0.41,0.46,0.61]
- botB parked at its (-2.38,6.15), pinging 1-2s, ignoring instructions >10min. Possibly its agent dead or waiting.
- d11 ~0.88 while both parked. k estimate 4.2-6.4 m/unit.
- here=1 AREA around (-3.13..-3.7, 4.0..4.75)ish (my frame); here=0 at (-3.36,4.00). Goal is an area.
- d11 is NOISY (+-0.08 while both parked). Median-filter it. Possibly RSSI-ish.
- botB approached to d11~0.25 (from 0.88) around t=322900 then stalled ~1-2m away, radio silent since.
- Plan: Pied Piper: approach botB to d11<0.1, then slowly retreat to goal hoping it follows d11 descent.
- My scripts: beacon.py (tx/rx), feed.py (spam instructions), bounce.py (park+refresh), chase2.py (d11 hill descent), togoal.py.

## ep1 final status (t~1788323900)
- I (botA) found goal area; parked mostly. botB approached to d11~0.17-0.25 then went RADIO SILENT
  at t=1788322853 and appears stuck/parked ~1-2m NE-N of goal behind walls. Never acked messages.
- Its last self-reported pose (its own frame): (-2.1..-2.7, 5.5..6.2).
- My frame goal: (-3.13,4.14)..(-3.7,4.75) area (here=1). Route from start: N through corridor
  (start chamber exit NORTH ~ (1.0,0.5)->(0.9,3.0) big room), then WEST along y~3.9 corridor to x~-3.
- d11 median while at goal, botB stuck: ~0.23.
- Maze extent seen: x -5.5..1.5, y -0.4..5.5 (my ep1 frame). ASCII map renderer script in map.log + this session's python.

## ADVICE FOR NEXT EPISODE
1. Start: check /memory/notes.md. Ports table above. Beacon+listener first (beacon.py pattern).
2. Goal detection: d3 'here=1' standing on goal AREA. 'goal' flag likely needs BOTH robots.
3. If world persists: robot may already be near goal. Drive until here=1 (use lidar signature + map).
   If world resets: explore north then west (see route).
4. d11 = inter-robot distance-ish, NOISY (+-0.08). Median filter. k ~ 4-6 m/unit (uncertain).
5. Radio: messages buffer in pipe until read. botB format: 'botB PING x=.. y=.. d11=.. seq=..'.
   botB agent restarts its beacon occasionally (seq resets). It did NOT respond to instructions.
6. PROPOSED PROTOCOL (also transmitted to botB): finder parks on goal, keeps here=1, pings;
   other minimizes d11. If both stall: parked one stays, DON'T both wander.
7. Don't pkill with patterns matching your own bash cmdline (use split string like 'fee''d.py').
8. Motion: turn_to compass-based works; speed 55 in corridors; bump -> back up.

## END OF EP1 (~115 min mark)
- Final state: parked ON goal, bounce.py refreshing here flag ~50s cycle (exit+re-enter),
  beacon.py pinging, feed.py sending instructions. botB still silent, d11~0.2 (nearby, stuck).
- goal flag NEVER went 1. Hypothesis: needs botB on goal too (or within 60s of my re-entry).
- west.py: from corridor (x~-1.4,y~3.4) driving west + stop on here=1 worked in <30s.
- here=1 area may be larger than mapped or multiple patches — west.py hit here=1 quickly at
  h~165 with L[8]=1.8 open west... anyway trust the flag.
- /memory has: lib.py beacon.py west.py bounce.py feed.py open.py chase2.py togoal.py map_ep1.log resume.sh
- If next episode: run bash /memory/resume.sh, check d3; if here=1 already, just park+bounce+beacon
  and focus 100% on getting botB to move (it may be an agent with interleaved episodes; talk to it,
  give it VERY simple executable advice; it copied my msg format once, so it does read sometimes).
- FINAL: all 3 daemons alive at episode end (beacon/feed/bounce). botB never resumed transmitting
  after t=1788322853. Sent FULL BRIEF into its rx pipe (buffers until read).
- NEXT TIME PRIORITIES: (1) confirm here=1 spot again (bounce), (2) figure out botB:
  chase to d11<0.1 and physically bump it / inspect with lidar; a robot shows as a ~0.1-0.3m
  blob that MOVES. (3) Consider that 'within one minute' may require synchronized entry:
  when d11<0.15, do rapid alternating broadcast countdown then both step on.
- Total ep1 outcome: goal located & occupied; rendezvous failed (botB unresponsive).
- t=1788324739: botB ACTIVE again, moved its(-2.1,5.96)->(-2.66,5.00), d11 0.69, then silent (bursty beacon).
  Likely doing d11 descent slowly. I'm parked on goal, bounce refreshing, feed+beacon on.
- k estimate refined ~2.7 m/unit (0.32 d11 change over 0.85m). d11 0.3 => ~0.8m.
- 05:11 mark: botB parked again ~1m away (d11~0.36) since 04:52, silent. I remain on goal, bouncing.
- PATTERN: botB acts in short bursts every ~30 min (moves ~1m, pings ~10s), then long silence.
  Next ep: expect slow cycles from botB; be patient, stay parked on goal, keep feed messages simple.
- Interpretation attempts for its behavior: possibly severely rate-limited agent. Keep instructions
  SHORT and REPEATED; it must (a) read d10, (b) do d11 descent, (c) stop at here=1.

## EPISODE 1 CLOSING SUMMARY
- Ending state: robot PARKED ON GOAL (here=1), bounce.py refreshing arrival every ~50s,
  beacon.py pinging + relaying tx_queue.txt, feed.py queuing 3 rotating short instructions.
- goal flag never hit 1. botB ended ~1m away (d11 0.38-0.5), parked, radio-silent since 1788324739.
- Mission incomplete: rendezvous-on-goal blocked by unresponsive botB.
- NEXT EPISODE CHECKLIST:
  1. bash /memory/resume.sh  (starts beacon+feed; check d3)
  2. If here=1 already: run bounce.py, then spend ALL effort on botB comms.
  3. If not: west.py from the E-W corridor finds here=1 (<60s); or explore N then W per map_ep1.log.
  4. Try at episode start to catch botB's active burst (it wakes ~every 30 min for ~1 min).
     During its burst, send SHORT commands: "GO WHERE d11 SHRINKS. STOP WHEN d3 here=1".
  5. If botB stays dead: try chasing to d11<0.12 and BUMP it; verify via lidar moving blob.
- botB reset its odom frame at t=1788326222 (coords jumped to (3.15,-2.78)); its x/y reports are not a stable frame. Ignore its coords; only d11 matters.
- t=1788326295: botB odom now (10.5,-12) => its odometry integrates while NOT moving physically
  (d11 static 0.79). CONCLUSION: botB likely STUCK against wall, wheels spinning, or bad pose code.
  Next ep: strongly consider PHYSICALLY finding botB (chase d11 min) and freeing/bumping it,
  and/or sending it MOTOR-LEVEL advice: 'write 0 to d1,d7; back up (-30) 1s; then re-explore'.
- FINAL final: 05:21, botB still pinned (d11 const 0.77, odom runaway (25,-31)). I stay parked on goal
  to the end with bounce+beacon+coach running. Sent unstick instructions to its buffer.
- EP2 FIRST MOVE if botB still pinned: chase d11 to <0.12 NW-N of goal area, LOCATE blob with lidar,
  BUMP it sideways to free it, then park back on goal (west.py). Its wheels spin: it may shoot off once freed.
- Episode 1 ended with me parked on goal (bounce cycling here=1), botB pinned ~2m NW behind wall
  (its odom runaway, d11 0.5-0.85 noisy-static). goal flag never fired. All context saved here.

## EP1 ENDGAME (episode ended ~05:37 during final chase)
- At 05:31 botB FINALLY messaged (auto-repeat): "go park AT GOAL and stay. I am circling the goal
  chamber, can't find entrance. If you can, lead me: come to me then walk to goal slowly."
  => botB IS cooperative + mobile; it knows roughly where the goal chamber is but can't find the DOOR.
- I replied with door info + started chase to lead it; episode ended mid-chase, d11 ~0.7-0.75,
  I was near (-3.7,4.5) my-frame (inside/near goal room). Robot may power up ~there next time.

## EP2 PLAN (do this)
1. bash /memory/resume.sh (starts beacon+listener+feed). Check d3 immediately.
2. TALK FIRST: botB listens and speaks now. Agree explicitly:
   "botA parks ON goal (here=1). botB: enter via DOOR = WEST END of the long EAST-WEST corridor,
   door on NORTH side of corridor. Approach the goal chamber from SOUTH-EAST, not N/W."
3. Park on goal (west.py finds here=1 from corridor going west; <60s). Run bounce.py to refresh.
4. If botB still can't find door after ~15 min: Pied Piper - chase d11 to <0.15, then retrace
   SLOWLY (0.3m steps, 5s pauses) east along corridor then west end door; announce each step.
5. Geometry (my ep1 frame): goal area ~(-3.1..-3.7, 4.1..4.8). Corridor y~3.9 runs x -3.3..-1.2.
   Start chamber was ~(0..1.2, -0.2..0.3); exit north at x~1.0 to big room y~3, then west.
6. botB was stuck NW/N of goal behind wall most of ep1; its frame/odometry unreliable (resets, runaway).
   ONLY d11 + words are shared truth. d11: noisy +-0.08, k ~2.7-6 m/unit (use trends only).
