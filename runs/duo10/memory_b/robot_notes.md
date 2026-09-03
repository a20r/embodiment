# Robot control knowledge (verified)
Ports /dev/robot/ (ASCII lines, use retry-on-empty reads):
- d0: WRITE radio transmit. d4: READ radio receive (blocks; use select).
- d1: heading deg (compass, noisy +/-4). Increases when turning clockwise.
- d2: unknown, always 0 so far (bump?). d9: unknown always 0.
- d3: 16 range beams, CSV. max 3.0, -1.0=bad reading.
  BEAM 2 = BODY FRONT. beam i at body angle 22.5*(i-2) deg CLOCKWISE.
  world azimuth of beam i = heading + 22.5*(i-2).
- d5: blocks on read AND write (unknown; maybe event port).
- d6: status "tick=N goal=0 here=0". tick ~100/s. here/goal flags presumably.
- d7/d8: wheel encoders left/right (rate = 5*cmd per sec).
- d10: left motor cmd. d11: right motor cmd. (write numbers)

# Motion model (verified by experiment)
- omega [deg/s] ~= 1.0 * (l - r)  (pure rotation in place when l=-r)
- v along beam2 dir ~= 0.04 * (l+r)/2 approx (5,5)->0.2-0.4 units/s. DEADZONE: (2,2) no motion.
- (6,6),(12,12) showed little motion in cramped spots - watch beam2 for blockage; robot blocked ~0.3 from wall.
- Wheel slip: encoders count even when blocked. Don't trust encoders for odometry when near walls.

# Env
- Maze with corridors ~0.5-1 wide. Cell/scale: ranges 0.15-3.0.
- Another robot exists; must both reach goal within 1 min of each other.
- Radio: write line to d0, other robot may hear; read d4.

# Tools
- /bot/src/rob.py helper lib. /tmp/tel.log background logger (logger.py).

# Radio / other robot (episode 1 findings)
- Other robot is ANOTHER AGENT, calls itself botA, broadcasts "HELLO botA x=.. y=.. d5=.."
  It has its own x,y odometry. It paces x~0.15, y 1.65..3.95; its d5: 0.64 at y=1.65 -> 0.97 at y=3.9.
- d5 = scalar sensor ~goal proximity signal in [0,1]. OUR d5 ~0.99 (very close to goal!).
- d4 emits ~4 lines/sec, blank if no message.
- Radio does NOT loop back own messages (botA tested XYZZY123).
- status d6: goal=0 here=0; presumably here=1 at goal, goal=1 when both.
# Movement gotchas
- Robot wedges in narrow corridors (walls ~0.1 on both sides): (8,8) then produces NO motion.
  Unstick: reverse, wiggle rotate. Corridors barely wider than robot.
- Rotation in place works reliably even when wedged: (12,-12) ~24deg/s CW.
# CRITICAL movement rule (ep1)
- Translation works reliably at commands (7,7)/(-7,-7): speed ~1.1 units/s. 
  (5,5),(9,9),(12,12),(2,2) often produce NO motion (stick-slip). USE 7!
- Rotation: (s,-s) works at s=6..14, ~2deg/s per unit... measured (12,-12)=24deg/s.
- Stall detect via front beam; unstick with (-7,-7).
- REFINED: translation ONLY with EXACTLY equal commands (7,7) or (-7,-7). Any asymmetry (7.5,6.5) stops translation!
  So: stop-and-go: rotate in place with (s,-s), then pure (7,7). Never blend steering.
# Maze facts (ep1, ~55min in)
- Maze corridor axes at compass ~33+90k deg (learned: 33/123/213/303).
- Corridors barely fit robot: must align within ~3deg to translate; else jam (stick-slip).
- hunt_drive in /bot/src/drv2.py: wiggle-hunt alignment, stall recovery. wf4.py explorer.
- Our pocket: corridor along 33/213 axis, ~2-3 units long. Other robot botA roams x 0..0.6, y 0..3.9 own frame.
- d5 = inter-robot proximity (confirmed): 1.0 adjacent, 0.26 when far.
- botA protocol active: it broadcasts HELLO botA x= y= [d5=]. It agreed: it wall-follows RIGHT, we LEFT; finder announces & other climbs d5.
# Translation mystery (unresolved, ep1 ~70min)
- Even exact (7,7) aligned to corridor often creeps ~0.02/s instead of ~1/s. Sometimes (4,4) catches. Stochastic.
- No clear dependence on heading/encoder phase/d9/d2 found (analysis of tel.log).
- d2=1 only during rotation sometimes (contact?). d9 flips 0/1 unrelated to motion (maybe other-bot proximity flag?).
- WORKAROUND: persistent pushes: alternate (7,7) 1.2s / (-7,-7) 0.3s kick / (4,4) / wiggle rot +-3deg. Catches within 5-30s usually. When caught, speed ~1 unit/s; KEEP pushing same command while progress continues.
- Successes seem more common in narrow corridor (front 1-2) than wide junctions (front 3.0) — unconfirmed.
- BREAKTHROUGH: pulsed drive can break stiction: motors(7,7) 0.5s on /0.06s off repeated ("D" pattern), also 0.18/0.08 sometimes. Solid (7,7) or (4,4) sometimes. STOCHASTIC: cycle through patterns, keep what catches.
- Corridor dirs LOCAL vary: use fine sweep with front beam while rotating slowly (man.py fsweep) instead of fixed axes. This pocket's corridors: ~30 and ~225-230 compass.
