# Robot port map (confirmed by experiment)
- /dev/robot/* are FIFOs, one line per open() read; reopen each poll.
- d0: read. "tick=N goal=0" (~100 ticks/s). goal flag -> presumably 1 when goal reached.
- d1: WRITE steering angle in degrees (like a car rudder). Effect only while moving.
      d1=90, d3=4 => heading rate ~ +3.5 deg/s. Small angles (<10) negligible.
- d2: read. 16 lidar beams CSV. beam i at (heading + i*22.5deg) world angle? beam0=forward.
      Max range ~1.6 (maybe clipped). -1.000 = no return/dropout (occasional).
- d3: WRITE throttle. Persists until overwritten! Always write 0 to stop.
      d3=3 => ~0.02-0.03 units/s. d3=4..10 similar-ish? (verify speed scaling)
- d4: read. compass heading deg 0-360, noise +-3deg. increases with d1>0 while moving fwd.
- d5: read. bump/stall flag (1 when pushing into wall).
- d6: read. always 0 so far. unknown.
- d7: read. noise +-0.03. unknown (gyro?).
- Write ports: sim reads d1,d3. Commands PERSIST (no watchdog): zero them when done.
- No help console. No network. Sim not inspectable (runs outside container).

## Dynamics (measured)
- speed ~0.064/s @ thr10, 0.088 @ thr20, 0.118 @ thr50 (saturating; inertia present)
- turn: heading rate ~ 0.64deg/s per throttle unit at steer=90 (max steer ~90), i.e.
  radius ~1.1-1.2 units at any speed. CAR-LIKE. Tight turns require fwd/rev wiggle:
  fwd steer=+90 raises heading; rev steer=+90 lowers it (like backing a car).
- wall pivot does NOT work (bump blocks rotation).
- bump at dist <~0.09 on beam0.
- beam i world angle = heading + 22.5*i (deg). beam0=front, beam4=side toward which
  steer>0 turns ("left"), beam8=rear, beam12="right".
- lidar max ~1.6-1.7 (clips), -1.0 = dropout, reading noise ~0.02.
- world scale: corridors ~0.3-1.5 wide. Turning radius >> corridor width!
- WARNING: invisible barriers exist (lidar sees through them, robot cannot pass;
  bump flag sometimes 0 while pushing). Detect stall by scan not changing while
  driving. Found one blocking ~heading 183 in first-episode area after long corridor.
