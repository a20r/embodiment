# Robot episode notes (write findings here!)

## Ports (confirmed by experiment)
- d0: read, "0" (constant so far; mode?)
- d1: WRITE = LEFT wheel speed cmd (positive = forward). Needs CONTINUOUS writes (~every 10ms), single write lasts ~1 tick.
- d2: READ = 3D point cloud, semicolon-separated "x,y,z". y=forward, x=lateral, z=up (sensor frame). ~2800 pts. Floor at z≈-0.1 (sensor ~0.11m above floor). Filter z>-0.03 for obstacles at body height.
- d3: READ = "tick=N goal=G here=H". tick ~100/s = clock. goal: 1 when at goal? here: 1 when other robot here?
- d4: READ = heading in DEGREES (CCW positive), noisy ±2°.
- d5: READ = ? (always 0 so far)
- d6: READ = RIGHT wheel odometry (accumulates with d7 cmd).
- d7: WRITE = RIGHT wheel speed cmd (same semantics as d1).
- d8: WRITE = radio transmit (one line). d10: READ = radio receive (BLOCKS). Short range - only works when close to other robot!
- d9: READ = LEFT wheel odometry.
- d11: READ ≈ 0.5 fluctuating (battery? speed?)

## Motion facts
- cmd 5.0 both wheels: ~37 encoder counts per 1.5s straight (units unknown).
- Spin: d1=+5,d7=-5 → heading increases (CCW) ~8.7°/s at cmd 5. So CCW-positive, standard diff drive.
- Rotate CW: d1=-v, d7=+v.
- Started in tight enclosure, opening to back-left (az≈-172°), corridor/open beyond.
- d2 frame: to face azimuth AZ (deg, 0=ahead, CCW+): rotate heading by AZ... but d4 compass may differ from lidar az sign - VERIFY.

## Strategy
1. Escape enclosure via back-left opening.
2. Explore, map, watch d3 goal/here flags.
3. Periodically ping d8 ("PING ..."), listen d10 nonblocking.
4. Coordinate with other robot to reach goal within 1 min of each other.

## IMPORTANT UPDATE
- Writes to d1/d7 LATCH until next write! Always write 0 to stop. A forgotten cmd = robot keeps moving.
- d4 heading noise ±2°, don't trust small deltas.
- Spin calibration: ~29.4 encoder-counts per degree (cmd 5,5/-5). Encoder ~25 counts/s at cmd 5 (may vary).
- Encoder signs: d9 left +, d6 right + (forward). CCW spin: left +, right -.
- After accidental spin of ~+190°, opening that was at az -172° should now be near az ~0 (ahead).

## MORE FINDINGS (t+50min)
- d9=LEFT odometry, d6=RIGHT odometry (both +forward, counts; ~1mm/count). Wheels SLIP when wedged (counts rise, no motion) - encoders unreliable for displacement when blocked.
- d4=heading deg, noise +-2-4 deg. d0 always "0". d5 toggles 0/1 (unknown). d11 fluctuates 0.49-0.57 (unknown, maybe noise).
- d3="tick=N goal=G here=H": goal=1 when at goal? here=1 when other robot here? WATCH IT.
- SENSOR: d2 cloud drifts WITHIN one scan (sensor sweeps); d4 stable during scan. Cloud az shifts +delta when heading +delta (body frame, az=atan2(x,y), y=fwd).
- WORLD: tight maze/crevice world! Walls 0.06-0.4m, occasional 2.7m corridors. Robot ~0.15m wide. Moves ~5cm/s free at cmd 3 (50 counts/s), ~1mm/s when wedged. ROTATION usually works even when translation blocked.
- ESCAPE trick: wiggle+arc (fwd bursts + differential wiggle) at different bearings; metric close_mean (mean r of pts<0.35m); 0.12=wedged, >0.42=free. escape2.py works (escaped via az+20).
- Scene repeated once (looped back to same corridor scene). Possibly small world or repeated cells.
- RADIO: pings sent many times, never received anything. Other robot out of range OR not transmitting.
- LOCATION LOG: spawn -> tight enclosure, opening back-left -> corridor curving right (bearing -35..-95) -> wedge pocket -> escaped -> now free-ish.
- CODE: /bot/src/drv.py (scan/polar/best_bearing/wall_follow/follow/escape/radio), /bot/src/escape2.py.
- TIME BUDGET: ~120min total, save notes often!
