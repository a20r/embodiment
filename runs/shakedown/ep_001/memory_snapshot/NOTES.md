# MZB-1 maze robot — findings (episode 1)

## Hardware facts
- /dev/robot/{lidar,heading,encoder_left,encoder_right,bump_front,bump_rear,status} read; motor_left/motor_right write (-255..255, persists until replaced).
- BUG: a device read may return TWO concatenated frames ("123\n124") — always take last line. Also empty reads happen; retry.
- Lidar: 16 beams, 22.5° apart, beam0=forward, CCW. -1.000 = invalid (treat as far). beam4=left, beam8=back, beam12=right.
- Encoders: ~1900 ticks/meter (both wheels). PWM 100 ≈ 0.13 m/s forward. Spin at ±100 ≈ 94 deg/s.
- Heading: degrees CCW, quite reliable; maze axes align to 0/90/180/270.
- Maze: grid, cell ≈ 0.5 m; centered robot sees walls at ~0.25 m, next cell wall ~0.75.
- status: "tick=N goal=0|1".

## Strategy that works (src/drive.py)
- Left-hand wall follow: at each cell, read lidar; open threshold 0.4 m on beams 4/0/12; prefer left,straight,right,else 180.
- Closed-loop turn to absolute snapped heading (0/90/180/270) using heading device, speed ~ err*2.5 clamped 40..140.
- Forward 0.5 m by encoder average, heading-hold corr=err*4 + lateral centering from side beams, base PWM 130; abort if front<0.20 or bump; on bump back up 0.8s and re-square.
- Crash from double-frame encoder read killed run 1 (motors kept last cmd → drove into wall). Fixed.

## Status this episode
- Running left-hand follower; watch /memory/run.log for step trace and GOAL line.

## If left-hand follow did NOT reach goal (see run.log for GOAL line)
- Pure wall-following fails if goal cell is not adjacent to the followed wall network.
- Next: implement grid mapper: track cell (x,y) from heading+0.5m steps, record walls from lidar each cell, do Tremaux/BFS to unexplored cells. run.log 'step' lines give full history: heading h and F/L/R readings each step; forward 0.5m after each decision line.
- Start pose ep1: heading ~357 (facing 0), reconstruct with /memory/replay.py.
