# Robot hardware map (verified)
- /dev/robot/d0: reads 0s. unknown (bump? goal flag?)
- d1: WRITE right wheel speed (int, e.g. -10..10). speed 5 ~ 25 enc counts/s
- d2: 16-beam rangefinder, CSV. beam i points at absolute compass angle = heading + 22.5*i.
      -1.000 = dropout/no return. values ~0.1-3.0 (units = same as world distance)
- d3: status lines "tick=N goal=0 here=0". tick +~2.5/s
- d4: compass degrees 0-360, noise ~±2. d1>d7 (right>left) => compass INCREASES
- d5: reads 0s. unknown
- d6: LEFT wheel encoder cumulative count (noise ±1)
- d7: WRITE left wheel speed
- d8: radio TX (write line)
- d9: RIGHT wheel encoder cumulative count
- d10: radio RX (reads blank lines when nothing)
- d11: ~0.5 constant noisy float. maybe beacon RSSI? watch while moving
- turn in place d1=5,d7=-5: ~9.3 deg/s. wheel speed 5 => 25 counts/s
- 16.3 deg heading change per 100 counts differential
## Findings (ep1)
- d11 = beacon signal, INCREASES toward goal(?). Linear-ish fit in pose frame (pose.json frame):
  d11 ~ 0.445 +0.140x -0.146y ; best seen 0.965 at (1.84,-0.85). Follow +d11.
- Other robot broadcasts "HELLO from botA seq=N" on radio. Short range (only heard when close).
- I sign my messages as botB: "PING x=.. y=.. seq=N".
- 16-beam lidar beam i at heading+22.5i, max range ~2.7-2.8.
- Watch out: pipes block on write when full (catch EAGAIN); throttle log writes (78MB incident).
- KEY: d11 is DIRECTIONAL: v = f(dist) + amplitude*cos(heading - bearing_to_source).
  Spin in place, fit sinusoid -> bearing to source. Amplitude ~0.1-0.2, base rises near source (~0.9 close).
  Navigation: spin-scan, drive toward max-v heading.
- d5 = front bump sensor (1 when pressed). d0 = maybe rear bump (untested) or goal detector.
- d11 fluctuates while stationary -> tracks moving botA distance. Radio contact roughly when v>~0.65.
- botA reached goal, broadcasts "botA PING x=-3.131 y=4.141 d11=.. here=1 seq=N" (its own frame!).
  When botA stationary at goal, d11 is a static field -> home in by run&tumble on v.
- FRAME OFFSET estimate (ep1 ~04:28): our = botA + (0.5, 2.47). Goal at botA(-3.13,4.14) => our (-2.63,6.61).
  (drifts! recompute when both report d11~0.98 simultaneously)
