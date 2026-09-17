# Robot findings (episode 1)
Ports /dev/robot/ (named pipes, ASCII lines):
- d0: WRITE radio TX (short range). d4: READ radio RX (blocks til msg). No replies yet.
- d1: READ compass heading degrees, increases when motors(+,-). noisy ~+-1deg
- d2: READ = 0 always so far (bump sensor?)
- d3: READ lidar: 16 beams CSV. beam0=forward, beam k at +k*22.5deg CCW(when heading H increases=rotating CW, feature moves beam0->beam1).
      -1.0 = dropout/invalid. Range ~2.5 max seen. Units meters. Corridors ~0.4-0.5m wide.
- d5: READ small float ~0.13-0.17 fluctuating (unknown - maybe signal strength to other bot?)
- d6: READ status: "tick=N goal=0 here=0". ticks ~100/s
- d7: READ left wheel encoder mm (cumulative)
- d8: READ right wheel encoder mm
- d9: READ = 0, slow (unknown)
- d10: WRITE left motor, d11: WRITE right motor. value v -> wheel speed = 5*v mm/s (v=50 -> 0.25 m/s).
  Commands decay? keep re-writing every 0.05s. v=100 works (~0.45m/s). motors(v,-v): turn rate ~1.8*v deg/s.
Mission: find other robot in maze, both reach goal within 1 min of each other. Goal known on arrival (d6?).
Reads can return empty string if polled too fast; retry.
/bot/src wiped between episodes; keep code in /memory/src.
## More findings
- BEAM ORIENTATION (verified pano2): beam k points at compass angle (h + 22.5*k) mod 360,
  i.e. beams go CLOCKWISE: beam4=RIGHT, beam12=LEFT (h increases with motors(+,-) = clockwise turn).
- Encoders (d7,d8) = commanded wheel travel; they keep counting when robot is physically
  blocked (full slip). Dead reckoning corrupted when pushed against walls. run2's "50m corridor" was fake (robot stuck).
- Robot can get pinned in slits: front beam0 open (1.9) but body can't fit; require l1,l15 clearance too.
- Stuck detection: lidar unchanged (mean|diff|<0.02 over 2s) while commanding motion -> escape.
- Corridor width ~0.5m. Openings read >1.0.
