# Robot notes (episode 1)
- /bot/README: find other robot; both must reach goal within 1 min of each other.
- Ports /dev/robot/:
  - d0 W radio tx (short range), d4 R radio rx (blank line = nothing)
  - d1 R compass heading deg (float)
  - d2 R unknown (0 so far; bump?)
  - d3 R lidar: 16 rays CSV, max ~3.0, -1.0 = no return(>3m)
  - d5 R slow scalar (~0.13-0.15 so far; beacon strength? poll)
  - d6 R status: "tick=N goal=0 here=0"
  - d7,d8 R wheel encoders (ticks). ~5.1 ticks/s per unit cmd, ~0.0013 m/tick
  - d9 R unknown 0
  - d10,d11 W wheel speed cmds (persist until changed)
- Motion: drive(v,v) moves in direction of lidar ray 12 ("front"=ray12).
  drive(+w,-w) (d10=+,d11=-) increases d1 heading ~1.8deg/s per unit w, and
  rotates front toward HIGHER ray indices. world angle of ray i ~ d1 + (i-12)*22.5 (+const).
- Speeds linear at least to 50 (0.33 m/s). ray values: index i angle=(i-12)*22.5 from front.
- read() may return empty; retry. Wheels unbalanced: heading-hold needed (drifts ~ -10deg/m).
- explorer: /bot/src/explore.py (step-based right-hand pref, dead-reckoned pose, logs explore.log)
- d9 flickers 0/1 sometimes; d5 wanders 0.13-0.19. Meaning unknown yet.
