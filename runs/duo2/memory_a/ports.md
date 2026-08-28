# Robot port map (confirmed by experiment)
- d0: read. 16-beam lidar, CSV floats. Beam spacing 22.5°. Beams indexed CCW
  (same sign as heading d10). Feature at world bearing B appears at beam
  ≈ (B - heading)/22.5 (beam0 = front, assumed). -1.000 = invalid (unknown if far or close).
  Max seen ~2.3.
- d1: write. Linear velocity cmd, persists until changed. d1=1 ≈ 0.1 m/s (est).
- d2: read. "tick=N goal=0/1". Goal flag! ticks ~100/s.
- d3: write. Transceiver TX (line).
- d5: read. Transceiver RX. Returns empty line if nothing. Nothing received yet.
- d4, d9: read. Always 0 so far. Maybe bumpers?
- d6: write. Angular velocity cmd, persists. d6=2 → ~1.8°/s (slow!). deg/s ≈ 0.9*d6?
- d7: read. Odometer counter (increases when moving; ~5/s at d1=1).
- d8: read. Rotation counter (wheel-based, NOT degrees; ~5x deg). Unbounded.
- d10: read. Compass heading in degrees [0,360), noise ±1.5°. CONFIRMED via lidar.
# Env: small room, walls ~0.2-2.3m. Goal unknown; check d2 goal flag.
# Motion calibration (important!)
- d1 linear cmd: effective speed ~0.45 units/s for cmd 20-60. cmd>=80 SLIPS (slower!).
  cmd<=10 barely moves (static friction). Use 20-40.
- d6 angular: deg/s ≈ 0.9*cmd, works up to ~100. cmd 200 = ignored/no motion.
- d7/d8 encoders count commanded rotation (5.24*cmd per s), NOT ground truth (slip!).
  Dead-reckon with time*0.45 instead, or lidar.
- Robot gets WEDGED near walls: forward cmd then causes rotation (one wheel gripping).
  Detect: heading drift while w=0. Escape: reverse ~1.2s at -25.
- Environment: maze-like, corridor half-widths ~0.15-0.4, lidar max ~2.3-2.5, -1=dropout(rare).
- Robot effective radius: wall contact when front lidar ~0.1-0.16.
- Beam i world bearing = heading(d10) + 22.5*i, beam0=front, CCW. VERIFIED.
- Nothing ever received on d5 yet (tx "hello" on d3 periodically).
- src code in /bot/src (robot.py, explore.py) - COPY TO /memory each run!
# CRITICAL CORRECTION (supersedes above!)
- d1 and d6 are WHEEL SPEED commands (differential drive), NOT linear/angular!
  d1=wheel that alone turns heading DOWN (right wheel in d10 convention... call it W_A)
  d6=wheel that alone turns heading UP.
  STRAIGHT: d1=d6=v. Wheel speed ≈ 0.0124 u/s per cmd unit (both@40 → ~0.5 u/s).
  TURN in place: d1=-w, d6=+w → deg/s ≈ 1.8*w (d10 increases). Verified.
  Keep |wheel cmd| <= 50; higher slips badly.
- d7 = encoder for d1 wheel, d8 = encoder for d6 wheel (5.24*cmd/s, counts commanded).
- All earlier weird "wedge/pivot" behavior was just wrong actuator model.
