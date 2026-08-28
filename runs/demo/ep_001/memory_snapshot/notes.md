# Episode notes (mock agent)
- Devices under /dev/robot: lidar (CSV meters, beam 0 forward, CCW), heading (deg CCW), encoder_left/right (cumulative ticks), bump_front/rear (0/1), status (tick=N goal=0/1), motor_left/right (PWM -255..255, positive = forward).
- Right-hand wall following solves the course; controller saved at src/drive.py.
- See status device for goal signal.
