# Major Discoveries!

## Position Tracking:
- **d9**: X coordinate (increases with forward movement, ~1100-1200 range)
- **d11**: Y coordinate (oscillates around 0.5, might be altitude or perpendicular axis)
- **d4**: Bearing/heading in degrees (0-360)

## Movement Test:
- Started at: d9=1127, d11=0.486
- After 10s forward: d9=1178, d11~0.5 (varies)
- Rate: ~5 units/second in d9

## Still Unknown:
- What are the goal coordinates?
- Where is the other robot?
- What triggers goal=1?

## Next Steps:
1. Try broadcasting position format: "POS:d9=X,d11=Y"
2. Try different positions to find goal
3. Listen for other robot's position
4. Navigate to meeting point
