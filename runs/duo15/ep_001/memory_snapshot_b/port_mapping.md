# Robot Control Port Mapping

## Confirmed:
- **d0**: Appears to be output (goal indicator?). Read as 0
- **d1**: Input - forward/movement command (write to move)
- **d2**: Output - Sensor data (16 float values, like LIDAR)
- **d3**: Output - Status string: `tick=X goal=Y here=Z` 
  - tick: incrementing counter
  - goal: 0 (not at goal), possibly 1 when goal is reached
  - here: Unknown (always 0 so far)
- **d4**: Output - Bearing/heading in degrees (float, ~60-190 range)
- **d5**: Input - Unknown purpose (writes don't seem to cause observable changes)
- **d6**: Input - Turn/heading control (write degree value)
  - Note: Doesn't seem to directly control bearing
- **d7**: Input - Unknown (writes don't cause obvious changes)
- **d8**: Input - Transmit to other robot (write messages here)
- **d9**: Output - Changes with movement (was 0-138 range)
- **d10**: Output - Receive from other robot (no messages received yet)
- **d11**: Output - Small float (~0.5)

## Communication Status:
- Sent multiple messages to d8
- d10 has NOT received any responses
- Other robot may not be within range, or may not be responding

## Next Ideas:
1. Try moving towards different directions to find other robot
2. Use sensor data (d2) to navigate toward detecting the other robot
3. Check if goal location might be indicated by sensor readings
4. Try different message formats or protocols
