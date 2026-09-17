# Robot exploration notes (episode 1)
## Port map (confirmed by experiment)
- d0: read, always 0 so far. Unknown (bumper? flag?)
- d1: WRITE. Motor command (left wheel?). Value persists. 1 unit cmd -> ~5 ticks/s on d9.
- d2: READ. 3D lidar point stream "x,y,z;x,y,z;..." ~2800-3400 pts/read. HUGE line (~55KB).
- d3: READ. "tick=N goal=0 here=0". tick ~ sim time (~40/s). goal/here flags.
- d4: READ. heading in DEGREES (changes when only one wheel driven; spin changes it). Noise +/-3 deg. Updates ~40Hz.
- d5: read, 0 so far. d6: read, right wheel odometer (signed ticks, correlates d7). d9: left wheel odometer (signed ticks, correlates d1).
- d7: WRITE. Motor (right wheel?). d8: WRITE = radio TX. d10: READ = radio RX (silent so far). d11: read ~0.5 (0.49-0.56) unknown!
## Behaviors
- spin (d1=6,d7=-6): d4 changes ~11 deg/s. d1=left(+ = CCW?)... d1=5 alone -> heading INCREASED (arc left). d7=5 -> heading decreased.
- DRIVE TEST: reverse d1=d7=-30 2s: odom -300 ticks but lidar cloud bbox UNCHANGED, heading unchanged. Either world-frame lidar or we cannot translate (blocked?).
- At spawn: clean raster cloud: object 0.2-0.24m ahead, ~12cm wide (z -0.06..+0.06), 26cm tall (y rows 0..0.258). Possibly THE OTHER ROBOT right in front at spawn!
- After rotations, that raster object no longer visible in same form; cloud scattered. If robot-frame: object should be at bearing 180-heading_delta.
- d11 changed 0.524->0.563 during reverse drive. Noise band normally +/-0.02.
- Radio: sent 'hello from A' + periodic PINGs, NO response yet. rx.log empty.
## Open questions
1. Can we translate at all? (world-frame lidar vs blocked)
2. What is d11? (distance to other robot? goal? battery?)
3. What are d0, d5?
4. Radio protocol with other robot?
5. Where is the goal? d3 goal flag; 'here' flag?
