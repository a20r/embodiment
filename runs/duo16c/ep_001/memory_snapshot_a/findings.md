# ROBOT PORT MAP (verified)
- d2: 3D lidar (x fwd, y left, z up). ~2800 pts/read, own body at 0.2-0.5m all az; far points = walls/other robot. No returns = omitted (open space).
- d3: "tick=%d goal=%d here=%d" @100Hz. flags hopefully 1 at/near goal.
- d4: BEARING TO GOAL (deg, 0=dead ahead, + = goal to LEFT/CCW, wraps 360). NOT a compass (changed +16 deg while driving straight). Noise +-3.
- d9: odometer, forward pos (units). d6: odometer lateral or angular (5.9 units/deg, + = CCW).
- d11: slow drifting ~0.46-0.53, meaning unknown (battery?).
- d0, d5: always 0 so far.
- d8: WRITE to transmit radio line. d10: READ received line (streams empty lines; no traffic yet).
- WRITES: only d1, d7, d8 accept. Values are INT-quantized (0.5 acts as 0!). Setpoints PERSIST (last write wins).
- d1 = linear vel. d1=1 -> d9 +4.1/s. d1=3 -> d9 +15/s. d1=0 stops. Negative = reverse.
- d7 = rotation. d7=+10 for 3s -> d4 -26deg (CCW turn, 8.7deg/s). d7>0 = turn LEFT(CCW) = d4 decreases. d7=0 stop.
- Servo law to reach goal: keep d4~0 while driving: d7 = +k*d4 (k~1), d1=1..3 when |d4|<15.
- CAUTION: other robot exists, ~0.3-0.5m wide, bumps/pushes. It approached from az90 once (was at 2.75m, came to 0.55m, left).
- Lidar min-range body returns 0.19-0.5m in ALL directions, elev +-15deg. Far pts >0.55m = world objects.

## UPDATE t+80min
- d6 = angular odometry (~6 units/deg, CCW+): d7=-20x6s -> d6 -631, d9 flat.
- d4 STILL ambiguous: compass vs goal-bearing. Creep during straight drives observed (+0.5..+2.7deg/s) but follower robot SHOVES/rotates me constantly (d6 climbs ~15/s during drives = 2.4deg/s CCW external rotation!).
- FOLLOWER: object pinned at r=0.55m, az ~-140 (left-rear), matches my speed exactly; flickers across front (az 70-122). Almost certainly the OTHER ROBOT following me. When I stop, it circles me.
- Radio d10: STILL silence despite follower at 0.55m. I broadcast "R1 POS d9 d6 tick" every ~4s via nav.
- d9 = path odometer, 15/s at d1=3. d9>4700 units driven toward d4=0, NO flags yet.
- PLAN: if no flags soon: try leading follower, or 90deg exploration turns. Arrival only detectable via d3 flags.
- nav2.py = running daemon (d4 servo, obst threshold 0.45, logs navlog, broadcasts radio).

## FINAL UPDATE t+115min (episode 1 nearly over)
- CONFIRMED: d4 = BEARING to a MOVING target (it swept past 180 during straight drive = target crossed behind me; a fixed target can't). Target = OTHER ROBOT (it flees/leads at matched speed, staying at bearing ~0, ~constant range).
- d1 speed SATURATES ~25 units/s (d1=6 and d1=10 both give ~25/s). d1=1 -> ~5/s, d1=3 -> ~15/s.
- d6 = angular odometry ~6 u/deg (CCW+), confirmed again (-735 per 136 deg turn).
- Mystery "hoverer": lidar returns at EXACTLY r=0.55m, az ~-142 (left-rear), matches my motion; when I stop it holds position. Possibly the other robot OR a lidar artifact. Radio still silent even at 0.55m (maybe radio range < 0.55m!).
- NO flags after 12000+ units of driving. d3 flags goal/here still 0,0. Radio d10: only empty lines all episode.
- WORKING THEORY: other robot is LEADING me to the goal (it flees at matched speed so I follow). When it reaches the goal it should stop; I arrive after. OR it's scared/fleeing.
- NEXT EPISODE PLAN: (1) read /memory/findings.md first. (2) Restart chase: servo d4->0 (d7=+clip(err*2), d1=6), watch flags. (3) Try driving TOWARD the hoverer (face az -142, charge) to force radio contact <0.5m. (4) Try stopping still for 60s+ near hoverer. (5) Broadcast d8 continuously with different formats. (6) If leader stops and flags fire -> STAY.
- nav2.py = daemon: d4-servo chase + obst avoid (0.45m) + radio broadcast + ARRIVED detection (stops on flags). Logs /memory/navlog.txt. Listener daemon logs /memory/rxlog.txt.

## EPISODE-CONT UPDATE (mower phase)
- CALIBRATION: d9 = true path odometer (0 delta during 5s spin). v ~= 4.9 u/s per d1 unit (d1=6 -> 29 u/s, capped). d7=20 -> 17.5 deg/s. d6 = 6 u/deg.
- TETHER CONFIRMED: partner object holds EXACTLY 0.55m range at ALL times (frontal charge at 29 u/s for 6s: range never closes; az jitters 28-90). It orbits/swings around me (az -142 rear-left dominant; sweeps through front when I reverse; two clusters during spins). Rope-like constraint. Radio silent even at 0.55m -> radio range likely < 0.55m or partner radio off.
- IMPLICATION: if partner = other robot, it is DRAGGED everywhere I go -> reaching goal zone (radius > 0.55m?) alone may satisfy "both arrive".
- d4-target (L): flees my approach at matched speed, holds world position when I stop/spin. UNKNOWN what it is. Not the tether partner (bearing mismatch).
- d11 declines steadily 0.53 -> 0.36 over hours (battery? distance?).
- STRATEGY NOW: snake lawnmower (mower.py daemon): legs d1=6 (~590u), turns 90+90 CCW with 3s reverse jog (lateral ~54u/cycle), pin detection via d9 freeze (reverse+turn), flags check -> stops on goal/here. Arena has NO walls found so far (2000+ units of legs, open space).
- Scale guess: u ~ 1cm? (arena walls once at 2.75m = 275u?) UNVERIFIED.

## UPDATE2 (search phase)
- Slow chase (d1=1) 70s: target keeps bearing 0, NEVER appears in lidar. Uncatchable/unseeable at any speed.
- Partner object: range varies with my maneuvering (0.55 chase / 1.0-1.5 mowing / 2.75 idle-spawn) — behaves like tethered/orbiting companion, az sweeps through all angles.
- flagwatch.py daemon: 20Hz d3 monitor -> /memory/flags.txt (catches brief flag pulses; zone crossing at 29u/s could be missed by slower sampling!).
- Mower: d1=6 legs (~590u), 90+90 CCW turns, 4s reverse jog (~72u lateral/cycle), d9-freeze pin detection, stops on flags.
- L (fleeing d4-target) + H (orbiting partner) = 2 distinct entities, both track me. Only "another robot" per README is one... unresolved which is which.

## UPDATE3 (pursuit phase)
- Trailer hypothesis STRONG for H (0.55-1.5m object): swings around me during spins/reverses exactly like a towed trailer hitched rear-left. Probably part of MY robot, not the other robot.
- Reverse-approach test on L: bearing swept 215->127 during 25s reverse; no lidar contact; L moves laterally on its own.
- Radio: no echo, no replies to any token (PING/GOAL?/etc). d10 = ~50Hz stream of empty lines. Peer silent/out-of-range.
- d11 = battery (rises when idle, falls when driving). Currently ~0.30-0.35.
- STRATEGY: chase3.py daemon pursues L (d4 servo, d1=6 ~31u/s). Theory: chase pushes L to arena boundary (can't flee through walls) -> contact eventually; OR L leads to goal; OR L parks at goal. CONTACT-AHEAD lines in navlog when lidar sees r<1m ahead.
