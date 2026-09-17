# READ ME FIRST (distilled from ep1+ep2, ~2h of data). Details in notes.md + logs.

## PORTS (confirmed)
d0=goal-signal (flickers 1 when near goal; was solid 1 for 8min inside pen approach; silent otherwise)
d1=RIGHT wheel cmd, d7=LEFT (int, latch; *** MUST WRITE WITH TRAILING \n *** else ignored!)
d2=3D lidar x,y,z;... self-arc: dense contiguous returns r<0.25 (radius VARIES 0.06-0.22 over time!) -> filter r<0.25, verify arc is self (moves with robot)
d3='tick=N goal=0 here=0' tick=100Hz simtime; goal/here NEVER 1 in 2h (goal flag maybe = other robot at goal?)
d4=heading deg CCW +-3 noise | d6=LEFT enc mm | d9=RIGHT enc mm | d11=battery (0.50 start, ~0.13/h idle, ~0.35/h driving)
d5=1 => OTHER ROBOT NEAR (medium range, can persist minutes while driving; NOT visible in lidar when on). d5=0 otherwise.
d8=radio TX | d10=radio RX (0.15-0.3s timeout reads) | pipes; use O_NONBLOCK writes, timeout-cat reads.

## FACTS
- World: big maze, corridors ~0.4-1m+ wide, circular PEN (R~0.85m, one opening) ~25m W of current end area.
- PEN PROBABLY NOT GOAL (d3 flags never fired inside). d0=1 happened ONLY while approaching/inside pen 04:17-04:25 (ep1). Cause unknown.
- Radio: ZERO RX in 2h despite constant TX + aggressive pause-ping bursts when d5=1. Maybe other agent never TXs, or radio needs something else.
- Other robot: alive, moves, once approached us (ep1), d5=1 tracks it being near. It may FOLLOW us.
- Robot: track 0.32m, cmd 25 ~ 0.13-0.2 m/s, spin 25/-25 ~ 30deg/s. Encoders can jump on slip. turn_to on d4 works.

## NEXT EP PLAN
1. Sample d0 x200 (if any 1s: STOP, gradient-ascend d0rate like hunt.py - it worked in ep1!).
2. Motor smoke test WITH '\n'. Then relaunch explore.py (it self-logs to /memory/epN.log, self-stops, TX radio, chases nothing yet).
3. Follow d5: when d5=1, pause 30s-cycle ping bursts + listen. Consider STOPPING entirely 60s when d5=1 - ep1 robot approached us when we sat still.
4. Watch battery: below 0.15 -> stop wasting, hold position & radio.
5. If goal found (d0=1 sustained or here=1): STAY, TX 'A GOAL x y H' every 1s forever, and if other robot near (d5=1) try to lead it in.
6. pkill: run in its OWN bash call; pattern must not appear anywhere else in the command (killed my shell twice!).
