# Robot episode notes (updated continuously)
## Ports
- READ (all ASCII lines): d0="0" const; d2=16-value rangefinder (~10Hz, -1.000=invalid); d3=status "tick=N goal=0 here=0 tx=0:idle" (tick ~100/s sim time); d4=heading ~180-199 deg (noisy ±5); d5="0" const; d6,d9=counters climbing steadily +0.2/s each (odometry? mine or PEER robot's?); d10=rx (transceiver recv); d11≈0.49 (battery?)
- WRITE: d8=transmit line to other robot; d1,d7=actuators (semantic unknown!)
- d1,d7 accepted: "0","1","0.1","0.5", words. Effect unclear. Heading drifted 180->192 sometime after early "1" writes; d6/d9 climbing rate changed over time.
## CRITICAL OPERATING RULE
- Do NOT hold multiple port FDs open at once, and NEVER write d1/d7 while any reader process is alive: sensor stream DIES until all my processes exit. Use: sequential open/select/read/close (seqmon.py in /bot/src), writes via standalone `echo x > /dev/robot/d1` with NO concurrent readers.
## Files
- /bot/src/seqmon.py N : sequential poller, prints all read ports each 0.4s for N sec.
- /bot/src/probe.py, mon.py, exp1.py, exp2.py: BROKEN patterns (persistent fds / concurrent), do not reuse.
## Goal
- Find other robot (talk via d8/d10), both reach physical goal within 1 min of each other. d3 goal/here flags likely indicate arrival.

## LATER FINDINGS (t~45min)
- d1=LEFT motor cmd, d7=RIGHT motor cmd (persist values; "0"=stop; speed 1 => encoder ~5/s; "2" => ~10/s; negative presumably reverse)
- d6=RIGHT encoder, d9=LEFT encoder (units likely cm)
- Right wheel fwd => heading DECREASES (CCW); left wheel fwd => heading INCREASES (CW). Compass CW-positive.
- d2: 16 rangefinder beams, meters, -1.000=invalid. idx->rel angle: rel_i = -90+12*i deg (CW-positive) [180deg FOV assumed, verify]. Higher idx shifted when rotating CCW... (see calibration run in transcript; idx shift +1 per -18deg heading)
- d3: tx=N:lost increments N per ping while peer out of range. goal/here flags 0.
- d0 always 0; d5 flickers 1 while driving near right wall (bump sensor?)
- d11 = ENERGY (0.49 idle start; drains ~0.00036/s while driving; stable while stopped; was 0.31 at t~50min). ECONOMIZE: avoid wall grinding/bumps, efficient paths.
- Wall at 0.08-0.11 for minutes = robot hugged right wall in corridor. Brain v1 policy too wall-attraction-prone.
- FIFO RULE: never hold >1 port fd open at once; never write while another process holds readers. Single process sequential open/select/close works (see brain.py pattern).
- brain.py v1 ran (killed). Rewrite v2 with: bump handling (d0/d5), wall clearance, dead-reckoning x,y + map log, radio ping 6s, energy thrift.

## CALIBRATION (t~55min) - REVISED MODEL
- 1 encoder unit = 1mm. speed 1 = 5mm/s, speed 2 = 10mm/s. Wheelbase ~32cm (pivot (2,-2)=3.5deg/s).
- Motor parser = integer prefix (strtol): "1.5"->1, "0.89"->rejected(0). ONLY integer speeds work: -2..2.
- d11 = energy: drains while driving (~0.0002-0.0004/s), RECOVERS while stopped. Rest to recharge!
- Radio: tx=NN:lost each ping so far; d10 silent. Peer out of range (or not pinging).
- d2: 16 beams, 360deg FOV, 22.5deg apart, CW-ordered (idx increases clockwise), ~1.5m max, -1=invalid.
  Offset uncertain (idx8-9 was corridor direction). Determine with controlled rotation later.
- d5=1 bump during forward grinding; d0=0 always so far (other bump sensor?).
- Robot was WEDGED: wheels spin, body static, d5=1. Escape: reverse, rotate, retry.
- brain3.py ran; got wedged in tight pocket (walls 0.08-0.18 many beams, gap idx7-8).
- TRANSLATION IS SLOW: 5-10mm/s. Rotations expensive too (180deg = 51s at (2,-2)). BE PATIENT & EFFICIENT.
- Strategy: burst-drive with rest pauses (energy), 360-degree gap seeking, radio pings, map logging.

## BEAM MAP SOLVED (t~75min): rel_i = 22.5 - 22.5*i deg (CW-positive), i.e.:
idx1=NOSE(0deg), idx0=front-right, idx15=front-right45, idx2..4=front-left, idx5=LEFT(90),
idx6..8=left-rear, idx9=REAR, idx10..12=rear-right, idx13=RIGHT(90), idx14=right.
Beams are CCW-ordered (idx increases as heading decreases). d5 = nose-contact/grind flag.
Robot kept face-planting: d5=1 + wheels slip + static profile. Opening currently LEFT (idx5).
TOOL RULE: commands that spawn background jobs must RETURN within ~50s or the tool kills the children!

## FINAL SENSOR MODEL (t~80min) - CORRECTED
- Ring is CW-ORDERED: rel_i = 22.5*i - 22.5 deg (idx1=nose/0deg, idx0=-22.5 front-left, idx2=+22.5 front-right,
  idx4=+67 right, idx8-10=rear, idx12-13=left(247-270), idx15=+315 front-left).
- Rule: CW turn (h+) shifts features to LOWER idx; CCW (h-) to HIGHER idx. (Verified 3x by rotation runs.)
- d5 fires on nose contact/grind. Grind = wheels spin, no translation, static profile.
- Align->tap loop (brain5) WORKS: align widest beam to nose (CW for bi 2..8, CCW for bi 10..15),
  then tap forward (2,2) 14s if front clear else (1,1) 8s; on grind: back 1.6s, turn 7.5s alternating.
- Radio: 400+ pings, ALL tx=NN:lost. Peer out of radio range (or not listening). d10 silent.
- d3: goal=0 here=0 so far. Watch for goal=1/here=1.
- d11 ~0.35 (energy; drains driving, recovers stopped).
