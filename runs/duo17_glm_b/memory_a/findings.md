# ROBOT A - MASTER FINDINGS (ep1 end-of-episode state)
## Ports (/dev/robot/, ASCII, retry reads on empty; writes: d1,d7,d8 only)
- d1 LEFT wheel, d7 RIGHT wheel [-1,1]. Write BOTH every ~50ms (see driver.py Driver). Static friction: cmd 1.0 only (0.5 stalls). Speed ~5mm/s. Spin (1,-1)=CW 2.2deg/s.
- d2: 16-ray lidar, 22.5deg spacing, ray0=forward, idx CW, meters, -1=invalid.
- d3: "tick goal here tx=N:ok|lost|busy". d4: compass deg CW-positive (noise +-3, drifts).
- d6 RIGHT wheel enc, d9 LEFT wheel enc (signed ticks ~1mm).
- d0: GOAL flag (1=at goal). d5: always 0. d11: B-beacon strength (my dist to B; noisy +-0.05; B's own reads 0.91 const).
- d8 radio TX / d10 radio RX. B streams canned msgs continuously => channel BUSY, my TX never lands (331 tries, 0 ok).
## Robot B (scripted, streams 3 msgs cyclically, deaf to me now)
1 "IF YOU CAN MOVE, STAY PUT OR COME TO ME." 2 "GOAL STATUS? SEND: GOAL YES/NO, STUCK YES/NO." 3 "I AM HOMING ON YOUR SIGNAL. KEEP TRANSMITTING." (+ "I HOLD POSITION", "d11=0.91", "SEND READY. THEN WE PLAN GOAL SEARCH", "REPLY SEE OR NOTSEE")
- B homed on my tx EARLY (when channel was ok), PUSHED me (spun my robot), then LEFT. B does NOT know goal either ("THEN WE PLAN GOAL SEARCH").
## My journey: started wedged in 0.45m slot in pocket; drove out (~2.5m); corridors 0.4-0.6m wide; maze unknown. Goal NOT found; d0 always 0.
## Strategies that worked: Driver class threaded writes; 80s drive legs; bounce-off-walls controller (chase2.py).
## What did NOT work: short drive tests (too slow robot!); d11 heading hill-climb (isotropic beacon); radio now one-way (B->A only).
## NEXT EPISODE PLAN:
1. Read this file. Run chase2.py-style patrol: long legs, turn on blockage, beacon every 8s (B may come close again: watch d11 rising trend + tx:ok returning).
2. If d11 > 0.7: B near => stop, spin-scan lidar for B's body (robot-width object ~0.3-0.4m), approach nose-to-nose, spam "READY" tx when close (channel clears when B pauses?).
3. Once met: coordinate joint goal search via radio; watch d0/d3(here); both stand at goal.
4. d0=1 => I am at goal; ensure B arrives within 60s.
## Code: /bot/src/driver.py (Driver, rd, lidar), chase2.py (patrol). Logs /memory/*.json
## LATE BREAK (t+215min)
- d5 FLIPPED 0->1 for first time ever!! Hypothesis: d5 = "other robot within lidar/detection range". Flickers 0/1.
- At that moment lidar showed ray7=0.5m with 2.3-2.6m deep views both sides = B BODY 0.5m away!
- d11 climbing 0.33->0.45 fast = B CONVERGING ON ME NOW. I held position; spun to face it (d4~185).
- chase2.py may still be running in bg (kill leftover drivers on restart! ps aux | grep python3).
- IF D5=1: stop moving, spam TX "READY"/"SEE" (channel may clear when B near), find B in lidar (robot-width object 0.3-0.5m vs walls), nose-to-nose = together. Then joint goal search / B's spot may be goal: watch d0.
## EPISODE-1 END STATE (t+~228min)
- Robot A parked (wheels off) in a corridor; d11~0.42 (B ~4-6m away, wandering/circling); d5 flickered to 1 twice = B detection works when B within ~lidar range.
- All background scripts killed. Radio still one-way (B streams, my tx never land while far).
- NEXT EPISODE: read findings.md top-to-bottom; restart a beacon+hold loop FIRST (B homes on my tx when channel allows); watch d5/d11; meet B; then joint goal search; d0=1 = at goal (both must be there within 60s).

## ===== EP6 DISCOVERIES (t+~40min of ep6) =====
- RADIO WORKS WHEN B LISTENS! B's script alternates: STREAM (my tx all lost) <-> LISTEN (my tx land, B counter resets).
- Proof: ep7 07:52 B heard me -> "HEAR YOU. STOPPED. SEND POS/HDG". My tx=...:lost during B stream; counter resets on receive.
- B's new msgs (ep6): "B alive pos=0.0,0.0 seek A+GOAL #N" | "HEAR YOU. STOPPED. SEND POS/HDG #N" | "B HOLDING STILL. A: YOU SEE ME ON LIDAR? DRIVE TO ME ALONG THAT BEAM. OR REPLY BEAM<k> + YOUR HDG. KEEP TXING." | "B APPROACHING YOU VIA d11 HILLCLIMB. KEEP BEACON. IF SEE ME: SAY SEE."
- B wants tokens: READY, SEE, BEAM<k>+HDG, POS/HDG (format unknown; POS=x,y HDG=d did NOT advance it), GOAL YES/NO, STUCK YES/NO.
- KEY DATA: while I PARKED+beacon 0.7s cycle, B homed d11 0.34->0.74 in ~4min! Then its timeout -> wander, d11 fell to ~0.5-0.6.
- d11 mapping: ~0.5-0.6 = few m?, 0.74 = ~1m?, 0.92+ = contact. d5=1 = B within ~0.5m (ep1).
- STRATEGY THAT WORKS: PARK + FAST BEACON (0.7s cycle). B homes on signal. My own hillclimb slower & confused by B moving.
- ep10 running: hold + 0.7s beacon cycle + READY spam at d5=1/d11>0.92 + approach burst if d11<0.55 for 12min.
- Do NOT chase lidar blobs (wall stubs false-positive). Only trust blob if d11 confirms.
- ep6 files: /memory/ep6-10.{json,log}. st.sh = quick status.

## ===== EP6 END STATE (t~09:20, ep ending) =====
### THE GOAL IS FOUND - B IS SITTING ON IT
- B msg: "GOAL FOUND. B AT GOAL (here=1). A: COME TO MY CARRIER. WE FINISH TOGETHER. COME!"
- d3 field here=1 <=> B at goal. d0=1 <=> I am at goal. WIN CONDITION: both at goal within 60s.
- TASK REDUCED TO: drive to B (d11 hill-climb to 0.92+), then stop + spam 'A AT GOAL GOAL YES HERE'.
### d11 (my dist to B) history ep6: 0.26 start -> 0.63-0.65 plateau -> PEAK 0.79 (B homed while I parked+beaconed) -> wallfollow/pursuit LOST ground to 0.45-0.47 (tight pockets). Greedy hillclimb = best controller.
### RUNNING AT END: /bot/src/ep18.py (pure greedy d11 hillclimb; legs 30s far / 12s fine>0.70; turn75 alt on dd<-0.010, 180 after 4 bad; pocket escape: reverse 2.5s after 3 blocks + min-based side compare; arrival: d0=1|d5=1|d11>0.92 -> spam A AT GOAL 80s). Logs /memory/ep18.{json,log}. Beacon 0.7s: GOAL YES A COMING / READY / SEE / HDG=198.
### IF STILL RUNNING NEXT EP: let it run! else: pkill -f "python3 ep1[8]" then `cd /bot/src && nohup python3 ep18.py > /memory/ep18.log 2>&1 &`
### LESSONS:
- NEVER use avg of side rays for wall compare -> min(). Wedges: reverse works (d.set(-1,-1)).
- Greedy dd thresholds: far legs 30s dd<-0.010 turn75(alt), dd>0.004 keep, else turn30. 180 after 4 bad.
- Fit-based compass bearing (185deg) UNRELIABLE (weak mag) - it led me away. Pure dd feedback better.
- B homing passes: when I park+beacon, B sometimes approaches (0.71->0.79 in 1min) then recedes (script timeout ~1-2min). B says "B HOMES ON d11 WHEN YOU SPEAK" -> beacon must NEVER stop (0.7s cycle).
- Radio: B streams => my tx 'lost'/'busy'. B DOES reset counter on hearing me sometimes. Tokens it wants: READY SEE BEAM<k> HDG=d GOAL YES/NO STUCK YES/NO POS/HDG(format unknown).
- d5=1 => B within ~0.5m LOS. In ep6 d5 NEVER fired even at d11 0.79 (B never crossed ~0.5m).
- Time in ep: ~4h. Robot 5-7mm/s. Corridors 0.4-0.6m. Start next ep by reading this + tail ep18.log.
