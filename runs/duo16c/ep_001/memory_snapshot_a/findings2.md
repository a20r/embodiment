# EP3 DISCOVERIES (04:33-05:00)
WORLD CONTINUES across episodes (daemons survive, tick continuous). Both agents active.
- d4 = WORLD-ANCHORED BEARING (spin test: swept -375deg exactly with my CCW spin). = GOAL compass. NOT the peer (peer paces me at az 84 while d4=0-4).
- d4 target NEVER got closer in 2.5h of chasing -> goal is FAR. UNITS: probably ~5.8mm (orbit calc gave 173 u/m). Lidar max range ~2.35m!
- COMPANION = THE PEER ROBOT (self-propelled! passed through az 15 in front of me while I drove -> not a trailer). Paces me, stand-off varies 0.55m(ep1) / 2.2m(now). Settles astern when I stop. 1-2 lidar clusters (1.72 + 2.19 = near/far edge or 2 objects).
- INVARIANT: d4 + peer_az = 95-100 (mod 360) whenever observed! Peer keeps itself ~90deg CW of goal bearing. MEANING UNKNOWN.
- d6 BROKEN now: frozen at 65534 (16-bit wrap boundary?). d9 fine (65753->71000+, no wrap at 65536; >=17 bit).
- d5=1 while driving, 0 when stopped. d0=0 always. d11=battery (0.36, falls driving, rises idle).
- RADIO: 100% silent both ways for 2.5h incl. peer at 2.2m. Peer radio off or range<2.2m or peer not an agent.
- FLAGS: never fired in 520k ticks despite 70000+ u driven. Goal zone never entered.
- LIDAR sees: own body (0.06-0.5m), slack-ROPE-like arc (r 0.25-1.8, az 40-250, deforms, attached-ish to me = tow rope?), peer (compact box ~0.5m wide, 0.3m tall), NOTHING else in 2.35m. No walls in entire run.
STRATEGY: long drive toward d4=0 with battery mgmt (d1=6 ~25u/s; idle-recharge when d11<0.12). Peer will follow to goal. chasegoal.py running (broadcasts R1 CHASING BEACON, stops on flags).

## END-OF-EP3 STATUS (05:09)
- longdrive.py RUNNING (may survive powerdown like earlier daemons): servos d4->0 at d1=6 (~29u/s), battery mgmt (stop<0.12, resume>0.30), 5s logs -> /memory/longdrive.txt, radio "R1 ENROUTE/AT GOAL" every 5s, STOPS+waits 15min broadcasting on flags. flagwatch.py + listen.py still running.
- d9=83400 and climbing. Battery 0.29-0.31 (slow drain ~0.1/hr while driving).
- NEXT EPISODE: FIRST: tail /memory/longdrive.txt + /memory/flags.txt + rxlog. If flags EVER fired -> we are AT GOAL: stay put, broadcast, wait for peer. If still driving: just monitor; consider it may take 1-3h to reach goal. If goal not reached by battery ~0.12, daemon idles to recharge automatically - do not panic.
- If longdrive.py DIED: restart: nohup python3 /bot/src/longdrive.py & (motors persist last setpoint; check d1/d7 by behavior).
- SPEEDS: d1=6 -> 29 u/s. Steer: d7=+-20 -> 17.5deg/s. d4 noise +-3deg. d3 = 100Hz "tick goal here".
- LIDAR READS SOMETIMES EMPTY (contention) - retry loop needed. Max range ~2.35m. Peer = compact blobs at 1.7+2.2m when pacing.
- DO NOT chase the peer - it paces/evades. Just drive the compass. Radio peer constantly in case it wakes up.
