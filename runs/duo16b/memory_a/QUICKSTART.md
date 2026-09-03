=== READ THIS FIRST (Episode 3 startup, after Episode 2) ===
1. Restart daemons: cd /bot/src; nohup python3 listener.py &  nohup python3 pinger.py &
2. Read /memory/NOTES.md sections "EPISODE 2". Old E1 notes below still valid for ports/motors.
3. PORT QUICK REF: d1/d7 motors (L/R, newline!, +fwd), d6/d9 enc, d4 compass CCW+,
   d2 lidar "r,e,az;" (e>0.35=tall, e<-0.9=deep/cliff, az ±0.25 cone), d3="tick=N goal=G here=H"
   (THE goal flag - poll at 25Hz), d5=contact latch, d11=distance-to-COMPANION (+-0.02 noise),
   d8 TX / d10 RX radio.
4. COMPANION: d11 ~0.25 when touching. It APPROACHES if we freeze, retreats if we move.
   We were IN CONTACT at bearing ~200 from us at ep2 end. d5=1 usually means touching it.
5. GOAL: never flagged yet (716k ticks). Read stepscan2.json (raw 360 scan at last spot).
   Candidate: FAR-tall anomaly headings ~210-293 (r~1.0-1.3). Also 268-beacon from E1.
   If nothing: descend deep cliffs (S ~103-146, N ~293-13 at last spot).
6. Radio almost certainly useless (0 replies in 100+ min incl. at 0.24 range). Still keep listener.
7. Move-stop rhythm keeps companion with us. STOP = it comes to ~0.26.
