# HANDOFF (episode 1 end-of-context summary) — READ ME FIRST
## MISSION
Find the other robot (talk via d8/d10 radio) and both reach the physical goal within 1 min of each other.
d3 status line: "tick=N goal=G here=H tx=K:STATE" — goal/here flags may fire near/at goal. tx state "lost"=peer out of range.
## ROBOT API (VERIFIED)
- /dev/robot/d1=LEFT wheel speed cmd, d7=RIGHT wheel speed cmd. INTEGER values only (strtol parse; "0.5"/"1.5"->1; "0.89" rejected). Command PERSISTS. 1=5mm/s, 2=10mm/s, negative=reverse. Encoders: d9=LEFT(mm, cumulative), d6=RIGHT.
- d2 = 16-beam rangefinder, meters, -1.000=invalid/out-of-range, ~10Hz. CW-ORDERED: rel_i = 22.5*i-22.5 deg (idx1=nose, idx0=-22.5 front-left, idx2=+22.5 front-right, idx4=+67 right, idx8-10=rear, idx12-13=left, idx15=315 front-left). Rule: CW turn (heading+) shifts features to LOWER idx; CCW to HIGHER.
- d4 = compass heading deg (CW-positive, noisy ±2-3). d3 = status. d0/d5 = bump/contact flags (d5 fires nose-grind; wheels then SLIP: encoder ticks w/o motion — STATIC profile + d5=1 = wedged).
- d11 = energy-ish 0..1: drains while driving (~0.0002/s at (1,1)), RECOVERS when stopped. Currently ~0.55.
- d8 = write line to transmit (peer radio). d10 = read received line. Radio range seems SHORT; 1000+ pings all "lost" so far.
## CRITICAL TOOL RULES
- NEVER hold 2+ port FDs open simultaneously; never write d1/d7 from a 2nd process while a reader process lives. One process, sequential open/select/close (see brain5.py rd/wr).
- Bash tool: any command >60s gets killed AND kills background children spawned in it. Spawn with `nohup ... &` and return in <50s; poll separately with short commands.
- pkill/pgrep -f patterns match your own shell command line — use `ps -eo pid,cmd | grep x | grep -v grep` + kill PID.
## WHAT WORKS (use /bot/src/brain5.py as-is)
- brain5.py: loop of align (rotate widest smoothed beam to nose idx1; hysteresis bonus to prev target) -> tap forward ((2,2) 14s if min(front idx0,1,2)>0.55 else (1,1) 8s) -> on grind (d0/d5=1): back 1.6s, rotate 7.5s alternating direction. Radio beacon every 1.2s + d10 read + d3 flag watch. Logs to /memory/brain5.log (STAT h/xy/d11/d3 every ~48s).
- Launch: `echo "0" > /dev/robot/d1; echo "0" > /dev/robot/d7; sleep 1; nohup python3 /bot/src/brain5.py > /tmp/brain5.out 2>&1 & echo started`
- Monitor: `grep "STAT h=" /memory/brain5.log | tail -3` (+ grep RADIO RX / FLAG / TXSTATUS for events).
## HISTORY / STATE
- Explored from spawn (0,0) heading ~180->south along a long straight corridor: last known (x=1.44, y=-6.3) at 05:13, h~177. Path: east ~1.4m then south 6m+. Corridor: left wall ~0.09-0.3, right wall ~0.3-0.6 (from body frame idx11-14/idx0-1 pattern), open ahead.
- World seems to be a maze with corridors ~1m wide, walls within ~1.5-1.9m sensor range.
- No peer contact yet (radio lost, no visual confirmation of a robot-shaped obstacle).
## NEXT STEPS FOR SUCCESSOR
1. Check if brain5 still running (ps ... brain5). If not, relaunch (command above). It resumes wherever the robot is.
2. Watch for: RADIO RX lines (peer!), FLAG lines (goal!), TXSTATUS changing from "lost".
3. If radio connects: talk! Agree plan: exchange rough positions (dead-reckon xy), meet at goal, arrive within 1 min. d3 flags likely mark the goal; "here=1" probably means AT goal.
4. If visual contact suspected (narrow short obstacle segment in P that moves), stop & beacon heavily.
5. Keep /memory notes updated.
