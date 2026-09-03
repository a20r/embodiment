# READ THIS FIRST (written end of episode 1)
You are botB, a robot in a maze with another robot botA. Goal: BOTH robots at the goal spot
within 60s of each other. My here-flag (d3 "here=") never fired in ep1; botA found the goal
and parked there (its pings: "botA PING x=-3.37 y=4.18 d11=.. here=1").

READ IN ORDER: hardware.md (device map, calibrations), strategy.md (lessons), log.md (ep1 story).
CODE: /memory/src_snapshot/ -- copy to /bot/src and reuse:
  robot.py (Bot: threaded readers, safe writes), explore.py (Nav, turn_to, drive),
  final.py (best controller: climb v-gradient -> wall-follow chamber, gap-shoot, botA protocol msgs).

FASTEST PATH TO WIN (ep1 knowledge):
1) rm /memory/pose.json (fresh frame). Start final.py immediately; it logs to /memory/final.log.
2) d11 (v) = proximity to botA. If botA parked at goal (check rx pings here=1), v is a static
   beacon field: climb to plateau ~0.8, then the goal chamber is behind a wall. Ep1 NEVER found
   the entrance by right-hand wall-follow (circled for ~30 min, v cycling 0.45-0.85).
3) UNTRIED/BEST IDEAS: (a) LEAD protocol -- ask botA (its agent reads radio & adapts!) to walk out,
   meet me (both d11->0.95+), then return to goal slowly pausing when my d11<0.85; follow it in.
   (b) left-hand wall-follow (side=-1) may find entrance right-hand missed.
   (c) map chamber precisely: stakeout at v-max, fine spin-scan for beams>1.0 (door signature).
4) here=1 => STOP, tx "botB AT GOAL here=1" repeatedly; both must be present within 60s.
Radio appears long-range. botA agent restarts periodically (silence = its episode gap).

## CRITICAL LATE DISCOVERY (last minute of ep1):
botA actively COACHES via radio: "botA COACH d11=0.778 HOTTER (keep direction). Goal=my position.
Stop when your here=1." => botA's agent watches my pings and replies hot/cold in real time!
NEXT EP: keep sending "botB PING d11=<v> seq=N" ~1/s and PARSE "COACH ... HOTTER/COLDER" replies:
HOTTER=keep heading, COLDER=turn. This + gradient-climb should reach the goal fast.
Also tell botA "STAY AT GOAL, coach me" immediately at episode start.
