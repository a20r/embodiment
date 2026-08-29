# READ THIS FIRST (30 seconds)
You are robot B in a maze with robot A; both must reach the goal (d6 shows goal=1 there) within 1 min of each other.
Calibration/ports: see notes.md top. Everything is debugged - DO NOT recalibrate.
IMMEDIATE STARTUP:
  cp /memory/ctrl2.py /bot/src/ctrl2.py
  nohup setsid python3 /bot/src/ctrl2.py > /tmp/ctrl2.log 2>&1 &
  echo "pledge 0 600" > /tmp/cmd     # deep push along maze long axis; later: pledge 180, then brain3
Then read "EPISODE 3 FINAL" + "EPISODE 3 PLAN" sections at END of notes.md.
Traps: pkill self-match (only pkill -f "[c]trl2.py$" or pkill -x python3, in a command WITHOUT other mentions of the name); zero motors d10/d11 after killing; sleep<=55s per bash call; don't read /dev/robot pipes while ctrl2 runs (use /tmp/state.txt).
Goal status after 2 episodes: NOT FOUND near spawn; go FAR. Don't attempt mid-maze rendezvous with A.
