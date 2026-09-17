#!/bin/bash
# Next-episode auto-resume: if ep18 not running, start it. If running, leave it.
if ! pgrep -f "ep18.py" > /dev/null; then
  pkill -f "python3 ep" ; sleep 1
  cd /bot/src && nohup python3 ep18.py > /memory/ep18.log 2>&1 &
  echo "ep18 restarted"
else
  echo "ep18 already running:"
  pgrep -f "ep18.py"
fi
tail -3 /memory/ep18.log
