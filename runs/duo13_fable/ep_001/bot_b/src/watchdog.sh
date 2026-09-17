#!/bin/bash
cd /bot/src
while true; do
  if ! pgrep -f "python3 lead.py" >/dev/null; then
    if tail -5 cell.txt | grep -q "GOAL"; then
      # on goal: wait there, message R2, log
      pgrep -f "python3 guide.py" >/dev/null || nohup python3 guide.py >/dev/null 2>&1 &
    else
      pkill -f "python3 guide.py"
      echo '{"pos":[0,0],"path":[],"cells":{}}' > /memory/map.json
      echo "=== watchdog restart $(date +%T) ===" >> cell.txt
      nohup python3 lead.py 60 > lead.out 2>&1 &
    fi
  fi
  sleep 20
done
