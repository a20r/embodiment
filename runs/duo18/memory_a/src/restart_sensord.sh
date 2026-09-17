#!/bin/bash
# restart the sensor daemon safely
for p in $(pgrep -f "python3 /bot/src/sensord.py"); do kill $p 2>/dev/null; done
sleep 0.3
cd /bot && nohup setsid python3 /bot/src/sensord.py > /tmp/sensord.out 2>&1 < /dev/null &
disown
sleep 1.5
pgrep -f "python3 /bot/src/sensord.py" >/dev/null && echo "sensord OK" || echo "sensord FAILED"
