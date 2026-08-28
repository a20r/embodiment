#!/bin/bash
while true; do
  if ! pgrep -f "python3 src/escort.py" > /dev/null; then
    echo "$(date) restarting escort" >> /memory/run.log
    cd /bot && nohup python3 src/escort.py >> /tmp/escort.out 2>&1 &
  fi
  sleep 10
done
