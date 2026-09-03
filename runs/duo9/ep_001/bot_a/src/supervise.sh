#!/bin/bash
while true; do
  if grep 'GOALFOUND' /memory/radio_rx.log 2>/dev/null | grep -vq 'token'; then
    for p in $(pgrep -f 'follow.py|auto.py|gobear.py|goto.py'); do kill -9 $p 2>/dev/null; done
    echo 0 > /dev/robot/d10; echo 0 > /dev/robot/d11
    nohup python3 /bot/src/homing2.py > /memory/homing2.out 2>&1 &
    echo "$(date +%s) switched to homing2" >> /memory/log.txt
    exit 0
  fi
  sleep 5
done
