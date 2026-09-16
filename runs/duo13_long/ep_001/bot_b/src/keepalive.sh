#!/bin/bash
# relaunch explore6 if dead, unless goal-seek/home mode is active
cd /bot/src
while true; do
  if [ ! -e GOALALERT.txt ] && [ ! -e SEEKING ] && [ ! -e HOMEMODE ]; then
    if ! pgrep -f "python3 /bot/src/explore6.py" > /dev/null; then
      nohup python3 /bot/src/explore6.py >> /bot/src/explore6i.out 2>&1 &
      echo "$(date +%s) keepalive relaunched explore6" >> keepalive.log
    fi
  fi
  if ! pgrep -f "python3 /bot/src/goalseek.py" > /dev/null; then
    nohup python3 /bot/src/goalseek.py >> /bot/src/goalseek.out 2>&1 &
    echo "$(date +%s) keepalive relaunched goalseek" >> keepalive.log
  fi
  if ! pgrep -f "python3 /bot/src/relay.py" > /dev/null; then
    nohup python3 /bot/src/relay.py >> /bot/src/relay.out 2>&1 &
  fi
  if ! pgrep -f "python3 /bot/src/comm7.py" > /dev/null; then
    nohup python3 /bot/src/comm7.py >> /bot/src/comm7.out 2>&1 &
  fi
  sleep 30
done
