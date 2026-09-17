#!/bin/bash
# duty-cycle tx: 25 pings over ~60s, then 75s silence
while true; do
  for i in $(seq 1 25); do
    echo "PING A$(date +%s)" > /dev/robot/d8 2>/dev/null
    sleep 2.4
  done
  sleep 75
done
