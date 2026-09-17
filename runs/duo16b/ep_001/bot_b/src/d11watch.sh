#!/bin/bash
# log d11 at 2Hz forever
while true; do
  v=$(timeout 0.3 cat /dev/robot/d11 2>/dev/null | tail -1)
  [ -n "$v" ] && echo "$(date +%H:%M:%S),$v" >> /memory/d11watch.log
  sleep 0.5
done
