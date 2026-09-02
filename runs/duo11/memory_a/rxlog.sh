#!/bin/bash
while true; do
  IFS= read -r line < /dev/robot/d10
  [ -n "$line" ] && echo "$(date +%s) $line" >> /memory/rx.log
done
