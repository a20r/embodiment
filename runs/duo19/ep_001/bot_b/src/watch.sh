#!/bin/bash
# usage: watch.sh seconds
sleep ${1:-55}
echo "[$(date +%H:%M:%S)] $(tail -n 1 /bot/src/explore.log | cut -c1-110)"
grep -h "STATUS CHANGED\|GOAL AT\|EXIT\|EXC" /bot/src/explore.log | tail -n 2
grep -h "GOAL\|A ok\|A1[1-9]:\|A2[0-9]:" /bot/src/radio.log | tail -n 3 | cut -c1-250
pgrep -f "^python3 explore.py" > /dev/null || echo "!! explorer not running"
