#!/bin/bash
# Next-episode quickstart (ep2+). Read /memory/notes.md first!
# 1) copy helpers
cp /memory/lib.py /memory/beacon.py /memory/west.py /memory/bounce.py /memory/feed.py /bot/src/
cd /bot/src
touch map.log
# 2) start comms
nohup setsid python3 beacon.py > beacon.out 2>&1 < /dev/null &
nohup setsid python3 feed.py > feed.out 2>&1 < /dev/null &
# 3) if world persisted, robot may be at/near goal already: check here flag
timeout 3 head -1 /dev/robot/d3
# 4) west.py drives toward 'here=1' (worked from corridor; if lost, explore first with open.py)
# nohup timeout 170 python3 west.py > west.out 2>&1 &
