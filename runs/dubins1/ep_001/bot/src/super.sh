#!/bin/bash
while true; do
  for side in right left; do
    python3 /bot/src/wf2.py $side >> /bot/src/wf2.err 2>&1 &
    P=$!
    sleep 270
    kill -9 $P 2>/dev/null
    python3 -c "
import sys,time; sys.path.insert(0,'/memory/code')
from robot import Robot
r=Robot(); time.sleep(0.5); r.stop()" 2>/dev/null
    if [ -f /memory/GOAL_REACHED ]; then exit 0; fi
  done
done
