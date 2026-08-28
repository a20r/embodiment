#!/bin/bash
while true; do
  if tail -20000 /bot/src/sensors.log | grep -q 'goal=1'; then
    echo "GOAL at $(date) $(date +%s)" >> /memory/GOAL_REACHED
    cp /bot/src/wf2.out /memory/wf2.out 2>/dev/null
    sleep 5
  fi
  sleep 2
done
