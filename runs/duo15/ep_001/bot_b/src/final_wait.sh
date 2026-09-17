#!/bin/bash

echo "Final broadcast and wait strategy..."
echo "Robot will stay in place and broadcast indefinitely"
echo "0" > /dev/robot/d1

echo "Starting 60-minute broadcast cycle..."
(
  counter=0
  while true; do
    counter=$((counter+1))
    d9=$(timeout 0.05 cat /dev/robot/d9 2>/dev/null)
    echo "ROBOT1_ACTIVE_WAITING_${d9}_${counter}" > /dev/robot/d8
    sleep 0.5
  done
) &

BCAST=$!

# Monitor for goal
echo "Monitoring for goal or message..."
start=$(date +%s)

while true; do
  now=$(date +%s)
  elapsed=$((now - start))
  
  # Check goal
  d3=$(timeout 0.05 cat /dev/robot/d3 2>/dev/null)
  if echo "$d3" | grep -q 'goal=1'; then
    echo "*** GOAL FOUND ***"
    echo "$d3"
    kill $BCAST 2>/dev/null
    break
  fi
  
  # Check for messages
  msg=$(timeout 0.05 cat /dev/robot/d10 2>/dev/null)
  if [ -n "$msg" ]; then
    echo "*** MESSAGE RECEIVED ***"
    echo "$msg"
  fi
  
  if [ $((elapsed % 60)) -eq 0 ] && [ $elapsed -gt 0 ]; then
    echo "[$elapsed seconds] Still waiting for goal/message..."
  fi
  
  if [ $elapsed -gt 3600 ]; then
    echo "Timeout after 1 hour"
    kill $BCAST 2>/dev/null
    break
  fi
  
  sleep 0.5
done

wait 2>/dev/null
echo "Final wait complete"

