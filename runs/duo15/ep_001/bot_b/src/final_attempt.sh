#!/bin/bash

echo "Final attempt: continuous movement and broadcasting..."

# Start continuous broadcast loop
(
  counter=0
  while true; do
    counter=$((counter + 1))
    d9=$(timeout 0.05 cat /dev/robot/d9 2>/dev/null)
    echo "ROBOT1_AT_${d9}_MSG_${counter}" > /dev/robot/d8
    sleep 0.5
  done
) &
BCAST_PID=$!

# Move back and forth
(
  direction=1
  for i in {1..120}; do
    if [ $((i % 30)) -eq 0 ]; then
      direction=$((direction * -1))
      echo $direction > /dev/robot/d1
    fi
    sleep 0.3
  done
  echo "0" > /dev/robot/d1
) &
MOVE_PID=$!

# Monitor for goal
for i in {1..180}; do
  d3=$(timeout 0.05 cat /dev/robot/d3 2>/dev/null)
  recv=$(timeout 0.05 cat /dev/robot/d10 2>/dev/null)
  goal=$(echo "$d3" | grep -o 'goal=[0-9]' | cut -d= -f2)
  
  if [ -n "$recv" ] && [ "$recv" != "" ]; then
    echo "*** RECEIVED: $recv ***"
  fi
  
  if [ "$goal" = "1" ]; then
    echo "*** GOAL FOUND! ***"
    echo "$d3"
    break
  fi
  
  if [ $((i % 30)) -eq 1 ]; then
    echo "[$i] goal=$goal"
  fi
  
  sleep 0.3
done

kill $BCAST_PID $MOVE_PID 2>/dev/null
wait 2>/dev/null
echo "Done"

