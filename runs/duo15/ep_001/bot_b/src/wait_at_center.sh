#!/bin/bash

echo "Moving to center position (d9~=2000) and waiting..."

# First, move backward to around 2000
d9=$(timeout 0.1 cat /dev/robot/d9)
echo "Current d9: $d9"

if [ $(echo "$d9 > 2000" | bc) -eq 1 ]; then
  echo "Moving backward..."
  echo "-1" > /dev/robot/d1
else
  echo "Moving forward..."
  echo "1" > /dev/robot/d1
fi

# Move until close to 2000
while true; do
  d9=$(timeout 0.1 cat /dev/robot/d9)
  if [ $(echo "$(echo "$d9" | cut -d. -f1) > 1990 && $(echo "$d9" | cut -d. -f1) < 2010" | bc) -eq 1 ]; then
    echo "Close enough: d9=$d9"
    break
  fi
  sleep 0.2
done

# Stop
echo "0" > /dev/robot/d1

# Stay here and broadcast
echo "Staying at position and broadcasting..."
(
  counter=0
  while true; do
    counter=$((counter + 1))
    echo "R1_WAITING_AT_CENTER_$counter" > /dev/robot/d8
    sleep 1
  done
) &

# Monitor
for i in {1..120}; do
  d3=$(timeout 0.1 cat /dev/robot/d3)
  recv=$(timeout 0.1 cat /dev/robot/d10)
  goal=$(echo "$d3" | grep -o 'goal=[0-9]' | cut -d= -f2)
  d9=$(timeout 0.1 cat /dev/robot/d9)
  
  if [ -n "$recv" ] && [ "$recv" != "" ]; then
    echo "*** MSG RECEIVED: $recv ***"
  fi
  
  if [ "$goal" = "1" ]; then
    echo "*** GOAL FOUND at d9=$d9! ***"
    break
  fi
  
  if [ $((i % 20)) -eq 1 ]; then
    echo "[$i] d9=$d9 goal=$goal"
  fi
  
  sleep 1
done

