#!/bin/bash

echo "Long monitoring - looking for goal=1..."

for i in {1..300}; do
  result=$(timeout 0.1 cat /dev/robot/d3 2>/dev/null)
  
  goal=$(echo "$result" | grep -o 'goal=[0-9]' | cut -d= -f2)
  
  if [ "$goal" = "1" ]; then
    echo "*** GOAL FOUND at tick $i! ***"
    echo "$result"
    break
  fi
  
  if [ $((i % 50)) -eq 1 ]; then
    d9=$(timeout 0.1 cat /dev/robot/d9 2>/dev/null)
    echo "[$i] d9=$d9 goal=$goal"
  fi
  
  sleep 0.2
done

