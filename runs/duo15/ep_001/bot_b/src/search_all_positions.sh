#!/bin/bash

# Final comprehensive search - try all discovered positions
echo "Final comprehensive position search..."

positions=(
  "1000:1"
  "2000:1"  
  "3000:1"
  "4000:1"
  "1500:1"
  "2500:1"
  "3500:1"
  "4000:-1"
  "3000:-1"
  "2000:-1"
  "1000:-1"
)

echo "0" > /dev/robot/d1

for pos_info in "${positions[@]}"; do
  d9_target=$(echo "$pos_info" | cut -d: -f1)
  direction=$(echo "$pos_info" | cut -d: -f2)
  
  echo ""
  echo "Target: d9=$d9_target direction=$direction"
  
  echo "$direction" > /dev/robot/d1
  
  # Move for 20 seconds or until goal found
  for i in {1..100}; do
    d9=$(timeout 0.05 cat /dev/robot/d9 2>/dev/null)
    d3=$(timeout 0.05 cat /dev/robot/d3 2>/dev/null)
    goal=$(echo "$d3" | grep -o 'goal=[0-9]' | cut -d= -f2)
    
    if [ -z "$d9" ] || [ -z "$goal" ]; then
      sleep 0.1
      continue
    fi
    
    d9_int=$(echo "$d9" | cut -d. -f1)
    
    if [ "$goal" = "1" ]; then
      echo "*** GOAL FOUND at d9=$d9 ***"
      echo "0" > /dev/robot/d1
      exit 0
    fi
    
    # Check if close to target or overshot
    diff=$((d9_int - d9_target))
    if [ $diff -lt 0 ]; then
      diff=$((-diff))
    fi
    
    if [ $diff -lt 50 ] && [ $i -gt 5 ]; then
      echo "  Reached $d9, checking..."
      sleep 1
      break
    fi
    
    if [ $((i % 20)) -eq 1 ]; then
      echo "  [$i] d9=$d9"
    fi
    
    sleep 0.1
  done
done

echo "0" > /dev/robot/d1
echo "Search complete - goal not found at any tested position"
