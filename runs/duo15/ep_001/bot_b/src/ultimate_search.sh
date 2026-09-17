#!/bin/bash

echo "=== ULTIMATE FINAL SEARCH ==="
echo "This will run until goal is found or time expires"
echo "Starting comprehensive search pattern..."
echo ""

# Stop everything first
echo "0" > /dev/robot/d1

# Start position broadcast loop
(
  while true; do
    d9=$(timeout 0.03 cat /dev/robot/d9 2>/dev/null)
    d11=$(timeout 0.03 cat /dev/robot/d11 2>/dev/null)
    
    # Try different message formats
    msgs=(
      "R1:$d9:$d11"
      "$d9"
      "POS:$d9"
      "ACTIVE"
    )
    
    for msg in "${msgs[@]}"; do
      echo "$msg" > /dev/robot/d8
    done
    
    sleep 0.2
  done
) &
BCAST_PID=$!

# Movement pattern: spiral through all positions
echo "1" > /dev/robot/d1

# Move for extended period
for i in {1..1000}; do
  # Every 50 iterations, change direction
  if [ $((i % 50)) -eq 0 ]; then
    if [ $((i / 50 % 2)) -eq 0 ]; then
      echo "1" > /dev/robot/d1
    else
      echo "-1" > /dev/robot/d1
    fi
  fi
  
  # Every 100 iterations, try a turn
  if [ $((i % 100)) -eq 0 ]; then
    angle=$((RANDOM % 360))
    echo "$angle" > /dev/robot/d6
  fi
  
  # Check for goal
  d3=$(timeout 0.03 cat /dev/robot/d3 2>/dev/null)
  if echo "$d3" | grep -q 'goal=1'; then
    echo ""
    echo "*** GOAL FOUND ***"
    echo "$d3"
    kill $BCAST_PID 2>/dev/null
    exit 0
  fi
  
  # Check for messages
  msg=$(timeout 0.03 cat /dev/robot/d10 2>/dev/null)
  if [ -n "$msg" ]; then
    echo "MESSAGE: $msg"
  fi
  
  if [ $((i % 100)) -eq 0 ]; then
    echo "Iteration $i..."
  fi
  
  sleep 0.05
done

# Final check
echo ""
echo "Search complete. Final status:"
timeout 0.1 cat /dev/robot/d3

kill $BCAST_PID 2>/dev/null

