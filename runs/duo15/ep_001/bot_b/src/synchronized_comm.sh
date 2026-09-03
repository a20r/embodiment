#!/bin/bash

echo "Attempting synchronized communication protocol..."

# Send a message and immediately try to read response
for i in {1..50}; do
  # Send
  echo "SYNC_$i" > /dev/robot/d8 &
  
  # Immediately read
  result=$(timeout 0.1 cat /dev/robot/d10 2>/dev/null)
  
  if [ -n "$result" ]; then
    echo "Response received: $result"
    break
  fi
  
  # Check goal
  d3=$(timeout 0.05 cat /dev/robot/d3 2>/dev/null)
  if echo "$d3" | grep -q 'goal=1'; then
    echo "GOAL!"
    break
  fi
  
  wait
  sleep 0.2
done

wait
echo "Done"
