#!/bin/bash

echo "Aggressive d10 reading strategy..."
echo "Theory: Other robot is broadcasting constantly, just need to read enough"
echo ""

echo "0" > /dev/robot/d1

echo "Beginning aggressive read loop..."
echo "Reading d10 extremely rapidly..."

messages_received=0

for i in {1..1000}; do
  # Try to read d10
  msg=$(timeout 0.02 cat /dev/robot/d10 2>/dev/null)
  
  if [ -n "$msg" ]; then
    messages_received=$((messages_received + 1))
    echo "[$i] RECEIVED: $msg"
    
    # If we got a message, maybe respond?
    echo "RESPONDING_TO_MESSAGE" > /dev/robot/d8
  fi
  
  # Also check goal
  d3=$(timeout 0.02 cat /dev/robot/d3 2>/dev/null)
  if echo "$d3" | grep -q 'goal=1'; then
    echo "GOAL FOUND!"
    echo "$d3"
    exit 0
  fi
  
  if [ $((i % 100)) -eq 0 ]; then
    echo "[$i] Still searching... (messages so far: $messages_received)"
  fi
done

echo ""
echo "Scan complete. Messages received: $messages_received"
timeout 0.1 cat /dev/robot/d3

