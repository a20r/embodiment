#!/bin/bash

echo "FINAL ATTEMPT: Trying all possible simple messages and values"
echo "0" > /dev/robot/d1

# Single digits
echo "Sending single digits 0-9..."
for digit in {0..9}; do
  echo "$digit" > /dev/robot/d8
  sleep 0.2
  
  d3=$(timeout 0.05 cat /dev/robot/d3 2>/dev/null)
  if echo "$d3" | grep -q 'goal=1'; then
    echo "GOAL at digit $digit!"
    exit 0
  fi
done

# Common single characters
echo "Sending single characters..."
for char in "A" "B" "X" "Y" "Z" "0" "1" "2" "*" "#" "@"; do
  echo "$char" > /dev/robot/d8
  sleep 0.2
  
  d3=$(timeout 0.05 cat /dev/robot/d3 2>/dev/null)
  if echo "$d3" | grep -q 'goal=1'; then
    echo "GOAL at char $char!"
    exit 0
  fi
done

# Empty message
echo "Sending empty message..."
echo "" > /dev/robot/d8
sleep 0.5

d3=$(timeout 0.05 cat /dev/robot/d3 2>/dev/null)
if echo "$d3" | grep -q 'goal=1'; then
  echo "GOAL at empty message!"
  exit 0
fi

# Try newline only
echo "Trying various control sequences..."
printf "\n" > /dev/robot/d8
sleep 0.2

d3=$(timeout 0.05 cat /dev/robot/d3 2>/dev/null)
if echo "$d3" | grep -q 'goal=1'; then
  echo "GOAL at newline!"
  exit 0
fi

echo "No goal found with any simple message"
echo "Final status:"
timeout 0.1 cat /dev/robot/d3

