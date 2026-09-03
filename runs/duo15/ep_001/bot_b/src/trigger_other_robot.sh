#!/bin/bash

echo "Attempting to trigger other robot..."
echo "0" > /dev/robot/d1

# Try various "activation" messages
messages=(
  "START"
  "ACTIVATE"
  "BEGIN"
  "ENABLE"
  "WAKE"
  "INIT"
  "SPAWN"
  "CREATE"
  "RUN"
  "ROBOT2_START"
  "ROBOT2_ONLINE"
  "SYSTEM_START"
  "CHALLENGE_BEGIN"
)

echo "Sending activation messages..."
for msg in "${messages[@]}"; do
  echo "$msg" > /dev/robot/d8
  echo "Sent: $msg"
  sleep 0.3
  
  # Check for any response
  resp=$(timeout 0.1 cat /dev/robot/d10 2>/dev/null)
  if [ -n "$resp" ]; then
    echo "  RESPONSE: $resp"
  fi
done

echo ""
echo "Now listening for 30 seconds for activation confirmation..."

for i in {1..30}; do
  d3=$(timeout 0.05 cat /dev/robot/d3 2>/dev/null)
  if echo "$d3" | grep -q 'goal=1'; then
    echo "GOAL activated!"
    break
  fi
  
  resp=$(timeout 0.05 cat /dev/robot/d10 2>/dev/null)
  if [ -n "$resp" ]; then
    echo "[$i] Received: $resp"
  fi
  
  sleep 1
done

echo "Activation attempt complete"

