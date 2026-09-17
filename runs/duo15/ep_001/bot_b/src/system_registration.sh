#!/bin/bash

echo "Attempting system registration..."
echo "0" > /dev/robot/d1

d9=$(timeout 0.1 cat /dev/robot/d9)
d11=$(timeout 0.1 cat /dev/robot/d11)
d4=$(timeout 0.1 cat /dev/robot/d4)

echo "Current position: d9=$d9, d11=$d11, bearing=$d4"
echo ""

# Try writing position info to different ports
echo "Attempting registration writes..."

# d5
echo "Registering on d5..."
echo "REG:$d9:$d11" > /dev/robot/d5 2>&1 &
sleep 0.2

# d7  
echo "Registering on d7..."
echo "LOC:$d9" > /dev/robot/d7 2>&1 &
sleep 0.2

# d8 with specific format
echo "Registering via d8..."
echo "REGISTER:$d9:$d11:$d4" > /dev/robot/d8
sleep 0.2

# Check for changes
echo ""
echo "Checking for response..."
sleep 1

d3=$(timeout 0.1 cat /dev/robot/d3)
echo "Status: $d3"

resp=$(timeout 0.1 cat /dev/robot/d10 2>/dev/null)
if [ -n "$resp" ]; then
  echo "d10 response: $resp"
fi

