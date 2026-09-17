#!/bin/bash

echo "Monitoring all ports..."

# Run for 30 seconds
for i in {1..30}; do
  echo "=== Tick $i ==="
  
  for port in {0..11}; do
    result=$(timeout 0.05 cat /dev/robot/d$port 2>/dev/null)
    if [ -n "$result" ]; then
      echo "d$port: $result"
    fi
  done
  
  sleep 1
done

