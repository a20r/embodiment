#!/bin/bash
T=${1:-58}; t0=$(date +%s)
while [ $(( $(date +%s) - t0 )) -lt $T ]; do
  if [ -s RX.txt ]; then echo "NEW RX:"; cat RX.txt | cut -c1-260; : > RX.txt; break; fi
  if tail -1 radio.log | grep -q "goal=1"; then echo "LINK/GOAL:"; tail -1 radio.log; break; fi
  sleep 3
done
echo "d11: $(grep TX radio.log | tail -6 | awk '{print $3}' | cut -d= -f2 | tr '\n' ' ') | $(tail -1 radio.log | cut -d' ' -f4-)"
