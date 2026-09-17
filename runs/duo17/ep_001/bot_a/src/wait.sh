#!/bin/bash
# wait up to $1 seconds for new RX or status change; print summary
T=${1:-55}; : > RX.txt; t0=$(date +%s)
while [ $(( $(date +%s) - t0 )) -lt $T ]; do
  if [ -s RX.txt ]; then echo "NEW RX:"; cat RX.txt; break; fi
  if grep -q "goal=1\|:ok" <(tail -1 radio.log); then echo "STATUS CHANGE:"; tail -1 radio.log; break; fi
  sleep 3
done
echo "d11 trend: $(grep TX radio.log | tail -10 | awk '{print $3}' | cut -d= -f2 | tr '\n' ' ')"; tail -1 radio.log
