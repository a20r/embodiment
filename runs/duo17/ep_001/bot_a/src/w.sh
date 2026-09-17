#!/bin/bash
t0=$(date +%s); : > RX.txt
while [ $(( $(date +%s) - t0 )) -lt 57 ]; do
  [ -f GOAL_DONE ] && { echo "GOAL DONE: $(cat GOAL_DONE)"; exit; }
  [ -s RX.txt ] && { echo "RX: $(tail -2 RX.txt | cut -c1-200)"; exit; }
  sleep 4
done
echo "$(tail -1 radio.log | cut -c1-12) d11=$(tail -1 radio.log | grep -o 'd11=[0-9.]*' | cut -d= -f2) $(tail -1 radio.log | grep -o 'goal=. here=.')"
