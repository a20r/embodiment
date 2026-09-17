#!/bin/bash

echo "Listening on d0, d10 while moving..."

# Start listeners
(while true; do 
  result=$(timeout 0.2 cat /dev/robot/d0)
  if [ -n "$result" ] && [ "$result" != "0" ]; then
    echo "[d0] $result"
  fi
done) &
PID1=$!

(while true; do 
  result=$(timeout 0.2 cat /dev/robot/d10)
  if [ -n "$result" ]; then
    echo "[d10] $result"
  fi
done) &
PID2=$!

# Move and broadcast
echo "1" > /dev/robot/d1
sleep 2

for i in {1..20}; do
  echo "SEEKING_ROBOT2" > /dev/robot/d8
  sleep 0.3
done

echo "0" > /dev/robot/d1

sleep 2
kill $PID1 $PID2 2>/dev/null
wait

