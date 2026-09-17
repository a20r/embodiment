#!/bin/bash
echo "Long broadcast test - sending on d8 and listening on d10..."
 
# Send continuous messages in background
(
  for i in {1..60}; do
    echo "ROBOT1_SEEKING_ROBOT2" > /dev/robot/d8
    sleep 0.3
  done
) &

# Move forward in background
(
  echo "1" > /dev/robot/d1
  sleep 30
  echo "0" > /dev/robot/d1
) &

# Listen on d10
timeout 35 bash -c 'while IFS= read -t 0.5 line; do echo "RECEIVED: $line"; done < /dev/robot/d10'

wait
echo "Done"
