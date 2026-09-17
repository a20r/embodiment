#!/bin/bash
for n in nav2 wf homing homing2; do pkill -f "^python3 $n\.py$"; done
sleep 0.3
echo 0 > /dev/robot/d10
echo 0 > /dev/robot/d11
