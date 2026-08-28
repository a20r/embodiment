#!/bin/bash
for d in 0 4 5 6 7; do printf "d$d: "; timeout 1 head -1 /dev/robot/d$d; done
timeout 1 head -1 /dev/robot/d2
