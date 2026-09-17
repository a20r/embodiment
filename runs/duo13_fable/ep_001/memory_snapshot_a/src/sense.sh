#!/bin/bash
# print all readable ports once
for i in 0 2 3 4 5 6 9 11; do printf "d%s: %s\n" $i "$(timeout 2 head -n1 /dev/robot/d$i)"; done
