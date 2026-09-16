#!/bin/bash
echo "=== $(date +%H:%M:%S)"
tail -2 /bot/src/exp6.log 2>/dev/null
tail -1 /bot/src/rx.log 2>/dev/null
[ -f /bot/src/GOALALERT.txt ] && echo "GOAL ALERT!" && tail -2 /bot/src/GOALALERT.txt
