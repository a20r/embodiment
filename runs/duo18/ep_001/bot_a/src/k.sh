#!/bin/bash
# kill python3 processes running src/$1.py (only real python3 procs)
for p in $(pgrep -f "src/$1.py"); do [ "$(cat /proc/$p/comm 2>/dev/null)" = "python3" ] && kill $p 2>/dev/null; done
sleep 0.3; echo "$1: $(pgrep -f "src/$1.py" | while read p; do [ "$(cat /proc/$p/comm 2>/dev/null)" = python3 ] && echo alive; done | head -1) done"
