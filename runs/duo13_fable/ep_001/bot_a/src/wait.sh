#!/bin/bash
# sleep then report compactly: rx count/last, status, sig
sleep ${1:-55}
n=$(wc -l < /tmp/rx.txt); last=$(tail -n 1 /tmp/rx.txt | cut -c1-200)
echo "rx=$n | $last"
echo "$(python3 /bot/src/rio.py r 3) sig=$(python3 /bot/src/rio.py r 11) $(date +%H:%M:%S)"
