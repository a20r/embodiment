#!/bin/bash
# print only new rx lines since last poll
N=$(cat /bot/src/poll.n 2>/dev/null || echo 0); T=$(wc -l < /bot/src/rx.log)
tail -n +$((N+1)) /bot/src/rx.log | cut -c1-230; echo $T > /bot/src/poll.n
echo "d11=$(python3 /bot/src/rio.py r d11) $(python3 /bot/src/rio.py r d3) route:$(tail -1 /bot/src/route.log | cut -c1-80)"
