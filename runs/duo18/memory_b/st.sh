#!/bin/sh
date +%H:%M:%S | tr '\n' ' '; python3 -c "
import json; r=json.load(open('/bot/pose.txt')); print('me:', r['x'], r['y'], int(r['h']), 'd11', r['d11'], 'goal', r['goal'], 'here', r['here'], 'd5', r['d5'])"
grep -h '"event"' /bot/track.log | grep -v CRASH | tail -1 | cut -c1-100
tail -n 1 /bot/rx.log | cut -c1-${1:-150}
grep -c "d11=[0-9.]* goal=1" /bot/rx.log
