#!/bin/sh
python3 -c "
import json,re,subprocess,time
r=json.load(open('/bot/pose.txt'))
last=open('/bot/rx.log').read().strip().split('\n')[-1]
m=re.search(r'd11=([0-9.]+) goal=(\d)', last)
print(time.strftime('%H:%M'), 'me', r['x'], r['y'], 'd11', r['d11'], 'goal', r['goal'], '| other', last[:8], m.groups() if m else last[9:60])
"
grep -c '"goal": "1"' /bot/track.log
