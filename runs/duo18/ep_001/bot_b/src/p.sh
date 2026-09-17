#!/bin/sh
python3 -c "
import json,re,time
r=json.load(open('/bot/pose.txt')); last=open('/bot/rx.log').read().strip().split('\n')[-1]
m=re.search(r'd11=([0-9.]+) goal=(\d)', last)
print(time.strftime('%H:%M'),'me(%.1f,%.1f) d11 %s g%s e%s | o %s %s'%(r['x'],r['y'],r['d11'],r['goal'],r.get('eff'),last[:5],m.groups() if m else last[9:70]))"
