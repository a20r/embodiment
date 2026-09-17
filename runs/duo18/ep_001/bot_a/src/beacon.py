#!/usr/bin/env python3
import time, sys, json
sys.path.insert(0,'/bot/src')
from rio import tx
i=0
while True:
    i+=1
    try: p=json.load(open('/tmp/pose.json')); s=json.load(open('/tmp/state.json'))
    except: p={'x':0,'y':0}; s={}
    extra=open('/tmp/beacon_extra.txt').read().strip() if __import__('os').path.exists('/tmp/beacon_extra.txt') else ''
    tx((f"MOVING robot #{i}: d11={s.get('d11','?')} goal=0. {extra}")[:250])
    time.sleep(10)
