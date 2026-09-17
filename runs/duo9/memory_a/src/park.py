import time,sys,json
sys.path.insert(0,'/bot/src')
from bot import *
last=0
while True:
    st=status()
    if st.get('here') or st.get('goal'):
        speed(0,0); tx('AT_GOAL B'); time.sleep(0.5); continue
    now=time.time()
    if now-last>3:
        try: v=float(rd('d5'))
        except: v=0
        tx(json.dumps({'who':'B','d5':round(v,2),'msg':'B STATIONARY+wiggling. hunt me. if you find goal instead, park there and tell me.'}))
        last=now
    # wiggle to be visible
    speed(18,18); time.sleep(0.25); speed(-18,-18); time.sleep(0.25); speed(0,0); time.sleep(1.2)
