import time,sys,random,json
sys.path.insert(0,'/bot/src')
from bot import *
g=open('/memory/homing.log','a',buffering=1)
def lg(m): g.write('%.0f %s\n'%(time.time(),m))
def d5v():
    try: return float(rd('d5'))
    except: return 0.0
def fr(L): 
    v=[x for x in (L[0],L[1],L[15]) if x>=0]
    return min(v) if v else 0.1
last_tx=0
while True:
    now=time.time()
    st=status()
    if st.get('here') or st.get('goal'):
        speed(0,0); tx('AT_GOAL'); lg('HERE %s'%st); time.sleep(0.5); continue
    v=d5v()
    if now-last_tx>3:
        tx(json.dumps({'who':'B','d5':round(v,2)})); last_tx=now; lg('d5=%.2f'%v)
    if False:
        speed(0,0); tx(json.dumps({'who':'B','msg':'B: d5>0.93 - we are together. now explore for goal? propose: A leads, B follows keeping d5>0.8','d5':round(v,2)}))
        lg('TOGETHER %.2f'%v); time.sleep(2); continue
    # try creep forward while d5 improving
    base=v; improved=False
    for k in range(8):
        L=lidar()
        if fr(L)<0.28: break
        speed(22,22); time.sleep(0.3)
        nv=d5v()
        if nv>base+0.005: base=nv; improved=True
        if nv<base-0.06: break
    speed(0,0)
    if not improved:
        n=random.choice([1,2,3,4,5])
        d=random.choice([1,-1])
        for _ in range(n):
            speed(20*d,-20*d); time.sleep(0.3); speed(0,0); time.sleep(0.1)
    time.sleep(0.1)
