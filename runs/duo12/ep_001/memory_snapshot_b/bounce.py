import time, math, json, sys
sys.path.insert(0,'/bot/src')
from lib import *
TICKS_PER_M=1600.0
def status():
    d={}
    for kv in r('d3').split():
        k,v=kv.split('='); d[k]=int(v)
    return d
log=open('map.log','a')
def logit(tag=''):
    st=status(); d=r('d11')
    log.write(json.dumps({'t':round(time.time(),1),'x':-3.37,'y':4.18,'h':heading(),
        'L':lidar(),'st':st,'d11':d,'tag':tag})+'\n'); log.flush()
    if st.get('goal'):
        open('/bot/src/GOAL_DONE','w').write(str(st))
    return st
def drive(hdg, secs, sp=35):
    turn_to(hdg, tol=10)
    w('d1',sp); w('d7',sp)
    ts=time.time()
    while time.time()-ts<secs:
        time.sleep(0.12)
        if bump(): w('d1',-30); w('d7',-30); time.sleep(0.5); break
        Lc=[v if v>0 else 2.0 for v in lidar()]
        if min(Lc[0],Lc[1]*1.4,Lc[15]*1.4)<0.2: break
    stop()
while True:
    # sit 40s
    for _ in range(20):
        st=logit('sit')
        time.sleep(2)
    # step off south, then back north
    drive(270, 2.2)   # south (heading 270 = -y in our convention? y=sin(h); 270 -> sin=-1 yes south)
    st=logit('off')
    print("off:",st,flush=True)
    time.sleep(2)
    drive(90, 2.2)
    # re-enter: seek here=1
    for attempt in range(8):
        st=logit('reenter')
        if st['here']==1: break
        drive(90+attempt*45, 1.2)
    print("back:",logit('back'),flush=True)
