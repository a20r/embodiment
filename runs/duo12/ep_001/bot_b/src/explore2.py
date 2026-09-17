import time, math, json, sys
sys.path.insert(0,'/bot/src')
from lib import *
TICKS_PER_M=1600.0
dur=float(sys.argv[1]) if len(sys.argv)>1 else 240
def status():
    d={}
    for kv in r('d3').split():
        k,v=kv.split('='); d[k]=int(v)
    return d
x=y=0.0
try:
    import os
    for line in open('map.log'):
        p=json.loads(line); x=p['x']; y=p['y']
except Exception: pass
l0=ri('d9'); r0=ri('d6')
log=open('map.log','a')
t0=time.time()
def upd():
    global l0,r0,x,y
    l=ri('d9'); rr=ri('d6')
    d=((l-l0)+(rr-r0))/2/TICKS_PER_M
    l0,r0=l,rr
    h=math.radians(heading())
    x+=d*math.cos(h); y+=d*math.sin(h)
try:
  while time.time()-t0<dur:
    L=lidar(); st=status(); upd()
    log.write(json.dumps({'t':round(time.time(),1),'x':round(x,3),'y':round(y,3),
        'h':heading(),'L':L,'st':st,'d11':r('d11')})+'\n'); log.flush()
    if st.get('here') or st.get('goal'):
        stop(); print("FLAG",st); break
    if bump():
        w('d1',-35); w('d7',-35); time.sleep(0.6)
        w('d1',-22); w('d7',22); time.sleep(0.6); stop(); continue
    Lc=[v if v>0 else 1.8 for v in L]
    front=min(Lc[0],Lc[1]*1.3,Lc[15]*1.3)
    right=min(Lc[3],Lc[4],Lc[5])
    if front<0.25:
        w('d1',-25); w('d7',25); time.sleep(0.35); continue
    if right>0.6 and Lc[4]>0.7:
        # lost wall: turn right toward it
        w('d1',35); w('d7',8); time.sleep(0.3); continue
    err=0.28-right
    steer=max(-14,min(14,err*70))
    sp=55 if front>0.5 else 28
    w('d1',sp-steer); w('d7',sp+steer)
    time.sleep(0.15)
finally:
  stop(); print("end",round(x,2),round(y,2),status())
