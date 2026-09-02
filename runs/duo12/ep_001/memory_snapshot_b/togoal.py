import time, math, json, sys
sys.path.insert(0,'/bot/src')
from lib import *
TICKS_PER_M=1600.0
def status():
    d={}
    for kv in r('d3').split():
        k,v=kv.split('='); d[k]=int(v)
    return d
# read last pose from map.log
import subprocess
p=json.loads(subprocess.check_output(['tail','-1','map.log']).decode())
x,y=p['x'],p['y']
gx,gy=-3.131,4.141
l0=ri('d9'); r0=ri('d6')
def upd():
    global l0,r0,x,y
    l=ri('d9'); rr=ri('d6')
    d=((l-l0)+(rr-r0))/2/TICKS_PER_M
    l0,r0=l,rr
    h=math.radians(heading())
    x+=d*math.cos(h); y+=d*math.sin(h)
log=open('map.log','a')
for it in range(40):
    st=status(); L=lidar(); upd()
    log.write(json.dumps({'t':round(time.time(),1),'x':round(x,3),'y':round(y,3),'h':heading(),'L':L,'st':st,'d11':r('d11')})+'\n'); log.flush()
    dist=math.hypot(gx-x,gy-y)
    print(f"pos {x:.2f},{y:.2f} dist {dist:.2f} here={st['here']}",flush=True)
    if dist<0.12 and st['here']==1:
        break
    if st['here']==1 and dist<0.25:
        break
    b=math.degrees(math.atan2(gy-y,gx-x))
    turn_to(b,tol=8)
    w('d1',40); w('d7',40)
    ts=time.time(); 
    while time.time()-ts<2:
        time.sleep(0.12); upd()
        if bump(): w('d1',-30); w('d7',-30); time.sleep(0.4); break
        Lc=[v if v>0 else 2.0 for v in lidar()]
        if min(Lc[0],Lc[1]*1.4,Lc[15]*1.4)<0.22: break
        if math.hypot(gx-x,gy-y)<0.1: break
    stop()
stop()
st=status()
print("parked at",x,y,"here",st,flush=True)
