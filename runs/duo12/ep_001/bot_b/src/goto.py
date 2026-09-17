import time, math, json, sys
sys.path.insert(0,'/bot/src')
from lib import *
TICKS_PER_M=1600.0
gx,gy=float(sys.argv[1]),float(sys.argv[2])
dur=float(sys.argv[3]) if len(sys.argv)>3 else 50
def status():
    d={}
    for kv in r('d3').split():
        k,v=kv.split('='); d[k]=int(v)
    return d
class Odo:
    def __init__(self,x,y):
        self.l=ri('d9'); self.r=ri('d6'); self.x=x; self.y=y
    def update(self):
        l=ri('d9'); rr=ri('d6')
        d=((l-self.l)+(rr-self.r))/2/TICKS_PER_M
        self.l=l; self.r=rr
        h=math.radians(heading())
        self.x+=d*math.cos(h); self.y+=d*math.sin(h)
# load last pose
x=y=0.0
try:
    for line in open('map.log'):
        p=json.loads(line); x=p['x']; y=p['y']
except Exception: pass
odo=Odo(x,y)
log=open('map.log','a')
t0=time.time()
try:
  while time.time()-t0<dur:
    L=lidar(); st=status(); odo.update()
    log.write(json.dumps({'t':round(time.time(),1),'x':round(odo.x,3),'y':round(odo.y,3),
        'h':heading(),'L':L,'st':st,'d11':r('d11')})+'\n'); log.flush()
    if st.get('here') or st.get('goal'):
        print("FLAG",st); stop(); break
    if bump():
        w('d1',-30); w('d7',-30); time.sleep(0.6); stop()
    Lc=[xx if xx>0 else 1.6 for xx in L]
    h=heading()
    bearing=math.degrees(math.atan2(gy-odo.y, gx-odo.x))
    dist=math.hypot(gx-odo.x, gy-odo.y)
    if dist<0.15:
        print("arrived at target, st",st); stop(); break
    # score each beam: beam i at angle h + 22.5*i (deg)
    best=None
    for i in range(16):
        ang=(h+22.5*i)
        clear=min(Lc[i], Lc[(i+1)%16]*1.1, Lc[(i-1)%16]*1.1)
        if clear<0.3: continue
        dd=abs(((ang-bearing)+180)%360-180)
        score=dd - min(clear,1.0)*20
        if best is None or score<best[0]: best=(score,i,ang,clear)
    if best is None:
        w('d1',-25); w('d7',25); time.sleep(0.5); continue
    _,i,ang,clear=best
    e=((ang-h)+180)%360-180   # 22.5*i normalized
    if abs(e)>30:
        s=max(8,min(20,abs(e)*0.3))
        if e>0: w('d1',s); w('d7',-s)
        else: w('d1',-s); w('d7',s)
        time.sleep(0.25)
    else:
        sp=40 if min(Lc[0],Lc[1],Lc[15])>0.35 else 22
        steer=max(-12,min(12,e*0.4))
        w('d1',sp+steer); w('d7',sp-steer)
        time.sleep(0.15)
finally:
  stop()
  print("end pose",round(odo.x,2),round(odo.y,2),"d11",r('d11'),"st",status())
