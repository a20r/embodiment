import time, math, json, sys
sys.path.insert(0,'/bot/src')
from lib import *

TICKS_PER_M = 1600.0
def status():
    s=r('d3')  # tick=N goal=0 here=0
    d={}
    for kv in s.split():
        k,v=kv.split('='); d[k]=int(v)
    return d

class Odo:
    def __init__(self):
        self.l=ri('d9'); self.r=ri('d6')
        self.x=0.0; self.y=0.0
    def update(self):
        l=ri('d9'); rr=ri('d6')
        dl=(l-self.l)/TICKS_PER_M; dr=(rr-self.r)/TICKS_PER_M
        self.l=l; self.r=rr
        d=(dl+dr)/2
        h=math.radians(heading())
        self.x+=d*math.cos(h); self.y+=d*math.sin(h)
        return d

odo=Odo()
log=open('/bot/src/map.log','a')
BASE=40
t_start=time.time()
last_log=0
mode='follow'
try:
  while time.time()-t_start < 50:
    L=lidar()
    st=status()
    odo.update()
    now=time.time()
    log.write(json.dumps({'t':round(now,1),'x':round(odo.x,3),'y':round(odo.y,3),
        'h':heading(),'L':L,'st':st,'d11':r('d11')})+'\n'); log.flush()
    if st.get('here') or st.get('goal'):
        print("STATUS FLAG!", st); stop(); break
    if bump():
        w('d1',-30); w('d7',-30); time.sleep(0.7)
        w('d1',-20); w('d7',20); time.sleep(0.8)  # turn left
        stop(); continue
    # replace -1 with 1.6
    L=[x if x>0 else 1.6 for x in L]
    front=min(L[0],L[1]*1.2,L[15]*1.2)
    right=min(L[3],L[4],L[5])
    if front<0.22:
        # turn left in place until front clear
        w('d1',-25); w('d7',25); time.sleep(0.4); continue
    # wall follow right: keep right dist ~0.25
    err=0.25-right   # >0 too close -> steer left(=heading-? ) 
    # heading+ = d1 fwd more. right side is beam4 = heading+90 (our convention)
    # to steer away from right wall (increase right dist) we want heading decrease? 
    # beam4 at h+90: moving heading- points front away from that wall -> right dist grows? Actually turning
    # heading- rotates beams: feature moves to higher index; wall at 4 -> 5.. still right side.
    # Steering: to move away from wall on right(beam4), turn left = heading such that front points away.
    # front beam0 at angle h. wall at h+90. turn left means decrease angle toward wall? Use sign empirically-ish:
    steer = max(-15,min(15, err*60))
    # steer>0 => turn away from right wall. try: away from beam4 direction = heading decrease => d1 slower than d7
    w('d1', BASE - steer); w('d7', BASE + steer)
    time.sleep(0.15)
finally:
  stop()
  print("done", status(), "pose", odo.x, odo.y)
