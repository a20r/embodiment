import time, math, json, sys
sys.path.insert(0,'/bot/src')
from bot import *

M_PER_COUNT=0.00078
LOG=open('/memory/maplog.jsonl','a',buffering=1)
def log(**kw):
    kw['t']=round(time.time(),1); LOG.write(json.dumps(kw)+'\n')

class Odo:
    def __init__(s):
        s.x=0.0; s.y=0.0; s.el,s.er=enc()
    def update(s,h):
        el,er=enc()
        d=((el-s.el)+(er-s.er))/2.0*M_PER_COUNT
        s.el,s.er=el,er
        th=math.radians(h)
        s.x+=d*math.cos(th); s.y+=d*math.sin(th)

odo=Odo()
visited={}
def vk(x,y): return (round(x/0.5),round(y/0.5))
tb=[0]
def tick():
    h=heading(); odo.update(h)
    st=status()
    now=time.time()
    if st.get('here') or st.get('goal'):
        speed(0,0); tx('AT_GOAL'); log(note='HERE',st=st)
        return h,st,True
    if now-tb[0]>2:
        tx('POS %.2f %.2f'%(odo.x,odo.y)); tb[0]=now
    return h,st,False

def turn_to(b,tol=7,timeout=10):
    t0=time.time()
    while time.time()-t0<timeout:
        h,st,g=tick()
        if g: return 'goal'
        err=(b-h+540)%360-180
        if abs(err)<=tol: speed(0,0); return 'ok'
        s=max(10,min(45,abs(err)*0.4))
        speed(s if err>0 else -s, -s if err>0 else s)
        time.sleep(0.06)
    speed(0,0); return 'timeout'

def escape():
    log(note='escape')
    import random
    for l,r,t in [(-200,-200,1.5),(random.choice([200,-200]),random.choice([-200,200]),1.2),(200,200,1.0)]:
        speed(l,r); time.sleep(t)
    speed(0,0)

def drive(bear,maxt=9):
    t0=time.time(); f0=None; lastprog=t0
    while time.time()-t0<maxt:
        h,st,g=tick()
        if g: return 'goal'
        L=lidar()
        front=min([x for x in (L[0],L[1],L[15]) if x>=0] or [2.5])
        if front<0.30: speed(0,0); return 'blocked'
        if f0 is None: f0=front
        if abs(front-f0)>0.15: f0=front; lastprog=time.time()
        if time.time()-lastprog>5: speed(0,0); return 'stuck'
        err=(bear-h+540)%360-180
        corr=max(-18,min(18,err*0.8))
        base=45 if front>0.7 else 25
        speed(base+corr,base-corr)
        time.sleep(0.07)
    speed(0,0); return 'time'

def choose(h,L):
    best=None; bestsc=-1e9
    for i in range(16):
        d=L[i]
        if d<0: d=2.5
        if d<0.35: continue
        b=(h+22.5*i)%360; th=math.radians(b)
        px=odo.x+min(d,1.0)*math.cos(th); py=odo.y+min(d,1.0)*math.sin(th)
        v=visited.get(vk(px,py),0)
        sc=min(d,2.0)-0.3*min(v,6)-min(i,16-i)*0.04
        if sc>bestsc: bestsc=sc; best=i
    return best

def main():
    nstuck=0
    while True:
        h,st,g=tick()
        if g: time.sleep(0.5); continue
        L=lidar()
        visited[vk(odo.x,odo.y)]=visited.get(vk(odo.x,odo.y),0)+1
        log(x=round(odo.x,2),y=round(odo.y,2),h=h,L=L,d5=rd('d5'))
        i=choose(h,L)
        if i is None:
            escape(); continue
        bear=(h+22.5*i)%360
        r=turn_to(bear)
        if r=='goal': continue
        r=drive(bear)
        if r in ('stuck',):
            nstuck+=1
            if nstuck>=2: escape(); nstuck=0
        else: nstuck=0

if __name__=='__main__':
    try: main()
    finally: speed(0,0)
