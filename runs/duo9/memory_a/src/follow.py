import time, math, json, sys, random
sys.path.insert(0,'/bot/src')
from bot import *
M=0.00078
LOG=open('/memory/maplog.jsonl','a',buffering=1)
def log(**kw):
    kw['t']=round(time.time(),1); LOG.write(json.dumps(kw)+'\n')
class Odo:
    def __init__(s):
        s.x=0.0; s.y=0.0; s.el,s.er=enc()
    def update(s):
        el,er=enc(); h=heading()
        d=((el-s.el)+(er-s.er))/2.0*M
        s.el,s.er=el,er
        th=math.radians(h)
        s.x+=d*math.cos(th); s.y+=d*math.sin(th)
        return h
odo=Odo()
visited={}
import collections
d5hist=collections.deque(maxlen=150)
grad=[0.0,0.0,False]
def upd_grad():
    n=len(d5hist)
    if n<40: grad[2]=False; return
    xs=[p[0] for p in d5hist]; ys=[p[1] for p in d5hist]; vs=[p[2] for p in d5hist]
    mx=sum(xs)/n; my=sum(ys)/n; mv=sum(vs)/n
    sxx=sum((a-mx)**2 for a in xs); syy=sum((a-my)**2 for a in ys)
    if sxx<0.05 or syy<0.05: grad[2]=False; return
    sxy=sum((a-mx)*(b-my) for a,b in zip(xs,ys))
    svx=sum((a-mx)*(v-mv) for a,v in zip(xs,vs))
    svy=sum((b-my)*(v-mv) for b,v in zip(ys,vs))
    det=sxx*syy-sxy*sxy
    if abs(det)<1e-6: grad[2]=False; return
    gx=(svx*syy-svy*sxy)/det; gy=(svy*sxx-svx*sxy)/det
    grad[0],grad[1]=gx,gy; grad[2]=True
def vk(x,y): return (round(x/0.5),round(y/0.5))
def mn(l,idx):
    v=[l[i] for i in idx if l[i]>=0]
    return min(v) if v else 0.15   # treat all -1 as very close
state={'tx':0,'log':0,'save':0}
def housekeep(L,note=None):
    h=odo.update() if True else 0
    st=status(); now=time.time()
    visited[vk(odo.x,odo.y)]=visited.get(vk(odo.x,odo.y),0)+1
    if st.get('here') or st.get('goal'):
        speed(0,0); tx('GOALFOUND B'); log(note='HERE',st=st,x=odo.x,y=odo.y)
        return True
    if now-state['tx']>2:
        tx(json.dumps({'who':'B','x':round(odo.x,2),'y':round(odo.y,2),'d5':rd('d5')}))
        state['tx']=now
    if now-state['log']>1.0:
        v=rd('d5')
        try: d5hist.append((odo.x,odo.y,float(v)))
        except: pass
        upd_grad()
        log(x=round(odo.x,2),y=round(odo.y,2),h=round(h,1),L=L,d5=v,d2=rd('d2'),d9=rd('d9'),n=note,g=[round(grad[0],2),round(grad[1],2)] if grad[2] else None); state['log']=now
    if now-state['save']>15:
        json.dump({'x':odo.x,'y':odo.y},open('/memory/pose.json','w')); state['save']=now
    return False
hist=[]
def static_too_long(L,secs=8.0):
    now=time.time(); hist.append((now,L))
    while hist and now-hist[0][0]>secs+0.5: hist.pop(0)
    if hist and now-hist[0][0]>secs:
        old=hist[0][1]
        diff=sum(abs(a-b) for a,b in zip(old,L) if a>=0 and b>=0)
        if diff<0.5: hist.clear(); return True
    return False
def rot_pulse(dir,power=20,dur=0.3):
    # dir=+1: shift features to lower idx (l>r). dir=-1: higher idx.
    speed(power*dir,-power*dir); time.sleep(dur); speed(0,0); time.sleep(0.12)
def choose(L,h):
    best=None; bsc=-1e9
    for i in range(16):
        d=L[i]
        if d<0: continue
        if d<0.4: continue
        th=math.radians((h+22.5*i)%360)
        px=odo.x+min(d,1.0)*math.cos(th); py=odo.y+min(d,1.0)*math.sin(th)
        v=visited.get(vk(px,py),0)
        sc=min(d,2.0)-0.15*min(v,10)-min(i,16-i)*0.04+random.uniform(0,0.05)
        if grad[2]:
            gm=math.hypot(grad[0],grad[1])
            if gm>1e-6:
                w=max(0.0,(0.88-(d5hist[-1][2] if d5hist else 0.5)))*12
                sc+=w*(math.cos(th)*grad[0]+math.sin(th)*grad[1])/gm
        if sc>bsc: bsc=sc; best=i
    return best
def main():
    rot_fail=0
    tgt=[None,0]
    while True:
        L=lidar()
        if not L or len(L)<16: continue
        if housekeep(L): time.sleep(0.5); continue
        if static_too_long(L):
            log(note='escape')
            speed(-40,-40); time.sleep(1.0)
            d=random.choice([1,-1])
            speed(200*d,-200*d); time.sleep(0.4)
            speed(0,0); time.sleep(0.2)
            continue
        f=mn(L,(15,0,1)); b=mn(L,(7,8,9))
        # de-crowd
        if f<0.24 and b>0.28:
            speed(-25,-25); time.sleep(0.35); speed(0,0); continue
        if b<0.24 and f>0.28:
            speed(25,25); time.sleep(0.35); speed(0,0); continue
        h=heading()
        now=time.time()
        if tgt[0] is not None and now<tgt[1]:
            i=int(round((((tgt[0]-h)%360)/22.5)))%16
            if L[i]<0.35: i=None; tgt[0]=None
        else:
            i=choose(L,h)
            if i is not None:
                tgt[0]=(h+22.5*i)%360; tgt[1]=now+4.0
        if i is None:
            # nothing open: escape
            log(note='boxed')
            speed(-30,-30); time.sleep(0.8); speed(0,0); continue
        if i in (15,0,1) and L[0]>=0.4:
            # drive forward, centering
            l1=L[1] if L[1]>=0 else 0.15; r15=L[15] if L[15]>=0 else 0.15
            corr=(l1-r15)*35    # if left(idx1?) side more open, turn toward it slightly
            corr=max(-10,min(10,corr))
            base=40 if f>0.7 else 24
            speed(base+corr,base-corr)
            time.sleep(0.25)
            rot_fail=0
        else:
            dir=1 if (1<=i<=8) else -1
            pre=L[:]
            rot_pulse(dir, 20 if rot_fail<3 else 150, 0.3)
            L2=lidar()
            diff=sum(abs(a-c) for a,c in zip(pre,L2) if a>=0 and c>=0)
            rot_fail = rot_fail+1 if diff<0.35 else 0
if __name__=='__main__':
    try: main()
    finally: speed(0,0)
