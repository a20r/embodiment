import robot as R, ctl, time, math, random, json, os
from collections import defaultdict, deque
hist=deque(maxlen=15)

LOG=open('/bot/src/explore.log','a',buffering=1)
SCAN=open('/bot/src/scan.log','a',buffering=1)
def log(*a):
    LOG.write(time.strftime('%H:%M:%S ')+' '.join(str(x) for x in a)+'\n')

TICK_M=ctl.TICK_M
pose=[0.0,0.0]
visits=defaultdict(int)
CELL=0.5
STATE='/memory/state.json'
if os.path.exists(STATE):
    try:
        st=json.load(open(STATE))
        pose=st['pose']
        for k,v in st['visits']: visits[tuple(k)]=v
    except Exception as e:
        log('state load fail',e)

def save_state():
    json.dump({'pose':pose,'visits':[[list(k),v] for k,v in visits.items()]},open(STATE,'w'))

def cell(x,y): return (round(x/CELL), round(y/CELL))

def rayval(l,i):
    v=l[i%16]
    return 3.0 if v<0 else v

def forward_hh(dist, v=35, min_front=0.20):
    h0=R.heading()
    e0=sum(R.enc())/2
    t0=time.time(); reason='dist'
    last_e=e0; last_t=t0
    while True:
        gone=(sum(R.enc())/2-e0)*TICK_M
        if gone>=dist: break
        if time.time()-t0>max(10,dist/0.08): reason='timeout'; break
        l=R.lidar()
        f=min(rayval(l,12), rayval(l,11)+0.08, rayval(l,13)+0.08)
        if f<min_front: reason='wall'; break
        h=R.heading()
        err=ctl.angdiff(h0,h)
        adj=max(-8,min(8,err*0.5))
        loA=min(rayval(l,13),rayval(l,14),rayval(l,15))
        loB=min(rayval(l,9),rayval(l,10),rayval(l,11))
        if loA<0.12: adj-=5
        if loB<0.12: adj+=5
        s=v if f>0.45 else 14
        R.drive(s+adj,s-adj)
        if time.time()-last_t>1.2:
            e=sum(R.enc())/2
            if abs(e-last_e)<10: reason='stuck'; break
            if 'lastl' in dir():
                ds=[abs(a-b) for a,b in zip(l,lastl) if 0<a<2.8 and 0<b<2.8]
                if len(ds)>=4 and max(ds)<0.05:
                    reason='slip'; break
            lastl=l
            last_e=e; last_t=time.time()
        time.sleep(0.06)
    R.stop()
    gone=(sum(R.enc())/2-e0)*TICK_M
    ha=math.radians((h0+R.heading())/2)
    pose[0]+=gone*math.cos(ha); pose[1]+=gone*math.sin(ha)
    n=max(1,int(gone/0.25))
    for k in range(n+1):
        x=pose[0]-gone*math.cos(ha)*(n-k)/n; y=pose[1]-gone*math.sin(ha)*(n-k)/n
        visits[cell(x,y)]+=1
    return gone, reason

def d5avg(n=4):
    v=0.0
    for _ in range(n):
        v+=float(R.read('d5')); time.sleep(0.05)
    return v/n

def grad():
    if len(hist)<6: return None
    xs=[p[0] for p in hist]; ys=[p[1] for p in hist]; vs=[p[2] for p in hist]
    mx=sum(xs)/len(xs); my=sum(ys)/len(ys); mv=sum(vs)/len(vs)
    sxx=sum((x-mx)**2 for x in xs); syy=sum((y-my)**2 for y in ys)
    sxy=sum((x-mx)*(y-my) for x,y in zip(xs,ys))
    sxv=sum((x-mx)*(v-mv) for x,v in zip(xs,vs))
    syv=sum((y-my)*(v-mv) for y,v in zip(ys,vs))
    den=sxx*syy-sxy*sxy
    if abs(den)<1e-6: return None
    gx=(syy*sxv-sxy*syv)/den; gy=(sxx*syv-sxy*sxv)/den
    m=math.hypot(gx,gy)
    if m<0.004: return None
    return gx/m, gy/m

def choose_dir(l,h):
    best=None
    for i in range(16):
        dr=rayval(l,i)
        if dr<0.5: continue
        a=math.radians(h+(i-12)*22.5)
        step=min(dr-0.25,2.0)
        ca,sa=math.cos(a),math.sin(a)
        vsum=0; n=max(1,int(step/0.25))
        for k in range(1,n+1):
            vsum+=visits[cell(pose[0]+ca*step*k/n, pose[1]+sa*step*k/n)]
        vend=visits[cell(pose[0]+ca*step, pose[1]+sa*step)]
        rel=abs(((i-12+8)%16)-8)
        backpen=3.0 if rel>=6 else 0.0
        score=vend*3+vsum/n+backpen-min(dr,2.5)*0.4
        g=GRAD
        if g: score-=2.0*(ca*g[0]+sa*g[1])
        if best is None or score<best[0]:
            best=(score,i)
    if best is None:
        return max(range(16), key=lambda i: rayval(l,i))
    return best[1]

step=0
GRAD=None
while True:
    step+=1
    l=R.lidar(); h=R.heading()
    GRAD=grad()
    i=choose_dir(l,h)
    ctl.face_ray(i)
    l2=R.lidar()
    d=min(rayval(l2,12)-0.25, 2.0)
    if d<0.05: d=0.05
    gone,reason=forward_hh(d)
    if reason in ('slip','stuck'):
        R.drive(-18,-18); time.sleep(1.2); R.stop()
        e=None
    d5v=d5avg(); d5='%.3f'%d5v; hist.append((pose[0],pose[1],d5v))
    d6=R.status(); d9=R.read('d9')
    R.tx('R1 pos %.2f %.2f'%(pose[0],pose[1]))
    m=R.rx()
    SCAN.write(json.dumps([round(pose[0],2),round(pose[1],2),round(R.heading(),1),l2])+'\n')
    log('step',step,'ray',i,'got %.2f'%gone,reason,
        'pose %.2f %.2f'%(pose[0],pose[1]),'h %.1f'%h,'d5',d5,'d9',d9,d6,'rx:',m)
    if m: log('RX!',m)
    if step%5==0: save_state()
    if 'here=1' in d6 or 'goal=1' in d6:
        log('*** STATUS',d6); save_state()
