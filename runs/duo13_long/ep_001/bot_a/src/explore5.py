import time, math, signal, sys, json, threading
from collections import deque
from robot import Robot

r = Robot()
def stopall(*a):
    r.stop(); sys.exit(0)
signal.signal(signal.SIGTERM, stopall)
signal.signal(signal.SIGINT, stopall)

logf = open('/bot/src/explore5.log','a', buffering=1)
def log(*a):
    logf.write(f'[{time.strftime("%H:%M:%S")}] ' + ' '.join(str(x) for x in a) + '\n')

TICKS=544.0
x=y=0.0
h = r.heading() or 0.0
r.stop(); time.sleep(0.3)
le, re = r.enc()

CELL=0.12
occ={}   # (gx,gy) -> [hits,misses]
soft = {} # edge blacklist: (cellA,cellB) -> time
def key(px,py): return (int(math.floor(px/CELL)), int(math.floor(py/CELL)))
def center(k): return ((k[0]+0.5)*CELL, (k[1]+0.5)*CELL)
def mark_free(x0,y0,x1,y1):
    dx,dy=x1-x0,y1-y0
    n=max(1,int(math.hypot(dx,dy)/CELL))
    for i in range(n):
        t=i/n
        c=occ.setdefault(key(x0+dx*t,y0+dy*t),[0,0]); c[1]+=1
def state(k):
    hh,mm=occ.get(k,[0,0])
    if hh+mm<2: return 0  # unknown
    if hh>=2 and hh>0.35*(hh+mm): return 2  # occ
    return 1  # free
def update_pose():
    global x,y,h,le,re
    l2,r2=r.enc()
    if l2 is None: return
    h2=r.heading()
    if h2 is None: return
    dl=(l2-le)/TICKS; dr=(r2-re)/TICKS
    le,re=l2,r2; h=h2
    d=(dl+dr)/2
    rad=math.radians(h)
    x+=d*math.sin(rad); y+=d*math.cos(rad)

def scan_update():
    lid=r.lidar()
    if not lid: return
    for i,d in enumerate(lid):
        if d is None or d<0.15: continue
        az=math.radians(h+i*22.5)
        ex=x+d*math.sin(az); ey=y+d*math.cos(az)
        if d<2.4:
            c=occ.setdefault(key(ex,ey),[0,0]); c[0]+=1
            mark_free(x,y,x+(d-0.08)*math.sin(az),y+(d-0.08)*math.cos(az))
        else:
            mark_free(x,y,x+2.3*math.sin(az),y+2.3*math.cos(az))

def neighbors(k):
    x0,y0=k
    for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
        yield (x0+dx,y0+dy)

def passable(a,b):
    if state(b)!=1: return False
    if time.time() < soft.get((a,b),0): return False
    return True

def bfs(start, goal_test):
    prev={start:None}
    q=deque([start])
    while q:
        k=q.popleft()
        if goal_test(k):
            path=[]
            while k is not None:
                path.append(k); k=prev[k]
            return path[::-1]
        for n in neighbors(k):
            if n in prev: continue
            if passable(k,n):
                prev[n]=k; q.append(n)
    return None

def find_frontier_path(start):
    return bfs(start, lambda k: state(k)==1 and any(state(n)==0 for n in neighbors(k)))

def clean(lid):
    out=[]
    for i,v in enumerate(lid):
        if v is None or v<0:
            a=lid[(i-1)%16]; b=lid[(i+1)%16]
            cc=[z for z in (a,b) if z and z>0]
            v=min(cc) if cc else 0.4
        out.append(v)
    return out

def rotto(tg, timeout=8, tol=5):
    t0=time.time()
    while time.time()-t0<timeout:
        hc=r.heading()
        if hc is None: continue
        e=(tg-hc+180)%360-180
        if abs(e)<=tol: break
        v=max(10,min(28,abs(e)*1.1))
        if e>0: r.wheels(v,-v)
        else: r.wheels(-v,v)
        time.sleep(0.04)
    r.stop(); time.sleep(0.1)

def goto(tx,ty,timeout=10):
    """drive toward world point with safety; returns True if reached"""
    global x,y
    t0=time.time()
    while time.time()-t0<timeout:
        dx=tx-x; dy=ty-y
        if math.hypot(dx,dy)<0.10: return True
        tg=math.degrees(math.atan2(dx,dy))%360
        rotto(tg,timeout=4,tol=7)
        l0,r0=r.enc()
        t1=time.time()
        while time.time()-t1<2.5:
            update_pose()
            dx=tx-x; dy=ty-y
            if math.hypot(dx,dy)<0.10: r.stop(); return True
            lid=r.lidar()
            if lid:
                c=clean(lid)
                f=min(c[15],c[0],c[1])
                if f<0.17: r.stop(); return False
                sp=20 if f>0.35 else 11
                err=(min(c[12],c[13])-min(c[3],c[4]))
                corr=max(-6,min(6,err*18))
                r.wheels(sp-corr,sp+corr)
            else:
                r.wheels(14,14)
            time.sleep(0.03)
        r.stop()
        if time.time()-t0>timeout-1: break
    update_pose()
    return math.hypot(tx-x,ty-y)<0.14

def radio():
    n=0
    while True:
        r.send(f'A PING {x:.1f} {y:.1f} {h:.0f} n={n}')
        n+=1
        t0=time.time()
        while time.time()-t0<7:
            v=r.recv(0.4)
            if v: log('RADIO RX:', v)
            time.sleep(0.2)
threading.Thread(target=radio, daemon=True).start()

log('=== explore5 start ===')
try:
    for round_i in range(80):
        st=r.status()
        if st and (st[1] or st[2]):
            log('FLAG!', st, f'pose {x:.2f},{y:.2f} h={h:.0f}')
            json.dump({'x':x,'y':y,'h':h,'flags':st}, open('/memory/goal_found.json','w'))
            break
        update_pose()
        scan_update()
        start=key(x,y)
        if state(start)!=1:
            # we're inside a smear: force current cell free
            occ.setdefault(start,[0,10])
        path=find_frontier_path(start)
        if path is None or len(path)<2:
            log(f'round {round_i}: no frontier from {x:.2f},{y:.2f}; wander back off')
            rotto((h+180)%360); 
            l0,r0=r.enc(); t0=time.time()
            while time.time()-t0<3:
                l1,r1=r.enc()
                if l1 is None: continue
                if ((l1-l0)+(r1-r0))/2>=0.2*TICKS: break
                r.wheels(-12,-12); time.sleep(0.05)
            r.stop()
            continue
        # follow path
        log(f'round {round_i}: path len {len(path)} from ({x:.2f},{y:.2f})')
        for k in path[1:]:
            tx,ty=center(k)
            ok=goto(tx,ty)
            update_pose(); scan_update()
            if not ok:
                prev=k
                soft[(key(x,y),k)]=time.time()+25
                log(f'  blocked at ({x:.2f},{y:.2f}) -> {k}; replan')
                break
        else:
            log(f'  path done at ({x:.2f},{y:.2f})')
        if round_i%5==0:
            log(f'round {round_i} pose {x:.2f},{y:.2f} h={h:.0f} batt={r.battery()} cells={len(occ)}')
            json.dump({'x':x,'y':y,'h':h,'occ':{f'{a},{b}':v for (a,b),v in occ.items()}}, open('/bot/src/map5.json','w'))
except Exception as e:
    import traceback; log('FATAL', traceback.format_exc())
finally:
    r.stop()
    json.dump({'x':x,'y':y,'h':h,'occ':{f'{a},{b}':v for (a,b),v in occ.items()}}, open('/bot/src/map5.json','w'))
    log('=== end ===', f'{x:.2f},{y:.2f} h={h:.0f}')
