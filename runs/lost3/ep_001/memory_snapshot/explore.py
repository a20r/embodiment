# v6 heading-hold explorer, anti-revisit, frustration escape
import time, math, random
from collections import defaultdict, deque

def rd(d):
    try:
        with open(f'/dev/robot/d{d}') as f: return f.readline().strip()
    except Exception: return ''
def wr(d,s):
    with open(f'/dev/robot/d{d}','w') as f: f.write(str(s)+'\n')
def lidar_med(n=2):
    scans=[]
    for _ in range(n+2):
        s=rd(4)
        try:
            v=[float(x) for x in s.split(',')]
            if len(v)==16: scans.append(v)
        except: pass
        if len(scans)>=n: break
    out=[]
    for i in range(16):
        vals=sorted(x[i] for x in scans if x[i]>=0)
        out.append(vals[len(vals)//2] if vals else 2.0)
    return out
def heading(n=3):
    vals=[]
    for _ in range(n+2):
        try: vals.append(float(rd(1)))
        except: pass
        if len(vals)>=n: break
    vals.sort(); return vals[len(vals)//2] if vals else 0.0
def stop(): wr(5,0); wr(6,0)
log=open('/memory/telemetry.csv','a')
def LOG(*a):
    log.write(','.join(str(x) for x in a)+'\n'); log.flush()

V=0.0025; RES=0.25
x,y=0.0,0.0
visits=defaultdict(int)
t0=time.time()
LOG('RESTART_V6', time.strftime('%H:%M:%S'))
last_cmd=(0,0); last_t=time.time(); cur_h=0.0
recent=deque(maxlen=45)

def setw(l,r):
    global last_cmd,last_t,x,y
    now=time.time(); dt=now-last_t
    v=V*(last_cmd[0]+last_cmd[1])/2
    hr=math.radians(cur_h)
    x+=v*dt*math.cos(hr); y+=v*dt*math.sin(hr)
    last_t=now
    if (l,r)!=last_cmd:
        wr(5,l); wr(6,r); last_cmd=(l,r)
def cell(px,py): return (int(px//RES),int(py//RES))
def goal_check():
    s0=rd(0)
    if 'goal=1' in s0:
        stop(); LOG('GOAL',s0,round(x,2),round(y,2)); print('GOAL',s0); return True
    return False
def turn_to(target):
    global cur_h
    for _ in range(50):
        cur_h=heading()
        err=(target-cur_h+540)%360-180
        if abs(err)<8: break
        rate=max(14,min(40,abs(err)*0.8))
        setw(int(-rate) if err>0 else int(rate), int(rate) if err>0 else int(-rate))
        time.sleep(0.12)
    setw(0,0)

def choose_dir(scan, ignore_visits=False):
    best=None; bestscore=1e9
    for k in range(16):
        clr=min(scan[k], scan[(k+1)%16]*1.25, scan[(k-1)%16]*1.25)
        if clr<0.38: continue
        a=math.radians(cur_h+22.5*k)
        score=random.uniform(0,0.4)
        if not ignore_visits:
            for dstep in (0.3,0.6,1.0):
                dd=min(dstep, scan[k]-0.1)
                score+=visits[cell(x+dd*math.cos(a), y+dd*math.sin(a))]
            if 6<=k<=10: score+=1.0
        else:
            score-=clr
        if score<bestscore: bestscore=score; best=k
    if best is None: best=max(range(16),key=lambda k:scan[k])
    return best

target_h=None
try:
    while True:
        if goal_check(): break
        scan=lidar_med(2)
        cur_h=heading(); setw(*last_cmd)
        bump=rd(2)=='1'
        visits[cell(x,y)]+=1
        recent.append((x,y))
        LOG(round(time.time()-t0,1), round(x,2), round(y,2), round(cur_h,1), int(bump), rd(3), ';'.join(f'{v:.2f}' for v in scan))
        frustrated = len(recent)==recent.maxlen and (max(p[0] for p in recent)-min(p[0] for p in recent) < 0.35) and (max(p[1] for p in recent)-min(p[1] for p in recent) < 0.35)
        front=min(scan[0],scan[15],scan[1])
        if bump:
            LOG('ESCAPE','bump')
            setw(-30,-30); time.sleep(1.1); setw(0,0)
            visits[cell(x,y)]+=3
            scan=lidar_med(3)
            k=choose_dir(scan)
            target_h=(cur_h+22.5*k)%360
            turn_to(target_h)
            continue
        if frustrated:
            LOG('FRUSTRATED')
            recent.clear()
            scan=lidar_med(3)
            k=choose_dir(scan, ignore_visits=True)
            target_h=(cur_h+22.5*k)%360
            turn_to(target_h)
            # forced drive
            for _ in range(14):
                if goal_check(): raise SystemExit
                scan=lidar_med(1)
                cur_h=heading(2)
                f=min(scan[0],scan[15],scan[1])
                if rd(2)=='1' or f<0.22: break
                err=(target_h-cur_h+540)%360-180
                c=max(-12,min(12,err*0.8))
                b=int(max(30,min(85,60*f)))
                setw(int(b-c),int(b+c))
                time.sleep(0.35)
            continue
        if front<0.28:
            scan=lidar_med(3)
            k=choose_dir(scan)
            target_h=(cur_h+22.5*k)%360
            turn_to(target_h)
            continue
        if target_h is None: target_h=cur_h
        err=(target_h-cur_h+540)%360-180
        # allow gentle gap centering: nudge target by clearance balance
        l=sum(scan[1:3]); r=sum(scan[14:16])
        nudge=max(-8,min(8,(l-r)*10))
        c=max(-14,min(14,err*0.8+nudge))
        b=int(max(34,min(95,70*front)))
        setw(int(b-c),int(b+c))
        time.sleep(0.4)
finally:
    stop(); LOG('END_V6',round(time.time()-t0,1))
