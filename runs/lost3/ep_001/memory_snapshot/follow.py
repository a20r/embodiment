# v4 alternating wall follower, dynamic speed
import time, math, random

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

def heading():
    vals=[]
    for _ in range(3):
        try: vals.append(float(rd(1)))
        except: pass
    vals.sort()
    return vals[len(vals)//2] if vals else 0.0

def stop(): wr(5,0); wr(6,0)
log = open('/memory/telemetry.csv','a')
def LOG(*a):
    log.write(','.join(str(x) for x in a)+'\n'); log.flush()

V=0.0025
x,y=0.0,0.0
t0=time.time()
LOG('RESTART_V4', time.strftime('%H:%M:%S'))
last_cmd=(0,0); last_t=time.time()
prev_scan=None; still=0
cur_h=0.0

def setw(l,r):
    global last_cmd,last_t,x,y
    now=time.time(); dt=now-last_t
    v=V*(last_cmd[0]+last_cmd[1])/2
    hr=math.radians(cur_h)
    x+=v*dt*math.cos(hr); y+=v*dt*math.sin(hr)
    last_t=now
    if (l,r)!=last_cmd:
        wr(5,l); wr(6,r); last_cmd=(l,r)

cur_h=heading()
mode_t=time.time(); hand=1   # 1=right wall, -1=left wall
try:
    while True:
        if time.time()-mode_t>150:
            hand=-hand; mode_t=time.time(); LOG('HAND',hand)
        s0=rd(0)
        if 'goal=1' in s0:
            stop(); LOG('GOAL',s0); print('GOAL',s0); break
        scan=lidar_med(2)
        cur_h=heading()
        bump=rd(2)=='1'
        d3=rd(3)
        LOG(round(time.time()-t0,1), round(x,2), round(y,2), round(cur_h,1), int(bump), d3, ';'.join(f'{v:.2f}' for v in scan))
        if prev_scan and sum(abs(a-b) for a,b in zip(scan,prev_scan))<0.6 and last_cmd!=(0,0):
            still+=1
        else: still=0
        prev_scan=scan
        front=min(scan[0],scan[1],scan[15])
        if hand==1:
            side=min(scan[12],scan[13]); sfront=scan[14]
        else:
            side=min(scan[3],scan[4]); sfront=scan[2]
        if bump or still>=4:
            LOG('ESCAPE','bump' if bump else 'stuck')
            still=0; prev_scan=None
            setw(-30,-30); time.sleep(1.0); setw(0,0)
            a=random.uniform(60,120)
            if hand==1: setw(-20,20)
            else: setw(20,-20)
            time.sleep(a/40.0); setw(0,0)
            continue
        if front<0.3:
            if hand==1: setw(-25,25)
            else: setw(25,-25)
            time.sleep(0.35)
            continue
        base=int(max(28,min(85, 55*front)))
        if side>0.9 and sfront>0.9:
            # lost wall: arc toward it
            setw(base, int(base*0.45)) if hand==1 else setw(int(base*0.45), base)
            time.sleep(0.4); continue
        err=side-0.35
        steer=max(-10,min(10, err*30))*hand
        if sfront<0.3: steer=(-8 if hand==1 else 8)
        setw(int(base+steer), int(base-steer))
        time.sleep(0.4)
finally:
    stop(); LOG('END_V4', round(time.time()-t0,1))
