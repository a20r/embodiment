# v2
import time, math, random

def rd(d):
    try:
        with open(f'/dev/robot/d{d}') as f: return f.readline().strip()
    except Exception: return ''
def wr(d,s):
    with open(f'/dev/robot/d{d}','w') as f: f.write(str(s)+'\n')

def lidar():
    for _ in range(3):
        s = rd(4)
        if s:
            try:
                v=[float(x) for x in s.split(',')]
                if len(v)==16: return v
            except: pass
    return None

def lidar_med(n=3):
    scans=[]
    for _ in range(n):
        v=lidar()
        if v: scans.append(v)
    out=[]
    for i in range(16):
        vals=sorted(x[i] for x in scans if x[i]>=0)
        out.append(vals[len(vals)//2] if vals else 1.0)
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

random.seed()

def turn_deg(ang):
    if ang>180: ang-=360
    if ang<-180: ang+=360
    dur = abs(ang)/40.0
    if ang>0: wr(5,-20); wr(6,20)
    else: wr(5,20); wr(6,-20)
    time.sleep(dur); stop()

t0=time.time()
prev_scan=None; still=0
LOG('RESTART', time.strftime('%H:%M:%S'))
try:
    while True:
        s0 = rd(0); d3 = rd(3)
        if 'goal=1' in s0:
            stop(); LOG('GOAL', s0); print("GOAL", s0); break
        scan = lidar_med(2)
        h = heading()
        bump = rd(2)=='1'
        LOG(round(time.time()-t0,1), s0.split()[0] if s0 else '?', round(h,1), int(bump), d3, ';'.join(f'{v:.2f}' for v in scan))
        if prev_scan and sum(abs(a-b) for a,b in zip(scan,prev_scan))<0.6:
            still += 1
        else:
            still = 0
        prev_scan = scan
        front = min(scan[0], scan[1], scan[15])
        if bump or front < 0.25 or still>=4:
            LOG('ESCAPE', 'bump' if bump else ('stuck' if still>=4 else 'front'))
            still=0; prev_scan=None
            wr(5,-30); wr(6,-30); time.sleep(1.2); stop()
            scan = lidar_med(3)
            idxs = sorted(range(16), key=lambda i:-scan[i])
            top = [i for i in idxs[:5] if scan[i]>0.5] or idxs[:3]
            choice = random.choice(top)
            turn_deg(22.5*choice + random.uniform(-8,8))
        else:
            l = sum(scan[1:4]); r = sum(scan[13:16])
            steer = max(-12, min(12, (l-r)*15))
            wr(5, 35-steer); wr(6, 35+steer)
            time.sleep(0.5)
finally:
    stop(); LOG('END', round(time.time()-t0,1))
