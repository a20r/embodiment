import time, math, statistics, threading, json
D='/dev/robot/'
def readl(p,tries=3):
    for _ in range(tries):
        try:
            with open(D+p) as f: s=f.read()
            lines=[x.strip() for x in s.split('\n') if x.strip()]
            if lines: return lines[-1]
        except Exception: pass
        time.sleep(0.02)
    return None
def fnum(p,tries=3):
    s=readl(p,tries)
    try: return float(s)
    except: return None
def lid():
    s=readl('d2')
    try: return [float(x) for x in s.split(',')]
    except: return None
def motor(l,r):
    try:
        with open(D+'d1','w') as f: f.write(f"{l:.1f}\n")
        with open(D+'d7','w') as f: f.write(f"{r:.1f}\n")
    except Exception: pass
def tx(m):
    try:
        with open(D+'d8','w') as f: f.write(m+"\n")
    except Exception: pass
def angdiff(a,b): return (a-b+180)%360-180
RX=[]
def listener():
    while True:
        try:
            with open(D+'d10') as f:
                s=f.read().strip()
                if s:
                    RX.append((time.time(),s))
                    with open('/memory/rx.log','a') as g: g.write(s+"\n")
                    print("RX:",s,flush=True)
        except Exception: pass
        time.sleep(0.02)
threading.Thread(target=listener,daemon=True).start()
def d11(n=3):
    vs=[]
    for _ in range(n):
        v=fnum('d11')
        if v is not None: vs.append(v)
        time.sleep(0.04)
    return statistics.median(vs) if vs else None
def rot_by(delta,spd=20):
    h0=fnum('d4')
    if h0 is None: return
    tgt=(h0+delta)%360
    t0=time.time()
    while time.time()-t0 < abs(delta)/9+1.5:
        h=fnum('d4')
        if h is None: motor(0,0); time.sleep(0.05); continue
        err=angdiff(tgt,h)
        if abs(err)<5: break
        motor(spd if err>0 else -spd, -spd if err>0 else spd); time.sleep(0.02)
    motor(0,0); time.sleep(0.1)
def front_clear():
    B=lid()
    if not B: return 9.9
    Bv=[v if v>0 else 9.9 for v in B]
    return min(Bv[15],Bv[0],Bv[1])
def advance(meters,spd=30,maxsec=3.0):
    r0=fnum('d6'); l0=fnum('d9')
    if r0 is None or l0 is None: return 0,True
    h_tgt=fnum('d4'); t0=time.time(); blocked=False
    while time.time()-t0<maxsec:
        r=fnum('d6'); l=fnum('d9')
        if r is None or l is None: continue
        if front_clear()<0.22: blocked=True; break
        h=fnum('d4')
        err=angdiff(h_tgt,h) if (h is not None and h_tgt is not None) else 0
        c=max(-10,min(10,err*1.4))
        motor(spd+c,spd-c); time.sleep(0.02)
    motor(0,0); time.sleep(0.1)
    r1=fnum('d6'); l1=fnum('d9')
    if None in (r1,l1): return 0,True
    return ((r1-r0)+(l1-l0))/2.0, blocked   # returns ticks

# local dead-reckon frame: integrate from start with KM=1 (unit*s), heading d4
pose={'x':0.0,'y':0.0,'t':time.time(),'r':None,'l':None}
TRILOG=open('/memory/tri.csv','a')
def snap(tag):
    r=fnum('d6'); l=fnum('d9'); h=fnum('d4'); v=d11(2)
    now=time.time()
    dt=now-pose['t']
    if pose['r'] is not None and r is not None and l is not None and dt>0:
        dr=(r-pose['r']); dl=(l-pose['l'])
        ds=(dr+dl)/2.0          # unit*s (KM=1)
        dhh=angdiff(h,pose['h']) if 'h' in pose and h is not None and pose['h'] is not None else 0
        # integrate with midpoint heading
        hm=pose['h']+dhh/2.0
        th=math.radians(hm)
        pose['x']+=ds*math.sin(th); pose['y']+=ds*math.cos(th)
    pose['r']=r; pose['l']=l; pose['h']=h; pose['t']=now
    TRILOG.write(f"{tag},{now:.2f},{pose['x']:.3f},{pose['y']:.3f},{h},{v},{(RX[-1][1] if RX else '')}\n")
    TRILOG.flush()
    return pose['x'],pose['y'],h,v

print("tri start",flush=True)
t_start=time.time(); tx_t=0; ask_i=0
try:
  while time.time()-t_start<2400:
    now=time.time()
    # drain RX: parse pings
    while RX:
        tt,s=RX.pop(0)
        if 'PING' in s and ('x=' in s):
            try:
                parts=dict(kv.split('=') for kv in s.split() if '=' in kv)
                px,py=float(parts['x']),float(parts['y'])
                # record sample: my local pose now (approx at ping time)
                x,y,h,v=snap('T')
                TRILOG.write(f"S,{tt:.2f},{px:.3f},{py:.3f},RX={s}\n"); TRILOG.flush()
            except Exception as e:
                print("parse err",e,s,flush=True)
    # talk every 2s
    if now-tx_t>2.0:
        tx_t=now; ask_i+=1
        if ask_i%5==1:
            tx("B1->B2: I trilaterate from your PINGs+d11. Please PING every 1-2s. HOLD STILL 30s if you can. Then both go GOAL x~0.4 y~-0.8. ACK?")
        else:
            v=d11(2)
            tx(f"B1 PING d11={v if v else -1:.3f} t={int(time.time())}")
    s3=readl('d3')
    if s3 and ('goal=1' in s3 or 'here=1' in s3):
        print("GOALFLAG",s3,flush=True)
        with open('/memory/goal_flag.log','a') as g: g.write(f"{time.time()} {s3}\n")
    # movement: periodic exploration moves to create trilateration geometry
    # L-shaped path segments, turning 90deg each time
    mvs,blk=advance(0.45)
    if blk:
        rot_by(90)
    else:
        rot_by(67)
    snap('M')
    time.sleep(0.2)
finally:
    motor(0,0)
