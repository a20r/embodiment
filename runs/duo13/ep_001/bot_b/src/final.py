import time, math, statistics, threading
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
                    RX.append(s)
                    with open('/memory/rx.log','a') as g: g.write(s+"\n")
                    if 'GOAL' in s.upper(): print("RX:",s,flush=True)
        except Exception: pass
        time.sleep(0.02)
threading.Thread(target=listener,daemon=True).start()
def d11(n=2):
    vs=[]
    for _ in range(n):
        v=fnum('d11')
        if v is not None: vs.append(v)
        time.sleep(0.03)
    return statistics.median(vs) if vs else None
def rot_to(tgt,spd=26,maxt=2.5):
    t0=time.time()
    while time.time()-t0<maxt:
        h=fnum('d4')
        if h is None: motor(0,0); time.sleep(0.05); continue
        e=angdiff(tgt,h)
        if abs(e)<6: break
        motor(spd if e>0 else -spd, -spd if e>0 else spd); time.sleep(0.02)
    motor(0,0)
LOG=open('/memory/final.csv','a')
print("FINAL descent start",flush=True)
tx("B1->B2: I am coming! FREEZE AT GOAL! Do not move!")
t_start=time.time(); tx_t=0
ema=None; side=1; lastprog=0
try:
  while time.time()-t_start<1500:
    now=time.time()
    while RX:
        m=RX.pop(0)
        if 'GOAL' in m.upper(): print("B2:",m,flush=True)
    if now-tx_t>3.0:
        tx_t=now
        tx(f"B1 homing d11={ema if ema else -1:.2f} B2 STAY FROZEN AT GOAL!")
    s3=readl('d3')
    if s3 and ('goal=1' in s3 or 'here=1' in s3):
        print("GOALFLAG!!!",s3,flush=True)
        with open('/memory/goal_flag.log','a') as g: g.write(f"{time.time()} {s3}\n")
    v=d11(2)
    if v is not None:
        ema=v if ema is None else 0.65*ema+0.35*v
        LOG.write(f"{now-t_start:.0f},{v:.3f},{ema:.3f}\n"); LOG.flush()
    if v is not None and v<0.17:
        print("*** CONTACT: d11=%.3f — HOLDING, blasting ***"%v,flush=True)
        tx("B1 ARRIVED! GOAL!")
        motor(0,0)
        t0=time.time()
        while time.time()-t0<30:
            tx("B1 AT YOU! GOAL!")
            time.sleep(0.6)
        continue
    B=lid()
    Bv=[b if b>0 else 9.9 for b in B] if B else None
    front=min(Bv[15],Bv[0],Bv[1]) if Bv else 9.9
    if v is not None and v<ema-0.004:
        lastprog=now
    if now-lastprog>4.0:
        # no progress: turn around-ish
        side=-side
        h=fnum('d4')
        if h is not None: rot_to((h+side*70)%360)
        ema=v; lastprog=now
        continue
    if front<0.24:
        motor(0,0)
        if Bv:
            bi=max(range(16),key=lambda i:Bv[i])
            h=fnum('d4')
            if h is not None: rot_to((h+22.5*bi)%360)
        continue
    motor(34,34)
    time.sleep(0.05)
finally:
    motor(0,0)
