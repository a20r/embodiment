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
        except Exception: pass
        time.sleep(0.02)
threading.Thread(target=listener,daemon=True).start()
def d11(n=3):
    vs=[]
    for _ in range(n):
        v=fnum('d11')
        if v is not None: vs.append(v)
        time.sleep(0.03)
    return statistics.median(vs) if vs else None
def rot_to(tgt,spd=28,maxt=2.4):
    t0=time.time()
    while time.time()-t0<maxt:
        h=fnum('d4')
        if h is None: motor(0,0); time.sleep(0.05); continue
        e=angdiff(tgt,h)
        if abs(e)<5: break
        motor(spd if e>0 else -spd, -spd if e>0 else spd); time.sleep(0.02)
    motor(0,0); time.sleep(0.05)
def burst(meters,spd=40,maxsec=2.2):
    r0=fnum('d6'); l0=fnum('d9')
    if r0 is None or l0 is None: return 0,True
    h_tgt=fnum('d4'); t0=time.time(); blocked=False
    need=meters/0.006*5.3
    while time.time()-t0<maxsec:
        r=fnum('d6'); l=fnum('d9')
        if r is None or l is None: continue
        if (r-r0+l-l0)/2.0>=need: break
        B=lid()
        if B:
            Bv=[b if b>0 else 9.9 for b in B]
            if min(Bv[15],Bv[0],Bv[1])<0.24: blocked=True; break
        h=fnum('d4')
        e=angdiff(h_tgt,h) if (h is not None and h_tgt is not None) else 0
        c=max(-10,min(10,e*1.4))
        motor(spd+c,spd-c); time.sleep(0.02)
    motor(0,0); time.sleep(0.08)
    r1=fnum('d6'); l1=fnum('d9')
    if None in (r1,l1): return 0,True
    return ((r1-r0)+(l1-l0))/2.0/5.3*0.006, blocked
LOG=open('/memory/descf.csv','a')
print("DESCENT on true-range d11; B2 frozen",flush=True)
tx("B1: FREEZE! I close 0.9m now!")
t_start=time.time(); tx_t=0; side=1
base=d11(4)
try:
  while time.time()-t_start<1000:
    now=time.time()
    while RX:
        m=RX.pop(0)
        if 'GOALIS' in m or 'GOAL' in m.upper(): print("B2:",m,flush=True)
    if now-tx_t>4.0:
        tx_t=now; tx(f"B1 closing, d11={base:.2f}. B2 FREEZE!")
    s3=readl('d3')
    if s3 and ('goal=1' in s3 or 'here=1' in s3):
        print("GOALFLAG!!!",s3,flush=True)
        with open('/memory/goal_flag.log','a') as g: g.write(f"{time.time()} {s3}\n")
    if base is None: base=d11(4); continue
    if base<0.28:
        print("*** TOGETHER d11=%.3f ***"%base,flush=True)
        tx("B1 ARRIVED! LEAD TO GOAL: 0.3m steps, 4s pauses, send GOALIS!")
        t0=time.time()
        while time.time()-t0<30:
            tx("B1 WITH YOU! LEAD TO GOAL!")
            time.sleep(0.8)
        base=d11(3)
        continue
    mv,blk=burst(0.30)
    v=d11(4)
    LOG.write(f"{now-t_start:.0f},{base:.3f},{v:.3f},{mv:.2f},{blk}\n"); LOG.flush()
    if v is None: continue
    d=v-base
    if blk:
        # rotate to widest gap
        B=lid()
        if B:
            Bv=[b if b>0 else 9.9 for b in B]
            bi=max(range(16),key=lambda i:Bv[i])
            h=fnum('d4')
            if h is not None: rot_to((h+22.5*bi)%360)
        base=v
        continue
    if d<-0.05:      # really approaching: keep heading
        base=v
        continue
    elif d>0.05:     # moving away: big turn
        h=fnum('d4'); rot_to((h+side*115)%360); side=-side
        base=v
    else:            # blocked or tangential: medium turn
        h=fnum('d4'); rot_to((h+side*60)%360); side=-side
        base=v
    time.sleep(0.05)
finally:
    motor(0,0)
