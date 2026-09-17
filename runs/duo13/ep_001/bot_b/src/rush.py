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
def rot_by(delta,spd=24):
    h0=fnum('d4')
    if h0 is None: return
    tgt=(h0+delta)%360
    t0=time.time()
    while time.time()-t0 < abs(delta)/11+1.5:
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
def advance(meters,spd=34,maxsec=3.5):
    r0=fnum('d6'); l0=fnum('d9')
    if r0 is None or l0 is None: return 0,True
    h_tgt=fnum('d4'); t0=time.time(); blocked=False
    TPS=5.3; KM=0.006
    need=meters/KM*TPS
    while time.time()-t0<maxsec:
        r=fnum('d6'); l=fnum('d9')
        if r is None or l is None: continue
        if (r-r0+l-l0)/2.0>=need: break
        if front_clear()<0.24: blocked=True; break
        h=fnum('d4')
        err=angdiff(h_tgt,h) if (h is not None and h_tgt is not None) else 0
        c=max(-10,min(10,err*1.4))
        motor(spd+c,spd-c); time.sleep(0.02)
    motor(0,0); time.sleep(0.1)
    r1=fnum('d6'); l1=fnum('d9')
    if None in (r1,l1): return 0,True
    return ((r1-r0)+(l1-l0))/2.0/TPS*KM, blocked

print("RUSH start: B2 is at goal; home on d11!",flush=True)
tx("B1->B2: STAY AT GOAL! I home to you via d11. ETA ~2min. HOLD!")
t_start=time.time(); tx_t=0
ema=None; state='RUN'; lastgood=None
LOG=open('/memory/rush.csv','a')
try:
  while time.time()-t_start<1500:
    now=time.time()
    while RX:
        m=RX.pop(0)
        if 'GOALSEEN' in m: print("B2 GOALSEEN:",m,flush=True)
        if 'HOLD' in m or 'ACK' in m or 'WAIT' in m or 'STAY' in m: print("B2 SAYS:",m,flush=True)
    if now-tx_t>2.0:
        tx_t=now
        v=d11(2)
        tx(f"B1 coming! d11={v if v else -1:.2f} STAY AT GOAL!")
    s3=readl('d3')
    if s3 and ('goal=1' in s3 or 'here=1' in s3):
        print("GOALFLAG!!!",s3,flush=True)
        with open('/memory/goal_flag.log','a') as g: g.write(f"{time.time()} {s3}\n")
    v=d11(3)
    if v is None: time.sleep(0.1); continue
    ema=v if ema is None else 0.5*ema+0.5*v
    LOG.write(f"{now-t_start:.0f},{v:.3f},{ema:.3f},{state}\n"); LOG.flush()
    if v<0.14:
        print("*** d11<0.14 — AT B2/GOAL? broadcasting ***",flush=True)
        tx("B1 ARRIVED AT YOUR POSITION. GOAL?")
        t0=time.time()
        while time.time()-t0<25:
            tx("B1 AT GOAL?")
            time.sleep(0.5)
        continue
    if state=='RUN':
        mvs,blk=advance(0.45)
        v2=d11(3)
        LOG.write(f"R,{v2},{mvs:.2f},{blk}\n"); LOG.flush()
        print(f"RUN d11={v2} mv={mvs:.2f} blk={blk}",flush=True)
        if v2 is not None and v2<0.14: continue
        if blk or (v2 is not None and v2>ema+0.015):
            state='TURN'; continue
    elif state=='TURN':
        # try a direction: descend check
        base=d11(3)
        best=(None,None)
        for c in [60,-60,120,-120,30,-30]:
            rot_by(c)
            mvs,blk=advance(0.35)
            v2=d11(3)
            LOG.write(f"T,{c},{v2},{mvs:.2f},{blk}\n"); LOG.flush()
            print(f"TURN c={c} d11={v2} mv={mvs:.2f} blk={blk}",flush=True)
            if v2 is not None and (best[0] is None or v2<best[0]): best=(v2,c)
            if v2 is not None and v2<base-0.02: break
        state='RUN'
    time.sleep(0.05)
finally:
    motor(0,0)
