import time, math, statistics
D='/dev/robot/'
def read(p, tries=4):
    for _ in range(tries):
        try:
            with open(D+p) as f: s=f.read().strip()
            if s: return s
        except Exception: pass
        time.sleep(0.02)
    return None
def fnum(p, tries=4):
    s=read(p,tries)
    try: return float(s)
    except: return None
def lidar():
    s=read('d2')
    try: return [float(x) for x in s.split(',')]
    except: return None
def motor(l,r):
    try:
        with open(D+'d1','w') as f: f.write(f"{l:.1f}\n")
        with open(D+'d7','w') as f: f.write(f"{r:.1f}\n")
    except Exception: pass
def radio_tx(m):
    try:
        with open(D+'d8','w') as f: f.write(m+"\n")
    except Exception: pass
def radio_rx():
    try:
        with open(D+'d10') as f:
            s=f.read().strip()
            return s if s else None
    except Exception: return None
def angdiff(a,b): return (a-b+180)%360-180
LOG=open('/memory/homelog.csv','a')
def log(s): LOG.write(s+"\n"); LOG.flush()

def odom():
    r=fnum('d6'); l=fnum('d9')
    return (r,l)

def d11(n=3):
    vs=[]
    for _ in range(n):
        v=fnum('d11')
        if v is not None: vs.append(v)
        time.sleep(0.05)
    return statistics.median(vs) if vs else None

def rot_by(delta, spd=22):
    h0=fnum('d4'); tgt=(h0+delta)%360
    t0=time.time()
    while time.time()-t0< abs(delta)/12+1.0:
        h=fnum('d4')
        if h is None: motor(0,0); time.sleep(0.05); continue
        err=angdiff(tgt,h)
        if abs(err)<5: break
        motor(spd if err>0 else -spd, -spd if err>0 else spd)
        time.sleep(0.02)
    motor(0,0)

def advance(dist_m, avoid=True, spd=32, maxsec=4.0):
    """drive forward ~dist_m using odometer; returns actual dist (m units via k_m)"""
    o0=odom(); t0=time.time(); h_tgt=fnum('d4')
    ticks_needed = dist_m/KM*5.3
    moved=0
    while time.time()-t0<maxsec:
        o=odom()
        if o[0] is None or o[1] is None or o0[0] is None or o0[1] is None: continue
        moved=((o[0]-o0[0])+(o[1]-o0[1]))/2.0
        if moved>=ticks_needed: break
        B=lidar()
        if B and avoid:
            Bv=[v if v>0 else 9.9 for v in B]
            front=min(Bv[15],Bv[0],Bv[1])
            if front<0.25:
                motor(0,0); return moved/KM/5.3, True
        h=fnum('d4')
        err=angdiff(h_tgt,h) if (h is not None and h_tgt is not None) else 0
        corr=max(-12,min(12,err*1.5))
        motor(spd+corr, spd-corr)
        time.sleep(0.02)
    motor(0,0)
    return moved/KM/5.3, False

KM=0.010   # meters per (unit*s) guess; calibrate
def calibrate_km():
    """drive toward nearest wall, measure closing rate vs odometer"""
    B=lidar()
    if not B: return
    Bv=[v if v>0 else 9.9 for v in B]
    # find a beam 0.5-1.5m roughly ahead: use front sector
    fi=min([15,0,1], key=lambda i:Bv[i])
    if Bv[fi]>1.6 or Bv[fi]<0.4: return
    o0=odom(); d0=fnum('d2').split(','); d0=[float(x) for x in d0]
    t0=time.time()
    while time.time()-t0<1.2:
        motor(25,25); time.sleep(0.02)
    motor(0,0); time.sleep(0.1)
    d1=[float(x) for x in read('d2').split(',')]
    o1=odom()
    if None in (o0[0],o1[0],o0[1],o1[1]): return
    ticks=((o1[0]-o0[0])+(o1[1]-o0[1]))/2.0
    # closing of the beam we aimed at
    dd = d0[fi]-d1[fi]
    if dd>0.02 and ticks>50:
        global KM
        KM = dd/(ticks/5.3)
        log(f"# KM calibrated: {KM:.4f} m per unit*s (dd={dd:.3f} ticks={ticks:.0f})")
        print("KM=",KM,flush=True)

print("home.py start", flush=True)
calibrate_km()
print("KM after cal:",KM, flush=True)

best=None; cur_h=None
t_start=time.time(); last_tx=0; phase='PROBE'
d_prev=d11(5)
log(f"# start d11={d_prev}")
try:
  while time.time()-t_start<3000:
    now=time.time()
    if now-last_tx>1.5:
        last_tx=now
        radio_tx(f"R1 t={now:.0f} d11={d_prev}")
        msg=radio_rx()
        if msg:
            log(f"RX,{msg}"); print("RX!!!",msg,flush=True)
    s3=read('d3')
    if 'goal=1' in s3 or 'here=1' in s3:
        log(f"GOALFLAG,{s3}"); print("GOALFLAG",s3,flush=True)
    B=lidar()
    if B:
        Bv=[v if v>0 else 9.9 for v in B]
        front=min(Bv[15],Bv[0],Bv[1])
    else: front=9.9
    if phase=='PROBE':
        # try candidate headings, pick best d11 improvement
        cands=[0,60,-60,120,-120,180]
        results=[]
        for c in cands:
            if c!=0: rot_by(c)
            adv,_=advance(0.30)
            dv=d11(3)
            results.append((dv if dv is not None else 9.9, c, adv))
            log(f"P,{c:.0f},{dv},{adv:.2f}")
        results.sort()
        bestd=results[0]
        # rotate to best candidate from current
        cur_abs = bestd[1]
        rot_by(cur_abs)
        d_prev=bestd[0]
        phase='RUN'
    elif phase=='RUN':
        # keep going in current heading while d11 improves; else PROBE
        adv,hit=advance(0.30)
        dv=d11(3)
        log(f"R,{dv},{adv:.2f},{hit}")
        if dv is None: continue
        if dv < d_prev-0.005:
            d_prev=dv; # keep heading
        elif dv > d_prev+0.01:
            phase='PROBE'
        else:
            d_prev=dv  # neutral: keep
        if hit:
            # blocked: sidestep via probe
            phase='PROBE'
    time.sleep(0.05)
finally:
    motor(0,0); log("# exit")
