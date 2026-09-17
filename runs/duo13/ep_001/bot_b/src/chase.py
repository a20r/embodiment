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
def d11(n=2):
    vs=[]
    for _ in range(n):
        v=fnum('d11')
        if v is not None: vs.append(v)
        time.sleep(0.03)
    return statistics.median(vs) if vs else None

LOG=open('/memory/chase2.csv','a')
print("chase start",flush=True)
tx("B1->B2 PLAN: SYNC phases. I broadcast MOVE or FREEZE. When I say FREEZE: you COME to me (your d11 falls). When I say MOVE: you HOLD. 30s phases. ACK!")

t_start=time.time(); tx_t=0; mode_t=0
MOVING=True
PHASE=25.0
ema=None; lasth=None
try:
  while time.time()-t_start<2400:
    now=time.time()-t_start
    # phase control
    if now-mode_t>PHASE:
        mode_t=now; MOVING=not MOVING
        print("PHASE ->", "MOVE" if MOVING else "FREEZE",flush=True)
    # talk
    if time.time()-tx_t>1.2:
        tx_t=time.time()
        v=d11(1)
        tx(f"B1 {'MOVE' if MOVING else 'FREEZE'} d11={v if v else -1:.2f} t={int(time.time())}")
    while RX:
        m=RX.pop(0)
        if 'GOAL' in m.upper() and 'B2' in m: print("B2 GOAL INFO:",m,flush=True)
    s3=readl('d3')
    if s3 and ('goal=1' in s3 or 'here=1' in s3):
        print("GOALFLAG!!!",s3,flush=True)
        with open('/memory/goal_flag.log','a') as g: g.write(f"{time.time()} {s3}\n")
    v=d11(2)
    if v is not None:
        ema=v if ema is None else 0.6*ema+0.4*v
        LOG.write(f"{now:.0f},{v:.3f},{ema:.3f},{'M' if MOVING else 'F'}\n"); LOG.flush()
    if v is not None and v<0.16:
        print("*** CONTACT RANGE ***",flush=True)
        tx("B1 ARRIVED! GOAL?")
        motor(0,0)
        t0=time.time()
        while time.time()-t0<20: tx("B1 HERE!"); time.sleep(0.5)
        continue
    B=lid()
    if MOVING:
        # decide: improving? keep; else turn
        improving = ema is not None and v is not None and v < ema+0.004
        if B:
            Bv=[b if b>0 else 9.9 for b in B]
            front=min(Bv[15],Bv[0],Bv[1])
        else: front=9.9
        if front<0.24:
            motor(0,0)
            # turn toward widest beam
            bi=max(range(16),key=lambda i:Bv[i])
            h=fnum('d4')
            if h is not None:
                tgt=(h+22.5*bi)%360
                t0=time.time()
                while time.time()-t0<2.0:
                    hh=fnum('d4')
                    if hh is None: break
                    e=angdiff(tgt,hh)
                    if abs(e)<6: break
                    motor(24 if e>0 else -24, -24 if e>0 else 24); time.sleep(0.02)
                motor(0,0)
            continue
        if improving:
            # straight, slight hold
            h=fnum('d4') or 0
            motor(32,32)
        else:
            # rotate toward open side, biased randomly
            motor(0,0)
            side=1 if (int(now)%2==0) else -1
            rot=45*side
            h=fnum('d4')
            if h is not None:
                tgt=(h+rot)%360
                t0=time.time()
                while time.time()-t0<1.6:
                    hh=fnum('d4')
                    if hh is None: break
                    e=angdiff(tgt,hh)
                    if abs(e)<6: break
                    motor(22 if e>0 else -22, -22 if e>0 else 22); time.sleep(0.02)
                motor(0,0)
            ema=v  # reset baseline after turn
    else:
        motor(0,0)
    time.sleep(0.06)
finally:
    motor(0,0)
