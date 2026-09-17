import time, math, statistics
D='/dev/robot/'
KM=0.006
TICKS_PER_UNIT_S=5.3
def read(p,tries=4):
    for _ in range(tries):
        try:
            with open(D+p) as f: s=f.read()
            lines=[x.strip() for x in s.split('\n') if x.strip()]
            if lines: return lines[-1]
        except Exception: pass
        time.sleep(0.02)
    return None
def fnum(p,tries=4):
    s=read(p,tries)
    try: return float(s)
    except: return None
def lid():
    s=read('d2')
    return [float(x) for x in s.split(',')] if s else None
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
import threading
RXHITS=[]
def listener():
    while True:
        try:
            with open(D+'d10') as f:
                s=f.read().strip()
                if s:
                    RXHITS.append(s)
                    with open('/memory/rx.log','a') as g: g.write(s+"\n")
                    print("RX!!!",s,flush=True)
        except Exception: pass
        time.sleep(0.02)
threading.Thread(target=listener,daemon=True).start()
LOG=open('/memory/homing3.csv','a')
def log(s): LOG.write(s+"\n"); LOG.flush()
def d11(n=4):
    vs=[]
    for _ in range(n):
        v=fnum('d11')
        if v is not None: vs.append(v)
        time.sleep(0.04)
    return statistics.median(vs) if vs else None
def rot_by(delta,spd=22):
    h0=fnum('d4')
    if h0 is None: return
    tgt=(h0+delta)%360
    t0=time.time()
    while time.time()-t0 < abs(delta)/9+1.5:
        h=fnum('d4')
        if h is None: motor(0,0); time.sleep(0.05); continue
        err=angdiff(tgt,h)
        if abs(err)<5: break
        motor(spd if err>0 else -spd, -spd if err>0 else spd)
        time.sleep(0.02)
    motor(0,0); time.sleep(0.1)
def front_clear():
    B=lid()
    if not B: return 9.9
    Bv=[v if v>0 else 9.9 for v in B]
    return min(Bv[15],Bv[0],Bv[1])
def advance(ticks_target,spd=30,maxsec=3.0):
    r0=fnum('d6'); l0=fnum('d9')
    if r0 is None or l0 is None: return 0,True
    h_tgt=fnum('d4'); t0=time.time(); blocked=False
    while time.time()-t0<maxsec:
        r=fnum('d6'); l=fnum('d9')
        if r is None or l is None: continue
        if (r-r0+l-l0)/2.0>=ticks_target: break
        if front_clear()<0.22: blocked=True; break
        h=fnum('d4')
        err=angdiff(h_tgt,h) if (h is not None and h_tgt is not None) else 0
        c=max(-10,min(10,err*1.4))
        motor(spd+c,spd-c); time.sleep(0.02)
    motor(0,0); time.sleep(0.15)
    r1=fnum('d6'); l1=fnum('d9')
    if None in (r1,l1): return 0,True
    return ((r1-r0)+(l1-l0))/2.0, blocked

print("homing3 start",flush=True)
ema=None; best_ema=None
t_start=time.time(); last_tx=0
MOV_TICKS=int(0.5/KM*TICKS_PER_UNIT_S)   # ~0.5m
log(f"# start MOV_TICKS={MOV_TICKS}")
state='PROBE'
round_no=0
try:
  while time.time()-t_start<3000:
    now=time.time()
    if now-last_tx>1.0:
        last_tx=now
        radio_tx(f"R1 d11={ema if ema else -1:.3f}")
        while RXHITS:
            m=RXHITS.pop(0)
            log(f"RX!!!,{m}")
    s3=read('d3')
    if 'goal=1' in s3 or 'here=1' in s3:
        log(f"GOALFLAG,{s3}"); print("GOALFLAG!!!",s3,flush=True)
    if state=='PROBE':
        round_no+=1
        base=d11(5)
        results=[]
        for c in [0,60,-60,120,-120,180]:
            if c!=0: rot_by(c)
            tk,blk=advance(MOV_TICKS)
            dv=d11(4)
            results.append((dv if dv is not None else 9.9, c, tk, blk))
            log(f"P,{c},{dv},{tk:.0f},{blk}")
            if dv is not None and dv<0.22: break
        results.sort(key=lambda r:(r[3], r[0]))
        b=results[0]
        log(f"# round{round_no} base={base} best=({b[0]:.3f},c={b[1]},tk={b[2]:.0f},blk={b[3]})")
        print(f"round{round_no} base={base} best d11={b[0]:.3f} at c={b[1]} blk={b[3]}",flush=True)
        if b[3]:  # best was blocked
            rot_by(90)
            state='PROBE'; continue
        if b[0]>base-0.005:
            # no improvement: rotate random and retry
            rot_by(77)
            state='PROBE'
        else:
            ema=b[0]; state='RUN'
    elif state=='RUN':
        tk,blk=advance(MOV_TICKS)
        dv=d11(4)
        if dv is not None:
            ema = dv if ema is None else 0.6*ema+0.4*dv
        log(f"R,{dv},{tk:.0f},{blk},{ema if ema else -1:.3f}")
        if blk or dv is None:
            state='PROBE'; continue
        if dv<0.22:
            # VERY CLOSE: stop, spin-scan, radio blast
            print("CLOSE! d11=",dv,flush=True)
            for k in range(12):
                radio_tx(f"R1 HERE d11={dv}")
                m=radio_rx()
                if m: log(f"RX!!!,{m}"); print("RX!!!",m,flush=True)
                rot_by(30,spd=14)
                dv2=d11(2)
                log(f"C,{dv2}")
                if dv2 is not None and dv2<0.12: break
            state='PROBE'
        elif dv>ema+0.02:
            state='PROBE'
    time.sleep(0.03)
finally:
    motor(0,0); log("# exit")
