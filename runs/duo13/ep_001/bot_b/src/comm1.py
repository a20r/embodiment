import time, math, statistics, threading
D='/dev/robot/'
KM=0.006; TPS=5.3
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
LOG=open('/memory/chase.csv','a')
def log(s): LOG.write(s+"\n"); LOG.flush()

RX=[]
LATEST={'b2':None,'t':0}
GOALINFO=[]
def listener():
    while True:
        try:
            with open(D+'d10') as f:
                s=f.read().strip()
                if s:
                    RX.append(s)
                    with open('/memory/rx.log','a') as g: g.write(s+"\n")
                    print("RX:",s,flush=True)
                    if 'B2' in s or 'B1' in s:
                        LATEST['b2']=s; LATEST['t']=time.time()
                        low=s.lower()
                        if 'goal' in low:
                            GOALINFO.append(s)
                            with open('/memory/goal_info.txt','a') as g: g.write(s+"\n")
                            print("*** GOAL INFO ***",s,flush=True)
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
    need=meters/KM*TPS
    h_tgt=fnum('d4'); t0=time.time(); blocked=False
    while time.time()-t0<maxsec:
        r=fnum('d6'); l=fnum('d9')
        if r is None or l is None: continue
        if (r-r0+l-l0)/2.0>=need: break
        if front_clear()<0.22: blocked=True; break
        h=fnum('d4')
        err=angdiff(h_tgt,h) if (h is not None and h_tgt is not None) else 0
        c=max(-10,min(10,err*1.4))
        motor(spd+c,spd-c); time.sleep(0.02)
    motor(0,0); time.sleep(0.1)
    r1=fnum('d6'); l1=fnum('d9')
    if None in (r1,l1): return 0,True
    return ((r1-r0)+(l1-l0))/2.0/TPS*KM, blocked

print("comm1 start",flush=True)
t_start=time.time(); ping_i=0
ema=None; state='DESCEND'; still_until=0
try:
  while time.time()-t_start<2400:
    now=time.time()
    # TALK
    if now-last_tx>1.5 if False else (ping_i==0 or now-(tx_t if 'tx_t' in dir() else 0)>1.5):
        pass
    # simpler talker below
    if now-(globals().get('tx_t',0))>1.5:
        globals()['tx_t']=now; ping_i+=1
        v=d11(2) or -1
        h=fnum('d4') or -1
        msg=f"B1 PING d11={v:.3f} h={h:.0f} t={int(time.time())}"
        if ping_i%4==0:
            msg="B1->B2 ACK! I hear you. Please PING every 2s. Q: have you found GOAL? reply 'B2 GOAL ...'. I am homing to you."
        tx(msg)
    if GOALINFO:
        print("GOALINFO:",GOALINFO,flush=True)
    # monitor flags
    s3=readl('d3')
    if s3 and ('goal=1' in s3 or 'here=1' in s3):
        log(f"GOALFLAG,{s3}"); print("GOALFLAG",s3,flush=True)
    v=d11(3)
    if v is not None:
        ema = v if ema is None else 0.5*ema+0.5*v
        log(f"L,{now-t_start:.0f},{v:.3f},{ema:.3f},{state}")
    fresh_b2 = (now-LATEST['t']<12) and LATEST['b2']
    if v is not None and v<0.16:
        print("VERY CLOSE — broadcasting",flush=True)
        t0=time.time()
        while time.time()-t0<20:
            tx(f"B1 AT d11={d11(1):.3f} COME!")
            time.sleep(0.4)
            if RX: break
        continue
    if state=='DESCEND':
        # alternate: brief still-period when B2 is fresh (let them home too)
        if fresh_b2 and now<still_until:
            time.sleep(0.2); continue
        cands=[0,60,-60,120,-120,180]
        base=ema if ema else d11(4)
        results=[]
        for c in cands:
            if c!=0: rot_by(c)
            mvs,blk=advance(0.40)
            dv=d11(3)
            results.append((dv if dv is not None else 9.9,c,mvs,blk))
            log(f"P,{c},{dv},{mvs:.2f},{blk}")
            if dv is not None and dv<0.20: break
        results.sort(key=lambda r:(r[3],r[0]))
        b=results[0]
        print(f"DESCEND base={base:.3f} best=({b[0]:.3f},c={b[1]},mv={b[2]:.2f},blk={b[3]})",flush=True)
        if b[3]: rot_by(90); continue
        if b[0]<base-0.01:
            if b[1]!=0: rot_by(b[1])
            ema=b[0]; state='RUN'
        else:
            rot_by(83); # random-ish redirect
            if fresh_b2:
                still_until=now+8  # hold still, let B2 approach
                print("holding still for B2",flush=True)
    elif state=='RUN':
        mvs,blk=advance(0.40)
        dv=d11(3)
        if dv is not None:
            ema=0.5*ema+0.5*dv
            log(f"R,{dv:.3f},{mvs:.2f},{blk}")
        print(f"RUN d11={dv} mv={mvs:.2f} blk={blk}",flush=True)
        if blk or dv is None: state='DESCEND'; continue
        if dv<0.20: continue
        if dv>ema+0.02: state='DESCEND'
    time.sleep(0.05)
finally:
    motor(0,0); log("# exit")
