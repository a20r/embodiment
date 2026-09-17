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
                    if 'GOAL' in s.upper() or 'ACK' in s.upper(): print("RX:",s,flush=True)
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
def rot_to(tgt,spd=26,maxt=2.2):
    t0=time.time()
    while time.time()-t0<maxt:
        h=fnum('d4')
        if h is None: motor(0,0); time.sleep(0.05); continue
        e=angdiff(tgt,h)
        if abs(e)<6: break
        motor(spd if e>0 else -spd, -spd if e>0 else spd); time.sleep(0.02)
    motor(0,0); time.sleep(0.08)
def advance(meters,spd=36,maxsec=2.6):
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
    motor(0,0); time.sleep(0.12)
    r1=fnum('d6'); l1=fnum('d9')
    if None in (r1,l1): return 0,True
    return ((r1-r0)+(l1-l0))/2.0/5.3*0.006, blocked
LOG=open('/memory/final2.csv','a')
print("final2 structured descent",flush=True)
t_start=time.time(); tx_t=0; cur_h=None; best_h=None
try:
  while time.time()-t_start<1300:
    now=time.time()
    while RX:
        m=RX.pop(0)
        if 'GOAL' in m.upper() or 'ACK' in m.upper(): print("B2:",m,flush=True)
    if now-tx_t>3.0:
        tx_t=now
        tx("B1 homing to you. FREEZE! After meet: you walk to goal, I follow.")
    s3=readl('d3')
    if s3 and ('goal=1' in s3 or 'here=1' in s3):
        print("GOALFLAG!!!",s3,flush=True)
        with open('/memory/goal_flag.log','a') as g: g.write(f"{time.time()} {s3}\n")
    base=d11(3)
    if base is None: time.sleep(0.2); continue
    if base<0.18:
        print("*** CONTACT d11=%.3f ***"%base,flush=True)
        tx("B1 ARRIVED AT YOU!")
        motor(0,0)
        t0=time.time()
        while time.time()-t0<30:
            tx("B1 HERE! YOUR TURN: WALK TO GOAL SLOWLY, I FOLLOW VIA d11!")
            time.sleep(0.7)
        continue
    # probe 5 headings: current, +-60, +-130
    h0=fnum('d4')
    cands=[0,60,-60,130,-130]
    results=[]
    for c in cands:
        if c!=0: rot_to((h0+c)%360)
        mv,blk=advance(0.42)
        v=d11(3)
        results.append((v if v is not None else 9.9, c, mv, blk))
        LOG.write(f"P,{c},{v},{mv:.2f},{blk}\n"); LOG.flush()
        if v is not None and v<0.18: break
    results.sort(key=lambda r:(r[3],r[0]))
    b=results[0]
    print(f"base={base:.3f} best c={b[1]} d11={b[0]:.3f} mv={b[2]:.2f} blk={b[3]}",flush=True)
    if b[3]:  # blocked: rotate elsewhere
        h=fnum('d4'); rot_to((h+90)%360); continue
    # go to best heading (we are already near it after last probe; go back if needed)
    if b[1]!=0:
        hcur=fnum('d4')
        rot_to((hcur+b[1])%360)
    base=b[0]
    # RUN: keep going while improving
    for k in range(4):
        mv,blk=advance(0.42)
        v=d11(3)
        LOG.write(f"R,{v},{mv:.2f},{blk}\n"); LOG.flush()
        if v is not None and v<0.18: break
        if blk or (v is not None and v>base+0.01): break
        if v is not None: base=v
finally:
    motor(0,0)
