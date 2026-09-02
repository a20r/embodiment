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
                    if 'ATGOAL' in s or 'FLAG' in s or 'HERE' in s: print("B2:",s,flush=True)
                    if 'ATGOAL' in s:
                        with open('/memory/b2_atgoal.flag','w') as g: g.write(s)
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
def rot_to(tgt,spd=28,maxt=2.2):
    t0=time.time()
    while time.time()-t0<maxt:
        h=fnum('d4')
        if h is None: motor(0,0); time.sleep(0.05); continue
        e=angdiff(tgt,h)
        if abs(e)<5: break
        motor(spd if e>0 else -spd, -spd if e>0 else spd); time.sleep(0.02)
    motor(0,0); time.sleep(0.04)
def burst(meters,spd=42,maxsec=2.0):
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
    motor(0,0); time.sleep(0.06)
    r1=fnum('d6'); l1=fnum('d9')
    if None in (r1,l1): return 0,True
    return ((r1-r0)+(l1-l0))/2.0/5.3*0.006, blocked
LOG=open('/memory/descp2.csv','a')
print("descp2 with backtracking",flush=True)
t_start=time.time(); tx_t=0; side=1; blocked_run=0; stall=0; last_best=9.9
try:
  while time.time()-t_start<2800:
    now=time.time()
    while RX:
        m=RX.pop(0)
        if 'ATGOAL' in m or 'FLAG' in m or 'HERE' in m: print("B2:",m,flush=True)
    if now-tx_t>4.0:
        tx_t=now
        tx(f"B1 coming! d11={last_best if last_best<9 else -1:.2f}. B2: STAY AT GOAL!")
    s3=readl('d3')
    if s3 and ('goal=1' in s3 or 'here=1' in s3):
        print("GOALFLAG!!!",s3,flush=True)
        with open('/memory/goal_flag.log','a') as g: g.write(f"{time.time()} {s3}\n")
    base=d11(4)
    if base is None: time.sleep(0.2); continue
    LOG.write(f"{now-t_start:.0f},{base:.3f},{blocked_run},{stall}\n"); LOG.flush()
    if base<0.18:
        print("*** AT B2 d11=%.3f — ask ATGOAL status ***"%base,flush=True)
        tx("B1 WITH YOU! Are you ON goal? Send B2 FLAG! If yes STAY, I stay too!")
        t0=time.time()
        while time.time()-t0<15: tx("B1 WITH YOU! FLAG?"); time.sleep(1.0)
        continue
    if blocked_run>=5:
        # hard backtrack
        h=fnum('d4')
        if h is not None: rot_to((h+160)%360)
        mv,blk=burst(0.55)
        blocked_run=1 if blk else 0
        continue
    h0=fnum('d4')
    if h0 is None: continue
    results=[]
    for c in [0,60,-60]:
        if c!=0: rot_to((h0+c)%360)
        mv,blk=burst(0.28)
        v=d11(3)
        results.append((v if v is not None else 9.9, c, mv, blk))
        LOG.write(f"P,{c},{v},{mv:.2f},{blk}\n"); LOG.flush()
        if v is not None and v<base-0.06: break
    results.sort(key=lambda r:(r[3],r[0]))
    b=results[0]
    if b[3]:
        blocked_run+=1
        B=lid()
        if B:
            Bv=[bb if bb>0 else 9.9 for bb in B]
            bi=max(range(16),key=lambda i:Bv[i])
            h=fnum('d4')
            if h is not None: rot_to((h+22.5*bi)%360)
        else:
            h=fnum('d4'); rot_to((h+side*95)%360); side=-side
        continue
    blocked_run=0
    if b[1]!=0:
        h=fnum('d4'); rot_to((h+b[1])%360)
    ref=b[0]
    gained=False
    for k in range(4):
        mv,blk=burst(0.30)
        v=d11(3)
        LOG.write(f"R,{v},{mv:.2f},{blk}\n"); LOG.flush()
        if blk or v is None: break
        if v>ref+0.04: break
        if v<ref-0.02: gained=True
        ref=v
        if v<0.18: break
    if not gained: stall+=1
    else: stall=0
    if stall>=3:
        h=fnum('d4'); rot_to((h+side*130)%360); side=-side; stall=0
finally:
    motor(0,0)
