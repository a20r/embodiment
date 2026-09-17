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
                    if 'B2' in s and 'PING' not in s: print("RX:",s,flush=True)
        except Exception: pass
        time.sleep(0.02)
threading.Thread(target=listener,daemon=True).start()
def rot_to(tgt,spd=26,maxt=2.2):
    t0=time.time()
    while time.time()-t0<maxt:
        h=fnum('d4')
        if h is None: motor(0,0); time.sleep(0.05); continue
        e=angdiff(tgt,h)
        if abs(e)<6: break
        motor(spd if e>0 else -spd, -spd if e>0 else spd); time.sleep(0.02)
    motor(0,0); time.sleep(0.05)
def advance(meters,spd=38,maxsec=2.4):
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
            if min(Bv[15],Bv[0],Bv[1])<0.25: blocked=True; break
        h=fnum('d4')
        e=angdiff(h_tgt,h) if (h is not None and h_tgt is not None) else 0
        c=max(-10,min(10,e*1.4))
        motor(spd+c,spd-c); time.sleep(0.02)
    motor(0,0); time.sleep(0.05)
    r1=fnum('d6'); l1=fnum('d9')
    if None in (r1,l1): return 0,True
    return ((r1-r0)+(l1-l0))/2.0/5.3*0.006, blocked
LOG=open('/memory/endgame.csv','a')
print("endgame: explore + follow",flush=True)
t_start=time.time(); tx_t=0
try:
  while time.time()-t_start<800:
    now=time.time()
    pings=[]
    while RX:
        m=RX.pop(0)
        if 'PING' in m and 'x=' in m:
            try:
                parts=dict(kv.split('=') for kv in m.split() if '=' in kv)
                pings.append((float(parts['x']),float(parts['y'])))
            except: pass
        else:
            print("RX:",m,flush=True)
    if now-tx_t>2.5:
        tx_t=now
        tx("B1 FOLLOWING. B2: PING x= y= every 1s + walk to goal 0.2m/3s pause!")
    s3=readl('d3')
    if s3 and ('goal=1' in s3 or 'here=1' in s3):
        print("GOALFLAG!!!",s3,flush=True)
        with open('/memory/goal_flag.log','a') as g: g.write(f"{time.time()} {s3}\n")
        tx("B1 SEES GOAL FLAG! coming!")
    v=fnum('d11')
    LOG.write(f"{now-t_start:.0f},{v},{len(pings)}\n"); LOG.flush()
    B=lid()
    if not B: time.sleep(0.05); continue
    Bv=[b if b>0 else 9.9 for b in B]
    front=min(Bv[15],Bv[0],Bv[1])
    if front<0.25:
        bi=max(range(16),key=lambda i:Bv[i])
        h=fnum('d4')
        if h is not None: rot_to((h+22.5*bi)%360)
    else:
        mv,blk=advance(0.45)
        if blk:
            h=fnum('d4'); rot_to((h+60)%360)
    time.sleep(0.05)
finally:
    motor(0,0)
