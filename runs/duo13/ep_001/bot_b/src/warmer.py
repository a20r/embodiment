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
    motor(0,0); time.sleep(0.1)
    r1=fnum('d6'); l1=fnum('d9')
    if None in (r1,l1): return 0,True
    return ((r1-r0)+(l1-l0))/2.0/5.3*0.006, blocked
LOG=open('/memory/warmer.csv','a')
print("warmer game start",flush=True)
t_start=time.time(); tx_t=0
h_dir=0.0   # current heading offset (deg, relative to initial)
best_reply=None
try:
  while time.time()-t_start<1100:
    now=time.time()
    if now-tx_t>4.0:
        tx_t=now
        tx(f"B1 MOVED & STOPPED. Send B2 WARMER/COLDER with your d11!")
    # wait for reply up to 4s
    reply=None; t0=time.time()
    while time.time()-t0<4.0:
        while RX:
            m=RX.pop(0)
            if 'WARMER' in m.upper() or 'COLDER' in m.upper():
                reply=m
            elif 'GOAL' in m.upper():
                print("B2:",m,flush=True)
        if reply: break
        time.sleep(0.1)
    val=None; warmer=None
    if reply:
        try:
            val=float(reply.split('=')[-1].replace('<','').replace('>','').strip())
        except: 
            import re
            mm=re.search(r'(-?\d+\.\d+)',reply)
            if mm: val=float(mm.group(1))
        warmer='WARMER' in reply.upper()
        LOG.write(f"{now-t_start:.0f},REPLY,{reply},{val}\n"); LOG.flush()
        print(f"ORACLE: {reply} val={val}",flush=True)
    else:
        LOG.write(f"{now-t_start:.0f},NOREPLY\n"); LOG.flush()
        print("no oracle reply",flush=True)
    # decision: if warmer than before -> keep direction; else rotate 65
    if warmer is True:
        pass  # keep heading
    elif warmer is False:
        h_dir += 65
    else:
        h_dir += 35   # blind rotate on no info
    h=fnum('d4')
    if h is not None:
        rot_to((h+ (65 if warmer is False else (35 if warmer is None else 0)))%360)
    mv,blk=advance(0.40)
    LOG.write(f"{now-t_start:.0f},MOVE,{mv:.2f},{blk}\n"); LOG.flush()
    print(f"moved {mv:.2f} blk={blk} dir_h={h_dir:.0f}",flush=True)
    if blk:
        h=fnum('d4'); rot_to((h+80)%360)
    time.sleep(0.5)
finally:
    motor(0,0)
