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
                    if 'ATGOAL' in s or 'FLAG' in s: print("B2:",s,flush=True)
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
def rot_to(tgt,spd=30,maxt=2.4):
    t0=time.time()
    while time.time()-t0<maxt:
        h=fnum('d4')
        if h is None: motor(0,0); time.sleep(0.05); continue
        e=angdiff(tgt,h)
        if abs(e)<5: break
        motor(spd if e>0 else -spd, -spd if e>0 else spd); time.sleep(0.02)
    motor(0,0); time.sleep(0.04)
def BV():
    B=lid()
    if not B: return None
    return [b if b>0 else 9.9 for b in B]
def drive_straight(meters,spd=40,maxsec=3.0):
    r0=fnum('d6'); l0=fnum('d9')
    if r0 is None or l0 is None: return 0,True
    h_tgt=fnum('d4'); t0=time.time(); blocked=False
    need=meters/0.006*5.3
    while time.time()-t0<maxsec:
        r=fnum('d6'); l=fnum('d9')
        if r is None or l is None: continue
        if (r-r0+l-l0)/2.0>=need: break
        B=BV()
        if B and min(B[15],B[0],B[1])<0.36: blocked=True; break
        h=fnum('d4')
        e=angdiff(h_tgt,h) if (h is not None and h_tgt is not None) else 0
        c=max(-10,min(10,e*1.4))
        motor(spd+c,spd-c); time.sleep(0.02)
    motor(0,0); time.sleep(0.05)
    r1=fnum('d6'); l1=fnum('d9')
    if None in (r1,l1): return 0,True
    return ((r1-r0)+(l1-l0))/2.0/5.3*0.006, blocked
LOG=open('/memory/wallbug.csv','a')
print("wallbug hybrid start",flush=True)
tx("B2: STAY AT GOAL (0.4,-0.8) or FREEZE. I am using wall-following to reach you. PING 1Hz please!")
t_start=time.time(); tx_t=0
mode='PROBE'; side=1
BAD=[]
FAILS=0
best=9.9; blocked_run=0; wf_dist=0.0
try:
  while time.time()-t_start<2800:
    now=time.time()
    while RX:
        m=RX.pop(0)
        if 'ATGOAL' in m or 'FLAG' in m: print("B2:",m,flush=True)
    if now-tx_t>4.0:
        tx_t=now; tx(f"B1 wallbug d11={best:.2f} B2 STAY!")
    s3=readl('d3')
    if s3 and ('goal=1' in s3 or 'here=1' in s3):
        print("GOALFLAG!!!",s3,flush=True)
        with open('/memory/goal_flag.log','a') as g: g.write(f"{time.time()} {s3}\n")
    base=d11(3)
    if base is None: time.sleep(0.15); continue
    if base<0.65 and mode=='PROBE':
        mode='ACQUIRE'; print('ACQUIRE at',base,flush=True)
    if base>=0.72 and mode=='ACQUIRE':
        mode='PROBE'; BAD.clear()
    if mode=='ACQUIRE':
        # test candidate directions: rotate, small burst, check d11 drop
        h0=fnum('d4')
        if h0 is None: continue
        cands=[]
        for k in range(16):
            B=BV()
            if B:
                for i in range(16):
                    if B[i]<9 and abs(B[i]-base)<0.18:
                        cands.append((h0+22.5*i)%360)
            rot_to((h0+22.5*(k+1))%360,spd=34,maxt=0.8)
        # dedupe & blacklist
        seen=[]; 
        for cd in cands:
            if all(abs(angdiff(cd,x))>25 for x in seen): seen.append(cd)
        cands=[cd for cd in seen if not any(abs(angdiff(cd,bad))<30 for bad in BAD)]
        if not cands:
            BAD.clear(); FAILS+=1
            print("acquire fails=",FAILS,flush=True)
            if FAILS>=2:
                mode='WALL'; wf_dist=0.0; FAILS=0
                side=1 if (int(now)%2==0) else -1
                print("-> WALL mode side=",side,flush=True)
            else:
                h=fnum('d4'); rot_to((h+50)%360)
            continue
        cd=cands[0]
        rot_to(cd)
        v0=base
        mv,blk=drive_straight(0.17)
        v=d11(3)
        LOG.write(f"A,{v},{cd:.0f},{mv:.2f},{blk},{v0-v:.3f}\n"); LOG.flush()
        print(f"A dir={cd:.0f} drop={v0-v:.3f} blk={blk}",flush=True)
        if blk or (v is not None and v0-v<0.08):
            BAD.append(cd)
            h=fnum('d4'); rot_to((h+40)%360)
        if v is not None and v<0.2:
            BAD.clear()
        continue
    if base<0.14:
        print("too close %.3f backing"%base,flush=True)
        r0=fnum('d6'); t0=time.time()
        while time.time()-t0<1.0:
            r=fnum('d6')
            if r is not None and r0 is not None and r-r0<-140: break
            motor(-30,-30); time.sleep(0.02)
        motor(0,0)
        tx("B1 WITH YOU! LEAD TO GOAL x=0.4 y=-0.8: 0.3m steps 3s pauses! I follow! Say B2 ATGOAL when on it!")
        continue
    if base<0.45 and int(now-t_start)%8==0:
        tx("B1 WITH YOU! LEAD TO GOAL: walk 0.3m steps toward x=0.4 y=-0.8! I follow! B2 ATGOAL when on it!")
    if base<best-0.02: best=base   # record progress
    LOG.write(f"{now-t_start:.0f},{base:.3f},{best:.3f},{mode}\n"); LOG.flush()
    if mode=='PROBE':
        h0=fnum('d4')
        if h0 is None: continue
        results=[]
        for c in [0,60,-60]:
            if c!=0: rot_to((h0+c)%360)
            mv,blk=drive_straight(0.28)
            v=d11(3)
            results.append((v if v is not None else 9.9, c, mv, blk))
            LOG.write(f"P,{c},{v},{mv:.2f},{blk}\n"); LOG.flush()
            if v is not None and v<base-0.06: break
        results.sort(key=lambda r:(r[3],r[0]))
        b=results[0]
        if b[3]:
            blocked_run+=1
            if blocked_run>=3:
                mode='WALL'; wf_dist=0.0; blocked_run=0
                side=1 if (int(now)%2==0) else -1
                print("switch WALL side=",side,flush=True)
            continue
        blocked_run=0
        if b[1]!=0:
            h=fnum('d4'); rot_to((h+b[1])%360)
        ref=b[0]
        for k in range(4):
            mv,blk=drive_straight(0.30)
            v=d11(3)
            LOG.write(f"R,{v},{mv:.2f},{blk}\n"); LOG.flush()
            if blk or v is None: break
            if v>ref+0.04: break
            ref=v
            if v<0.19: break
    elif mode=='WALL':
        # wall-follow: keep wall on 'side' at ~0.25-0.45m, drive forward
        B=BV()
        if not B: time.sleep(0.1); continue
        front=min(B[15],B[0],B[1])
        # beams for left/right side: left = 9..14, right = 1..6 (approx side walls)
        if side>0:  # wall on LEFT
            wmin=min(B[10],B[11],B[12],B[13])
        else:
            wmin=min(B[2],B[3],B[4],B[5])
        if front<0.34:
            # corner: turn away from wall
            h=fnum('d4')
            if h is not None: rot_to((h+side*75)%360)
            continue
        if wmin>0.55:
            # wall lost: turn toward wall side
            h=fnum('d4')
            if h is not None: rot_to((h-side*30)%360)
            continue
        if wmin<0.16:
            # too close to wall: veer away
            h=fnum('d4')
            if h is not None: rot_to((h+side*20)%360)
            continue
        mv,blk=drive_straight(0.22)
        wf_dist+=mv
        LOG.write(f"W,{d11(2)},{mv:.2f},{wf_dist:.1f}\n"); LOG.flush()
        if wf_dist>0.9:
            # periodic re-probe
            mode='PROBE'; blocked_run=0
finally:
    motor(0,0)
