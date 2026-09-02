import time, math, statistics
D='/dev/robot/'
KM=0.008
def read(p,tries=4):
    for _ in range(tries):
        try:
            with open(D+p) as f: s=f.read().strip()
            if s: return s
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
LOG=open('/memory/homing2.csv','a')
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
    while time.time()-t0 < abs(delta)/10+1.5:
        h=fnum('d4')
        if h is None: motor(0,0); time.sleep(0.05); continue
        err=angdiff(tgt,h)
        if abs(err)<5: break
        motor(spd if err>0 else -spd, -spd if err>0 else spd)
        time.sleep(0.02)
    motor(0,0); time.sleep(0.1)

def advance(dist_m,spd=30,maxsec=3.5):
    """straight forward ~dist_m; stop early on wall. returns (moved_m, blocked)"""
    r0=fnum('d6'); l0=fnum('d9')
    if r0 is None or l0 is None: return 0.0,True
    need=dist_m/KM*5.3
    h_tgt=fnum('d4'); t0=time.time(); blocked=False
    while time.time()-t0<maxsec:
        r=fnum('d6'); l=fnum('d9')
        if r is None or l is None: continue
        if (r-r0+l-l0)/2.0>=need: break
        B=lid()
        if B:
            Bv=[v if v>0 else 9.9 for v in B]
            fr=min(Bv[15],Bv[0],Bv[1])
            if fr<0.22: blocked=True; break
        h=fnum('d4')
        err=angdiff(h_tgt,h) if (h is not None and h_tgt is not None) else 0
        c=max(-10,min(10,err*1.4))
        motor(spd+c,spd-c); time.sleep(0.02)
    motor(0,0); time.sleep(0.15)
    r1=fnum('d6'); l1=fnum('d9')
    if None in (r1,l1,r0,l0): return 0.0,True
    moved=((r1-r0)+(l1-l0))/2.0/KM/5.3
    return moved,blocked

# pose tracking
pose={'x':0.0,'y':0.0,'h':fnum('d4') or 0.0,'t':time.time()}
def update_pose():
    h=fnum('d4')
    if h is None: return
    dt=time.time()-pose['t']
    r=fnum('d6'); l=fnum('d9')
    # crude: use odom rate over dt
    if dt>0.8 and r is not None and l is not None and 'or' in pose and 'ol' in pose:
        vr=(r-pose['or'])/5.3/dt; vl=(l-pose['ol'])/5.3/dt
        ds=(vr+vl)/2.0*KM*dt
        th=math.radians(pose['h'])
        pose['x']+=ds*math.sin(th); pose['y']+=ds*math.cos(th)
        pose['t']=time.time()
    pose['or']=r; pose['ol']=l
    pose['h']=h

print("homing2 start",flush=True)
d_prev=d11(5)
log(f"# start d11={d_prev} KM={KM}")
t_start=time.time(); last_tx=0; stuck=0
state='PROBE'
cand_idx=0
try:
  while time.time()-t_start<3000:
    now=time.time()
    if now-last_tx>1.2:
        last_tx=now
        radio_tx(f"R1 d11={d_prev:.3f} st={state}")
        msg=radio_rx()
        if msg:
            log(f"RX!!!,{msg}"); print("RX!!!",msg,flush=True)
    s3=read('d3')
    if 'goal=1' in s3 or 'here=1' in s3:
        log(f"GOALFLAG,{s3}"); print("GOALFLAG",s3,flush=True)
    update_pose()
    if state=='PROBE':
        cands=[0,60,-60,120,-120,180]
        if cand_idx>=len(cands):
            cand_idx=0
            d_prev=d11(4)
            if d_prev is None: time.sleep(0.2); continue
            log(f"# new probe round d_prev={d_prev:.3f}")
        c=cands[cand_idx]; cand_idx+=1
        if c!=0: rot_by(c)
        mov,blk=advance(0.30)
        dv=d11(4)
        log(f"P,{c},{dv},{mov:.2f},{blk},{pose['x']:.2f},{pose['y']:.2f}")
        print(f"P c={c} d11={dv} mov={mov:.2f} blk={blk} pose=({pose['x']:.1f},{pose['y']:.1f})",flush=True)
        if dv is not None and d_prev is not None and dv<d_prev-0.02:
            d_prev=dv; state='RUN'; cand_idx=0
        elif blk:
            rot_by(90); 
        elif cand_idx>=len(cands):
            state='RUN' if False else 'PROBE'
    elif state=='RUN':
        mov,blk=advance(0.30)
        dv=d11(4)
        log(f"R,{dv},{mov:.2f},{blk},{pose['x']:.2f},{pose['y']:.2f}")
        if dv is None: continue
        if blk: state='PROBE'; cand_idx=1; continue
        if dv<d_prev-0.004: d_prev=dv
        elif dv>d_prev+0.015: state='PROBE'; cand_idx=0
        else: d_prev=dv
    time.sleep(0.03)
finally:
    motor(0,0); log("# exit")
