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
def d11(n=3):
    vs=[]
    for _ in range(n):
        v=fnum('d11')
        if v is not None: vs.append(v)
        time.sleep(0.04)
    return statistics.median(vs) if vs else None

# local pose integration (units: ticks->'us' via KM applied later)
KM=0.006; TPS=5.3
pose={'x':0.0,'y':0.0,'t':time.time(),'r':None,'l':None,'h':None}
TRI=open('/memory/tri2.csv','a')
def integrate():
    r=fnum('d6'); l=fnum('d9'); h=fnum('d4')
    now=time.time()
    if pose['r'] is not None and None not in (r,l,h) and pose['h'] is not None:
        dt=now-pose['t']
        if dt>0:
            ds=((r-pose['r'])+(l-pose['l']))/2.0/TPS*KM
            dh=angdiff(h,pose['h'])
            hm=math.radians(pose['h']+dh/2.0)
            pose['x']+=ds*math.sin(hm); pose['y']+=ds*math.cos(hm)
    pose['r']=r; pose['l']=l; pose['h']=h; pose['t']=now
    return pose['x'],pose['y'],h

EVENTS=[]   # (px_local,py_local,h,d11,qx,qy,t)
GOALS=[]    # (t,x,y)
def parse_rx():
    while RX:
        s=RX.pop(0)
        now=time.time()
        if 'PING' in s and 'x=' in s:
            try:
                parts=dict(kv.split('=') for kv in s.split() if '=' in kv)
                qx,qy=float(parts['x']),float(parts['y'])
                x,y,h=integrate()
                v=d11(2)
                EVENTS.append((x,y,h,v,qx,qy,now))
                TRI.write(f"S,{now:.1f},{x:.3f},{y:.3f},{h},{v},{qx},{qy}\n"); TRI.flush()
            except Exception as e: print("perr",e,flush=True)
        elif 'GOALSEEN' in s and 'x=' in s:
            try:
                parts=dict(kv.split('=') for kv in s.split() if '=' in kv)
                GOALS.append((now,float(parts['x']),float(parts['y'])))
            except Exception: pass
def fit_recent(win=180.0):
    now=time.time()
    ev=[e for e in EVENTS if now-e[6]<win]
    if len(ev)<5: return None
    best=None
    for s in [x*0.0005 for x in range(4,25)]:
        for thd in range(0,360,4):
            th=math.radians(thd); c,sn=math.cos(th),math.sin(th)
            for mir in (False,True):
                tx_=sum(e[4]-(s*(c*((e[1] if mir else e[0]))-sn*((e[0] if mir else e[1])))) for e in ev)/len(ev)
                ty_=sum(e[5]-(s*(sn*((e[1] if mir else e[0]))+c*((e[0] if mir else e[1])))) for e in ev)/len(ev)
                r=0
                for (x,y,h,v,qx,qy,_) in ev:
                    px,py=(y,x) if mir else (x,y)
                    wx=s*(c*px-sn*py)+tx_; wy=s*(sn*px+c*py)+ty_
                    r+=(math.hypot(wx-qx,wy-qy)-v)**2
                rms=math.sqrt(r/len(ev))
                if best is None or rms<best[0]: best=(rms,s,thd,tx_,ty_,mir)
    return best


def rot_by(delta,spd=24):
    h0=fnum('d4')
    if h0 is None: return
    tgt=(h0+delta)%360
    t0=time.time()
    while time.time()-t0 < abs(delta)/11+1.5:
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
def advance(meters,spd=34,maxsec=3.5):
    r0=fnum('d6'); l0=fnum('d9')
    if r0 is None or l0 is None: return 0,True
    h_tgt=fnum('d4'); t0=time.time(); blocked=False
    need=meters/KM*TPS
    while time.time()-t0<maxsec:
        r=fnum('d6'); l=fnum('d9')
        if r is None or l is None: continue
        if (r-r0+l-l0)/2.0>=need: break
        if front_clear()<0.24: blocked=True; break
        h=fnum('d4')
        err=angdiff(h_tgt,h) if (h is not None and h_tgt is not None) else 0
        c=max(-10,min(10,err*1.4))
        motor(spd+c,spd-c); time.sleep(0.02)
    motor(0,0); time.sleep(0.1)
    r1=fnum('d6'); l1=fnum('d9')
    if None in (r1,l1): return 0,True
    return ((r1-r0)+(l1-l0))/2.0/TPS*KM, blocked

print("rush2 start",flush=True)
tx("B1->B2: keep PINGing! I am navigating to your GOALSEEN. Please also send GOALSEEN often!")
t_start=time.time(); tx_t=0; fit_t=0
FIT=None; nav_h=None
try:
  while time.time()-t_start<2400:
    now=time.time()
    parse_rx()
    if now-tx_t>2.0:
        tx_t=now
        v=d11(2)
        tx(f"B1 PING d11={v if v else -1:.2f} t={int(now)}")
    s3=readl('d3')
    if s3 and ('goal=1' in s3 or 'here=1' in s3):
        print("GOALFLAG!!!",s3,flush=True)
        with open('/memory/goal_flag.log','a') as g: g.write(f"{time.time()} {s3}\n")
    if now-fit_t>40:
        fit_t=now
        FIT=fit_recent()
        if FIT:
            rms,s,thd,tx_,ty_,mir=FIT
            print(f"FIT rms={rms:.3f} s={s:.4f} th={thd} T=({tx_:.2f},{ty_:.2f}) mir={mir} n>=",flush=True)
            with open('/memory/fit.txt','w') as g:
                g.write(f"{rms} {s} {thd} {tx_} {ty_} {mir} {time.time()}\n")
    v=d11(3)
    if v is not None and v<0.15:
        print("*** VERY CLOSE TO B2 ***",flush=True)
        tx("B1 ARRIVED AT YOU. GOAL STATE?")
        t0=time.time()
        while time.time()-t0<20: tx("B1 HERE!"); time.sleep(0.5)
        continue
    # NAVIGATION
    if FIT and FIT[0]<0.13:
        rms,s,thd,tx_,ty_,mir=FIT
        th=math.radians(thd); c,sn=math.cos(th),math.sin(th)
        x,y,h=integrate()
        px,py=(y,x) if mir else (x,y)
        wx=s*(c*px-sn*py)+tx_; wy=s*(sn*px+c*py)+ty_
        # goal = recent GOALSEEN average
        grec=[g for g in GOALS if now-g[0]<240]
        if grec:
            gx=sum(g[1] for g in grec)/len(grec); gy=sum(g[2] for g in grec)/len(grec)
            dist_goal=math.hypot(gx-wx,gy-wy)
            # desired world dir (in their frame) -> inverse rotate to my local
            dgx,dgy=(gx-wx)/max(dist_goal,0.01),(gy-wy)/max(dist_goal,0.01)
            # inverse of R(θ): rotate by -θ ; mirror inverse = same mirror
            lx=(c*dgx+sn*dgy); ly=(-sn*dgx+c*dgy)
            if mir: lx,ly=ly,lx
            h_tgt=math.degrees(math.atan2(lx,ly))%360
            if abs(angdiff(h_tgt,h))>10:
                rot_by(angdiff(h_tgt,h))
            mvs,blk=advance(0.35)
            print(f"NAV dist_goal={dist_goal:.2f} pos=({wx:.2f},{wy:.2f}) h_tgt={h_tgt:.0f} mv={mvs:.2f} blk={blk}",flush=True)
            if blk: rot_by(75)
            if dist_goal<0.22:
                print("*** SHOULD BE AT GOAL ***",flush=True)
                tx("B1 AT GOAL COORDS!")
                t0=time.time()
                while time.time()-t0<30:
                    tx("B1 AT GOAL. WHERE ARE YOU?")
                    time.sleep(1.0)
                continue
            continue
    # fallback: d11 descent
    if v is None: time.sleep(0.1); continue
    base=v
    best=(None,None)
    for cc in [0,60,-60,120,-120]:
        if cc!=0: rot_by(cc)
        # gap-aware advance: rotate toward widest beam first? keep simple
        mvs,blk=advance(0.35)
        v2=d11(3)
        print(f"FALLBACK c={cc} d11={v2} mv={mvs:.2f} blk={blk}",flush=True)
        if v2 is not None and (best[0] is None or v2<best[0]): best=(v2,cc)
        if v2 is not None and v2<base-0.02: break
    if best[1] not in (0,None) and best[0] is not None and best[0]<base:
        pass
    time.sleep(0.1)
finally:
    motor(0,0)
