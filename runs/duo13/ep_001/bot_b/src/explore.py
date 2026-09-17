import time, math
D='/dev/robot/'
def read(p, tries=3):
    for _ in range(tries):
        try:
            with open(D+p) as f: s=f.read().strip()
            if s: return s
        except Exception: pass
        time.sleep(0.02)
    return ""
def lidar():
    s=read('d2')
    try: return [float(x) for x in s.split(',')]
    except: return None
def status():
    s=read('d3')
    try:
        parts=dict(p.split('=') for p in s.split())
        return int(parts['tick']), int(parts['goal']), int(parts['here'])
    except: return 0,0,0
def hd():
    try: return float(read('d4'))
    except: return None
def motor(l,r):
    try:
        with open(D+'d1','w') as f: f.write(f"{l:.1f}\n")
        with open(D+'d7','w') as f: f.write(f"{r:.1f}\n")
    except Exception: pass
def radio_tx(msg):
    try:
        with open(D+'d8','w') as f: f.write(msg+"\n")
    except Exception: pass
def radio_rx():
    try:
        with open(D+'d10') as f:
            s=f.read().strip()
            return s if s else None
    except Exception: return None
def angdiff(a,b): return (a-b+180)%360-180
LOG=open('/memory/maplog.csv','a')
def log(line): LOG.write(line+"\n"); LOG.flush()

CRUISE=35; TURN_S=20
state='ESCAPE'; tgt_h=None; last_tx=0; last_log=0; same_cnt=0; last_front=None
tick,goal,here=status()
log(f"# start tick={tick} goal={goal} here={here}")
t_start=time.time()
try:
  while time.time()-t_start < 3000:
    now=time.time()
    tick,goal,here=status()
    if now-last_tx>2.0:
        last_tx=now
        radio_tx(f"R1 t={tick} h={hd()} st={state}")
        msg=radio_rx()
        if msg: log(f"RX,{tick},{msg}"); print("RX:",msg,flush=True)
    if goal or here:
        log(f"GOALFLAG,{tick},{goal},{here},{read('d0')},{read('d11')}")
        print("GOALFLAG",goal,here,flush=True)
    L=lidar(); h=hd()
    if now-last_log>1.0 and L and h is not None:
        last_log=now
        log(f"S,{tick},{h:.1f},{read('d6')},{read('d5')},{read('d11')},{read('d0')},"+",".join(f"{v:.2f}" for v in L))
    if L is None or h is None: motor(0,0); time.sleep(0.05); continue
    B=[v if v>0 else 9.9 for v in L]
    front=min(B[15],B[0],B[1])
    if state=='ESCAPE':
        # rotate toward widest beam then creep
        bi=max(range(16), key=lambda i:B[i])
        tgt=(h+22.5*bi)%360
        err=angdiff(tgt,h)
        if abs(err)<10: state='ROAM'; tgt_h=h
        else:
            s=TURN_S
            motor(s if err>0 else -s, -s if err>0 else s)
    elif state=='ROAM':
        if tgt_h is None: tgt_h=h
        err=angdiff(tgt_h,h)
        if front<0.28:
            motor(0,0); time.sleep(0.05)
            left=min(B[9:15]); right=min(B[1:8])
            if left>right: tgt_h=(h-45)%360
            elif right>left: tgt_h=(h+45)%360
            else: tgt_h=(h+90)%360
            state='TURN'
        elif front<0.5 and abs(err)<20:
            # open space ahead-ish: adopt new heading toward widest nearby gap
            bi=max(range(15,16+2), key=lambda i:B[i%16])
            pass
        corr=max(-14,min(14,err*1.6))
        spd=CRUISE if front>0.8 else (22 if front>0.45 else 14)
        motor(spd+corr, spd-corr)
    elif state=='TURN':
        err=angdiff(tgt_h,h)
        if abs(err)<6: state='ROAM'
        else:
            s=TURN_S
            motor(s if err>0 else -s, -s if err>0 else s)
    time.sleep(0.02)
finally:
    motor(0,0); log("# exit")
