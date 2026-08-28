import sys, time, math, statistics, threading
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
log=open('/bot/src/home.out','a')
def P(*a):
    log.write(f"{time.strftime('%H:%M:%S')} {' '.join(str(x) for x in a)}\n"); log.flush()
P("=== home start ===")

# blip heading recorder thread
bliphs=[]   # (t, heading)
def mon():
    while True:
        v=r.data.get(6); h=r.data.get(4)
        if v and v[1]=='1' and h and abs(v[0]-h[0])<0.3:
            bliphs.append((v[0], float(h[1])))
        time.sleep(0.02)
threading.Thread(target=mon,daemon=True).start()

def goalcheck():
    if r.goal():
        P("GOAL!!!", r.get(0)); n.stop()
        open('/memory/GOAL.txt','a').write(f"{time.time()} GOAL reached! {r.get(0)}\n")
        return True
    return False

def recent_blips(within=8):
    t=time.time()
    return [h for tt,h in bliphs if t-tt<within]

def circmean(hs):
    x=sum(math.cos(math.radians(h)) for h in hs)
    y=sum(math.sin(math.radians(h)) for h in hs)
    return math.degrees(math.atan2(y,x))%360

def stroke(sign, fwd, dur):
    """arc stroke: sign=+1 increase heading. fwd True/False."""
    steer = 90*sign if fwd else -90*sign
    thr = 8 if fwd else -8
    n.cmd(steer,thr)
    t0=time.time()
    while time.time()-t0<dur:
        time.sleep(0.06)
        if r.bump(): break
        s=r.scan()
        if s:
            if fwd and 0<s[0]<0.12: break
            if not fwd and 0<s[8]<0.12: break
    n.stop()

def sweep(deg=380, sign=1):
    """rotate ~deg degrees total, collecting blip headings. Returns band centers."""
    P(f"sweep {deg} sign={sign}")
    marks=len(bliphs)
    acc=0; last=r.heading(); fwd=n.clearance(0)>=n.clearance(8)
    t0=time.time()
    while acc<deg and time.time()-t0<240:
        if goalcheck(): return 'goal'
        room = n.clearance(0) if fwd else n.clearance(8)
        if room<0.14:
            fwd=not fwd
            room=n.clearance(0) if fwd else n.clearance(8)
            if room<0.14:
                n.cmd(0, -6 if fwd else 6); time.sleep(0.7); n.stop()
        dur=min(2.2,max(0.7,(room-0.08)/0.05))
        stroke(sign,fwd,dur)
        fwd=not fwd
        time.sleep(0.15)
        h=r.heading()
        if h is not None and last is not None:
            acc+=abs(angdiff(h,last)); last=h
    news=bliphs[marks:]
    if not news: return []
    # cluster into bands
    hs=sorted(h for t,h in news)
    bands=[[hs[0]]]
    for h in hs[1:]:
        if abs(angdiff(h,bands[-1][-1]))<10: bands[-1].append(h)
        else: bands.append([h])
    return [circmean(b) for b in bands]

def creep(b, dist=0.30):
    """drive along bearing b for ~dist, weaving +-8deg; return blips seen."""
    marks=len(bliphs)
    t0=time.time(); dur=dist/0.055
    while time.time()-t0<dur:
        if goalcheck(): return 'goal'
        h=r.heading(); s=r.scan()
        if h is None or not s: time.sleep(0.1); continue
        if r.bump() or (0<s[0]<0.13):
            n.stop(); P("creep blocked"); break
        weave = 10*math.sin((time.time()-t0)*1.5)
        e=angdiff((b+weave)%360,h)
        n.cmd(max(-90,min(90,4*e)), 9)
        time.sleep(0.08)
    n.stop()
    return bliphs[marks:]

target=None
nosweep=0
while True:
    if goalcheck(): break
    if target is None:
        res=sweep()
        if res=='goal': break
        if res:
            P("bands:", [round(x,1) for x in res])
            target=res[0] if len(res)==1 else circmean(res)
            # if two bands (edges of band), average; else first
        else:
            P("no blips in sweep; relocating")
            # relocate: drive forward-ish following clearest direction for ~25s
            t0=time.time()
            while time.time()-t0<25:
                if goalcheck(): break
                s=r.scan(); h=r.heading()
                if not s or h is None: time.sleep(0.1); continue
                if r.bump() or (0<s[0]<0.18):
                    # turn toward most open of beams 2,14 (or back off)
                    n.cmd(0,-10); time.sleep(0.8); n.stop()
                    sgn = 1 if (s[2] if s[2]>0 else 0) >= (s[14] if s[14]>0 else 0) else -1
                    stroke(sgn, True, 1.5); stroke(sgn, False, 1.5)
                    continue
                n.cmd(0,18); time.sleep(0.12)
            n.stop()
            continue
    P("homing along", round(target,1))
    res=creep(target)
    if res=='goal': break
    if res:
        hs=[h for t,h in res]
        target=circmean(hs)
        P("blips during creep; new target", round(target,1))
    else:
        # lost: mini sweep +-25 around target
        P("mini sweep")
        marks=len(bliphs)
        from nav2 import turn_to2
        turn_to2(n,r,(target-25)%360,tol=8,timeout=30)
        res=sweep(55,sign=1)
        if res=='goal': break
        if res:
            target=circmean(res) if len(res)>1 else res[0]
            P("re-acquired", round(target,1))
        else:
            P("band lost; full sweep next")
            target=None
n.stop()
P("home exit")
