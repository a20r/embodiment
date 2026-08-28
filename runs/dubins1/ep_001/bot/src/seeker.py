import sys, time, math, threading
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
from nav2 import turn_to2
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
log=open('/bot/src/seeker.out','a')
blog=open('/memory/blips.log','a')
def P(*a):
    log.write(f"{time.strftime('%H:%M:%S')} {' '.join(str(x) for x in a)}\n"); log.flush()
P("=== seeker start ===")

bliphs=[]
def mon():
    while True:
        v=r.data.get(6); h=r.data.get(4)
        if v and v[1]=='1' and h and abs(v[0]-h[0])<0.3:
            bliphs.append((v[0], float(h[1])))
            blog.write(f"{v[0]:.2f} h={h[1]}\n"); blog.flush()
        time.sleep(0.02)
threading.Thread(target=mon,daemon=True).start()

def goalcheck():
    if r.goal():
        P("GOAL!!!", r.get(0)); n.stop()
        open('/memory/GOAL.txt','a').write(f"{time.time()} GOAL! {r.get(0)}\n")
        return True
    return False

def circmean(hs):
    x=sum(math.cos(math.radians(h)) for h in hs); y=sum(math.sin(math.radians(h)) for h in hs)
    return math.degrees(math.atan2(y,x))%360

def stroke(sign,fwd,dur):
    steer=90*sign if fwd else -90*sign
    n.cmd(steer, 8 if fwd else -8)
    t0=time.time()
    while time.time()-t0<dur:
        time.sleep(0.06)
        if r.bump(): break
        s=r.scan()
        if s and ((fwd and 0<s[0]<0.12) or (not fwd and 0<s[8]<0.12)): break
    n.stop()

def sweep(deg,sign=1,tlim=200):
    marks=len(bliphs)
    acc=0; last=r.heading(); fwd=n.clearance(0)>=n.clearance(8)
    t0=time.time()
    while acc<deg and time.time()-t0<tlim:
        if goalcheck(): return 'goal'
        room=n.clearance(0) if fwd else n.clearance(8)
        if room<0.14:
            fwd=not fwd
            room=n.clearance(0) if fwd else n.clearance(8)
            if room<0.14:
                n.cmd(0,-6 if fwd else 6); time.sleep(0.7); n.stop()
        stroke(sign,fwd,min(2.2,max(0.7,(room-0.08)/0.05)))
        fwd=not fwd
        time.sleep(0.12)
        h=r.heading()
        if h is not None and last is not None:
            acc+=abs(angdiff(h,last)); last=h
    news=[h for t,h in bliphs[marks:]]
    return circmean(news) if news else None

def creep(b,dist=0.3):
    marks=len(bliphs)
    t0=time.time(); dur=dist/0.055
    while time.time()-t0<dur:
        if goalcheck(): return 'goal'
        h=r.heading(); s=r.scan()
        if h is None or not s: time.sleep(0.1); continue
        if r.bump() or (0<s[0]<0.13):
            n.stop(); return 'blocked'
        weave=10*math.sin((time.time()-t0)*1.5)
        n.cmd(max(-90,min(90,4*angdiff((b+weave)%360,h))),9)
        time.sleep(0.08)
    n.stop()
    news=[h for t,h in bliphs[marks:]]
    return circmean(news) if news else None

def home(b0):
    P("HOME start b=%.1f"%b0)
    b=b0; fails=0
    while fails<3:
        if goalcheck(): return True
        res=creep(b)
        if res=='goal': return True
        if res=='blocked':
            P("blocked while homing; nudge around")
            # try to go around: back off, turn 45 toward more open side, forward, re-sweep
            n.cmd(0,-10); time.sleep(1.0); n.stop()
            s=r.scan(); h=r.heading()
            sgn=1 if s and (s[2] if s[2]>0 else 0)>=(s[14] if s[14]>0 else 0) else -1
            turn_to2(n,r,(h+50*sgn)%360,tol=15,timeout=30)
            t0=time.time()
            while time.time()-t0<3:
                if goalcheck(): return True
                ss=r.scan()
                if not ss or (0<ss[0]<0.2) or r.bump(): break
                n.cmd(0,12); time.sleep(0.12)
            n.stop()
            res=None
        if isinstance(res,float):
            b=res; fails=0; P("track b=%.1f"%b)
        else:
            P("mini sweep around b")
            turn_to2(n,r,(b-30)%360,tol=10,timeout=40)
            res=sweep(65,1,90)
            if res=='goal': return True
            if isinstance(res,float):
                b=res; fails=0; P("reacq b=%.1f"%b)
            else:
                fails+=1
                turn_to2(n,r,b,tol=10,timeout=40)
    P("home lost band")
    return False

W,WF,WR,SGN=12,13,11,1
def sv(s,i):
    v=s[i%16]; return v if v and v>0 else 2.5
def fc(s): return min(sv(s,0),sv(s,1)+0.12,sv(s,15)+0.12)

lastn=len(bliphs)
while True:
    if goalcheck(): break
    if len(bliphs)>lastn:
        b=circmean([h for t,h in bliphs[lastn:]])
        lastn=len(bliphs)
        n.stop()
        if home(b): break
        lastn=len(bliphs)
        continue
    h=n.upd(); s=r.scan()
    if h is None or not s: time.sleep(0.3); continue
    f=fc(s); w=sv(s,W); wf=sv(s,WF); wr=sv(s,WR)
    if r.bump() or f<0.22:
        n.cmd(0,-12); time.sleep(1.0); n.stop()
        h0=r.heading()
        if h0 is not None: turn_to2(n,r,(h0+60)%360,tol=15,timeout=40)
        continue
    if w>0.75 and wf>0.75:
        h0=r.heading()
        if h0 is not None: turn_to2(n,r,(h0-50)%360,tol=15,timeout=40)
        t0=time.time()
        while time.time()-t0<4:
            if r.goal() or len(bliphs)>lastn: break
            s2=r.scan()
            if not s2 or fc(s2)<0.25 or r.bump(): break
            n.cmd(0,12); time.sleep(0.15)
        n.stop()
        continue
    err=(0.30-w); align=(wr-wf)
    steer=max(-90,min(90,SGN*260*err-SGN*120*align))
    n.cmd(steer, 18 if f>0.5 else 9)
    time.sleep(0.15)
n.stop()
P("seeker exit")
