import sys, time, math
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
from nav2 import turn_to2
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
log=open('/bot/src/wf2.out','a')
def P(*a):
    log.write(f"{time.strftime('%H:%M:%S')} {' '.join(str(x) for x in a)}\n"); log.flush()
P("=== wf2 start ===")
W,WF,WR,SGN=12,13,11,1
def sv(s,i):
    v=s[i%16]; return v if v and v>0 else 2.5
def fc(s): return min(sv(s,0),sv(s,1)+0.12,sv(s,15)+0.12)

def pursue():
    P("d6=1 PURSUIT")
    n.stop(); time.sleep(0.4)
    t0=time.time(); prev=None; prevh=None; target=None
    while time.time()-t0<40:
        if r.goal(): P("GOAL!!!",r.get(0)); n.stop(); return True
        if r.get(6)!='1' and time.time()-r.d6t>10:
            P("lost d6"); return False
        h=r.heading(); s=r.scan()
        if h is None or not s: time.sleep(0.1); continue
        if prev is not None and abs(angdiff(h,prevh))<4:
            for i in range(16):
                a,b=prev[i],s[i]
                if a>0 and b>0 and abs(b-a)>0.15 and min(a,b)<1.3:
                    target=(h+22.5*i)%360
                    P(f"blob beam{i} wa={target:.0f} d={min(a,b):.2f}")
        prev,prevh=s,h
        if target is not None:
            e=angdiff(target,h)
            if abs(e)>25:
                turn_to2(n,r,target,tol=20,timeout=25)
            else:
                f=s[0]
                if 0<f<0.11 or r.bump():
                    n.cmd(0,-8); time.sleep(0.6); n.stop()
                else:
                    n.cmd(max(-90,min(90,3*e)),14); time.sleep(0.15)
        else:
            time.sleep(0.3)
    return False

while True:
    if r.goal(): P("GOAL!!!",r.get(0)); n.stop(); break
    if r.get(6)=='1' or time.time()-r.d6t<8:
        if pursue(): break
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
            if r.goal() or r.get(6)=='1': break
            s2=r.scan()
            if not s2 or fc(s2)<0.25 or r.bump(): break
            n.cmd(0,12); time.sleep(0.15)
        n.stop()
        continue
    err=(0.30-w); align=(wr-wf)
    steer=max(-90,min(90,SGN*260*err - SGN*120*align))
    n.cmd(steer, 16 if f>0.5 else 9)
    time.sleep(0.15)
n.stop()
P("wf2 exit")
