import sys, time, math, statistics
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
from nav2 import turn_to2
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
log=open('/bot/src/wf3.out','a')
blog=open('/memory/blips.log','a')
def P(*a):
    log.write(f"{time.strftime('%H:%M:%S')} {' '.join(str(x) for x in a)}\n"); log.flush()
def B(*a):
    blog.write(f"{time.time():.2f} {' '.join(str(x) for x in a)}\n"); blog.flush()
P("=== wf3 start ===")
W,WF,WR,SGN=12,13,11,1
def sv(s,i):
    v=s[i%16]; return v if v and v>0 else 2.5
def fc(s): return min(sv(s,0),sv(s,1)+0.12,sv(s,15)+0.12)
def goalcheck():
    if r.goal():
        P("GOAL!!!", r.get(0)); B("GOAL", r.get(0)); n.stop(); return True
    return False

def baseline(dur=1.2):
    t0=time.time(); acc=[]
    while time.time()-t0<dur:
        s=r.scan()
        if s: acc.append(s)
        time.sleep(0.08)
    if not acc: return None
    return [statistics.median(s[i] for s in acc) for i in range(16)]

def pursue():
    n.stop(); time.sleep(0.3)
    h=r.heading(); s=r.scan()
    B("blip h=%.1f"%(h or -1), "scan="+(",".join("%.2f"%x for x in s) if s else "?"))
    P("BLIP pursue start h=",h)
    tstart=time.time()
    while time.time()-tstart<90:
        if goalcheck(): return True
        base=baseline(1.0)
        h=r.heading()
        if base is None or h is None: continue
        # watch for moving blob
        blob=None; t0=time.time()
        while time.time()-t0<8:
            if goalcheck(): return True
            s=r.scan()
            if s:
                cand=[]
                for i in range(16):
                    a,b=base[i],s[i]
                    if a>0 and b>0 and abs(b-a)>0.15 and min(a,b)<1.5:
                        cand.append((abs(b-a),i,min(a,b)))
                if cand:
                    cand.sort(reverse=True)
                    _,i,d=cand[0]
                    blob=((h+22.5*i)%360, d)
                    break
            time.sleep(0.08)
        if blob is None:
            P("no blob; recent d6?", time.time()-r.d6t)
            if time.time()-r.d6t>25: return False
            continue
        wa,d=blob
        P(f"blob wa={wa:.0f} d={d:.2f} chasing")
        B(f"blob wa={wa:.0f} d={d:.2f}")
        # chase: steer toward wa
        e=angdiff(wa,r.heading() or wa)
        if abs(e)>100:
            # drive backward away? just turn
            turn_to2(n,r,wa,tol=30,timeout=20)
        t0=time.time()
        while time.time()-t0<12:
            if goalcheck(): return True
            hh=r.heading(); ss=r.scan()
            if hh is None or not ss: time.sleep(0.1); continue
            f=ss[0]
            if 0<f<0.12 or r.bump():
                n.cmd(0,-10); time.sleep(0.5); n.stop(); break
            n.cmd(max(-90,min(90,3*angdiff(wa,hh))), 30)
            time.sleep(0.12)
        n.stop()
    return False

last_d6t=r.d6t
while True:
    if goalcheck(): break
    if r.get(6)=='1' or r.d6t>last_d6t:
        last_d6t=r.d6t
        if pursue(): break
        last_d6t=max(last_d6t,r.d6t)
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
    n.cmd(steer, 18 if f>0.5 else 9)
    time.sleep(0.15)
n.stop()
P("wf3 exit")
