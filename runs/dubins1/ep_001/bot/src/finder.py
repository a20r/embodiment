import sys, time, math
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff
from nav2 import turn_to2
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
log=open('/bot/src/finder.out','a')
def P(*a): log.write(f"{time.strftime('%H:%M:%S')} {' '.join(str(x) for x in a)}\n"); log.flush()
P("finder start")
def sig(h,s):
    ok3=any(s[i]>1.3 and abs(angdiff((h+22.5*i)%360,5))<22 for i in range(16))
    ok93=any(s[i]>2.3 and abs(angdiff((h+22.5*i)%360,86))<22 for i in range(16))
    return ok3 and ok93
def pick_side(t):
    return (12,13,11,1) if int(t/150)%2==0 else (4,3,5,-1)
W,WF,WR,SGN=12,13,11,1
def sv(s,i):
    v=s[i%16]; return v if v and v>0 else 2.5
def fc(s): return min(sv(s,0),sv(s,1)+0.12,sv(s,15)+0.12)
t0=time.time(); found=False
while time.time()-t0<1500:
    if r.goal(): P("GOAL"); n.stop(); sys.exit()
    h=n.upd(); s=r.scan()
    if h is None or not s: time.sleep(0.2); continue
    if r.get(6)=='1' or (r.d6t and time.time()-r.d6t<10): P("d6 hot! stop"); n.stop(); found=True; break
    if sig(h,s): P(f"signature at h={h:.0f}"); n.stop(); found=True; break
    mi=max(range(16),key=lambda i:s[i])
    if s[mi]>2.2:
        wa=(h+22.5*mi)%360
        P(f"long beam {s[mi]:.2f} wa={wa:.0f}: dashing")
        turn_to2(n,r,wa,tol=18,timeout=60)
        td=time.time()
        while time.time()-td<20:
            if r.goal() or r.get(6)=='1' or (r.d6t and time.time()-r.d6t<5): break
            s2=r.scan(); h2=r.heading()
            if not s2 or not h2: time.sleep(0.1); continue
            if 0<s2[0]<0.18 or r.bump(): break
            n.cmd(max(-90,min(90,3*angdiff(wa,h2))),16); time.sleep(0.15)
        n.stop(); continue
    W,WF,WR,SGN=pick_side(time.time()-t0)
    f=fc(s); w=sv(s,W); wf=sv(s,WF); wr=sv(s,WR)
    if r.bump() or f<0.22:
        n.cmd(0,-12); time.sleep(1.0); n.stop()
        turn_to2(n,r,(h+60)%360,tol=15,timeout=40); continue
    if w>0.75 and wf>0.75:
        turn_to2(n,r,(h-50)%360,tol=15,timeout=40)
        te=time.time()
        while time.time()-te<4:
            s2=r.scan()
            if not s2 or fc(s2)<0.25 or r.bump(): break
            n.cmd(0,12); time.sleep(0.15)
        n.stop(); continue
    steer=max(-90,min(90,SGN*260*(0.30-w)-SGN*120*(wr-wf)))
    n.cmd(steer,16 if f>0.5 else 9); time.sleep(0.15)
n.stop()
P("phase A done found=",found)
def stroke(steer,thr,dur):
    n.cmd(steer,thr); te=time.time()
    while time.time()-te<dur:
        time.sleep(0.12)
        if r.goal(): P("GOAL!!!"); n.stop(); return 'goal'
        if r.get(6)=='1' or (r.d6t and time.time()-r.d6t<5): P("d6 hot!!",r.scan()); n.stop(); return 'd6'
        if r.bump(): break
        ss=r.scan()
        if ss and thr>0 and 0<ss[0]<0.12: break
        if ss and thr<0 and 0<ss[8]<0.12: break
    n.stop(); return None
res=None
for cycle in range(6):
    for steer in (0,45,90,-45,-90,70,-70,20,-20):
        res=stroke(steer,8,3.0)
        if res: break
        res=stroke(steer,-8,3.2)
        if res: break
    P(f"cycle {cycle} d6t={r.d6t}")
    if res: break
P("phase B end res=",res,"d0=",r.get(0))
if res=='d6':
    for k in range(60):
        if r.goal(): P("GOAL!!!",r.get(0)); break
        time.sleep(1)
n.stop()
