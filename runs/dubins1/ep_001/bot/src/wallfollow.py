import sys, time, math
sys.path.insert(0,'/memory/code')
from robot import Robot
from nav import Nav, angdiff

SIDE = sys.argv[1] if len(sys.argv)>1 else 'right'
r=Robot('/bot/src/sensors.log'); time.sleep(1)
n=Nav(r)
log=open('/bot/src/wf.out','a')
def P(*a):
    log.write(f"{time.strftime('%H:%M:%S')} {' '.join(str(x) for x in a)}\n"); log.flush()
P(f"=== wallfollow {SIDE} start ===")

# index helpers: for right-follow, wall at beam12; for left, beam4
if SIDE=='right':
    W, WF, WR, SGN = 12, 13, 11, 1   # steer>0 = away from wall(left)
else:
    W, WF, WR, SGN = 4, 3, 5, -1

def sv(s,i):
    v=s[i%16]
    return v if v and v>0 else 2.5

def wiggle(deg_target_delta, timeout=60):
    """relative turn by wiggling; positive delta = toward beam4 (steer>0 fwd)."""
    h0=None
    t0=time.time()
    while h0 is None: h0=r.heading(); time.sleep(0.1)
    target=(h0+deg_target_delta)%360
    res=n.turn_to(target, tol=15, timeout=timeout)
    return res

def front_clear(s):
    return min(sv(s,0), sv(s,1)+0.12, sv(s,15)+0.12)

goal=False
state='follow'
last_progress=time.time()
while not goal:
    if r.goal(): P("GOAL!!!", r.get(0)); n.stop(); goal=True; break
    h=n.upd(); s=r.scan()
    if h is None or not s: time.sleep(0.3); continue
    f=front_clear(s)
    w=sv(s,W); wf=sv(s,WF); wr=sv(s,WR)
    if r.bump() or f<0.22:
        # blocked ahead: back up slightly, turn away from wall
        n.cmd(0,-12); time.sleep(1.0); n.stop()
        P(f"blocked f={f:.2f} w={w:.2f}: turning {'left' if SIDE=='right' else 'right'} 60")
        res=wiggle(SGN*60)
        if res=='goal': P("GOAL"); break
        continue
    if w>0.75 and wf>0.75:
        # lost wall: turn toward wall side and creep
        P(f"lost wall w={w:.2f} wf={wf:.2f}: turn toward wall 45 + creep")
        res=wiggle(-SGN*50)
        if res=='goal': P("GOAL"); break
        # creep forward
        t0=time.time()
        while time.time()-t0<4:
            if r.goal(): P("GOAL"); goal=True; break
            s2=r.scan()
            if not s2 or front_clear(s2)<0.25 or r.bump(): break
            n.cmd(0,12); time.sleep(0.15)
        n.stop()
        continue
    # normal follow
    err = (0.30 - w)   # positive => too close => steer away (SGN*+)
    align = (wr - wf)  # wall veers away ahead => steer toward wall
    steer = max(-90, min(90, SGN*(260*err) + SGN*(-120*align)))
    thr = 16 if f>0.5 else 9
    n.cmd(steer, thr)
    time.sleep(0.15)
n.stop()
