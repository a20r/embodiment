import sys,time,math,json
sys.path.insert(0,'/bot/src'); import rio as m
def f(x,default=None):
    try: return float(x)
    except: return default
def heading():
    for _ in range(3):
        h=f(m.rd('d4'))
        if h is not None: return h
    return None
def enc():
    a=f(m.rd('d9')); b=f(m.rd('d6')); return a,b
def lidar():
    s=m.rd('d2')
    if not s: return None
    v=[f(x,-1.0) for x in s.split(',')]
    return v if len(v)==16 else None
def status(): return m.rd('d3')
def stop(): m.wr('d1','0'); m.wr('d7','0')
def drive(a,b): m.wr('d1',str(int(a))); m.wr('d7',str(int(b)))
def angdiff(a,b):
    d=(a-b+180)%360-180; return d
def turn_to(target, tol=4, spd=40, tmax=8):
    t0=time.time()
    while time.time()-t0<tmax:
        h=heading()
        if h is None: continue
        d=angdiff(target,h)
        if abs(d)<=tol: stop(); time.sleep(0.15); h=heading(); 
        if abs(d)<=tol: 
            if abs(angdiff(target,h))<=tol: return h
            continue
        s=spd if abs(d)>25 else max(15,spd//2)
        if d>0: drive(s,-s)   # need heading increase: d1 forward
        else: drive(-s,s)
        time.sleep(0.05)
    stop(); return heading()
def forward(counts, spd=60, tmax=15, min_front=0.15, hold_heading=None):
    """drive straight by encoder counts (neg = reverse). Stops if front lidar < min_front."""
    a0,b0=enc(); t0=time.time(); sgn=1 if counts>0 else -1
    while time.time()-t0<tmax:
        a,b=enc()
        if a is None or b is None: continue
        da=a-a0; db=b-b0; done=(da+db)/2*sgn
        if done>=abs(counts): break
        if sgn>0:
            L=lidar()
            if L and 0<L[0]<min_front: print('front obstacle',L[0]); break
        corr=0
        if hold_heading is not None:
            h=heading()
            if h is not None:
                e=angdiff(hold_heading,h)   # >0 need heading increase => more d1
                corr=max(-15,min(15,e*1.0))
        else:
            corr=max(-15,min(15,(db-da)*0.3))
        drive(sgn*spd+corr, sgn*spd-corr)
        time.sleep(0.05)
    stop(); time.sleep(0.15); a,b=enc(); return (a-a0,b-b0)
def scan_abs():
    """return list of (abs_angle_deg, range) from current pose"""
    h=heading(); L=lidar()
    if h is None or L is None: return None,[]
    return h,[((h+22.5*k)%360, L[k]) for k in range(16)]
if __name__=='__main__':
    cmd=sys.argv[1]
    if cmd=='turn': print(turn_to(float(sys.argv[2])))
    elif cmd=='fwd': print(forward(int(sys.argv[2]), spd=int(sys.argv[3]) if len(sys.argv)>3 else 60))
    elif cmd=='stop': stop()
    elif cmd=='scan':
        h,s=scan_abs(); print('heading',h); 
        for a,r in sorted(s): print('  %6.1f deg: %.3f'%(a,r))
    elif cmd=='st': print(status(), 'hd',heading(), 'enc',enc(), 'd11',m.rd('d11'),'d0',m.rd('d0'),'d5',m.rd('d5'))
