from rob import *
import math, statistics
def wrap(a): return (a+180)%360-180
def heading(n=3):
    v=[]
    for i in range(n):
        s=rd('d4',0.3)
        if s: v.append(float(s))
    if not v: return None
    # circular median approx: use mean of unit vectors
    x=sum(math.cos(math.radians(a)) for a in v); y=sum(math.sin(math.radians(a)) for a in v)
    return math.degrees(math.atan2(y,x))%360
def enc():
    a=rd('d9',0.3); b=rd('d6',0.3)
    return (int(a) if a else None, int(b) if b else None)
def stop(): wr('d1','0'); wr('d7','0')
def motors(a,b): wr('d1',str(int(a))); wr('d7',str(int(b)))
def spin_to(target, tol=5, maxs=80, timeout=15):
    """turn in place to absolute heading target (deg, d4 units)"""
    t0=time.time()
    while time.time()-t0<timeout:
        h=heading(3)
        if h is None: continue
        err=wrap(target-h)
        if abs(err)<tol:
            stop(); time.sleep(0.2)
            h=heading(3); err=wrap(target-h)
            if abs(err)<tol: return h
            continue
        s=max(15,min(maxs,abs(err)*1.5))
        if err>0: motors(s,-s)   # d1+ raises heading
        else: motors(-s,s)
        time.sleep(0.05)
    stop(); return heading(3)
def spin_by(delta, **kw):
    h=heading(3); return spin_to((h+delta)%360, **kw)
def drive(speed, ticks=None, dur=None, hold=None, minfront=0.12, log=None):
    """drive with heading hold; stop on bumper, front range<minfront, ticks, or duration"""
    if hold is None: hold=heading(3)
    e0=enc(); t0=time.time(); reason='timeout'
    while True:
        if dur and time.time()-t0>dur: reason='dur'; break
        h=heading(1); 
        if h is None: continue
        err=wrap(hold-h)
        corr=max(-20,min(20,err*1.0))
        if speed>=0: motors(speed+corr, speed-corr)
        else: motors(speed+corr, speed-corr)
        bf=rd('d5',0.2); bb=rd('d0',0.2)
        if speed>0 and bf=='1': reason='front_bump'; break
        if speed<0 and bb=='1': reason='rear_bump'; break
        r=ranges()
        if r and speed>0 and 0<r[0]<minfront: reason='front_range'; break
        if r and speed<0 and 0<r[8]<minfront: reason='rear_range'; break
        if ticks:
            e=enc()
            if e[0] is not None and e0[0] is not None and abs(e[0]-e0[0])+abs(e[1]-e0[1])>=2*ticks: reason='ticks'; break
        if log: log(h,r)
        time.sleep(0.03)
    stop()
    e1=enc()
    return reason, (e1[0]-e0[0] if None not in (e1[0],e0[0]) else None), (e1[1]-e0[1] if None not in (e1[1],e0[1]) else None)
def status():
    r=ranges(); h=heading(3); e=enc()
    return f"h={h:.1f} enc={e} d3={rd('d3')} d0={rd('d0')} d5={rd('d5')} d11={rd('d11')}\n r={r}"
