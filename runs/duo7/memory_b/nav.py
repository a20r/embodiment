import time, math, threading, sys
sys.path.insert(0,'/bot/src')
from lib import rd, wr, drive, stop, tx

LOG='/tmp/nav.log'
def log(m):
    with open(LOG,'a') as f: f.write('%.1f %s\n'%(time.time(),m))

def lidar(prev=[None]):
    while True:
        try:
            v=[float(x) for x in rd('d3').split(',')]
            if len(v)==16:
                if prev[0]:
                    v=[v[i] if v[i]>=0 else prev[0][i] for i in range(16)]
                else:
                    v=[x if x>=0 else 3.0 for x in v]
                prev[0]=v
                return v
        except: pass
def heading():
    while True:
        try: return float(rd('d1'))
        except: pass
def goal():
    try:
        return int(rd('d6').split('goal=')[1].split()[0])
    except: return 0

def angdiff(a,b):
    d=(a-b)%360
    if d>180: d-=360
    return d

def ray_at(l,h,world_ang):
    # ray i world angle = h + 22.5*i ; find nearest ray
    i=int(round(((world_ang-h)%360)/22.5))%16
    return l[i]

def turn_to(target):
    # increase heading with drive(+,-)
    while True:
        h=heading()
        e=angdiff(target,h)
        if abs(e)<4:
            stop(); time.sleep(0.15)
            h=heading(); e=angdiff(target,h)
            if abs(e)<6: return
            continue
        s=max(12,min(60,abs(e)*1.2))
        if e>0: drive(int(s),int(-s))
        else: drive(int(-s),int(s))
        time.sleep(0.04)

# dead reckoning
X=[0.0]; Y=[0.0]
K=0.0065
def integrate(cmdl,cmdr,dt,h):
    v=K*(cmdl+cmdr)/2.0
    hr=math.radians(h)
    X[0]+=v*dt*math.cos(hr); Y[0]+=v*dt*math.sin(hr)

def straight(target_h, maxtime=8):
    """Drive straight along target_h until front blocked or opening appears.
       Returns reason."""
    t0=time.time(); tprev=t0
    # initial side state
    l=lidar(); h=heading()
    右0=ray_at(l,h,(target_h+90)%360)
    左0=ray_at(l,h,(target_h-90)%360)
    right_closed = 右0<0.45
    left_closed = 左0<0.45
    traveled_est=0.0
    cl=cr=0
    while True:
        now=time.time(); dt=now-tprev; tprev=now
        h=heading()
        integrate(cl,cr,dt,h)
        traveled_est+=K*(cl+cr)/2.0*dt
        l=lidar()
        if goal(): stop(); return 'GOAL'
        front=ray_at(l,h,target_h)
        f2=min(front, ray_at(l,h,(target_h+22.5)%360), ray_at(l,h,(target_h-22.5)%360))
        r=ray_at(l,h,(target_h+90)%360)
        lf=ray_at(l,h,(target_h-90)%360)
        if front<0.22 or f2<0.13:
            stop(); return 'blocked'
        if right_closed and r>0.6 and traveled_est>0.15:
            stop(); return 'open_right'
        if left_closed and lf>0.6 and traveled_est>0.15:
            stop(); return 'open_left'
        if not right_closed and r<0.45: right_closed=True
        if not left_closed and lf<0.45: left_closed=True
        if now-t0>maxtime: stop(); return 'timeout'
        # steering: heading hold + centering
        e=angdiff(target_h,h)
        cen=0.0
        if r<0.5 and lf<0.5: cen=(r-lf)*40
        steer=max(-28,min(28, e*1.5+cen))
        base=60 if front>0.5 else 35
        cl=int(base+steer); cr=int(base-steer)
        drive(cl,cr)
        time.sleep(0.06)

def scan4(axis0):
    stop(); time.sleep(0.2)
    l=lidar(); h=heading()
    d={}
    for k,ang in enumerate([axis0,(axis0+90)%360,(axis0+180)%360,(axis0+270)%360]):
        d[ang]=max(ray_at(l,h,ang), ray_at(l,h,(ang+10)%360), ray_at(l,h,(ang-10)%360))
    return d

def main():
    # determine maze axis from most open direction
    l=lidar(); h=heading()
    best=max(range(16),key=lambda i:l[i])
    axis=(h+22.5*best)%360
    axis0=axis%90
    log('axis0=%.1f start h=%.1f'%(axis0,h))
    cur=axis
    last_bcast=0
    visits={}
    while True:
        if goal():
            stop(); log('GOAL at %.2f %.2f'%(X[0],Y[0])); tx('A GOAL'); time.sleep(2); continue
        # choose direction: right-hand rule relative to cur
        d=scan4(axis0)
        prefs=[(cur+90)%360, cur, (cur+270)%360, (cur+180)%360]  # right, straight, left, back
        choice=None
        for p in prefs:
            key=min(d.keys(), key=lambda a: min((a-p)%360,(p-a)%360))
            if d[key]>0.55:
                choice=key; break
        if choice is None:
            choice=max(d.keys(), key=lambda a:d[a])
        log('at %.2f,%.2f cur=%.0f scan=%s -> %.0f'%(X[0],Y[0],cur,{int(a):round(v,2) for a,v in d.items()},choice))
        turn_to(choice)
        cur=choice
        r=straight(cur)
        log('straight done: %s pos %.2f %.2f'%(r,X[0],Y[0]))
        if r=='GOAL':
            log('GOAL!'); tx('A GOAL'); time.sleep(2)
        now=time.time()
        if now-last_bcast>4:
            last_bcast=now; tx('A pos %.2f %.2f'%(X[0],Y[0]))
main()
