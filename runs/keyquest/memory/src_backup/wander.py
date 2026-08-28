import sys, time, math, json
sys.path.insert(0,'/bot/src')
from lib import *

LOG=open('/bot/wander.log','a',buffering=1)
def log(*a): print(time.strftime('%H:%M:%S'),*a,file=LOG)

def lid_unused():
    for _ in range(3):
        try:
            v=lidar()
            if len(v)==16: return v
        except Exception: pass
        time.sleep(0.05)
    return v

def smooth(v, prev):
    # replace -1 dropouts with prev
    return [ (prev[i] if v[i]<0 else v[i]) for i in range(16) ]

def angdiff(a,b):
    d=(a-b)%360
    if d>180: d-=360
    return d

def turn_to(target, tol=12, tmax=8):
    t0=time.time()
    while time.time()-t0<tmax:
        h=hdg()
        d=angdiff(target,h)
        if abs(d)<tol:
            drive(0,0); return True
        # positive d => need heading increase => drive(-s, s)
        s=4 if abs(d)<40 else 7
        if d>0: drive(-s,s)
        else: drive(s,-s)
        time.sleep(0.15)
    drive(0,0); return False

def enc():
    try: return (float(rd(6)), float(rd(2)))  # left, right
    except: return None

# pose integration
x,y=0.0,0.0
last=enc()
prev=[0.5]*16
pose_log=open('/memory/trace.csv','a',buffering=1)
t_start=time.time()
i=0
while True:
    i+=1
    if goal():
        drive(0,0)
        log('GOAL REACHED at pose',x,y)
        with open('/memory/GOAL.txt','a') as f:
            f.write(f'GOAL reached, pose=({x:.1f},{y:.1f}) counts, time={time.time()-t_start:.0f}s\n')
        break
    lv=lidar()
    if lv is None: time.sleep(0.1); continue
    v=smooth(lv,prev); prev=v
    h=hdg()
    # update odometry
    e=enc()
    if e and last:
        dl=e[0]-last[0]; dr=e[1]-last[1]
        d=(dl+dr)/2.0
        x+=d*math.cos(math.radians(h)); y+=d*math.sin(math.radians(h))
    last=e
    front=min(v[15],v[0],v[1])
    if i%10==0:
        pose_log.write(f'{time.time()-t_start:.1f},{x:.1f},{y:.1f},{h:.1f},{front:.3f},{rd(4)},{rd(8)}\n')
    if front>0.22:
        # steer toward more open side gently
        l=min(v[1],v[2]); r=min(v[14],v[15])
        if l>r+0.1: drive(4,6)
        elif r>l+0.1: drive(6,4)
        else: drive(6,6)
        time.sleep(0.2)
    else:
        drive(0,0)
        # choose widest direction: score each beam by window min
        best=None;bs=-1
        for k in range(16):
            w=min(v[(k-1)%16],v[k],v[(k+1)%16])
            if w>bs: bs=w;best=k
        target=(h+best*22.5)%360
        log(f'blocked front={front:.2f} turn to beam {best} (clr {bs:.2f}) hdg {h:.0f}->{target:.0f}')
        turn_to(target)
