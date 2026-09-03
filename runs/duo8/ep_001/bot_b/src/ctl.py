import robot as R, time, math

TICK_M = 0.0013  # m per encoder tick
FRONT = 12

def angdiff(a,b):
    d=(a-b)%360
    if d>180: d-=360
    return d

def turn_deg(delta, w=15):
    """delta>0 -> heading increases (front moves toward higher ray idx)."""
    target=(R.heading()+delta)%360
    t0=time.time()
    while time.time()-t0<20:
        d=angdiff(target,R.heading())
        if abs(d)<3: break
        s=max(4,min(w,abs(d)*0.6))
        if d>0: R.drive(s,-s)
        else: R.drive(-s,s)
        time.sleep(0.08)
    R.stop()

def face_ray(i, w=15):
    turn_deg(((i-FRONT+8)%16-8)*22.5, w)

def front_dist(l=None):
    l=l or R.lidar()
    v=l[FRONT]
    return 3.0 if v<0 else v

def forward(dist, v=30, min_front=0.18):
    e0=sum(R.enc())/2
    t0=time.time()
    while time.time()-t0<30:
        gone=(sum(R.enc())/2-e0)*TICK_M
        if gone>=dist: break
        l=R.lidar()
        f=front_dist(l)
        # also check diagonal front rays
        f11=l[11] if l[11]>0 else 3.0
        f13=l[13] if l[13]>0 else 3.0
        if f<min_front or min(f11,f13)<0.12:
            break
        s=v if f>0.4 else 12
        R.drive(s,s)
        time.sleep(0.05)
    R.stop()
    return (sum(R.enc())/2-e0)*TICK_M
