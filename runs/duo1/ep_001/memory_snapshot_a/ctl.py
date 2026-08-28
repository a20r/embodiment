import rob, time, math

def h_avg(n=4):
    xs=ys=0.0
    for _ in range(n):
        h=math.radians(rob.heading()); xs+=math.cos(h); ys+=math.sin(h)
        time.sleep(0.03)
    return math.degrees(math.atan2(ys,xs))%360

def angdiff(a,b):
    return (a-b+540)%360-180

def turn_to(target, tol=4, timeout=30):
    t0=time.time()
    while time.time()-t0<timeout:
        h=h_avg(3)
        e=angdiff(target,h)   # want heading to increase by e
        if abs(e)<tol:
            rob.motors(0,0); return True
        # motors(l>r) decreases heading => to increase heading use r>l
        s=max(5,min(60,abs(e)*1.2))
        if e>0: rob.motors(-s,s)
        else:   rob.motors(s,-s)
        time.sleep(0.15)
    rob.motors(0,0); return False

def face_beam(i):
    """turn so that what is currently at beam i becomes beam 0"""
    h=h_avg()
    target=(h+22.5*i)%360
    return turn_to(target)

def forward(speed=20, dur=2.0):
    o0=rob.odo()
    rob.motors(speed,speed); time.sleep(dur); rob.motors(0,0)
    return rob.odo()-o0
