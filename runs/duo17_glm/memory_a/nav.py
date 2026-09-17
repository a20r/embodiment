import sys, time, math
sys.path.insert(0,"/bot/src")
from robot import speed, turn, stop, send, recv, read
from robust import rline

def hd():
    v=rline("d4",0.5)
    try: return float(v)
    except: return None

def sc():
    v=rline("d2",0.5)
    try: return [float(x) for x in v.split(",")]
    except: return None

def odo():
    v=rline("d9",0.5)
    try: return float(v)
    except: return None

def ang_err(cur, target):
    return (target-cur+540)%360-180

def turn_to(target, tol=4, maxt=12):
    # d7>0 => heading decreases (clockwise). e = target-cur (CCW positive)
    t0=time.time()
    while time.time()-t0<maxt:
        c=hd()
        if c is None: continue
        e=ang_err(c,target)
        if abs(e)<tol:
            turn(0); return c
        turn(max(-50,min(50, -e*2)))   # e<0 (need CW) -> positive d7
        time.sleep(0.03)
    turn(0)
    return hd()

def drive(dist, sp=3, stop_at=0.12, maxt=30):
    t0=time.time()
    s0=odo()
    while time.time()-t0<maxt:
        s=odo()
        if s is not None and s0 is not None and abs(s-s0)>=dist: break
        f=sc()
        if f:
            front=min(f[15],f[0],f[1],f[2])
            if 0<=front<stop_at:
                break
        time.sleep(0.03)
    speed(0)
