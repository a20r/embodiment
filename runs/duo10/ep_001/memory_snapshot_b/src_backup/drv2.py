import time, math
from rob import *
from nav import rot_to, d5val

AXES={0:35.0, 1:125.0, 2:215.0, 3:305.0}  # learned estimates
def near_axis_key(h):
    return min(AXES, key=lambda k: abs((h-AXES[k]+180)%360-180))
def front(): return ranges()[2]

def try_move(secs=1.0):
    f0=front()
    if f0<0: f0=front()
    t0=time.time()
    while time.time()-t0<secs:
        motors(7,7); time.sleep(0.12)
    f1=front()
    if f1<0: f1=front()
    return f0-f1, f1

def hunt_drive(axkey, tmax=20, dist_stop=0.32, log=print):
    """align to axis, wiggle until movement catches, then drive; update axis estimate"""
    nom=AXES[axkey]
    rot_to(nom, tol=2.5)
    offsets=[0,3,-3,6,-6,9,-9,12,-12]
    t0=time.time()
    for off in offsets:
        if time.time()-t0>tmax: break
        rot_to(nom+off, tol=2.5)
        d,f=try_move(0.9)
        if f<dist_stop: stop(); return 'wall',0
        if d>0.12:
            h=heading()
            AXES[axkey]=(AXES[axkey]*3+ (nom+off))/4.0
            # locked in: keep driving
            f0=f+d; fmin=f; lastprog=time.time()
            while time.time()-t0<tmax:
                motors(7,7); time.sleep(0.13)
                st=rd('d6')
                if 'here=1' in st or 'goal=1' in st: stop(); return 'HERE',f0-fmin
                fc=front()
                if fc<0: continue
                if fc<dist_stop: stop(); return 'wall',f0-fc
                if fc<fmin-0.05: fmin=fc; lastprog=time.time()
                if fc>fmin+0.5: fmin=fc; lastprog=time.time()
                if time.time()-lastprog>1.4:
                    stop(); return 'stuck',f0-fc
            stop(); return 'time',f0-front()
    stop(); return 'nolock',0
