import time, math, sys
sys.path.insert(0,'/bot/src')
from lib import rd, wr, drive, stop, tx

LOG='/tmp/wf.log'
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
def goal():
    try: return int(rd('d6').split('goal=')[1].split()[0])
    except: return 0
def bump():
    try: return rd('d9')=='1'
    except: return False

# ray 0 = forward; +i = 22.5*i toward 'plus' side (call it RIGHT: drive(l>r) turns that way)
import random
def main():
    last_g=0; last_ping=0
    side=1  # 1 = right wall, -1 = left wall
    next_switch=time.time()+random.uniform(35,70)
    while True:
        now=time.time()
        if now>next_switch:
            side=random.choice([1,-1])
            next_switch=now+random.uniform(35,70)
            log('side switch -> %d'%side)
        if goal():
            stop()
            if now-last_g>8:
                last_g=now; tx('GOALFOUND from=beta (I am standing on goal)'); log('GOAL standing')
            time.sleep(0.3); continue
        l=lidar()
        if side==-1:
            l=[l[0]]+l[1:][::-1]  # mirror rays
        front=min(l[0], l[1]*1.3, l[15]*1.3)
        rdiag=min(l[2],l[3])
        right=min(l[4],l[5])
        if bump():
            drive(-45,-45); time.sleep(0.45)
            drive(-40,40); time.sleep(0.25)   # rotate left
            stop(); log('bump recover'); continue
        if front<0.3 or min(l[1],l[15])<0.16:
            # turn away from followed wall
            drive(-42*side,42*side)
            t0=time.time()
            while time.time()-t0<4:
                l=lidar()
                if min(l[0],l[1]*1.2,l[15]*1.2)>0.45: break
                time.sleep(0.03)
            stop()
            continue
        # steering: keep right wall at ~0.25
        e = (right-0.25)
        steer = e*90
        # diag repulsion (both sides)
        ldiag=min(l[13],l[14])
        steer += -14.0*(1.0/max(rdiag,0.07)) + 14.0*(1.0/max(ldiag,0.07))
        # if right wide open, arc right to hug corner
        if right>0.8 and rdiag>0.5:
            steer=38
        steer=max(-40,min(40,steer))
        base=85 if front>0.7 else (60 if front>0.45 else 38)
        if side==-1: steer=-steer
        drive(int(base+steer), int(base-steer))
        if now-last_ping>4:
            last_ping=now; tx('PING from=beta wf')
        try:
            v2=rd('d2')
            if v2 not in ('0',''):
                with open('/tmp/ALERT','a') as f: f.write('%.0f D2=%s\n'%(now,v2))
                log('D2 NONZERO %s'%v2)
        except: pass
        time.sleep(0.06)
main()
