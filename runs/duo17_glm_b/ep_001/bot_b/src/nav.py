import sys, time, math
sys.path.insert(0,'/bot/src')
from robot import *

def rel_bearing(idx):
    # beam k is 22.5deg*k counterclockwise from front (verified)
    return 22.5*idx

def beam_abs_bearing(h, idx):
    return (h + rel_bearing(idx)) % 360

class Pose:
    def __init__(self):
        self.x=0.0; self.y=0.0; self.h=heading()
        self.lo,self.ro = odom()
    def update(self):
        lo,ro = odom()
        dl = lo-self.lo; dr = ro-self.ro
        self.lo,self.ro = lo,ro
        # left odom positive forward; right odom positive forward too (signed)
        d = (dl+dr)/2.0 * 0.002  # guess scale: ~0.5m per 250 counts
        dh = (dr-dl)*0.0  # derive from heading sensor instead (cleaner)
        self.h = heading()
        self.x += d*math.cos(math.radians(self.h))
        self.y += d*math.sin(math.radians(self.h))
        return self.x,self.y,self.h

def pick_dir(l, min_clear=0.35):
    # score each direction: prefer forward-ish with clearance; weighted sum of window
    best=None; bestscore=-1
    for k in range(16):
        w=[l[(k+j)%16] for j in (-2,-1,0,1,2)]
        w=[x for x in w if x>0]
        if not w: continue
        m=min(w)
        if m<min_clear: continue
        # angular cost: prefer small |offset|
        off=min(k,16-k)
        score=m - 0.06*off
        if score>bestscore:
            bestscore=score; best=k
    return best

def heading_for_beam(h, idx):
    # absolute heading to steer to = current heading + rel bearing of beam
    return (h + rel_bearing(idx)) % 360

def steer_to(target_h, speed=28):
    h=heading()
    err=((target_h-h+180)%360)-180
    if abs(err)<3: motors(speed,speed); return
    if err>0: motors(speed,-speed)  # CW increase: left fwd, right back
    else: motors(-speed,speed)

def avoid_drive(duration=30, log=None):
    p=Pose(); t0=time.time()
    while time.time()-t0<duration:
        l=lidar()
        k=pick_dir(l)
        if k is None:
            # boxed in: back up
            motors(-25,-25); time.sleep(0.7); stop(); continue
        x,y,h=p.update()
        th=heading_for_beam(h,k)
        steer_to(th)
        time.sleep(0.12)
        if log: log.write('%.1f,%.2f,%.2f,%.1f,%s,%s,%s\n'%(time.time()-t0,x,y,h,','.join('%.2f'%v for v in l),rd('d0'),rd('d5')))
    stop()
    return p

if __name__=='__main__':
    with open('/memory/traj.log','a') as log:
        p=avoid_drive(float(sys.argv[1]) if len(sys.argv)>1 else 30, log)
    print('pose', p.x,p.y,p.h, 'status', status())
