import time, math, sys
sys.path.insert(0,'/bot/src')
from robot import Robot

def angdiff(a,b):
    d=(a-b)%360
    if d>180: d-=360
    return d

class Drive:
    def __init__(self,r):
        self.r=r
    def settle(self,t=0.4):
        self.r.wheels(0,0)
        end=time.time()+t
        while time.time()<end:
            self.r.update(); time.sleep(0.05)
    def turn_to(self,target,tol=6):
        r=self.r
        for attempt in range(4):
            r.update()
            if r.h is None: time.sleep(0.05); continue
            d=angdiff(target,r.h)
            if abs(d)<tol: break
            t0=time.time()
            while time.time()-t0<8:
                r.update()
                d=angdiff(target,r.h)
                if abs(d)<tol: break
                sp=max(4,min(11,abs(d)*0.35))
                if d>0: r.wheels(sp,-sp)
                else: r.wheels(-sp,sp)
                time.sleep(0.04)
            self.settle(0.3)
        r.update()
        return r.h
    def forward(self,dist,target_h=None,speed=60,front_stop=0.22):
        """drive dist meters holding heading target_h, centering with side rays.
        returns (traveled, reason)"""
        r=self.r
        if target_h is None:
            r.update(); target_h=r.h
        v=0.0060*speed
        traveled=0.0
        last=time.time()
        reason='dist'
        while traveled<dist:
            r.update()
            now=time.time(); dt=now-last; last=now
            traveled+=v*dt if (r.cl or r.cr) else 0
            f=r.rmin([0])
            f2=r.rmin([15,1])
            if f<front_stop or f2<0.13:
                reason='wall'; break
            # heading hold
            corr=0.0
            if r.h is not None:
                corr += 1.2*angdiff(target_h,r.h)
            # centering: ray4 (h+90) vs ray12 (h-90)
            s4=r.ray(4); s12=r.ray(12)
            if s4 is not None and s12 is not None and s4<0.5 and s12<0.5:
                corr += 60*(s4-s12)   # if ray4 side farther, steer that way
            elif s4 is not None and s4<0.28:
                corr -= 40*(0.28-s4)*4
            elif s12 is not None and s12<0.28:
                corr += 40*(0.28-s12)*4
            corr=max(-20,min(20,corr))
            r.wheels(speed+corr,speed-corr)
            time.sleep(0.05)
        self.settle(0.2)
        return traveled,reason
