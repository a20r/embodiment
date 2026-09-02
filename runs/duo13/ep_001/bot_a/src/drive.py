import sys, time, math; sys.path.insert(0,'/bot/src')
from robot import R
def wrap(a): 
    while a>180: a-=360
    while a<-180: a+=360
    return a
class Driver:
    def __init__(self): self.r=R()
    def turnto(self, target, tol=4.0, maxs=5.0):
        r=self.r
        for _ in range(120):
            h=r.heading()
            if h is None: continue
            e=wrap(target-h)
            if abs(e)<tol: r.motors(0,0); return True
            v=max(-100,min(100, 2.5*e))
            r.motors(int(v), int(-v))   # d1 up -> heading up
            time.sleep(0.1)
        r.motors(0,0); return False
    def fwd(self, speed, stopfn, maxt=8.0):
        r=self.r
        e0=r.enc(); t0=time.time()
        dl=dr=0
        pl,pr=e0
        while time.time()-t0<maxt:
            if stopfn(): break
            h=r.heading()
            # heading hold toward initial dir
            err = wrap(self.fwdtarget-h) if hasattr(self,'fwdtarget') else 0
            a=int(speed+1.0*err); b=int(speed-1.0*err)
            r.motors(a,b)
            time.sleep(0.1)
        r.motors(0,0)
        cl,cr=r.enc()
        return cl,cr
