import sys, time, math
sys.path.insert(0,'/memory/code')
from robot import Robot

def angdiff(a,b):  # a-b in [-180,180)
    return ((a-b+180)%360)-180

class Nav:
    def __init__(self, r, log=print):
        self.r=r; self.log=log
        self.x=0.0; self.y=0.0  # dead reckoning, arbitrary frame (heading CCW+)
        self.last=time.time()
        self.thr=0
        self.trail=[]

    def upd(self):
        now=time.time(); dt=now-self.last; self.last=now
        h=self.r.heading()
        if h is not None and self.thr:
            v=0.0064*self.thr if abs(self.thr)<=10 else math.copysign(0.0044*abs(self.thr)**0.83+0.02,self.thr)
            v=math.copysign(min(abs(v),0.12),self.thr)
            self.x+=v*dt*math.cos(math.radians(h))
            self.y+=v*dt*math.sin(math.radians(h))
        return h

    def cmd(self,steer,thr):
        self.upd()
        self.thr=thr
        self.r.cmd(steer,thr)

    def stop(self):
        self.cmd(0,0)

    def clearance(self, idx, width=1):
        s=self.r.scan()
        if not s: return 0
        vals=[]
        for k in range(-width,width+1):
            v=s[(idx+k)%16]
            if v>0: vals.append(v)
        return min(vals) if vals else 2.0

    def goal(self):
        return self.r.goal()

    def turn_to(self, target, tol=15, timeout=90):
        """wiggle-turn until heading within tol of target"""
        t0=time.time()
        while time.time()-t0<timeout:
            if self.goal(): return 'goal'
            h=self.upd()
            if h is None: time.sleep(0.3); continue
            err=angdiff(target,h)
            if abs(err)<=tol:
                self.stop(); return True
            front=self.clearance(0); back=self.clearance(8)
            # choose stroke direction with more room
            fwd = front>=back
            room = front if fwd else back
            if room<0.12:
                fwd = not fwd
                room = back if fwd==False else front
            steer = 90 if err>0 else -90
            if not fwd: steer=-steer
            thr = 10 if fwd else -10
            dur = min(2.0, max(0.6, (room-0.1)/0.07))
            self.cmd(steer,thr)
            t1=time.time()
            while time.time()-t1<dur:
                time.sleep(0.15)
                if self.goal(): self.stop(); return 'goal'
                if self.r.bump(): break
                h=self.upd()
                if h is not None and abs(angdiff(target,h))<=tol:
                    self.stop(); return True
            self.stop()
            time.sleep(0.2)
        self.stop(); return False

    def drive(self, target_hdg=None, dur=6, thr=15, min_front=0.16):
        """drive forward, steering toward target heading; stop on obstacle/bump.
        returns reason"""
        t0=time.time()
        while time.time()-t0<dur:
            if self.goal(): self.stop(); return 'goal'
            h=self.upd()
            front=self.clearance(0)
            if self.r.bump():
                self.cmd(0,-10); time.sleep(1.2); self.stop(); return 'bump'
            if front<min_front:
                self.stop(); return 'blocked'
            steer=0
            if target_hdg is not None and h is not None:
                steer=max(-90,min(90,3*angdiff(target_hdg,h)))
            self.cmd(steer, thr if front>0.35 else 8)
            time.sleep(0.15)
        self.stop(); return 'time'
