import os, time, math, sys, json

DEV='/dev/robot/'
class RPort:
    def __init__(self,name):
        self.fd=os.open(DEV+name, os.O_RDONLY|os.O_NONBLOCK)
        self.buf=b''
        self.last=None
    def poll(self):
        # read all available, keep last complete line; return list of new lines
        lines=[]
        try:
            while True:
                d=os.read(self.fd,65536)
                if not d: break
                self.buf+=d
        except BlockingIOError:
            pass
        while b'\n' in self.buf:
            line,self.buf=self.buf.split(b'\n',1)
            s=line.decode(errors='replace').strip()
            if s!='':
                self.last=s; lines.append(s)
        return lines

class WPort:
    def __init__(self,name):
        self.fd=os.open(DEV+name, os.O_WRONLY)
    def write(self,s):
        os.write(self.fd,(str(s)+'\n').encode())

class Robot:
    def __init__(self):
        self.heading=RPort('d1'); self.lidar=RPort('d3'); self.d2=RPort('d2')
        self.d5=RPort('d5'); self.d6=RPort('d6'); self.d7=RPort('d7')
        self.d8=RPort('d8'); self.d9=RPort('d9'); self.rx=RPort('d4')
        self.tx=WPort('d0'); self.ml=WPort('d10'); self.mr=WPort('d11')
        self.h=None; self.rays=None; self.goal=None; self.tick=None
        self.x=0.0; self.y=0.0   # dead reckoning
        self.cl=0; self.cr=0
        self.lastt=time.time()
        self.msgs=[]
        self.events=[]
    def wheels(self,l,r):
        self.ml.write(round(l,1)); self.mr.write(round(r,1))
        self.cl=l; self.cr=r
    def update(self):
        now=time.time(); dt=now-self.lastt; self.lastt=now
        self.heading.poll(); self.lidar.poll(); self.d2.poll(); self.d5.poll()
        self.d7.poll(); self.d8.poll(); self.d9.poll()
        self.msgs += [m for m in self.rx.poll() if m]
        for line in self.d6.poll():
            self.events.append(line)
            for kv in line.split():
                if '=' in kv:
                    k,v=kv.split('=',1)
                    if k=='goal': self.goal=v
                    if k=='tick': self.tick=v
        if self.heading.last: self.h=float(self.heading.last)
        if self.lidar.last:
            try:
                self.rays=[float(v) for v in self.lidar.last.split(',')]
            except: pass
        # dead reckon with commanded speeds
        if self.h is not None:
            v=0.0028*(self.cl+self.cr)/2.0
            a=math.radians(self.h)
            self.x+=v*math.cos(a)*dt; self.y+=v*math.sin(a)*dt
    def ray(self,i):
        if not self.rays: return None
        v=self.rays[i%16]
        return None if v<0 else v
    def rmin(self,idxs,default=3.0):
        vals=[self.ray(i) for i in idxs]
        vals=[v for v in vals if v is not None]
        return min(vals) if vals else default
