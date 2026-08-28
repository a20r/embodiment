import os, time, math

DEV='/dev/robot/'
class Port:
    def __init__(self, name, mode):
        if mode=='r':
            self.fd=os.open(DEV+name, os.O_RDONLY|os.O_NONBLOCK); self.buf=b''
        else:
            self.fd=os.open(DEV+name, os.O_WRONLY)
    def last_line(self, wait=0.5):
        t0=time.time(); last=None
        while True:
            try:
                d=os.read(self.fd,65536)
                if d: self.buf+=d
            except BlockingIOError: pass
            if b'\n' in self.buf:
                lines=self.buf.split(b'\n'); self.buf=lines[-1]
                for l in reversed(lines[:-1]):
                    if l.strip(): last=l.decode().strip(); break
                if last is not None: return last
            if time.time()-t0>wait: return last
            time.sleep(0.02)
    def write(self, s): os.write(self.fd,(str(s)+'\n').encode())

class Robot:
    def __init__(self):
        self.lidar=Port('d0','r'); self.status=Port('d2','r')
        self.compass=Port('d10','r'); self.rx=Port('d5','r')
        self.wa=Port('d1','w'); self.wb=Port('d6','w'); self.tx=Port('d3','w')
    def drive(self,v,w):
        # v forward cmd, w turn cmd (positive = heading increases)
        a=max(min(v-w,50),-50); b=max(min(v+w,50),-50)
        self.wa.write(round(a,1)); self.wb.write(round(b,1))
    def stop(self): self.drive(0,0)
    def scan(self):
        l=self.lidar.last_line(1.0)
        if not l: return None
        try:
            s=[float(x) for x in l.split(',')]
            return s if len(s)==16 else None
        except: return None
    def heading(self):
        try: return float(self.compass.last_line(1.0))
        except: return None
    def goal(self):
        l=self.status.last_line(1.0)
        if l and 'goal=' in l:
            try: return int(l.split('goal=')[1].split()[0])
            except: return 0
        return 0
    def turn_to(self,target,tol=5,timeout=15):
        t0=time.time()
        while time.time()-t0<timeout:
            h=self.heading()
            if h is None: continue
            err=(target-h+180)%360-180
            if abs(err)<=tol: self.stop(); return True
            w=max(min(err*0.8,28),-28)
            if abs(w)<6: w=6*(1 if err>0 else -1)
            self.drive(0,w); time.sleep(0.08)
        self.stop(); return False
