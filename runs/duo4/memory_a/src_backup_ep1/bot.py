import os, select, time, math, sys

DEV='/dev/robot/d'

class IO:
    def __init__(self):
        self.rfds = {}
        self.buf = {}
        self.latest = {}
        self.msgs = []
        for n in [0,1,2,3,6,7,9]:
            fd = os.open(DEV+str(n), os.O_RDONLY|os.O_NONBLOCK)
            self.rfds[fd]=n
            self.buf[fd]=b''
        self.w4 = open(DEV+'4','w', buffering=1)
        self.w5 = open(DEV+'5','w', buffering=1)
        self.w8 = open(DEV+'8','w', buffering=1)
    def poll(self, t=0.05):
        r,_,_ = select.select(list(self.rfds.keys()),[],[],t)
        for fd in r:
            try: chunk = os.read(fd,4096)
            except BlockingIOError: continue
            if not chunk: continue
            self.buf[fd]+=chunk
            while b'\n' in self.buf[fd]:
                line,self.buf[fd] = self.buf[fd].split(b'\n',1)
                n = self.rfds[fd]
                s = line.decode().strip()
                if n==3:
                    if s: self.msgs.append(s)
                else:
                    self.latest[n]=s
    def lidar(self):
        s=self.latest.get(1)
        if not s: return None
        return [float(x) for x in s.split(',')]
    def heading(self):
        s=self.latest.get(2)
        return float(s) if s else None
    def drive(self,turn,fwd):
        self.w4.write('%g\n'%turn); self.w5.write('%g\n'%fwd)
    def send(self,msg):
        self.w8.write(msg+'\n')

def clean(l, prev):
    # replace -1 with prev value or 2.5
    out=[]
    for i,v in enumerate(l):
        if v<0:
            out.append(prev[i] if prev and prev[i] and prev[i]>0 else 2.5)
        else: out.append(v)
    return out
