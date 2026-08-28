import os, select, time, math

class Port:
    def __init__(self, n):
        self.n=n
        self.fd=os.open(f'/dev/robot/d{n}', os.O_RDONLY|os.O_NONBLOCK)
        self.buf=b''
        self.last=None
        self.queue=[]
    def poll(self):
        while True:
            r,_,_=select.select([self.fd],[],[],0)
            if not r: break
            try: d=os.read(self.fd,65536)
            except BlockingIOError: break
            if not d: break
            self.buf+=d
        while b'\n' in self.buf:
            line,self.buf=self.buf.split(b'\n',1)
            s=line.decode().strip()
            if s:
                self.last=s
                self.queue.append(s)
    def latest(self, wait=1.5):
        # drain, then wait for a fresh line
        self.poll(); self.last=None
        t0=time.time()
        while time.time()-t0<wait:
            self.poll()
            if self.last is not None: return self.last
            time.sleep(0.02)
        return self.last

class Bot:
    def __init__(self):
        self.p={n:Port(n) for n in [0,1,2,3,6,7,9]}
        self.wfd={}
    def wr(self,n,s):
        fd=os.open(f'/dev/robot/d{n}', os.O_WRONLY)
        os.write(fd,(s+'\n').encode()); os.close(fd)
    def heading(self):
        s=self.p[2].latest(); return float(s) if s else None
    def scan(self):
        s=self.p[1].latest(); return [float(x) for x in s.split(',')] if s else None
    def move(self,d):
        self.p[6].poll(); self.p[6].queue.clear()
        self.wr(4,str(d))
        t0=time.time()
        while time.time()-t0<10:
            self.p[6].poll()
            if self.p[6].queue: return float(self.p[6].queue.pop(0))
            time.sleep(0.02)
        return None
    def turn(self,a):
        self.wr(5,str(a)); time.sleep(0.3)
    def status(self):
        return self.p[9].latest()
    def radio_send(self,s): self.wr(8,s)
    def radio_recv(self):
        self.p[3].poll(); q=self.p[3].queue[:]; self.p[3].queue.clear(); return q
