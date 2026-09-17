import os, time, threading

class Port:
    def __init__(self, path, mode):
        if mode=='r':
            self.fd = os.open(path, os.O_RDONLY|os.O_NONBLOCK)
        else:
            self.fd = os.open(path, os.O_WRONLY|os.O_NONBLOCK)
    def write(self, s):
        try: os.write(self.fd, (s+"\n").encode())
        except BlockingIOError: pass

class Robot:
    def __init__(self):
        self.lidar=None; self.hdg=None; self.stat={}; self.flags={}
        self.rx=[]
        self.lock=threading.Lock()
        self._rd("/dev/robot/d2", self._lidar)
        self._rd("/dev/robot/d4", self._hdg)
        self._rd("/dev/robot/d3", self._stat)
        self._rd("/dev/robot/d0", lambda l: self.flags.__setitem__('d0', l))
        self._rd("/dev/robot/d5", lambda l: self.flags.__setitem__('d5', l))
        self._rd("/dev/robot/d10", self._rx)
        self.L=Port("/dev/robot/d1",'w')
        self.R=Port("/dev/robot/d7",'w')
        self.TX=Port("/dev/robot/d8",'w')
        time.sleep(0.3)
    def _rd(self, path, cb):
        def run():
            fd=os.open(path, os.O_RDONLY|os.O_NONBLOCK)
            buf=b""
            while True:
                try:
                    d=os.read(fd,4096)
                    if d:
                        buf+=d
                        parts=buf.split(b"\n"); buf=parts[-1]
                        for p in parts[:-1]:
                            cb(p.decode())
                    else: time.sleep(0.01)
                except BlockingIOError: time.sleep(0.01)
        threading.Thread(target=run,daemon=True).start()
    def _lidar(self,l):
        try: self.lidar=[float(x) for x in l.split(",")]
        except: pass
    def _hdg(self,l):
        try: self.hdg=float(l)
        except: pass
    def _stat(self,l):
        d={}
        for kv in l.split():
            k,v=kv.split("=")
            d[k]=v
        self.stat=d
    def _rx(self,l):
        if l.strip():
            with self.lock: self.rx.append(l)
    def motors(self,l,r):
        self.L.write(str(l)); self.R.write(str(r))
    def stop(self): self.motors(0,0)
    def send(self,msg): self.TX.write(msg)
    def get_rx(self):
        with self.lock:
            out=self.rx[:]; self.rx=[]
        return out
