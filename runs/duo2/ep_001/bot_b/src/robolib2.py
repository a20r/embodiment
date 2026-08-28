import time, os

DEV='/dev/robot/'
def poll(name, timeout=1.5):
    # open fifo nonblocking-ish: normal open then read one line
    import select
    fd=os.open(DEV+name, os.O_RDONLY|os.O_NONBLOCK)
    try:
        buf=b''
        t0=time.time()
        while time.time()-t0<timeout:
            r,_,_=select.select([fd],[],[],0.1)
            if r:
                try:
                    chunk=os.read(fd,4096)
                except BlockingIOError:
                    continue
                if chunk:
                    buf+=chunk
                    if b'\n' in buf:
                        return buf.split(b'\n')[0].decode().strip()
        return None
    finally:
        os.close(fd)

class Bot:
    def __init__(self):
        self.fl=open(DEV+'d1','w',buffering=1); self.fr=open(DEV+'d6','w',buffering=1)
        self.last=[0.5]*16
    def wheels(self,l,r):
        self.fl.write(f"{l}\n"); self.fr.write(f"{r}\n")
    def stop(self): self.wheels(0,0)
    def lidar(self):
        s=poll('d0')
        if not s: return self.last
        try: v=[float(x) for x in s.split(',')]
        except: return self.last
        v=[self.last[i] if x<0 else x for i,x in enumerate(v)]
        self.last=v
        return v
    def heading(self):
        s=poll('d10')
        try: return float(s)
        except: return None
    def status(self):
        s=poll('d2')
        if s and 'goal=' in s:
            t=int(s.split('tick=')[1].split()[0]); g=int(s.split('goal=')[1].split()[0])
            return t,g
        return -1,0
    def odo(self):
        s=poll('d8')
        try: return int(s)
        except: return 0
    def rx(self):
        return poll('d5', timeout=0.15)
    def tx(self,msg):
        with open(DEV+'d3','w') as f: f.write(msg+'\n')

def angdiff(a,b):
    d=(a-b)%360
    if d>180: d-=360
    return d
