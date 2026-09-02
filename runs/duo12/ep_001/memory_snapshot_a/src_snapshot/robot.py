import os, time, statistics, threading

DEV="/dev/robot/"

class Reader(threading.Thread):
    """Continuously reads a device, keeps latest lines."""
    def __init__(self, name):
        super().__init__(daemon=True)
        self.dev=DEV+name
        self.latest=None
        self.lines=[]
        self.lock=threading.Lock()
        self.start()
    def run(self):
        fd=os.open(self.dev, os.O_RDONLY|os.O_NONBLOCK)
        buf=b""
        while True:
            try:
                b=os.read(fd,4096)
                if b:
                    buf+=b
                    while b"\n" in buf:
                        line,buf=buf.split(b"\n",1)
                        s=line.decode(errors="replace")
                        with self.lock:
                            self.latest=s
                            self.lines.append(s)
                            if len(self.lines)>200: self.lines=self.lines[-100:]
                else: time.sleep(0.02)
            except BlockingIOError:
                time.sleep(0.02)
    def last(self,n=5):
        with self.lock: return self.lines[-n:]
    def take(self):
        with self.lock:
            l=self.lines; self.lines=[]
            return l

class Bot:
    def __init__(self):
        self.rng=Reader("d2"); self.comp=Reader("d4")
        self.stat=Reader("d3"); self.rx=Reader("d10")
        self.encL=Reader("d6"); self.encR=Reader("d9")
        self.d11=Reader("d11"); self.d0=Reader("d0"); self.d5=Reader("d5")
        self.fdR=os.open(DEV+"d1", os.O_WRONLY|os.O_NONBLOCK)
        self.fdL=os.open(DEV+"d7", os.O_WRONLY|os.O_NONBLOCK)
        self.fdTX=os.open(DEV+"d8", os.O_WRONLY|os.O_NONBLOCK)
        time.sleep(0.6)
    def _w(self,fd,val):
        try: os.write(fd,f"{val}\n".encode())
        except BlockingIOError: pass
    def wheels(self,l,r):
        lr=(l,r)
        if getattr(self,"_lastw",None)==lr: 
            # still refresh occasionally
            import time as _t
            if _t.time()-getattr(self,"_lastwt",0)<0.5: return
        self._lastw=lr; import time as _t; self._lastwt=_t.time()
        self._w(self.fdL,l); self._w(self.fdR,r)
    def stop(self): self.wheels(0,0)
    def tx(self,msg):
        try: os.write(self.fdTX,(msg+"\n").encode())
        except BlockingIOError: pass
    def heading(self):
        ls=self.comp.last(5)
        vs=[float(x) for x in ls if x]
        if not vs: return None
        # circular median-ish: use last
        import math
        s=sum(math.sin(math.radians(v)) for v in vs)
        c=sum(math.cos(math.radians(v)) for v in vs)
        return math.degrees(math.atan2(s,c))%360
    def ranges(self):
        ls=self.rng.last(4)
        arrs=[[float(x) for x in l.split(",")] for l in ls if l]
        if not arrs: return None
        n=len(arrs[0])
        out=[]
        for i in range(n):
            vals=[a[i] for a in arrs if a[i]>=0]
            out.append(statistics.median(vals) if vals else -1.0)
        return out
    def enc(self):
        try:
            l=statistics.median(int(x) for x in self.encL.last(3))
            r=statistics.median(int(x) for x in self.encR.last(3))
            return l,r
        except: return None
    def status(self):
        s=self.stat.latest or ""
        d={}
        for kv in s.split():
            if "=" in kv:
                k,v=kv.split("="); d[k]=int(v)
        return d

def angdiff(a,b):
    """a-b wrapped to [-180,180]"""
    d=(a-b+180)%360-180
    return d

def turn_to(bot, target, tol=6, speed=4, timeout=30):
    end=time.time()+timeout
    while time.time()<end:
        h=bot.heading()
        d=angdiff(target,h)
        if abs(d)<tol:
            bot.stop(); return True
        s=speed if abs(d)>25 else 2
        if d>0: bot.wheels(-s,s)   # increase compass: right>left
        else: bot.wheels(s,-s)
        time.sleep(0.15)
    bot.stop(); return False

def forward(bot, dist_counts, speed=8, timeout=60, min_front=0.25):
    """drive straight-ish by encoder counts; stop early if obstacle in front"""
    e0=bot.enc(); h0=bot.heading()
    end=time.time()+timeout
    while time.time()<end:
        e=bot.enc()
        trav=((e[0]-e0[0])+(e[1]-e0[1]))/2
        if trav>=dist_counts: break
        r=bot.ranges()
        if r:
            front=min(x for x in [r[0],r[1],r[15]] if x>=0) if any(x>=0 for x in [r[0],r[1],r[15]]) else 9
            if front<min_front: break
        h=bot.heading()
        corr=angdiff(h0,h)*0.15
        corr=max(-3,min(3,corr))
        bot.wheels(speed-corr, speed+corr)
        time.sleep(0.15)
    bot.stop()
    e=bot.enc()
    return ((e[0]-e0[0])+(e[1]-e0[1]))/2
