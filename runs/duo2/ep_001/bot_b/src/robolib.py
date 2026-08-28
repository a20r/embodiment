import time, os, threading

DEV='/dev/robot/'

class Reader(threading.Thread):
    """Continuously read a fifo, keep the latest line."""
    def __init__(self, name):
        super().__init__(daemon=True)
        self.path=DEV+name; self.latest=None; self.lock=threading.Lock()
        self.start()
    def run(self):
        while True:
            try:
                with open(self.path) as f:
                    for line in f:
                        line=line.strip()
                        if line:
                            with self.lock: self.latest=line
            except Exception:
                time.sleep(0.1)
    def get(self):
        with self.lock: return self.latest

class Bot:
    def __init__(self):
        self.lidar_r=Reader('d0'); self.stat_r=Reader('d2'); self.head_r=Reader('d10')
        self.odo_r=Reader('d8'); self.d4=Reader('d4'); self.d7=Reader('d7'); self.d9=Reader('d9')
        self.rx=Reader('d5')
        self.fl=open(DEV+'d1','w',buffering=1); self.fr=open(DEV+'d6','w',buffering=1)
        time.sleep(0.5)
    def wheels(self,l,r):
        self.fl.write(f"{l}\n"); self.fr.write(f"{r}\n")
    def stop(self): self.wheels(0,0)
    def lidar(self):
        s=self.lidar_r.get()
        if not s: return None
        try: return [float(x) for x in s.split(',')]
        except: return None
    def heading(self):
        s=self.head_r.get()
        try: return float(s)
        except: return None
    def goal(self):
        s=self.stat_r.get()
        if s and 'goal=' in s:
            return int(s.split('goal=')[1].split()[0])
        return 0
    def tick(self):
        s=self.stat_r.get()
        if s and 'tick=' in s:
            return int(s.split('tick=')[1].split()[0])
        return -1
    def odo(self):
        s=self.odo_r.get()
        try: return int(s)
        except: return None
    def tx(self,msg):
        with open(DEV+'d3','w') as f: f.write(msg+'\n')

def angdiff(a,b):
    d=(a-b)%360
    if d>180: d-=360
    return d
