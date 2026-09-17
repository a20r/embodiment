import os, time, threading

class Driver:
    def __init__(self, period=0.05):
        self.period = period
        self.l = 0.0
        self.r = 0.0
        self.running = True
        self.fds = {}
        for p in ('d1','d7'):
            self.fds[p] = os.open('/dev/robot/'+p, os.O_WRONLY)
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
    def _loop(self):
        while self.running:
            os.write(self.fds['d1'], ('%.3f\n'%self.l).encode())
            os.write(self.fds['d7'], ('%.3f\n'%self.r).encode())
            time.sleep(self.period)
    def set(self, l, r):
        self.l = max(-1,min(1,l)); self.r = max(-1,min(1,r))
    def stop(self):
        self.l = self.r = 0.0
    def close(self):
        self.l=self.r=0.0; time.sleep(0.1); self.running=False; self.thread.join(timeout=1)
        for fd in self.fds.values(): os.close(fd)

def rd(p, tries=200):
    for _ in range(tries):
        with open('/dev/robot/'+p) as f:
            v = f.read().strip()
        if v: return v
        time.sleep(0.01)
    raise RuntimeError('no data from '+p)

def lidar():
    return [float(x) for x in rd('d2').split(',')]
