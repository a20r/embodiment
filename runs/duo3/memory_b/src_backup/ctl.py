from drv import Bot
import time, math

def angdiff(a,b):
    d=(a-b)%360
    if d>180: d-=360
    return d

class Ctl(Bot):
    def turn_by(self, deg):
        h0=self.heading()
        if h0 is None: return False
        return self.turn_to((h0+deg)%360)
    def turn_to(self, target):
        t0=time.time()
        while time.time()-t0<15:
            h=self.heading()
            if h is None: continue
            e=angdiff(target,h)
            if abs(e)<4:
                self.wr(5,'0'); time.sleep(0.15)
                h=self.heading(); e=angdiff(target,h)
                if abs(e)<6: return True
                continue
            rate=max(min(e*2.0,90),-90)
            if abs(rate)<15: rate=15*(1 if rate>0 else -1)
            self.wr(5,str(round(rate,1)))
            time.sleep(0.1)
        self.wr(5,'0')
        return False
    def stop(self):
        self.wr(4,'0'); self.wr(5,'0')
