import os, select, time, math

D = '/dev/robot/'
class Robot:
    def rd(self, p, tmo=0.2):
        try:
            fd = os.open(D+p, os.O_RDONLY | os.O_NONBLOCK)
            r,_,_ = select.select([fd],[],[],tmo)
            v = None
            if r:
                try: v = os.read(fd, 4096).decode().strip()
                except: v = ''
            os.close(fd)
            return v
        except Exception:
            return None
    def wr(self, p, s):
        try:
            fd = os.open(D+p, os.O_WRONLY | os.O_NONBLOCK)
            os.write(fd, (s+'\n').encode()); os.close(fd)
        except Exception: pass
    def wheels(self, l, r):
        self.wr('d1', str(l)); self.wr('d7', str(r))
    def stop(self):
        self.wheels(0,0)
    def status(self):
        s = self.rd('d3')
        tick=goal=here=None
        if s:
            for part in s.split():
                if '=' in part:
                    k,v = part.split('=')
                    try:
                        if k=='tick': tick=int(v)
                        elif k=='goal': goal=int(v)
                        elif k=='here': here=int(v)
                    except: pass
        return tick, goal, here
    def heading(self):
        for _ in range(3):
            h = self.rd('d4')
            try: return float(h)
            except: pass
        return None
    def enc(self):
        for _ in range(3):
            l = self.rd('d9'); rr = self.rd('d6')
            try: return float(l), float(rr)
            except: pass
        return None, None
    def lidar(self):
        for _ in range(3):
            s = self.rd('d2')
            if s:
                try:
                    return [float(x) for x in s.split(',')]
                except: pass
        return None
    def battery(self):
        try: return float(self.rd('d11'))
        except: return None
    def send(self, line):
        self.wr('d8', line)
    def recv(self, tmo=0.5):
        return self.rd('d10', tmo)
    # --- motion helpers ---
    TICKS_PER_M = 544.0
    WHEELBASE = 0.56  # m, refine
    def rot(self, deg, speed=30, timeout=30):
        # rotate in place; positive deg = clockwise (heading increases)
        l,r = self.enc()
        if l is None: return
        l0, r0 = l, r
        ticks = abs(deg)*self.DEG2TICK
        sgn = 1 if deg > 0 else -1
        t0 = time.time()
        while time.time()-t0 < timeout:
            l,r = self.enc()
            if l is None: continue
            done = (l-l0)*sgn + (r-r0)*(-sgn)  # left fwd for cw? left wheel fwd => cw
            # clockwise: left forward, right backward
            prog = ((l-l0)*sgn - (r-r0)*sgn)/2.0
            if abs(prog) >= ticks: break
            err = ticks - abs(prog)
            v = max(8, min(speed, err/3+8))
            if deg>0: self.wheels(v, -v)
            else: self.wheels(-v, v)
            time.sleep(0.05)
        self.stop()
        time.sleep(0.3)
    DEG2TICK = 2.7  # encoder ticks per degree of robot rotation (spin-in-place, per wheel)
    def drive(self, meters, speed=25, timeout=30):
        l,r = self.enc()
        if l is None: return
        l0, r0 = l, r
        ticks = meters*self.TICKS_PER_M
        sgn = 1 if meters > 0 else -1
        t0 = time.time()
        while time.time()-t0 < timeout:
            l,r = self.enc()
            if l is None: continue
            prog = ((l-l0)+(r-r0))/2.0*sgn
            if prog >= ticks: break
            err = ticks - prog
            base = max(8, min(speed, err*0.3+10))
            # heading hold
            hdg_err = 0.0
            if hasattr(self,'_hold_hdg') and self._hold_hdg is not None:
                h = self.heading()
                if h is not None:
                    d = (h - self._hold_hdg + 180) % 360 - 180
                    hdg_err = -d*0.5
            self.wheels(base*sgn + hdg_err, base*sgn - hdg_err)
            time.sleep(0.05)
        self.stop()
        time.sleep(0.3)
