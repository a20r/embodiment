import os, time, select
BASE="/dev/robot/"
class R:
    def _open_r(self, i):
        return os.open(BASE+f"d{i}", os.O_RDONLY|os.O_NONBLOCK)
    def read(self, i, to=0.05):
        try: fd=self._open_r(i)
        except OSError: return None
        rr,_,_=select.select([fd],[],[],to)
        if rr:
            try: d=os.read(fd,1024).decode().strip()
            except OSError: d=None
        else: d=None
        os.close(fd); return d
    def write(self, i, s):
        try:
            fd=os.open(BASE+f"d{i}", os.O_WRONLY)
            os.write(fd, (str(s)+"\n").encode()); os.close(fd); return True
        except OSError: return False
    def motors(self, a, b):  # a->d1, b->d7
        self.write(1, a); self.write(7, b)
    def stop(self): self.motors(0,0)
    def status(self):
        s=self.read(3) or ""
        tick=goal=here=None
        for part in s.split():
            if part.startswith("tick="): tick=int(part[5:])
            if part.startswith("goal="): goal=int(part[5:])
            if part.startswith("here="): here=int(part[5:])
        return tick,goal,here
    def heading(self):
        h=self.read(4)
        try: return float(h)
        except: return None
    def ranges(self):
        s=self.read(2)
        if not s: return None
        try: return [float(x) for x in s.split(",")]
        except: return None
    def enc(self):
        a=self.read(6); b=self.read(9)
        try: return (int(a),int(b))
        except: return (None,None)
    def bat(self): return self.read(11)
