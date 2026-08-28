import threading, time, json

class Robot:
    def __init__(self, log_path=None):
        self.data = {}   # name -> (t, value string)
        self.log = open(log_path,'a') if log_path else None
        self.d6t = 0
        for d in [0,2,4,5,6,7]:
            t = threading.Thread(target=self._reader, args=(d,), daemon=True)
            t.start()
        time.sleep(0.5)

    def _reader(self, d):
        path=f"/dev/robot/d{d}"
        while True:
            try:
                with open(path) as f:
                    line = f.readline().strip()
                if line:
                    rec = (time.time(), line)
                    self.data[d] = rec
                    if d==6 and line=='1': self.d6t=rec[0]
                    if self.log:
                        self.log.write(f"{rec[0]:.2f} d{d} {line}\n"); self.log.flush()
            except Exception:
                time.sleep(0.2)

    def _w(self, d, v):
        with open(f'/dev/robot/d{d}','w') as f: f.write(f"{v}\n")

    def cmd(self, steer, throttle):
        self._w(1, steer); self._w(3, throttle)

    def stop(self):
        self.cmd(0,0)

    # sensor accessors (fresh within max_age seconds, else None)
    def get(self, d, max_age=1.5):
        rec = self.data.get(d)
        if rec and time.time()-rec[0] < max_age:
            return rec[1]
        return None

    def heading(self):
        v = self.get(4)
        return float(v) if v else None

    def scan(self):
        v = self.get(2)
        if not v: return None
        try:
            return [float(x) for x in v.split(',')]
        except ValueError:
            return None

    def bump(self):
        v = self.get(5)
        return v == '1'

    def goal(self):
        v = self.get(0)
        return v and 'goal=1' in v

    def tick(self):
        v = self.get(0)
        if v and v.startswith('tick='):
            return int(v.split()[0][5:])
        return None
