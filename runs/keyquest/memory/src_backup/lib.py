import time
def rd_raw(p):
    with open(f'/dev/robot/d{p}') as f: return f.readline().strip()
def rd(p, tries=10):
    for _ in range(tries):
        try:
            s=rd_raw(p)
            if s: return s
        except Exception: pass
        time.sleep(0.03)
    return ''
def rdf(p, default=None):
    for _ in range(10):
        s=rd(p)
        try: return float(s)
        except Exception: time.sleep(0.03)
    return default
def wr(p,v):
    for _ in range(5):
        try:
            with open(f'/dev/robot/d{p}','w') as f: f.write(str(v)+'\n')
            return
        except Exception: time.sleep(0.03)
def lidar():
    for _ in range(10):
        s=rd(7)
        try:
            v=[float(x) for x in s.split(',')]
            if len(v)==16: return v
        except Exception: pass
        time.sleep(0.03)
    return None
def hdg():
    h=rdf(0)
    return h if h is not None else 0.0
def goal():
    s=rd(9)
    return 'goal=1' in s
def drive(l,r): wr(3,l); wr(5,r)
