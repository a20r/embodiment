import time, statistics
D='/dev/robot/'
def read(p, tries=3):
    for _ in range(tries):
        try:
            with open(D+p) as f: s=f.read().strip()
            if s: return s
        except Exception: pass
        time.sleep(0.02)
    return ""
def lidar():
    s=read('d2')
    try: return [float(x) for x in s.split(',')]
    except: return None
def hd():
    try: return float(read('d4'))
    except: return None
def motor(l,r):
    with open(D+'d1','w') as f: f.write(f"{l:.1f}\n")
    with open(D+'d7','w') as f: f.write(f"{r:.1f}\n")

def angdiff(a,b): return (a-b+180)%360-180

def rot_to(target, maxsec=3.0):
    t0=time.time()
    while time.time()-t0<maxsec:
        h=hd()
        if h is None: motor(0,0); time.sleep(0.05); continue
        err=angdiff(target,h)
        if abs(err)<4: motor(0,0); return h
        s=18
        motor(s if err>0 else -s, -s if err>0 else s)
        time.sleep(0.02)
    motor(0,0); return hd()

def rot_by(delta):
    h0=hd(); tgt=(h0+delta)%360
    return rot_to(tgt), h0

samples=[]
h0=hd()
print("start heading",h0)
for step in range(8):
    L=lidar(); h=hd()
    samples.append((h,L))
    rot_by(45.0)
    time.sleep(0.15)
L=lidar(); h=hd(); samples.append((h,L))
motor(0,0)
for i in range(1,len(samples)):
    h0,L0=samples[i-1]; h1,L1=samples[i]
    dh=angdiff(h1,h0)
    best=None
    for k in range(-16,17):
        errs=[abs(L1[j]-L0[(j-k)%16]) for j in range(16) if L1[j]>0 and L0[(j-k)%16]>0]
        if len(errs)>=12:
            m=statistics.mean(errs)
            if best is None or m<best[1]: best=(k,m)
    print(f"step{i}: dh={dh:+.1f} bestshift={best}")
print("full-circle check: heading drift over 8 steps =", angdiff(samples[-1][0], samples[0][0]))
