import time
def read(p, tries=3):
    for _ in range(tries):
        try:
            with open(p) as f: s=f.read().strip()
            if s: return s
        except Exception: pass
        time.sleep(0.03)
    return ""
def lidar(): return [float(x) for x in read('/dev/robot/d2').split(',')]
def drive(l, r, secs):
    t0=time.time()
    while time.time()-t0 < secs:
        with open('/dev/robot/d1','w') as f: f.write(f"{l}\n")
        with open('/dev/robot/d7','w') as f: f.write(f"{r}\n")
        time.sleep(0.02)
L0=lidar(); h0=float(read('/dev/robot/d4'))
drive(60,-60,1.5)   # spin CW ~90deg
L1=lidar(); h1=float(read('/dev/robot/d4'))
print("h0=%.1f h1=%.1f dh=%.1f"%(h0,h1,h1-h0))
print("L0:",L0); print("L1:",L1)
print("shift check: L1[i] vs L0[i-k] for k in -3..3")
import statistics
best=None
for k in range(-4,5):
    errs=[]
    for i in range(16):
        j=(i+k)%16
        if L1[i]>0 and L0[j]>0: errs.append(abs(L1[i]-L0[j]))
    if len(errs)>=10:
        m=statistics.mean(errs)
        print(f"k={k:+d}: meanErr={m:.3f}")
        if best is None or m<best[1]: best=(k,m)
print("BEST shift:",best)
print("d11=",read('/dev/robot/d11'),"d3=",read('/dev/robot/d3'))
