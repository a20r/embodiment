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
for v in [40, 20]:
    L0=lidar(); h0=float(read('/dev/robot/d4'))
    print(f"drive {v},{v}: L0={L0} h0={h0}")
    drive(v,v,2.0)
    L1=lidar(); h1=float(read('/dev/robot/d4'))
    print(f"  -> L1={L1} dh={h1-h0:+.1f} d3={read('/dev/robot/d3')} d5={read('/dev/robot/d5')} d6={read('/dev/robot/d6')} d9={read('/dev/robot/d9')} d11={read('/dev/robot/d11')}")
    time.sleep(0.5)
