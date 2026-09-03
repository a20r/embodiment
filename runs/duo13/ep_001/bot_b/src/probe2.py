import time
def read(p, tries=3):
    for _ in range(tries):
        try:
            with open(p) as f: s=f.read().strip()
            if s: return s
        except Exception: pass
        time.sleep(0.05)
    return ""
def lidar(): return [float(x) for x in read('/dev/robot/d2').split(',')]
def stream(port, val, secs=1.5, hz=50):
    L0=lidar(); h0=float(read('/dev/robot/d4'))
    for i in range(int(secs*hz)):
        with open(port,'w') as f: f.write(val+"\n")
        time.sleep(1.0/hz)
    L1=lidar(); h1=float(read('/dev/robot/d4'))
    d=[abs(a-b) for a,b in zip(L0,L1)]
    return sum(d)/len(d), max(d), (h1-h0), L1

for port in ['/dev/robot/d7','/dev/robot/d1']:
    for val in ['100','50','-100','30']:
        avg,mx,dh,L = stream(port,val)
        print(f"{port[-2:]} val={val:>5} avgDL={avg:.3f} maxDL={mx:.3f} dh={dh:+.1f} head_now={L and read('/dev/robot/d4')}")
        time.sleep(0.5)
print("lidar now:", read('/dev/robot/d2'))
print("d3:", read('/dev/robot/d3'), "d9:", read('/dev/robot/d9'), "d5:", read('/dev/robot/d5'), "d6:", read('/dev/robot/d6'), "d0:", read('/dev/robot/d0'), "d11:", read('/dev/robot/d11'))
