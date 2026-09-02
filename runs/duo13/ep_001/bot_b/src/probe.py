import time

def read(p):
    try:
        with open(p) as f: return f.read().strip()
    except Exception as e: return "ERR:"+str(e)

def lidar():
    s = read('/dev/robot/d2')
    return [float(x) for x in s.split(',')]

def stream(port, val, secs=1.5, hz=50):
    L0 = lidar(); h0 = float(read('/dev/robot/d4'))
    n = int(secs*hz); dt = 1.0/hz
    t0=time.time()
    ok=True
    for i in range(n):
        try:
            with open(port,'w') as f: f.write(val+"\n")
        except Exception as e:
            ok=False; break
        time.sleep(dt)
    el=time.time()-t0
    L1 = lidar(); h1 = float(read('/dev/robot/d4'))
    d = [abs(a-b) for a,b in zip(L0,L1)]
    return ok, el, sum(d)/len(d), max(d), (h1-h0), L1

tests = []
for port in ['/dev/robot/d1','/dev/robot/d7']:
    for val in ['0.3','-0.3','1','100','-1','0.3,0.3','fwd 0.3','go','move 1','0.5','0']:
        tests.append((port,val))

for port,val in tests:
    ok,el,avg,mx,dh,L = stream(port,val)
    print(f"{port.split('/')[-1]:>3} val={val!r:12} ok={ok} el={el:.1f} avgDL={avg:.3f} maxDL={mx:.3f} dh={dh:+.1f}")
print("final lidar:", L)
