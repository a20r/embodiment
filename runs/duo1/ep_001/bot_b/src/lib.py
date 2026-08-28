import time

def read(port):
    # one snapshot line
    for _ in range(3):
        try:
            with open(f"/dev/robot/{port}") as f:
                line = f.readline().strip()
            if line: return line
        except Exception: pass
        time.sleep(0.1)
    return ""

def write(port, val):
    with open(f"/dev/robot/{port}","w") as f:
        f.write(f"{val}\n")

def lidar():
    for _ in range(5):
        s = read("d0")
        try:
            v = [float(x) for x in s.split(",")]
            if len(v)==16: return v
        except Exception: pass
        time.sleep(0.1)
    return [3.0]*16

def heading():
    try: return float(read("d10"))
    except: return 0.0

def goal():
    s = read("d2")
    return "goal=1" in s or "goal=0" not in s, s

def wheels(l, r):
    write("d1", l); write("d6", r)

def stop(): wheels(0,0)

def norm(a): return (a+180)%360-180

def turn_by(delta, speed=15):
    h0 = heading(); target = (h0+delta)%360
    s = speed if delta>0 else -speed
    wheels(-s, s)
    t0=time.time()
    while time.time()-t0 < abs(delta)/8 + 8:
        e = norm(target-heading())
        if abs(e) < 6: break
        w = max(4, min(20, abs(e)/3))
        s2 = w if e>0 else -w
        wheels(-s2, s2)
        time.sleep(0.15)
    stop()

def drive(l, r, dur):
    wheels(l, r); time.sleep(dur); stop()
