import time

def rd(port, tries=5):
    for _ in range(tries):
        with open(f"/dev/robot/d{port}") as f:
            s = f.readline().strip()
        if s: return s
        time.sleep(0.05)
    return s

def wr(port, val):
    with open(f"/dev/robot/d{port}", "w") as f:
        f.write(str(val) + "\n")

def lidar():
    for _ in range(5):
        s = rd(0)
        try:
            return [float(x) for x in s.split(",")]
        except ValueError:
            time.sleep(0.05)
    raise RuntimeError("lidar fail")

def heading():
    return float(rd(10))

def motors(l, r):
    wr(1, l); wr(3, r)

def status():
    return rd(2)

def goal():
    s = status()
    return "goal=1" in s

def odo():
    return int(rd(7))
