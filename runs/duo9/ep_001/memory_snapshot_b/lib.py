import time

def read_port(name, retries=5):
    for _ in range(retries):
        try:
            with open(f"/dev/robot/{name}") as f:
                line = f.readline().strip()
            if line: return line
        except Exception:
            pass
        time.sleep(0.05)
    return ""

def read_float(name):
    for _ in range(10):
        s = read_port(name)
        try: return float(s)
        except: time.sleep(0.05)
    raise RuntimeError(f"no float from {name}")

def write_port(name, line):
    with open(f"/dev/robot/{name}","w") as f:
        f.write(str(line)+"\n")

def motors(l, r):
    write_port("d10", l); write_port("d11", r)

def lidar():
    for _ in range(10):
        s = read_port("d3")
        try: return [float(x) for x in s.split(",")]
        except: time.sleep(0.05)
    raise RuntimeError("no lidar")
