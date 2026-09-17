import os, time, math

R = "/dev/robot/"

def read(port, timeout=2.0):
    # read one line from a port
    cmd = f"timeout {timeout} head -1 {R}{port}"
    with os.popen(cmd) as p:
        return p.read().strip()

def write(port, val):
    try:
        with open(R+port, "w") as f:
            f.write(str(val) + "\n")
        return True
    except Exception:
        return False

def speed(v): write("d1", v)
def turn(v): write("d7", v)
def stop():
    speed(0); turn(0)

def scan():
    s = read("d2")
    try:
        return [float(x) for x in s.split(",")]
    except Exception:
        return None

def heading(): 
    try: return float(read("d4"))
    except: return None

def odo():
    try: return float(read("d9"))
    except: return None

def status(): return read("d3")
def battery(): return read("d11")
def send(line): return write("d8", line)
def recv(timeout=2.0): return read("d10", timeout)
