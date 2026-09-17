#!/usr/bin/env python3
"""Control helpers built on rio."""
import rio, time, math

def motors(l, r):
    rio.write_port(1, str(int(l))); rio.write_port(7, str(int(r)))

def stop():
    motors(0, 0)

def heading(n=3):
    """Average n compass readings (circular mean)."""
    xs = ys = 0.0; k = 0
    for _ in range(n):
        h = rio.read_port(4)
        if h is None: continue
        a = math.radians(float(h)); xs += math.cos(a); ys += math.sin(a); k += 1
    if k == 0: return None
    return math.degrees(math.atan2(ys, xs)) % 360

def enc():
    r = rio.read_port(6); l = rio.read_port(9)
    return (int(l) if l else 0, int(r) if r else 0)

def scan(n=1):
    """Return 16 ranges (None for dropouts); if n>1, median per beam."""
    cols = [[] for _ in range(16)]
    for _ in range(n):
        s = rio.read_port(2)
        if not s: continue
        for i, v in enumerate(s.split(",")):
            v = float(v)
            if v >= 0: cols[i].append(v)
    out = []
    for c in cols:
        if not c: out.append(None)
        else:
            c.sort(); out.append(c[len(c)//2])
    return out

def status():
    s = rio.read_port(3) or ""
    d = {}
    for kv in s.split():
        if "=" in kv:
            k, v = kv.split("=", 1); d[k] = v
    return d

def angdiff(a, b):
    """signed a-b in [-180,180)"""
    return (a - b + 180) % 360 - 180

def turn_to(target, tol=4, maxspeed=40, timeout=15):
    """Turn in place to compass heading target (deg)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        h = heading(3)
        if h is None: continue
        d = angdiff(target, h)
        if abs(d) <= tol:
            stop(); time.sleep(0.15)
            h = heading(3); d = angdiff(target, h)
            if abs(d) <= tol: return h
            continue
        sp = max(8, min(maxspeed, abs(d) * 1.2))
        if d > 0: motors(sp, -sp)   # clockwise => heading increases
        else: motors(-sp, sp)
        time.sleep(0.05)
    stop(); return heading(3)

def drive(counts, speed=40, min_front=0.15, timeout=20):
    """Drive straight for encoder counts (avg of wheels), stop if obstacle ahead closer than min_front."""
    l0, r0 = enc(); t0 = time.time(); sign = 1 if counts > 0 else -1
    motors(sign*speed, sign*speed)
    reason = "done"
    try:
        while time.time() - t0 < timeout:
            l, r = enc()
            prog = ((l - l0) + (r - r0)) / 2.0
            if abs(prog) >= abs(counts): break
            s = scan()
            front = [v for v in (s[0], s[1], s[15]) if v is not None] if sign > 0 else [v for v in (s[7], s[8], s[9]) if v is not None]
            if front and min(front) < min_front:
                reason = "obstacle"; break
            st = status()
            if st.get("goal") == "1": reason = "goal"; break
            time.sleep(0.02)
    finally:
        stop()
    l, r = enc()
    return ((l - l0) + (r - r0)) / 2.0, reason

if __name__ == "__main__":
    print("hdg", heading(), "enc", enc(), "status", status()); print("scan", scan(3))
