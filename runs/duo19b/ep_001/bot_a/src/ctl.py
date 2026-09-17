import rb, time, statistics, math

def set_wheels(l, r):
    rb.write_line(1, str(l)); rb.write_line(7, str(r))

def stop():
    set_wheels(0, 0)

def hdg(n=3):
    vals = []
    for _ in range(n):
        h = rb.heading()
        if h is not None: vals.append(h)
    if not vals: return None
    # circular median approx: use mean of unit vectors
    x = sum(math.cos(math.radians(v)) for v in vals); y = sum(math.sin(math.radians(v)) for v in vals)
    return math.degrees(math.atan2(y, x)) % 360

def angdiff(a, b):
    """signed a-b in (-180,180]"""
    d = (a - b + 180) % 360 - 180
    return d

def rotate_to(target, speed=30, tol=4, timeout=15):
    t0 = time.time()
    while time.time() - t0 < timeout:
        h = hdg(3)
        d = angdiff(target, h)
        if abs(d) <= tol:
            stop(); return h
        s = speed if abs(d) > 20 else max(15, speed // 2)
        if d > 0: set_wheels(s, -s)   # turn right (CW), heading increases
        else: set_wheels(-s, s)
        time.sleep(0.1)
    stop(); return hdg(3)

def scan():
    s = rb.scan()
    if s is None: return None
    # replace -1 with None
    return [None if x < 0 else x for x in s]

def scan_med(n=3):
    ss = [scan() for _ in range(n)]
    ss = [s for s in ss if s]
    out = []
    for i in range(16):
        v = [s[i] for s in ss if s[i] is not None]
        out.append(statistics.median(v) if v else None)
    return out

def fmt(s):
    return "[" + ",".join("  -- " if x is None else "%5.2f" % x for x in s) + "]"
