import sys; sys.path.insert(0,'/bot/src')
import rio, time, math

def drive(l, r):
    rio.write_line('d1', str(l)); rio.write_line('d7', str(r))

def stop(): drive(0, 0)

def heading():
    for _ in range(3):
        v = rio.read_line('d4', 1.0)
        if v is not None:
            try: return float(v)
            except: pass
    return None

def enc():
    l = rio.read_line('d9', 1.0); r = rio.read_line('d6', 1.0)
    return int(l), int(r)

def scan():
    for _ in range(3):
        s = rio.read_line('d2', 1.0)
        if s:
            try: return [float(x) for x in s.split(',')]
            except: pass
    return None

def status():
    s = rio.read_line('d3', 1.0)
    d = {}
    if s:
        for kv in s.split():
            k, v = kv.split('='); d[k] = int(v)
    return d

def angdiff(a, b):
    """signed a-b in (-180,180]"""
    d = (a - b + 180) % 360 - 180
    return d

def turn_to(target, tol=3.0, maxspeed=40, timeout=15):
    t0 = time.time()
    while time.time() - t0 < timeout:
        h = heading()
        if h is None: continue
        e = angdiff(target, h)
        if abs(e) < tol:
            stop(); time.sleep(0.2)
            h = heading(); e = angdiff(target, h)
            if abs(e) < tol: return True
            continue
        sp = max(8, min(maxspeed, abs(e) * 1.2))
        if e > 0: drive(sp, -sp)   # clockwise
        else: drive(-sp, sp)
        time.sleep(0.1)
    stop(); return False

def forward(ticks, speed=60, hold_heading=None, min_front=0.15, timeout=30):
    """Drive forward `ticks` encoder ticks (avg of both wheels). Stops early if front range < min_front.
    Returns (ticks_travelled, reason)"""
    l0, r0 = enc()
    if hold_heading is None: hold_heading = heading()
    t0 = time.time()
    reason = 'done'
    sgn = 1 if ticks >= 0 else -1
    while time.time() - t0 < timeout:
        l, r = enc()
        trav = ((l - l0) + (r - r0)) / 2
        if abs(trav) >= abs(ticks): break
        s = scan()
        if s and sgn > 0:
            f = [x for x in (s[0], s[1], s[15]) if x > 0]
            if f and min(f) < min_front: reason = 'obstacle'; break
        if s and sgn < 0:
            f = [x for x in (s[7], s[8], s[9]) if x > 0]
            if f and min(f) < min_front: reason = 'obstacle'; break
        h = heading()
        corr = 0
        if h is not None and hold_heading is not None:
            e = angdiff(hold_heading, h)
            corr = max(-15, min(15, e * 0.8))
        drive(sgn*speed + corr, sgn*speed - corr)
        time.sleep(0.05)
    stop()
    l, r = enc()
    return ((l - l0) + (r - r0)) / 2, reason
