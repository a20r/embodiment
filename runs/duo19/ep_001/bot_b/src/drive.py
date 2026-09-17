import sys, time, math
sys.path.insert(0, '/bot/src')
from rob import read_line, write_line

def heading():
    for _ in range(3):
        l = read_line(4, 0.5)
        if l:
            try: return float(l)
            except: pass
    return None

def ranges():
    for _ in range(3):
        l = read_line(2, 0.5)
        if l:
            try: return [float(x) for x in l.split(',')]
            except: pass
    return None

def enc():
    a = read_line(9, 0.5); b = read_line(6, 0.5)
    try: return int(a), int(b)
    except: return None

def motors(a, b):
    write_line(1, str(int(a))); write_line(7, str(int(b)))

def stop():
    motors(0, 0)

def angdiff(a, b):
    return (a - b + 180) % 360 - 180

def turn_to(target, tol=3.0, maxspeed=30, timeout=8.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        h = heading()
        if h is None: continue
        d = angdiff(target, h)
        if abs(d) <= tol:
            stop(); time.sleep(0.15)
            h = heading()
            if h is not None and abs(angdiff(target, h)) <= tol:
                return h
            continue
        sp = max(8, min(maxspeed, abs(d) * 0.6))
        if d > 0: motors(sp, -sp)   # increase heading
        else: motors(-sp, sp)
        time.sleep(0.05)
    stop()
    return heading()

def forward(ticks, speed=60, min_front=0.15, timeout=20.0):
    """Drive forward until encoders advance by `ticks` or front obstacle closer than min_front."""
    e0 = enc()
    t0 = time.time()
    reason = 'done'
    while time.time() - t0 < timeout:
        r = ranges()
        if r:
            front = [x for x in (r[0], r[1], r[15]) if x > 0]
            if front and min(front) < min_front:
                reason = 'obstacle'; break
        e = enc()
        if e and e0:
            adv = ((e[0]-e0[0]) + (e[1]-e0[1])) / 2.0
            if adv >= ticks: break
        motors(speed, speed)
        time.sleep(0.05)
    stop()
    time.sleep(0.2)
    e1 = enc()
    return reason, ((e1[0]-e0[0]) + (e1[1]-e0[1])) / 2.0 if (e1 and e0) else None

if __name__ == '__main__':
    print('heading', heading()); print('ranges', ranges()); print('enc', enc())
