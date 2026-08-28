import sys, os, time, math, json
sys.path.insert(0,'/bot/src')
from rio import read_port, write_port, lidar, compass

def motors(l, r):
    write_port("d4", str(int(l))); write_port("d5", str(int(r)))

def stop(): motors(0,0)

def turn_by(delta, tol=5):
    """delta: compass-positive degrees (d5>d4 direction)."""
    c0 = compass()
    while c0 is None: c0 = compass()
    tgt = (c0 + delta) % 360
    return turn_to(tgt, tol)

def turn_to(tgt, tol=5):
    for _ in range(80):
        c = compass()
        if c is None: continue
        err = (tgt - c + 540) % 360 - 180
        if abs(err) < tol:
            stop(); return c
        sp = max(min(abs(err)*2.0, 70), 12)
        if err > 0: motors(-sp, sp)
        else: motors(sp, -sp)
        time.sleep(0.06)
    stop(); return compass()
