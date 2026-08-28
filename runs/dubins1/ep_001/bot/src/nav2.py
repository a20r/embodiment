import time, math
from nav import angdiff

def turn_to2(n, r, target, tol=14, timeout=120, thr=7):
    """Strict alternating wiggle turn, bump-aware, low speed for tight radius."""
    t0=time.time()
    # start with roomier direction
    fwd = n.clearance(0) >= n.clearance(8)
    while time.time()-t0 < timeout:
        if r.goal(): n.stop(); return 'goal'
        h=None
        while h is None and time.time()-t0<timeout:
            h=r.heading()
            if h is None: time.sleep(0.2)
        err=angdiff(target,h)
        if abs(err)<=tol: n.stop(); return True
        room = n.clearance(0) if fwd else n.clearance(8)
        if room < 0.14:
            fwd = not fwd
            room = n.clearance(0) if fwd else n.clearance(8)
            if room < 0.14:
                # both tight: tiny escape stroke
                n.cmd(0, -6 if fwd else 6); time.sleep(0.6); n.stop()
        steer = 90 if err>0 else -90
        t = thr
        if not fwd: steer, t = -steer, -t
        dur = min(2.5, max(0.8, (room-0.08)/0.05))
        n.cmd(steer, t)
        te=time.time()
        while time.time()-te<dur:
            time.sleep(0.12)
            if r.goal(): n.stop(); return 'goal'
            if r.bump(): break
            hh=r.heading()
            if hh is not None and abs(angdiff(target,hh))<=tol:
                n.stop(); return True
        n.stop(); time.sleep(0.25)
        fwd = not fwd
    n.stop(); return False
