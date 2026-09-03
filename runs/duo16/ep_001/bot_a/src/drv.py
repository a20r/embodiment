import time, threading, math, subprocess

def _w(p, s):
    try:
        f = open('/dev/robot/'+p, 'w'); f.write(s+'\n'); f.flush(); f.close()
    except Exception:
        pass

def _r(p, timeout=0.8):
    try:
        f = open('/dev/robot/'+p, 'r')
        line = [None]
        def rd(): line[0] = f.readline().strip()
        t = threading.Thread(target=rd, daemon=True); t.start(); t.join(timeout)
        f.close()
        return line[0]
    except Exception:
        return None

def stop():
    _w('d1','0'); _w('d7','0')

def heading():
    h = _r('d4')
    try: return float(h)
    except: return None

def enc():
    l = _r('d9'); r = _r('d6')
    try: return float(l), float(r)
    except: return None, None

def status():
    return _r('d3')

def scan():
    try:
        out = subprocess.run(['timeout','2','cat','/dev/robot/d2'],capture_output=True,text=True).stdout
        pts = []
        for p in out.strip().split(';'):
            if p.strip():
                a,b,c = p.split(',')[:3]
                pts.append((float(a),float(b),float(c)))
        return pts
    except Exception:
        return []

def polar(pts, zmin=-0.03, zmax=0.4, sector=10):
    """min horizontal range per azimuth sector (deg, 0=fwd CCW+), y=fwd x=lateral"""
    bins = {}
    for x,y,z in pts:
        if not (zmin < z < zmax): continue
        az = math.degrees(math.atan2(x,y))
        rh = math.hypot(x,y)
        b = int((az+180)//sector)
        if b not in bins or rh < bins[b][0]: bins[b] = (rh, az)
    return bins

def min_ahead(pts, width=40):
    """min range within +/-width deg of forward"""
    m = 99.0
    for x,y,z in pts:
        if z < -0.03 or z > 0.4: continue
        az = math.degrees(math.atan2(x,y))
        if abs(az) <= width:
            m = min(m, math.hypot(x,y))
    return m

def spin(cmd, dur):
    """raw wheel cmd for dur seconds, then stop"""
    t0 = time.time()
    while time.time()-t0 < dur:
        _w('d1', str(cmd[0])); _w('d7', str(cmd[1]))
        time.sleep(0.008)
    stop()

def rot_by(delta_deg, v=5.0, timeout=60):
    """rotate heading by delta_deg (CCW+), closed loop on d4"""
    h0 = heading()
    if h0 is None: return None
    target = h0 + delta_deg
    dirn = 1.0 if delta_deg > 0 else -1.0
    t0 = time.time()
    last = [0,0]
    while time.time()-t0 < timeout:
        h = heading()
        if h is None: continue
        err = target - h
        # shortest-path on continuous accumulation: we track raw heading, may wrap
        if abs(err) < 3: break
        if dirn*err > 0:
            c = (v*dirn, -v*dirn)  # CCW: left fwd right back
        else:
            stop(); break
        spin_frac = min(1.0, abs(err)/30.0)
        cv = max(1.5, v*spin_frac)
        _w('d1', str(cv*dirn if dirn>0 else -cv))
        _w('d7', str(-cv*dirn if dirn>0 else cv))
        time.sleep(0.01)
    stop()
    return heading()

def goto_az(az_deg, v=5.0, timeout=90):
    """rotate so that lidar azimuth az_deg points forward: rotate by -az? verify sign at runtime"""
    return rot_by(-az_deg, v, timeout)

def drive(dur, v=4.0):
    t0 = time.time()
    while time.time()-t0 < dur:
        pts = scan()
        m = min_ahead(pts) if pts else 99
        if m < 0.18:
            stop(); return False
        _w('d1', str(v)); _w('d7', str(v))
        time.sleep(0.05)
    stop(); return True

def gap_az(pts=None, thr=0.5):
    """azimuth (deg, CCW+ from forward) of centroid of open sectors (minR>thr)"""
    if pts is None: pts = scan()
    bins = polar(pts, sector=5)
    opens = [(b*5-180+2.5, rh) for b,(rh,az) in bins.items() if rh > thr]
    if not opens: return None
    # contiguous run around the widest gap: just take weighted centroid of all open az
    sx = sum(math.cos(math.radians(a)) for a,_ in opens)
    sy = sum(math.sin(math.radians(a)) for a,_ in opens)
    return math.degrees(math.atan2(sy, sx))

def turn_to_az(az_target=0.0, tol=8.0, vmax=5.0, timeout=120):
    t0=time.time()
    while time.time()-t0 < timeout:
        pts = scan()
        if not pts: continue
        g = gap_az(pts)
        if g is None:
            stop(); return False
        err = az_target - g
        if abs(err) < tol:
            stop(); return True
        v = max(1.2, min(vmax, abs(err)*0.08))
        if err > 0: _w('d1',str(v)); _w('d7',str(-v))
        else:       _w('d1',str(-v)); _w('d7',str(v))
        time.sleep(0.15)
    stop(); return False

def drive_fwd(seconds, v=4.0, stop_at=0.20):
    t0=time.time()
    ok=True
    while time.time()-t0 < seconds:
        pts = scan()
        m = min_ahead(pts) if pts else 99
        if m < stop_at:
            stop(); return False
        _w('d1',str(v)); _w('d7',str(v))
        time.sleep(0.06)
    stop(); return ok

def best_bearing(pts=None, fov=100.0, zmin=-0.03, zmax=0.4, sector=10):
    """bearing (deg) of the most open direction within ±fov, using sector min ranges"""
    if pts is None: pts = scan()
    bins = {}
    for x,y,z in pts:
        if not (zmin < z < zmax): continue
        az = math.degrees(math.atan2(x,y)); rh = math.hypot(x,y)
        if abs(az) > fov: continue
        b = int((az+fov)//sector)
        if b not in bins or rh < bins[b][0]: bins[b] = (rh, az)
    if not bins: return None, 0
    best_b, (best_r, _) = max(bins.items(), key=lambda kv: kv[1][0])
    # centroid of near-best sectors for stability
    cand = [(b*sector - fov + sector/2.0, r) for b,(r,_) in bins.items() if r > best_r*0.7]
    sx = sum(math.cos(math.radians(a))*r for a,r in cand)
    sy = sum(math.sin(math.radians(a))*r for a,r in cand)
    return math.degrees(math.atan2(sy,sx)), best_r

def escape(steps=40, log=print):
    """reactive: aim at most open bearing, creep forward"""
    for i in range(steps):
        pts = scan()
        if not pts: continue
        b, r = best_bearing(pts)
        m = min_ahead(pts, 45)
        if b is None:
            stop(); log("no bearing"); return False
        log(f"  step{i}: bearing={b:.0f} openR={r:.2f} minAhead45={m:.2f} h={heading()}")
        # steer: rotate toward bearing, but creep simultaneously if ahead is clear
        if abs(b) > 20 or m < 0.22:
            v = max(1.5, min(5.0, abs(b)*0.12))
            if b > 0: _w('d1',str(v)); _w('d7',str(-v))
            else:     _w('d1',str(-v)); _w('d7',str(v))
            time.sleep(0.25)
            stop(); time.sleep(0.05)
        else:
            # creep with bias away from bearing=0 offset
            lv = 3.0 - max(0.0,b)*0.05
            rv = 3.0 + min(0.0,b)*0.05
            _w('d1',str(lv)); _w('d7',str(rv))
            time.sleep(0.4)
            stop(); time.sleep(0.05)
    stop(); return True

def radio_send(line):
    try:
        f=open('/dev/robot/d8','w'); f.write(line+'\n'); f.flush(); f.close(); return True
    except Exception: return False

def radio_recv(timeout=0.4):
    try:
        f=open('/dev/robot/d10','r')
        import threading as th
        line=[None]
        def rd(): line[0]=f.readline().strip()
        t=th.Thread(target=rd,daemon=True); t.start(); t.join(timeout); f.close()
        return line[0]
    except Exception: return None

def wander(steps=50, log=print, radio_every=5):
    """explore: reactive escape + periodic radio ping"""
    last_open=0
    for i in range(steps):
        pts = scan()
        if not pts: continue
        b, r = best_bearing(pts)
        m = min_ahead(pts, 45)
        st = _r('d3')
        if i % radio_every == 0:
            radio_send(f"PING h={heading()} t={time.time():.0f}")
            msg = radio_recv(0.3)
            if msg: log("RADIO<<<", msg)
        log(f"  w{i}: bearing={b:.0f} openR={r:.2f} minA={m:.2f} h={heading()} st={st}")
        if abs(b) > 20 or m < 0.22:
            v = max(1.5, min(5.0, abs(b)*0.12))
            if b > 0: _w('d1',str(v)); _w('d7',str(-v))
            else:     _w('d1',str(-v)); _w('d7',str(v))
            time.sleep(0.25)
            stop(); time.sleep(0.05)
        else:
            lv = 3.0 - max(0.0,b)*0.05; rv = 3.0 + min(0.0,b)*0.05
            _w('d1',str(lv)); _w('d7',str(rv))
            time.sleep(0.4)
            stop(); time.sleep(0.05)
    stop()

def follow(steps=60, log=print, radio_every=6, v=3.0):
    """corridor follower: steer to gap centroid, stop-cone only +/-12deg, abort if wedged"""
    for i in range(steps):
        pts = scan()
        if not pts: continue
        b, r = best_bearing(pts, fov=70, sector=5)
        # clear cone straight ahead
        cone = 99.0
        for x,y,z in pts:
            if -0.03<z<0.4:
                az = math.degrees(math.atan2(x,y))
                if abs(az) < 12: cone = min(cone, math.hypot(x,y))
        if i % radio_every == 0:
            radio_send("PING1")
            msg = radio_recv(0.25)
            if msg: log("RADIO<<<", msg)
        log(f"f{i}: b={b:.0f} open={r:.2f} cone={cone:.2f} h={heading()}")
        if cone < 0.16:
            log("  cone blocked"); stop(); return False
        if b is None:
            stop(); return False
        # steer
        err = b
        if abs(err) > 25:
            stop(); time.sleep(0.02)
            vv = max(1.5, min(4.0, abs(err)*0.1))
            if err>0: _w('d1',str(vv)); _w('d7',str(-vv))
            else:     _w('d1',str(-vv)); _w('d7',str(vv))
            time.sleep(0.2); stop(); time.sleep(0.03)
        else:
            vv = v
            lv = vv - err*0.04; rv = vv + err*0.04
            _w('d1',str(max(1.0,lv))); _w('d7',str(max(1.0,rv)))
            time.sleep(0.35); stop(); time.sleep(0.03)
    stop(); return True

def wall_follow(steps=80, side=+1, target=0.14, log=print, radio_every=10, v=2.6):
    """follow wall on +az (side=+1) or -az (side=-1) at distance target"""
    for i in range(steps):
        pts = scan()
        if not pts: continue
        # wall distance on chosen side
        W = None
        for x,y,z in pts:
            if -0.03<z<0.4:
                az = math.degrees(math.atan2(x,y))
                if side*50 < az < side*130:
                    r = math.hypot(x,y)
                    if W is None or r < W: W = r
        # front cone
        F = 99.0
        Faz = None
        for x,y,z in pts:
            if -0.03<z<0.4:
                az = math.degrees(math.atan2(x,y)); r = math.hypot(x,y)
                if abs(az) < 15 and r < F: F, Faz = r, az
        if i % radio_every == 0:
            radio_send("PING1")
            msg = radio_recv(0.2)
            if msg: log("RADIO<<<", msg)
        if i % 3 == 0:
            log(f"wl{i}: W={None if W is None else round(W,2)} F={round(F,2)} h={heading()}")
        if W is None:
            # lost wall: drive straight slowly
            _w('d1',str(v*0.7)); _w('d7',str(v*0.7)); time.sleep(0.3); stop(); continue
        if F < 0.19:
            # wall ahead: turn away from wall side
            stop(); time.sleep(0.02)
            if side>0: _w('d1',str(-2.2)); _w('d7',str(2.2))   # CW
            else:      _w('d1',str(2.2));  _w('d7',str(-2.2))
            time.sleep(0.30); stop(); time.sleep(0.04)
            continue
        # steer to keep wall distance
        err = (W - target)*side   # >0: too far from wall, steer toward (+az) side
        vv = v
        c = max(-1.4, min(1.4, err*8.0))
        if side>0:
            lv = vv - c; rv = vv + c      # err>0 -> curve CCW toward +az wall? check
        else:
            lv = vv + c; rv = vv - c
        _w('d1',str(max(0.8,lv))); _w('d7',str(max(0.8,rv)))
        time.sleep(0.3); stop(); time.sleep(0.03)
    stop()
