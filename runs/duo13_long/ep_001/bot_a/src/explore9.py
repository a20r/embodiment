import time, math, signal, sys, json, threading, random, os
from robot import Robot

r = Robot()
TICKS = 544.0

logf = open('/bot/src/explore9.log', 'a', buffering=1)
def log(*a):
    logf.write(f'[{time.strftime("%H:%M:%S")}] ' + ' '.join(str(x) for x in a) + '\n')

def stopall(*a):
    try:
        r.stop(); log('SIGNAL EXIT'); 
    except Exception: pass
    sys.exit(0)
signal.signal(signal.SIGTERM, stopall)
signal.signal(signal.SIGINT, stopall)

x = y = 0.0
def integrate(d, hdeg):
    global x, y
    rad = math.radians(hdeg); x += d*math.sin(rad); y += d*math.cos(rad)

def clean(lid):
    out = []
    for i in range(16):
        v = lid[i]
        if v is None or v < 0:
            a = lid[(i-1) % 16]; b = lid[(i+1) % 16]
            cc = [z for z in (a, b) if z and z > 0]
            v = min(cc) if cc else 0.4
        out.append(v)
    return out

def rotto(tg, timeout=7, tol=7):
    t0 = time.time()
    while time.time()-t0 < timeout:
        hc = r.heading()
        if hc is None: continue
        e = (tg-hc+180) % 360-180
        if abs(e) <= tol: break
        v = max(12, min(34, abs(e)*1.4))
        if e > 0: r.wheels(v, -v)
        else: r.wheels(-v, v)
        time.sleep(0.04)
    r.stop(); time.sleep(0.08)

def drive_until(tg, maxdist=3.5, stop_front=0.24, sp_hi=24, sp_lo=15):
    """compass-held drive; returns (traveled_m, reason). reason in FLAG/wall/max"""
    l0, r0 = r.enc()
    if l0 is None: return 0.0, 'enc'
    t0 = time.time(); traveled = 0.0; reason = 'max'
    while time.time()-t0 < maxdist/6.0+10:
        st = r.status()
        if st and st[1] is not None and (st[1] or st[2]):
            r.stop(); return traveled, 'FLAG'
        lid = r.lidar()
        if not lid:
            r.wheels(12, 12); time.sleep(0.05); continue
        c = clean(lid)
        f = min(c[15], c[0], c[1])
        if f < stop_front:
            reason = 'wall'; break
        h = r.heading()
        d = ((h-tg+180) % 360-180) if h is not None else 0.0
        steer = -d*1.4
        minL = min(c[11], c[12], c[13]); minR = min(c[3], c[4], c[5])
        if minL < 0.22: steer += (0.22-minL)*30
        if minR < 0.22: steer -= (0.22-minR)*30
        steer = max(-7, min(7, steer))
        sp = sp_hi if f > 0.5 else sp_lo
        r.wheels(sp+steer, sp-steer)
        l1, r1 = r.enc()
        if l1 is not None:
            traveled = ((l1-l0)+(r1-r0))/2.0/TICKS
            if traveled >= maxdist: break
        time.sleep(0.05)
    r.stop(); time.sleep(0.06)
    l1, r1 = r.enc()
    if l1 is not None: traveled = ((l1-l0)+(r1-r0))/2.0/TICKS
    return traveled, reason

def rev(dist=0.35, timeout=8):
    """reverse straight (no rotation needed); returns distance"""
    l0, r0 = r.enc()
    if l0 is None: return 0.0
    t0 = time.time(); moved = 0.0
    while time.time()-t0 < timeout:
        lid = r.lidar()
        if lid:
            c = clean(lid)
            rear = min(c[6], c[7], c[8], c[9], c[10])
            if rear < 0.15: break
        r.wheels(-14, -14)
        l1, r1 = r.enc()
        if l1 is not None:
            moved = ((l0-l1)+(r0-r1))/2.0/TICKS
            if moved >= dist: break
        time.sleep(0.05)
    r.stop(); time.sleep(0.06)
    return moved

def escape2():
    """pocket escape: back out straight, then rotate to longest ray and go"""
    m = rev(0.35)
    lid = r.lidar()
    best_d = 0.0
    if lid:
        c = clean(lid)
        best_i = max(range(16), key=lambda i: c[i])
        best_d = c[best_i]
        if best_d > 0.55:
            az = ((r.heading() or 0) + best_i*22.5) % 360
            rotto(az, timeout=9)
            d, _ = drive_until(az, maxdist=1.2, stop_front=0.20)
            return m + d
    # fallback: pick random heading and try
    az = random.randrange(0, 360)
    rotto(az, timeout=9)
    d, _ = drive_until(az, maxdist=1.0, stop_front=0.20)
    return m + d

def detect_robot():
    lid = r.lidar()
    if not lid: return None
    c = clean(lid)
    for i in range(16):
        if min(c[(i-1) % 16], c[i], c[(i+1) % 16]) < 0.5:
            if min(c[(i+3) % 16], c[(i+4) % 16], c[(i-3) % 16], c[(i-4) % 16]) > 0.75:
                return i
    return None

# ---------------- radio ----------------
rx_count = [0]; rx_last = [0.0]; rx_times = []
b_at_goal = [False]
def radio():
    n = 0
    while True:
        st = r.status()
        g = st[1] if st and st[1] is not None else 0
        he = st[2] if st and st[2] is not None else 0
        hh = r.heading(); 
        r.send(f'A PING {x:.2f} {y:.2f} {hh if hh is not None else 0:.0f} n={n} goal={g} here={he} MOW')
        if n % 12 == 5:
            r.send('A TIP: goal may be in room interior; do lawnmower strips N-S spacing 0.35m; beacon if GOAL=1/HERE=1')
        n += 1
        t0 = time.time()
        while time.time()-t0 < 1.5:
            v = r.recv(0.12)
            if v:
                rx_count[0] += 1; rx_last[0] = time.time(); rx_times.append(time.time())
                if len(rx_times) > 400: del rx_times[:200]
                hnow = r.heading()
                log('!!!RX h=%s pose=(%.2f,%.2f):' % (hnow, x, y), v)
                with open('/memory/rx_log.txt', 'a') as f:
                    f.write('%s h=%s pose=(%.2f,%.2f) %s\n' % (time.strftime('%H:%M:%S'), hnow, x, y, v))
                r.send('A ACK %d %s' % (time.time(), v.strip()[:30]))
                U = v.upper()
                if 'GOAL=1' in U or 'HERE=1' in U or 'AT GOAL' in U:
                    json.dump({'msg': v, 't': time.time()}, open('/memory/goal_from_B.json', 'w'))
                    b_at_goal[0] = True
            time.sleep(0.1)
threading.Thread(target=radio, daemon=True).start()

def rate_sample(t=2.5):
    c0 = rx_count[0]; t0 = time.time()
    while time.time()-t0 < t: time.sleep(0.1)
    return rx_count[0]-c0

# ---------------- goal mode ----------------
def goal_mode(st):
    log('GOAL MODE!!! flags', st, 'pose %.2f %.2f' % (x, y))
    json.dump({'x': x, 'y': y, 'h': r.heading(), 'flags': st, 't': time.time()},
              open('/memory/goal_found.json', 'w'))
    r.stop(); n = 0
    while True:
        r.stop()
        st2 = r.status()
        lid = r.lidar()
        r.send(f'A AT GOAL n={n} flags={st2} h={r.heading() if r.heading() is not None else 0:.0f}')
        if n % 6 == 2 and lid:
            r.send('A GOAL LID ' + ','.join('%.2f' % v for v in clean(lid)))
        v = r.recv(0.5)
        if v:
            log('GOALMODE RX:', v)
            with open('/memory/rx_log.txt', 'a') as f:
                f.write('%s GOALMODE %s\n' % (time.strftime('%H:%M:%S'), v))
        n += 1; time.sleep(1.2)

# ---------------- homing (B at goal) ----------------
def homing():
    log('HOMING start (B at goal)')
    best = rate_sample(2.0)
    t_last_rx = time.time()
    tries = 0
    while tries < 600:
        tries += 1
        st = r.status()
        if st and st[1] is not None and (st[1] or st[2]):
            goal_mode(st); return
        if rx_times and rx_times[-1] > time.time()-5: t_last_rx = time.time()
        if time.time()-t_last_rx > 150:
            log('homing: no RX 150s -> resume mow'); return
        tgt = detect_robot()
        if tgt is not None:
            az = ((r.heading() or 0) + tgt*22.5) % 360
            log('homing: sighting beam', tgt, 'az %.0f' % az)
            rotto(az, timeout=5)
            d, _ = drive_until(az, maxdist=0.5, stop_front=0.20)
            c = rate_sample(1.5)
            if c >= best: best = c
            continue
        improved = False
        for hh in (0, 45, -45, 90, -90, 135, -135, 180):
            cur = r.heading() or 0
            tg = (cur+hh) % 360
            rotto(tg, timeout=6)
            before = rx_count[0]
            d, _ = drive_until(tg, maxdist=0.30, stop_front=0.20)
            c = rate_sample(2.0)
            log('homing try hh=%d d=%.2f rate=%d best=%d' % (hh, d, c, best))
            if c > best:
                best = c; improved = True; break
        if not improved:
            if best >= 4:
                log('homing strong contact; holding')
                time.sleep(2.0)
            else:
                # bigger exploratory hop toward last-known direction
                tg = (r.heading() or 0) + random.choice([-120, 120, 60, -60])
                rotto(tg % 360, timeout=6)
                drive_until(tg % 360, maxdist=0.6, stop_front=0.20)

# ---------------- main mow loop ----------------
align = {}
log('=== explore9 (lawnmower) start ===')
r.stop(); time.sleep(0.3)
strip_h = 0          # N
adv = 1              # +1 = east
netE = 0.0
pocket_ct = 0
step = 0
t_heart = time.time()
last_pos = (x, y); t_progress = time.time()
try:
    while True:
        step += 1
        st = r.status()
        if st and st[1] is not None and (st[1] or st[2]):
            goal_mode(st)
        if b_at_goal[0]:
            homing(); continue
        rotto(strip_h)
        d, reason = drive_until(strip_h, maxdist=3.5)
        integrate(d, strip_h)
        if reason == 'FLAG':
            st = r.status()
            goal_mode(st)
        # lateral offset for next strip: try preferred side, then other; escape if both blocked
        got = 0.0
        for attempt in range(2):
            adv_h = 90 if adv > 0 else 270
            rotto(adv_h)
            g, _ = drive_until(adv_h, maxdist=0.50+random.random()*0.12, stop_front=0.20)
            integrate(g, adv_h)
            if g >= 0.15:
                got = g
                break
            adv = -adv
        if got == 0.0:
            # pocket: both sides blocked -> reverse out, then longest ray
            ge = escape2()
            integrate(ge, r.heading() or 0)
            log('POCKET escape moved=%.2f' % ge)
            strip_h = random.choice([0, 180])
        if d >= 0.6:
            pocket_ct = 0
        elif d < 0.15:
            pocket_ct += 1
            if pocket_ct >= 2:
                pocket_ct = 0
                log('short strips x2; escape2')
                ge = escape2()
                integrate(ge, r.heading() or 0)
        strip_h = 180 if strip_h == 0 else 0
        # stuck detection
        if (abs(x-last_pos[0])+abs(y-last_pos[1])) > 0.8:
            last_pos = (x, y); t_progress = time.time()
        if time.time()-t_progress > 240:
            log('STUCK 4min; escape maneuver')
            tg = random.randrange(0, 360)
            rotto(tg)
            drive_until(tg, maxdist=1.0, stop_front=0.22)
            last_pos = (x, y); t_progress = time.time()
        if step % 5 == 0 or time.time()-t_heart > 60:
            t_heart = time.time()
            log('%d: strip=%d adv=%+d d=%.2f %s pkt=%d pose %.1f,%.1f rx=%d' %
                (step, strip_h, adv, d, reason, netE, x, y, rx_count[0]))
except Exception as e:
    import traceback; log('FATAL', traceback.format_exc())
finally:
    r.stop()
    log('=== end ===', '%.2f,%.2f' % (x, y))
