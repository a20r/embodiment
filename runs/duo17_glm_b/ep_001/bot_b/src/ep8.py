import sys, time, math
sys.path.insert(0, '/bot/src')
import robot2 as R

LOG = open('/memory/ep8.log', 'a', buffering=1)
def L(s):
    LOG.write('%9.1f %s\n' % (time.time() % 1000000, s))

def here(): return 'here=1' in R.status()
def agoal(): return 'goal=1' in R.status()
def txstate():
    st = R.status()
    for tok in st.split():
        if tok.startswith('tx='):
            return tok.split(':', 1)[1] if ':' in tok else 'idle'
    return 'idle'

def front():
    for _ in range(14):
        l = R.lidar()
        if len(l) == 16:
            xs = [x for x in (l[15], l[0], l[1]) if x > 0]
            return min(xs) if xs else 9.0
        time.sleep(0.03)
    return 9.0

def turnto(tgt, sp=24, tol=8, timeout=4):
    t0 = time.time()
    while time.time() - t0 < timeout:
        err = ((tgt - R.heading() + 180) % 360) - 180
        if abs(err) < tol: break
        if err > 0: R.motors(sp, -sp)
        else: R.motors(-sp, sp)
        time.sleep(0.06)
    R.stop(); time.sleep(0.1)

BEACON = 'B-TO-A: B AT GOAL ZONE (B here=1). A: DRIVE TOWARD B CARRIER (d11 RISING). STOP WHEN YOUR here=1. HOLD. ACK'
def beacon():
    R.tx(BEACON)

class Vec:  # dead-reckoning displacement from start, meters, +y = initial heading? use world compass
    def __init__(self):
        l9, l6 = R.odom()
        self.o9, self.o6 = l9, l6
        self.x = 0.0; self.y = 0.0
    def poll(self):
        l9, l6 = R.odom()
        d9 = l9 - self.o9; d6 = l6 - self.o6
        self.o9, self.o6 = l9, l6
        ds = (d9 + d6) / 2.0 * 0.00066
        h = math.radians(R.heading())
        self.x += ds * math.sin(h); self.y += ds * math.cos(h)
        return self.x, self.y

def drive_guard(vec, dur, sp=20, msg=None):
    t0 = time.time()
    while time.time() - t0 < dur:
        vec.poll()
        if here(): R.stop(); return 'here'
        if agoal() and vec is V and math.hypot(V.x, V.y) > 2.5:
            R.stop(); return 'agoal'
        f = front()
        if f < 0.30:
            R.stop(); return 'blocked'
        R.motors(sp, sp)
        if msg: R.tx(msg)
        time.sleep(0.12)
    R.stop(); return 'ok'

def reacquire():
    L('reacquire zone from (%.1f,%.1f)' % (V.x, V.y))
    for i in range(60):
        if here() or agoal(): return True
        f = front()
        if f < 0.30:
            l = R.lidar()
            if len(l) == 16:
                rgt = min([x for x in (l[10], l[11], l[12]) if x > 0] or [9])
                lft = min([x for x in (l[4], l[5], l[6]) if x > 0] or [9])
                turnto(R.heading() + (70 if rgt > lft else -70))
        drive_guard(V, 1.3, 20, 'B-TO-A: B SEEKING GOAL ZONE')
    return here()

def go_home_vec(maxs=25.0):
    # drive back toward vector origin (goal anchor)
    t0 = time.time()
    while time.time() - t0 < 60:
        x, y = V.poll()
        d = math.hypot(x, y)
        if here(): R.stop(); return True
        if d < 0.4:
            R.stop()
            if here(): return True
            for _ in range(20):
                if here(): return True
                drive_guard(V, 1.2, 18, None)
            return here()
        bearing = math.degrees(math.atan2(-x, -y)) % 360
        turnto(bearing, tol=10)
        f = front()
        if f < 0.30:
            turnto(R.heading() + (85 if (V.y * math.sin(math.radians(R.heading())) + V.x * math.cos(math.radians(R.heading()))) < 0 else -85))
        R.motors(20, 20)
        R.tx('B-TO-A: B RETURNING TO GOAL ZONE. A: GO TO GOAL. YOUR here=1.')
        time.sleep(0.15)
    return here()

# ---------------- SEARCH ----------------
def search():
    L('SEARCH begin')
    radii = [5.0, 9.0, 13.0]
    dirs = [0, 45, 90, 135, 180, 225, 270, 315]
    for rad in radii:
        for d in dirs:
            if here() is False:
                if not reacquire(): L('cannot reacquire!'); time.sleep(2)
            turnto(d)
            # outbound
            leg = 0; hit = False
            while leg < int(rad / 0.15):
                st = drive_guard(V, 0.75, 20, BEACON)
                leg += 1
                if st == 'agoal':
                    if not go_home_vec(): reacquire()
                    hold(7200); return
                if st == 'blocked':
                    turnto(R.heading() + 70); drive_guard(V, 0.6, 18, None); turnto(d)
                if st == 'here': break
                if agoal() and not here():
                    L('A AT GOAL WHILE I SEARCH -> RUSH BACK'); hit = False
                    if not go_home_vec(): reacquire()
                    hold(7200); return
                # detection while away from goal
                x, y = V.poll(); dm = math.hypot(x, y)
                ts = txstate(); v11 = R.fget('d11')
                m = R.rx()
                if m: L('RX %s' % m[:100])
                if dm > 3.0 and ts in ('ok', 'busy'):
                    L('DETECT tx=%s d=%.1f d11=%.3f' % (ts, dm, v11)); hit = True; break
                if dm > 3.0 and v11 > 0.72:
                    L('DETECT d11=%.3f d=%.1f' % (v11, dm)); hit = True; break
            if hit:
                r = anchor(rad)
                if r == 'done': return
                continue
            # return to goal
            if not go_home_vec():
                L('return failed at dir %d' % d); reacquire()
            L('leg dir %d r %.0f done' % (d, rad))
        L('radius %.0f complete' % rad)
    if not here(): reacquire()
    L('SEARCH exhausted; back to HOLD')

def anchor(rad):
    L('ANCHOR: stop+beacon, waiting for A')
    R.stop()
    t0 = time.time(); lastsig = time.time()
    while time.time() - t0 < 600:
        beacon(); time.sleep(0.3)
        v11 = R.fget('d11'); ts = txstate()
        m = R.rx()
        if m: L('RX %s' % m[:100]); lastsig = time.time()
        if ts in ('ok', 'busy'): lastsig = time.time()
        x, y = V.poll(); dm = math.hypot(x, y)
        if v11 > 0.80:
            L('A CLOSE d11=%.3f - LEAD TO GOAL' % v11)
            # guide A: I go to goal, A follows carrier
            if not here():
                if go_home_vec(): L('led A home, here=1')
            break
        if agoal():
            L('goal=1 while anchored')
            if not here(): go_home_vec()
            break
        if time.time() - lastsig > 120:
            L('lost A signal; resume search')
            go_home_vec(); return 'lost'
    # final: hold at goal (escorted or A at goal)
    hold(7200); return 'done'

# ---------------- HOLD / DANCE ----------------
def hold(dur):
    L('HOLD %.0fs here=%s goal=%s' % (dur, here(), agoal()))
    t0 = time.time(); peak = 0; nclose = 0; both_t = None
    k = 0; hi = 0.0
    while time.time() - t0 < dur:
        k = (k + 1) % 8
        if k < 5: beacon()
        v11 = R.fget('d11')
        peak = max(peak, v11)
        if v11 > hi + 0.06 and v11 > 0.60:
            hi = v11; L('d11 rising %.3f (txstate busy may be goal carrier)' % v11)
        if v11 < hi - 0.15: hi = max(0.0, v11)
        m = R.rx()
        if m: L('RX %s' % m[:100])
        st = R.status()
        if 'here=0' in st:
            L('lost zone; reacquire'); reacquire()
        if agoal() and here():
            if both_t is None:
                both_t = time.time(); L('*** BOTH FLAGS 1 ***')
            time.sleep(0.3)
            if time.time() - both_t > 75:
                L('pingpong fresh-arrival hedge')
                pingpong()
                both_t = time.time()
            continue
        if agoal() and not here():
            L('A IN ZONE, B OUT -> RUSH'); rush(); return
        if not agoal():
            nclose = nclose + 1 if v11 > 0.84 else 0
            if nclose >= 6:
                L('A ADJACENT d11=%.3f -> DANCE' % v11)
                dance(); return
            if v11 > 0.68 and peak > 0.68:
                L('carrier pulse d11=%.3f - talking' % v11)
        time.sleep(0.25 if k < 5 else 1.1)
    L('HOLD timeout -> SEARCH')
    search()

def pingpong():
    # exit 1.5m, re-enter, refresh arrival
    if not here(): return
    h0 = R.heading()
    turnto((h0 + 180) % 360)
    for _ in range(14):
        if 'here=0' in R.status(): break
        if drive_guard(V, 0.5, 20, 'B-TO-A: A HOLD POSITION. B PINGPONG 20s.') == 'blocked':
            turnto(R.heading() + 70)
    time.sleep(1.0)
    for _ in range(25):
        R.tx('B-TO-A: A STAY IN ZONE. B RE-ENTERING NOW.')
        if here(): break
        if agoal(): break
        f = front()
        if f < 0.30: turnto(R.heading() + 80)
        R.motors(20, 20); time.sleep(0.15)
    R.stop()
    L('pingpong done here=%s goal=%s' % (here(), agoal()))

def dance():
    h0 = R.heading()
    beacon()
    R.tx('B-TO-A: A: GO INTO GOAL ZONE (your here=1 fires) AND STOP THERE. B STEPS OUT 1 MIN THEN RETURNS.')
    turnto((h0 + 180) % 360)
    for _ in range(16):
        if 'here=0' in R.status(): break
        if drive_guard(V, 0.55, 20, None) == 'blocked':
            turnto(R.heading() + 70)
    L('exited zone here=%s goal=%s' % (here(), agoal()))
    t0 = time.time()
    while time.time() - t0 < 300:
        R.tx('B-TO-A: A: ENTER ZONE (your here=1) AND STOP. B RETURNS WHEN YOU ARE IN.')
        if agoal() and not here():
            L('A IN ZONE -> RUSH'); break
        if R.fget('d11') > 0.88:
            L('A ADJACENT OUTSIDE -> LEAD IN'); break
        if time.time() - t0 > 300: break
        time.sleep(0.3)
    # re-enter
    if not here():
        go_home_vec()
    if here():
        L('B back in zone here=1 goal=%s' % agoal())
        if not agoal():
            t1 = time.time()
            while time.time() - t1 < 240:
                R.tx('B-TO-A: B IN ZONE. A: COME IN (your here=1) AND STOP.')
                if agoal():
                    L('*** BOTH IN ***'); break
                m = R.rx()
                if m: L('RX %s' % m[:100])
                time.sleep(0.3)
    hold(7200)

def rush():
    if not go_home_vec():
        reacquire()
    hold(7200)

V = Vec()
if __name__ == '__main__':
    R.stop()
    L('=== ep8 start here=%s goal=%s odom=%s,%s d11=%.3f' % (here(), agoal(), V.o9, V.o6, R.fget('d11')))
    try:
        while True:
            hold(360)   # first hold short, then search; later cycles 25min
            search()
    except Exception as e:
        L('FATAL %r' % e)
        while True:
            try:
                beacon(); time.sleep(1)
            except Exception: pass
