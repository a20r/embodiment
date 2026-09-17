import sys; sys.path.insert(0,'/bot/src')
import ctl, rio, time, math, json
TPU = 1850.0
PF = '/bot/pose.json'
def load(): return json.load(open(PF))
def save(st): json.dump(st, open(PF, 'w'))
def dir_vec(h): r = math.radians(h); return math.sin(r), math.cos(r)
def log(m):
    with open('/bot/explore.log', 'a') as f: f.write(f"{time.strftime('%H:%M:%S')} NAV {m}\n")

def goto(tx, ty, tol=0.15, timeout=90, stop_on_here=False):
    st = load(); x, y = st['x'], st['y']
    t0 = time.time()
    while time.time() - t0 < timeout:
        d = math.hypot(tx - x, ty - y)
        if d < tol: ctl.stop(); log(f'arrived ({x:.2f},{y:.2f})'); break
        want = math.degrees(math.atan2(tx - x, ty - y)) % 360
        s = ctl.scan(); h = ctl.heading()
        if s is None or h is None: continue
        # choose beam closest to want that is open enough
        best = None
        for k in range(16):
            r = s[k]
            if r < 0 or r < 0.3: continue
            rl = s[(k-1)%16]; rr = s[(k+1)%16]
            if (0 <= rl < 0.15) or (0 <= rr < 0.15): continue
            ang = (h + 22.5*k) % 360
            dev = abs(ctl.angdiff(ang, want))
            sc = -dev + 30*min(r, 1.0)
            if best is None or sc > best[0]: best = (sc, ang, r)
        if best is None:
            log('blocked, backing'); l0, r0 = ctl.enc(); ctl.drive(-40, -40); time.sleep(0.8); ctl.stop(); l1, r1 = ctl.enc()
            dd = ((l1-l0)+(r1-r0))/2/TPU; dx, dy = dir_vec(h); x += dx*dd; y += dy*dd; continue
        _, ang, r = best
        if abs(ctl.angdiff(ang, h)) > 6: ctl.turn_to(ang, tol=4)
        step = min(d, r - 0.2, 0.5)
        l0, r0 = ctl.enc()
        trav, reason = ctl.forward(step*TPU, speed=60, hold_heading=ang, min_front=0.18, timeout=8)
        l1, r1 = ctl.enc(); hh = ctl.heading()
        dd = ((l1-l0)+(r1-r0))/2/TPU; dx, dy = dir_vec(ang); x += dx*dd; y += dy*dd
        stt = ctl.status()
        log(f'pose=({x:.2f},{y:.2f}) hdg={hh:.0f} moved={trav:.0f} {reason} st={stt} d11={rio.read_line("d11",0.5)}')
        st['x'], st['y'] = x, y; save(st)
        if stop_on_here and stt.get('here') == 1: log('HERE=1, stopping'); break
    ctl.stop(); st['x'], st['y'] = x, y; save(st)
    return x, y

if __name__ == '__main__':
    tx, ty = float(sys.argv[1]), float(sys.argv[2])
    soh = len(sys.argv) > 3 and sys.argv[3] == 'here'
    print(goto(tx, ty, stop_on_here=soh))
