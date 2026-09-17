import sys; sys.path.insert(0,'/bot/src')
import ctl, rio, time, math, json, random
TPU = 1850.0  # encoder ticks per range unit
import os
BIAS=float(os.environ.get('BIAS','0.4')); GX=float(os.environ.get('GX','-0.5')); GY=float(os.environ.get('GY','-0.87'))
CELL = 0.25
state_file = '/bot/pose.json'
try:
    st = json.load(open(state_file))
except Exception:
    st = {'x': 0.0, 'y': 0.0, 'visits': {}}
x, y = st['x'], st['y']
visits = {tuple(map(int, k.split(','))): v for k, v in st.get('visits', {}).items()}
goal_pos = st.get('goal_pos')

def save():
    json.dump({'x': x, 'y': y, 'visits': {f'{k[0]},{k[1]}': v for k, v in visits.items()}, 'goal_pos': goal_pos}, open(state_file, 'w'))

def cell(px, py): return (int(math.floor(px / CELL)), int(math.floor(py / CELL)))

def log(msg):
    with open('/bot/explore.log', 'a') as f: f.write(f"{time.strftime('%H:%M:%S')} {msg}\n")

def mark_visit():
    c = cell(x, y); visits[c] = visits.get(c, 0) + 1

def dir_vec(hdg):  # heading deg clockwise from north; x east, y north
    r = math.radians(hdg); return math.sin(r), math.cos(r)

def score_dirs(s, hdg):
    best = None
    for k in range(16):
        r = s[k]
        if r < 0: continue
        ang = (hdg + 22.5 * k) % 360
        # require neighbours not too tight
        rl = s[(k - 1) % 16]; rr = s[(k + 1) % 16]
        width_ok = (rl < 0 or rl > 0.18) and (rr < 0 or rr > 0.18)
        if r < 0.35 or not width_ok: continue
        dx, dy = dir_vec(ang)
        # visit penalty along the ray
        pen = 0.0
        for d in (0.3, 0.6, 0.9):
            if d > r: break
            pen += visits.get(cell(x + dx * d, y + dy * d), 0)
        sc = min(r, 1.4) - 0.35 * pen + random.uniform(0, 0.05)
        if k == 0: sc += 0.1  # slight preference to continue straight
        sc += BIAS * (dx * GX + dy * GY)
        if best is None or sc > best[0]: best = (sc, ang, r)
    return best

def step_odometry(l0, r0, l1, r1, hdg):
    global x, y
    d = ((l1 - l0) + (r1 - r0)) / 2 / TPU
    dx, dy = dir_vec(hdg); x += dx * d; y += dy * d

last_tx = 0
log(f'explorer start pose=({x:.2f},{y:.2f})')
stuck = 0
while True:
    stt = ctl.status()
    if stt.get('goal') == 1 and goal_pos is None:
        goal_pos = [x, y]; log(f'*** GOAL FLAG at ({x:.2f},{y:.2f}) status={stt}'); save()
        with open('/memory/NOTES.md', 'a') as f: f.write(f'GOAL FLAG seen at odom ({x:.2f},{y:.2f}) status={stt}\n')
    if stt.get('here') == 1:
        log(f'*** HERE FLAG status={stt} at ({x:.2f},{y:.2f})')
        if os.environ.get('STOP_ON_HERE'): ctl.stop(); save(); log('stopping on goal'); break
    s = ctl.scan(); h = ctl.heading()
    if s is None or h is None: time.sleep(0.2); continue
    mark_visit()
    with open('/bot/map.log', 'a') as f: f.write(json.dumps({'x': round(x, 3), 'y': round(y, 3), 'h': h, 's': s, 't': time.time()}) + '\n')
    if time.time() - last_tx > 20:
        rio.write_line('d8', f'Robot A here. My odom pos x={x:.2f} y={y:.2f} (x east,y north, units=range units). Goal found: {goal_pos}. Reply with your status.'); last_tx = time.time()
    best = score_dirs(s, h)
    if best is None:
        log('no open dir, backing up'); l0, r0 = ctl.enc(); ctl.drive(-40, -40); time.sleep(1.0); ctl.stop(); l1, r1 = ctl.enc(); step_odometry(l0, r0, l1, r1, h); continue
    sc, ang, r = best
    if abs(ctl.angdiff(ang, h)) > 8: ctl.turn_to(ang, tol=4)
    h = ctl.heading(); l0, r0 = ctl.enc()
    dist = min(r - 0.2, 0.7) * TPU
    trav, reason = ctl.forward(dist, speed=70, hold_heading=ang, min_front=0.2, timeout=12)
    l1, r1 = ctl.enc(); hh = ctl.heading()
    step_odometry(l0, r0, l1, r1, (ang + hh) / 2 if abs(ctl.angdiff(ang, hh)) < 30 else hh)
    log(f'pose=({x:.2f},{y:.2f}) hdg={hh:.0f} moved={trav:.0f} {reason} d11={rio.read_line("d11",0.5)} st={stt}')
    if abs(trav) < 40: stuck += 1
    else: stuck = 0
    if stuck >= 2:
        log('stuck, reversing'); l0, r0 = ctl.enc(); ctl.drive(-50, -50); time.sleep(1.2); ctl.stop(); l1, r1 = ctl.enc(); step_odometry(l0, r0, l1, r1, hh); stuck = 0
    save()
