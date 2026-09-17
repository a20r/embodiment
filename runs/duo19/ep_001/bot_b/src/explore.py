import sys, time, math, json, os, threading, heapq
sys.path.insert(0, '/bot/src')
from rob import read_line, write_line
from drive import heading, ranges, enc, motors, stop, angdiff

TPU = 1820.0; CELL = 0.05; MAXR = 1.5
HARD = 1      # cells around occupied that are impassable
SOFT = 3      # cells around occupied that are costly
LOG = open('/bot/src/explore.log', 'a')
def log(*a):
    LOG.write(time.strftime('%H:%M:%S ') + ' '.join(str(x) for x in a) + '\n'); LOG.flush()

class Pose:
    def __init__(self, x0=0.0, y0=0.0):
        self.x = x0; self.y = y0; self.th = heading() or 0.0
        self.e = enc(); self.lock = threading.Lock(); self.run = True
        threading.Thread(target=self.loop, daemon=True).start()
    def loop(self):
        while self.run:
            h = heading(); e = enc()
            if h is not None: self.th = h
            if e and self.e:
                d = ((e[0]-self.e[0]) + (e[1]-self.e[1])) / 2.0 / TPU
                with self.lock:
                    self.x += d * math.cos(math.radians(self.th)); self.y += d * math.sin(math.radians(self.th))
                self.e = e
            time.sleep(0.03)
    def get(self):
        with self.lock: return self.x, self.y, self.th

score = {}   # cell -> int; >=2 occupied, <=-1 free (or known), else unknown-ish
d11log = []; POSE = None
def cell(x, y): return (int(math.floor(x / CELL)), int(math.floor(y / CELL)))
def state(c):
    s = score.get(c)
    if s is None: return None
    return 'O' if s >= 2 else 'F'
def scan_into_grid(pose):
    r = ranges()
    if not r: return
    x, y, th = pose
    score[cell(x, y)] = score.get(cell(x, y), 0) - 1
    for i, d in enumerate(r):
        if d <= 0: continue
        b = math.radians(th + 22.5 * i); dd = min(d, MAXR)
        n = int(dd / (CELL / 2))
        seen = set()
        for k in range(1, n):
            c = cell(x + k * (CELL/2) * math.cos(b), y + k * (CELL/2) * math.sin(b))
            if c in seen: continue
            seen.add(c); score[c] = score.get(c, 0) - 1
        if d < MAXR:
            c = cell(x + d * math.cos(b), y + d * math.sin(b))
            score[c] = score.get(c, 0) + 3
    v = read_line(11, 0.3)
    try: d11log.append((round(x,3), round(y,3), float(v)))
    except: pass
    return r

def scan_match(pose, r):
    x, y, th = pose.get()
    pts = [(math.radians(th + 22.5*i), d) for i, d in enumerate(r) if 0 < d < MAXR]
    if len(pts) < 8: return
    def matches(dx, dy):
        n = 0
        for b, d in pts:
            c = cell(x + dx + d*math.cos(b), y + dy + d*math.sin(b))
            if score.get(c, 0) >= 2 or any(score.get((c[0]+ex, c[1]+ey), 0) >= 2 for ex in (-1,0,1) for ey in (-1,0,1)): n += 1
        return n
    base = matches(0, 0); best = (base, 0, 0)
    for i in range(-6, 7):
        for j in range(-6, 7):
            if i == 0 and j == 0: continue
            m = matches(i*CELL, j*CELL)
            if m > best[0] or (m == best[0] and abs(i)+abs(j) < abs(best[1])+abs(best[2])): best = (m, i, j)
    if best[0] >= base + 3 and (best[1] or best[2]):
        dx, dy = best[1]*CELL*0.5, best[2]*CELL*0.5   # apply half the correction
        with pose.lock: pose.x += dx; pose.y += dy
        log('scan-match correction dx=%.2f dy=%.2f (%d->%d matches)' % (dx, dy, base, best[0]))

def save_map():
    if not score: return
    xs = [c[0] for c in score]; ys = [c[1] for c in score]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    px, py, _ = POSE.get(); pc = cell(px, py)
    lines = []
    for iy in range(y1, y0 - 1, -1):
        row = ''
        for ix in range(x0, x1 + 1):
            v = state((ix, iy)); ch = '#' if v == 'O' else ('.' if v == 'F' else ' ')
            if (ix, iy) == pc: ch = 'R'
            row += ch
        lines.append(row)
    with open('/bot/src/map.txt', 'w') as f:
        f.write('x0=%d y0=%d CELL=%.2f top_y=%d pose=%.2f,%.2f\n' % (x0, y0, CELL, y1, px, py))
        f.write('\n'.join(lines) + '\n')
    with open('/bot/src/d11.json', 'w') as f: json.dump(d11log, f)
    with open('/bot/src/grid.json', 'w') as f: json.dump({'%d,%d' % c: v for c, v in score.items()}, f)
    with open('/bot/src/pose.txt', 'w') as f: f.write('%.3f %.3f %.1f\n' % POSE.get())
    os.system('cp /bot/src/map.txt /bot/src/pose.txt /bot/src/d11.json /bot/src/grid.json /memory/ 2>/dev/null')

def neighbors(c):
    for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
        yield (c[0]+dx, c[1]+dy)

def cost_maps():
    hard = set(); soft = set()
    for c, s in score.items():
        if s >= 2:
            for dx in range(-SOFT, SOFT + 1):
                for dy in range(-SOFT, SOFT + 1):
                    cc = (c[0]+dx, c[1]+dy); soft.add(cc)
                    if abs(dx) <= HARD and abs(dy) <= HARD: hard.add(cc)
    return hard, soft

OLD = set()
if os.path.exists('/memory/grid_old.json'):
    for k in json.load(open('/memory/grid_old.json')):
        a, b = k.split(','); OLD.add((int(a) // 6, int(b) // 6))
def is_frontier(c):
    if (c[0] // 6, c[1] // 6) in OLD: return False
    return state(c) == 'F' and any(state(nb) is None for nb in neighbors(c))

RELAX = [0]
def plan(start, hard, soft, goal_test, maxn=60000):
    """Dijkstra over known-free cells (unknown not traversable). Start allowed even if hard."""
    dist = {start: 0}; prev = {start: None}; pq = [(0, start)]; n = 0
    while pq and n < maxn:
        d, c = heapq.heappop(pq); n += 1
        if d > dist.get(c, 1e9): continue
        if c != start and goal_test(c):
            path = []
            while c is not None: path.append(c); c = prev[c]
            return path[::-1]
        for nb in neighbors(c):
            if state(nb) != 'F': continue
            if nb in hard and math.hypot(nb[0]-start[0], nb[1]-start[1]) > 5 and RELAX[0] < 2: continue
            w = 1 + (6 if nb in soft else 0) + (12 if nb in hard else 0)
            nd = d + w
            if nd < dist.get(nb, 1e9):
                dist[nb] = nd; prev[nb] = c; heapq.heappush(pq, (nd, nb))
    return None

def turn_to_h(pose, target, tol=4.0):
    t0 = time.time()
    while time.time() - t0 < 6:
        d = angdiff(target, pose.get()[2])
        if abs(d) <= tol:
            stop(); time.sleep(0.12)
            if abs(angdiff(target, pose.get()[2])) <= tol: return
            continue
        sp = max(8, min(30, abs(d) * 0.6))
        motors(sp, -sp) if d > 0 else motors(-sp, sp)
        time.sleep(0.04)
    stop()

def go_to_cell(pose, target, speed=50):
    tx = (target[0] + 0.5) * CELL; ty = (target[1] + 0.5) * CELL
    x, y, th = pose.get()
    if math.hypot(tx - x, ty - y) < CELL: return True
    bearing = math.degrees(math.atan2(ty - y, tx - x))
    if abs(angdiff(bearing, th)) > 8: turn_to_h(pose, bearing)
    t0 = time.time()
    while time.time() - t0 < 6:
        x, y, th = pose.get(); dist = math.hypot(tx - x, ty - y)
        if dist < 0.6 * CELL: break
        bearing = math.degrees(math.atan2(ty - y, tx - x)); err = angdiff(bearing, th)
        if abs(err) > 60 and dist > 2 * CELL: break
        r = ranges()
        if r:
            f = [v for v in (r[0], r[1], r[15]) if v > 0]
            if f and min(f) < 0.12:
                stop(); log('obstacle stop front=%.2f' % min(f))
                scan_into_grid(pose.get())
                px, py, pth = pose.get()
                for dd in (0.08, 0.12, 0.16):
                    c = cell(px + dd*math.cos(math.radians(pth)), py + dd*math.sin(math.radians(pth)))
                    score[c] = max(score.get(c, 0), 0) + 4
                motors(-40, -40); time.sleep(0.4); stop(); time.sleep(0.1)
                return False
        corr = max(-15, min(15, err * 0.5))
        motors(speed + corr, speed - corr); time.sleep(0.04)
    stop(); return True

def reactive_step(pose, tx, ty, steplen=0.3):
    x, y, th = pose.get(); want = math.degrees(math.atan2(ty - y, tx - x)); dist = math.hypot(tx - x, ty - y)
    r = ranges()
    if not r: return False
    best = None
    for i in range(16):
        ri = r[i] if r[i] > 0 else 0.25
        side = min(r[(i-1) % 16] if r[(i-1) % 16] > 0 else 0.25, r[(i+1) % 16] if r[(i+1) % 16] > 0 else 0.25)
        if ri < 0.3 or side < 0.15: continue
        b = th + 22.5 * i
        sc = abs(angdiff(b, want)) + (40 if ri < 0.5 else 0) - min(ri, 1.5) * 10
        if best is None or sc < best[0]: best = (sc, i, ri, b)
    if best is None:
        i = max(range(16), key=lambda k: r[k]); best = (0, i, r[i], th + 22.5 * i)
        motors(-40, -40); time.sleep(0.4); stop()
    _, i, ri, b = best
    turn_to_h(pose, b)
    goal_d = min(steplen, max(0.05, ri - 0.2), dist)
    e0 = enc(); t0 = time.time()
    while time.time() - t0 < 5:
        rr = ranges()
        if rr:
            f = [v for v in (rr[0], rr[1], rr[15]) if v > 0]
            if f and min(f) < 0.13: break
            l = rr[2] if rr[2] > 0 else 1; rgt = rr[14] if rr[14] > 0 else 1
            corr = 0
            if l < 0.14: corr = -12
            if rgt < 0.14: corr = 12
            motors(50 + corr, 50 - corr)
        e = enc()
        if e and e0 and ((e[0]-e0[0]) + (e[1]-e0[1])) / 2.0 / TPU >= goal_d: break
        time.sleep(0.04)
    stop(); scan_into_grid(pose.get())
    return True

def check_status():
    return read_line(3, 0.3)

def main():
    global POSE
    x0 = float(sys.argv[1]) if len(sys.argv) > 2 else 0.0
    y0 = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    POSE = pose = Pose(x0, y0)
    if os.path.exists('/bot/src/grid.json'):
        for k, v in json.load(open('/bot/src/grid.json')).items():
            a, b = k.split(','); score[(int(a), int(b))] = v
        log('loaded grid cells', len(score))
    if os.path.exists('/bot/src/d11.json'):
        d11log.extend(tuple(t) for t in json.load(open('/bot/src/d11.json')))
    time.sleep(0.3); log('START pose', pose.get())
    for k in range(4):
        scan_into_grid(pose.get()); turn_to_h(pose, pose.get()[2] + 90)
    scan_into_grid(pose.get()); save_map()
    fails = 0; tried = {}; step = 0; lastpos = (0, 0); stuck = 0
    while True:
        step += 1
        st = check_status(); x, y, th = pose.get()
        if math.hypot(x - lastpos[0], y - lastpos[1]) < 0.06: stuck += 1
        else: stuck = 0
        lastpos = (x, y)
        if stuck >= 4:
            r = ranges() or []
            if r:
                i = max(range(16), key=lambda k: r[k] if r[k] > 0 else -1)
                log('STUCK escape: turning to beam %d (range %.2f)' % (i, r[i]))
                turn_to_h(pose, th + 22.5 * i)
                t0 = time.time()
                while time.time() - t0 < 2.0:
                    rr = ranges()
                    if rr and min([v for v in (rr[0], rr[1], rr[15]) if v > 0] or [9]) < 0.15: break
                    motors(50, 50); time.sleep(0.05)
                stop(); stuck = 0; tried.clear()
                scan_into_grid(pose.get())
                continue
        if st and 'goal=0' not in st: log('STATUS CHANGED:', st, 'at', (x, y)); save_map()
        rr = scan_into_grid((x, y, th))
        pass
        if step % 4 == 0: save_map()
        if step % 6 == 0:
            write_line(8, 'Hello from robot at approx (%.1f,%.1f). I am mapping. Where are you? Reply please.' % (x, y))
        hard, soft = cost_maps(); start = cell(x, y)
        score[start] = min(score.get(start, 0), -1)
        mine = lambda c: c[0] * CELL > -2.0 and c[1] * CELL > 3.3
        path = plan(start, hard, soft, lambda c: is_frontier(c) and tried.get(c, 0) < 2 and mine(c) and
                    math.hypot(c[0]-start[0], c[1]-start[1]) >= 4)
        if not path:
            path = plan(start, hard, soft, lambda c: is_frontier(c) and tried.get(c, 0) < 2 and
                    math.hypot(c[0]-start[0], c[1]-start[1]) >= 4)
        if not path:
            fails += 1; log('no frontier path (fail %d); cells %d' % (fails, len(score))); save_map()
            # reactive move toward nearest frontier (euclidean) in my region, else any
            fr = [c for c in score if is_frontier(c) and tried.get(c, 0) < 3]
            pri = [c for c in fr if mine(c)] or fr
            if pri:
                tgt = min(pri, key=lambda c: math.hypot(c[0]-start[0], c[1]-start[1]) + (0 if math.hypot(c[0]-start[0], c[1]-start[1]) > 6 else 100))
                tried[tgt] = tried.get(tgt, 0) + 1
                log('reactive toward %s' % (tgt,))
                for _ in range(3): reactive_step(pose, (tgt[0]+0.5)*CELL, (tgt[1]+0.5)*CELL)
                continue
            if fails % 3 == 0: tried.clear()
            RELAX[0] = fails
            # robot is physically here: clear cells around it
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    c = (start[0]+dx, start[1]+dy)
                    if score.get(c, 0) >= 2: score[c] = -1
            if fails >= 4:
                # wipe the map region within 0.6 of robot (drift makes it stale) and rescan
                for c in [c for c in score if math.hypot(c[0]-start[0], c[1]-start[1]) <= 12]: del score[c]
                for k in range(4):
                    scan_into_grid(pose.get()); turn_to_h(pose, pose.get()[2] + 90)
                continue
            turn_to_h(pose, th + 90)
            if fails > 12: log('giving up'); break
            continue
        fails = 0; RELAX[0] = 0
        target = path[-1]; tried[target] = tried.get(target, 0) + 1
        log('step %d pose=(%.2f,%.2f,%.0f) target=%s pathlen=%d d11=%s %s' % (step, x, y, th, target, len(path), d11log[-1][2] if d11log else None, st))
        i = 0
        while i < len(path) - 1:
            j = min(len(path) - 1, i + 6)
            ok = go_to_cell(pose, path[j]); scan_into_grid(pose.get())
            if not ok:
                tx, ty = (path[-1][0]+0.5)*CELL, (path[-1][1]+0.5)*CELL
                for _ in range(2): reactive_step(pose, tx, ty)
                break
            i = j
            st = check_status()
            if st and 'goal=0' not in st: log('STATUS CHANGED:', st, 'at', pose.get()); save_map()
        save_map()

def goto_main(tx, ty):
    global POSE
    POSE = pose = Pose(float(sys.argv[1]), float(sys.argv[2]))
    for k, v in json.load(open('/bot/src/grid.json')).items():
        a, b = k.split(','); score[(int(a), int(b))] = v
    time.sleep(0.3); log('GOTO start', pose.get(), 'target', (tx, ty))
    tc = cell(tx, ty); t0 = time.time()
    while time.time() - t0 < 240:
        x, y, th = pose.get()
        if math.hypot(tx - x, ty - y) < 0.15: log('GOTO reached', pose.get()); break
        scan_into_grid((x, y, th)); save_map()
        hard, soft = cost_maps(); start = cell(x, y); score[start] = min(score.get(start, 0), -1)
        path = plan(start, hard, soft, lambda c: math.hypot(c[0]-tc[0], c[1]-tc[1]) <= 3)
        st = read_line(3, 0.3); v = read_line(11, 0.3)
        log('GOTO pose=(%.2f,%.2f) pathlen=%s d11=%s %s' % (x, y, len(path) if path else None, v, st))
        if not path:
            reactive_step(pose, tx, ty); continue
        i = 0
        while i < len(path) - 1:
            j = min(len(path) - 1, i + 6)
            ok = go_to_cell(pose, path[j]); scan_into_grid(pose.get())
            if not ok: reactive_step(pose, tx, ty); break
            i = j
    save_map(); log('GOTO end', pose.get())

if __name__ == '__main__':
    if len(sys.argv) > 5 and sys.argv[3] == 'goto':
        try: goto_main(float(sys.argv[4]), float(sys.argv[5]))
        except Exception: import traceback; log('EXC', traceback.format_exc())
        finally: stop(); log('EXIT')
        sys.exit(0)
    try: main()
    except Exception: import traceback; log('EXC', traceback.format_exc())
    finally:
        stop()
        try: save_map()
        except Exception: pass
        log('EXIT')
