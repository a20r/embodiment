#!/usr/bin/env python3
"""Autonomous brain: odometry + occupancy grid + frontier exploration.
Mode file /bot/mode: 'explore' | 'stop' | 'goto X Y' | 'spin'. Logs: /bot/brain.log, map: /bot/map.txt, pose: /bot/pose.txt"""
import rio, ctl, time, math, json, os, collections, sys

K = 0.00058           # units per encoder count
CELL = 0.05           # grid cell size
N = 400               # grid is N x N, origin at center
MAXR = 1.3            # treat readings >= this as "no hit" (free along ray only)
INFL = 2              # inflation cells for planning
LOG = open("/bot/brain.log", "a")
def log(*a):
    LOG.write(time.strftime("%H:%M:%S ") + " ".join(str(x) for x in a) + "\n"); LOG.flush()

# grid: 0 unknown, 1 free, 2 obstacle (counts-based)
occ = bytearray(N*N)   # 0 unknown,1 free,2 obst
hits = collections.Counter()
frees = collections.Counter()
def idx(x, y):
    i = int(round(x / CELL)) + N//2; j = int(round(y / CELL)) + N//2
    if 0 <= i < N and 0 <= j < N: return i, j
    return None

class Odo:
    def __init__(self):
        self.x = 0.0; self.y = 0.0; self.h = ctl.heading(5) or 0.0
        self.l, self.r = ctl.enc()
        self.extra = {}
    def update(self):
        l, r = ctl.enc(); h = ctl.heading(1)
        if h is not None:
            # light smoothing
            d = ctl.angdiff(h, self.h); self.h = (self.h + 0.8*d) % 360
        dl = l - self.l; dr = r - self.r; self.l, self.r = l, r
        # ignore encoder noise of +-1
        if abs(dl) <= 1: dl = 0
        if abs(dr) <= 1: dr = 0
        dist = (dl + dr) / 2.0 * K
        a = math.radians(self.h)
        self.x += dist * math.sin(a); self.y += dist * math.cos(a)
        return dist

odo = Odo()

def integrate_scan(s):
    """Mark free cells along beams and obstacle at hit."""
    for i, v in enumerate(s):
        if v is None: continue
        ang = math.radians(odo.h + i * 22.5)
        hit = v < MAXR
        rng = v if hit else MAXR
        steps = int(rng / (CELL*0.7))
        for k in range(1, steps):
            d = k * CELL * 0.7
            if d >= rng - CELL*0.6: break
            c = idx(odo.x + d*math.sin(ang), odo.y + d*math.cos(ang))
            if c:
                for di in (-1,0,1):
                    for dj in (-1,0,1):
                        cc = (c[0]+di, c[1]+dj)
                        if 0 <= cc[0] < N and 0 <= cc[1] < N and occ[cc[1]*N+cc[0]] != 2:
                            occ[cc[1]*N+cc[0]] = 1
        if hit:
            c = idx(odo.x + v*math.sin(ang), odo.y + v*math.cos(ang))
            if c:
                hits[c] += 1
                if hits[c] >= 2: occ[c[1]*N+c[0]] = 2
    # robot's own footprint is free
    c = idx(odo.x, odo.y)
    if c:
        for di in (-1,0,1):
            for dj in (-1,0,1):
                occ[(c[1]+dj)*N + c[0]+di] = 1

def sense(nscan=1):
    odo.update(); s = ctl.scan(nscan); integrate_scan(s)
    st = ctl.status()
    d0 = rio.read_port(0); d5 = rio.read_port(5); d11 = rio.read_port(11)
    odo.extra = dict(d0=d0, d5=d5, d11=d11, **st)
    with open("/bot/pose.txt", "w") as f:
        f.write(json.dumps(dict(x=round(odo.x,3), y=round(odo.y,3), h=round(odo.h,1), scan=s, **odo.extra)))
    return s, st

def dump_map():
    # find bounds of known cells
    xs = [i for j in range(N) for i in range(N) if occ[j*N+i]] 
    if not xs: return
    js = [j for j in range(N) for i in range(N) if occ[j*N+i]]
    i0, i1, j0, j1 = min(xs), max(xs), min(js), max(js)
    rc = idx(odo.x, odo.y)
    lines = []
    for j in range(j1, j0-1, -1):
        row = []
        for i in range(i0, i1+1):
            if rc and (i, j) == rc: row.append("R")
            else: row.append({0:" ",1:".",2:"#"}[occ[j*N+i]])
        lines.append("".join(row))
    with open("/bot/map.txt", "w") as f:
        f.write(f"# x from {(i0-N//2)*CELL:.2f} to {(i1-N//2)*CELL:.2f}, y from {(j0-N//2)*CELL:.2f} to {(j1-N//2)*CELL:.2f} (top row = max y). R=robot at ({odo.x:.2f},{odo.y:.2f}) h={odo.h:.0f}\n")
        f.write("\n".join(lines) + "\n")

def inflated_blocked():
    """Return set of blocked cells (obstacles inflated)."""
    blocked = set()
    for j in range(N):
        base = j*N
        for i in range(N):
            if occ[base+i] == 2:
                for di in range(-INFL, INFL+1):
                    for dj in range(-INFL, INFL+1):
                        blocked.add((i+di, j+dj))
    return blocked

def bfs(start, goal_test, blocked, allow_unknown=False):
    """BFS over free cells from start; returns path to first cell satisfying goal_test."""
    q = collections.deque([start]); prev = {start: None}
    while q:
        c = q.popleft()
        if goal_test(c) and c != start:
            path = []
            while c: path.append(c); c = prev[c]
            return path[::-1]
        for di, dj in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            n = (c[0]+di, c[1]+dj)
            if n in prev or not (0 <= n[0] < N and 0 <= n[1] < N): continue
            v = occ[n[1]*N+n[0]]
            if n in blocked: continue
            if v == 1 or (allow_unknown and v == 0): prev[n] = c; q.append(n)
    return None

def is_frontier(c, blocked):
    if occ[c[1]*N+c[0]] != 1 or c in blocked: return False
    for di, dj in ((1,0),(-1,0),(0,1),(0,-1)):
        n = (c[0]+di, c[1]+dj)
        if 0 <= n[0] < N and 0 <= n[1] < N and occ[n[1]*N+n[0]] == 0: return True
    return False

def follow(path, maxseg=0.5):
    """Follow a cell path: take a straight segment as far as the path stays roughly straight, then turn+drive."""
    if not path or len(path) < 2: return "short"
    # choose target: farthest cell along path within maxseg with clear line (just use path index)
    tx, ty = None, None
    for c in path[1:]:
        wx, wy = (c[0]-N//2)*CELL, (c[1]-N//2)*CELL
        d = math.hypot(wx-odo.x, wy-odo.y)
        if d > maxseg: break
        tx, ty = wx, wy
    if tx is None:
        c = path[1]; tx, ty = (c[0]-N//2)*CELL, (c[1]-N//2)*CELL
    dx, dy = tx-odo.x, ty-odo.y; d = math.hypot(dx, dy)
    tgt = math.degrees(math.atan2(dx, dy)) % 360
    ctl.turn_to(tgt, tol=5, maxspeed=40)
    sense()
    # drive in small chunks, updating odometry
    counts = d / K; done = 0.0; reason = "done"
    l0, r0 = ctl.enc()
    ctl.motors(45, 45)
    t0 = time.time()
    while done < counts and time.time()-t0 < 10:
        s, st = sense()
        l, r = ctl.enc(); done = ((l-l0)+(r-r0))/2.0
        front = [v for v in (s[0], s[1], s[15]) if v is not None]
        if front and min(front) < 0.14: reason = "obstacle"; break
        if st.get("goal") == "1": reason = "goal"; break
        # heading correction
        e = ctl.angdiff(tgt, odo.h)
        corr = max(-15, min(15, e*0.8))
        ctl.motors(45+corr, 45-corr)
    ctl.stop(); sense()
    return reason

def read_mode():
    try: return open("/bot/mode").read().strip()
    except: return "explore"

def main():
    log("brain start; pose reset to 0,0 h=", odo.h)
    for _ in range(3): sense(3)
    dump_map(); last_dump = time.time(); fail = 0
    while True:
        mode = read_mode()
        if mode == "stop":
            ctl.stop(); sense(); time.sleep(0.5)
            if time.time()-last_dump > 3: dump_map(); last_dump = time.time()
            continue
        if mode == "spin":
            for h in (0, 90, 180, 270):
                ctl.turn_to(h); sense(3)
            open("/bot/mode","w").write("stop"); dump_map(); continue
        s, st = sense(2)
        if st.get("goal") == "1": log("GOAL FLAG at", odo.x, odo.y); open("/bot/mode","w").write("stop"); continue
        blocked = inflated_blocked()
        start = idx(odo.x, odo.y)
        if start in blocked:
            blocked = set(c for c in blocked if occ[c[1]*N+c[0]] == 2)  # fall back to raw obstacles
            for di in (-1,0,1):
                for dj in (-1,0,1): blocked.discard((start[0]+di, start[1]+dj))
            log("start in inflation; using raw obstacles")
        if mode.startswith("goto"):
            _, gx, gy = mode.split(); g = idx(float(gx), float(gy))
            path = bfs(start, lambda c: abs(c[0]-g[0])+abs(c[1]-g[1]) <= 1, blocked, allow_unknown=True)
            if not path or len(path) < 3:
                log("goto reached/unreachable", path is not None); open("/bot/mode","w").write("stop"); continue
            r = follow(path); log("goto seg", r, "pose", round(odo.x,2), round(odo.y,2), round(odo.h))
        else:
            path = bfs(start, lambda c: is_frontier(c, blocked), blocked)
            if not path:
                fail += 1; log("no frontier found (fail", fail, ")"); 
                if fail > 3: open("/bot/mode","w").write("stop"); log("exploration complete")
                ctl.turn_to((odo.h + 90) % 360); continue
            fail = 0
            r = follow(path); log("explore seg", r, "pathlen", len(path), "pose", round(odo.x,2), round(odo.y,2), round(odo.h), "extra", odo.extra)
        if time.time()-last_dump > 3: dump_map(); last_dump = time.time()

if __name__ == "__main__":
    try: main()
    except Exception as e:
        import traceback; log("CRASH", traceback.format_exc()); ctl.stop()
