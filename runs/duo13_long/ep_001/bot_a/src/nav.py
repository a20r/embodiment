import math, time, json, os
from robot import Robot

BEAMS = 16
BEAM_W = 22.5
CELL = 0.15
ROBOT_R = 0.03  # inflation radius m (corridors ~0.3m! rely on reactive safety)

def wrap(a): return (a+180)%360-180

class Grid:
    def __init__(self):
        self.occ = {}   # (gx,gy) -> [hits, misses]
    def key(self, x, y):
        return (int(math.floor(x/CELL)), int(math.floor(y/CELL)))
    def mark_free(self, x0,y0,x1,y1):
        # walk ray, mark free
        dx,dy = x1-x0, y1-y0
        dist = math.hypot(dx,dy)
        n = max(1,int(dist/CELL)+1)
        for i in range(n):
            t = i/n
            k = self.key(x0+dx*t, y0+dy*t)
            c = self.occ.setdefault(k,[0,0]); c[1]+=1
    def mark_hit(self, x,y):
        k = self.key(x,y)
        c = self.occ.setdefault(k,[0,0]); c[0]+=1
    def state(self, k):
        h,m = self.occ.get(k,[0,0])
        if h+m < 2: return 'unk'
        if h >= 3 and h > 0.3*(h+m): return 'occ'
        return 'free'
    def is_free(self, k): return self.state(k)=='free'
    def is_occ(self, k): return self.state(k)=='occ'

class Nav:
    def __init__(self, log=print):
        self.r = Robot()
        self.log = log
        self.grid = Grid()
        self.x = 0.0; self.y = 0.0
        self.hdg = self.r.heading() or 0.0
        self.hdg0 = self.hdg
        self.stuck = 0
        self.goal_seen = False
    def pose_str(self):
        return f'({self.x:.2f},{self.y:.2f})h={self.hdg:.0f}'
    def update_pose(self):
        l,rr = self.r.enc()
        if l is None: return
        h = self.r.heading()
        if h is None: return
        dl = (l - self._le)/544.0 if hasattr(self,'_le') else 0
        dr = (rr - self._re)/544.0 if hasattr(self,'_re') else 0
        self._le, self._re = l, rr
        self.hdg = h
        if abs(dl)+abs(dr) > 0:
            dist = (dl+dr)/2
            rad = math.radians(self.hdg)
            # midpoint integration
            self.x += dist*math.sin(rad)
            self.y += dist*math.cos(rad)
    def sync_enc(self):
        l,rr = self.r.enc()
        self._le, self._re = l, rr
    def scan_update(self):
        # update grid with current lidar from current pose
        lid = self.r.lidar()
        if not lid: return
        for i, d in enumerate(lid):
            if d is None or d < 0: continue
            az = self.hdg + i*BEAM_W
            rad = math.radians(az)
            ex = self.x + d*math.sin(rad); ey = self.y + d*math.cos(rad)
            if 0.15 < d < 2.4:
                self.grid.mark_hit(ex, ey)
                self.grid.mark_free(self.x, self.y, self.x + (d-0.05)*math.sin(rad), self.y + (d-0.05)*math.cos(rad))
            else:
                self.grid.mark_free(self.x, self.y, self.x + 2.3*math.sin(rad), self.y + 2.3*math.cos(rad))
    def neighbors4(self, k):
        x,y = k
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            yield (x+dx, y+dy)
    def passable(self, k):
        if not self.grid.is_free(k): return False
        cx = (k[0]+0.5)*CELL; cy = (k[1]+0.5)*CELL
        need = (ROBOT_R + CELL)**2
        for (ox,oy),(h,m) in self.grid.occ.items():
            if h < 2: continue
            if h > 0.3*(h+m) or h >= 3:
                ox = (ox+0.5)*CELL; oy = (oy+0.5)*CELL
                if (ox-cx)**2 + (oy-cy)**2 < need: return False
        return True
    def find_frontier(self, start):
        from collections import deque
        seen = {start}
        q = deque([start])
        while q:
            k = q.popleft()
            for n in self.neighbors4(k):
                if n in seen: continue
                st = self.grid.state(n)
                if st=='unk':
                    # frontier if any diag neighbor unknown & path to it free
                    return k, n
                seen.add(n)
                if st=='free' and self.passable(n):
                    q.append(n)
        return None, None
    def plan(self, start, goal):
        from collections import deque
        if not self.passable(start):
            return None
        prev = {start: None}
        q = deque([start])
        while q:
            k = q.popleft()
            if k == goal: break
            for n in self.neighbors4(k):
                if n in prev: continue
                if self.passable(n):
                    prev[n] = k; q.append(n)
        if goal not in prev: return None
        path = []
        k = goal
        while k is not None:
            path.append(k); k = prev[k]
        path.reverse()
        return path
    def cell_center(self, k):
        return ((k[0]+0.5)*CELL, (k[1]+0.5)*CELL)
    def rotate_to(self, target, tol=6, timeout=15):
        t0 = time.time()
        while time.time()-t0 < timeout:
            h = self.r.heading()
            if h is None: time.sleep(0.05); continue
            err = wrap(target - h)
            if abs(err) <= tol: break
            v = max(10, min(40, abs(err)*1.2))
            if err > 0: self.r.wheels(v, -v)   # need CW
            else: self.r.wheels(-v, v)
            time.sleep(0.05)
        self.r.stop(); time.sleep(0.2)
    def goto(self, tx, ty, timeout=25):
        # drive toward world point with heading hold + safety
        t0 = time.time()
        while time.time()-t0 < timeout:
            dx = tx-self.x; dy = ty-self.y
            dist = math.hypot(dx,dy)
            if dist < 0.12: break
            target = math.degrees(math.atan2(dx,dy)) % 360
            self.rotate_to(target, tol=8, timeout=4)
            # drive straight until close or blocked
            self.sync_enc()
            t1 = time.time()
            while time.time()-t1 < 3.0:
                self.update_pose()
                dx = tx-self.x; dy = ty-self.y
                dist = math.hypot(dx,dy)
                if dist < 0.12: break
                h = self.r.heading()
                if h is None: continue
                err = wrap(target - h)
                base = 22
                corr = max(-8, min(8, -err*0.8))
                lid = self.r.lidar()
                if lid:
                    front = min(lid[15], lid[0], lid[1])
                    if front < 0.15:
                        self.r.stop(); self.log(f'BLOCKED front={front:.2f}'); return False
                    if front < 0.35: base = 10
                self.r.wheels(base+corr, base-corr)
                time.sleep(0.05)
            self.r.stop()
        self.r.stop()
        return True
    def follow_path(self, path):
        for k in path[1:]:
            tx,ty = self.cell_center(k)
            ok = self.goto(tx,ty, timeout=20)
            self.scan_update()
            if not ok: return False
        return True
    def full_scan(self, log_prefix=''):
        # rotate 360 in 24 steps, scanning
        results = []
        for i in range(24):
            self.update_pose()
            self.scan_update()
            h = self.r.heading()
            st = self.r.status()
            if st and (st[1] or st[2]):
                self.log(f'FLAG! goal={st[1]} here={st[2]} at {self.pose_str()}')
            self.rotate_to((self.hdg0 + (i+1)*15)%360, tol=4, timeout=8)
        self.update_pose()
        self.scan_update()
        return results
