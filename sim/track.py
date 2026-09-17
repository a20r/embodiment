"""Race-track scene: a closed circuit built from a centerline with
per-side widths (TUM racetrack-database CSV: x_m,y_m,w_tr_right_m,
w_tr_left_m, CC-BY-4.0), scaled down to robot size.

Duck-types the Maze surface World and the daemon use (segments, hash,
to_dict, start pose, the locked-exit attributes) so the rest of the sim
is untouched; World branches on `kind == "track"` only for lap timing.
Track edges are hard barriers: the car collides with them like walls.
"""

import csv
import hashlib
import math
import os
import zlib

TRACK_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "tracks")


class Track:
    kind = "track"
    has_exit = False
    locked = False
    key_pos = None
    key_cell = None
    door_segments = []
    spawn_b_cell = None
    cell_size = 1.0
    start_cell = (0, 0)
    goal_cell = (0, 0)

    def __init__(self, name="austin", scale=0.07, margin=1.0,
                 checkpoints=20, width_scale=1.0):
        self.name = name
        self.scale = float(scale)
        self.margin = float(margin)
        self.checkpoints = int(checkpoints)
        self.width_scale = float(width_scale)
        raw = self._load(name)
        # Orientation is the file's driving direction; the left/right
        # widths are relative to it.
        n = len(raw)
        tang = []
        for i in range(n):
            x0, y0 = raw[i - 1][0], raw[i - 1][1]
            x1, y1 = raw[(i + 1) % n][0], raw[(i + 1) % n][1]
            d = math.hypot(x1 - x0, y1 - y0) or 1.0
            tang.append(((x1 - x0) / d, (y1 - y0) / d))
        s, ws = self.scale, self.scale * self.width_scale
        left, right, center = [], [], []
        for (x, y, wr, wl), (tx, ty) in zip(raw, tang):
            nx, ny = -ty, tx                 # left of travel
            center.append((x * s, y * s))
            left.append((x * s + nx * wl * ws, y * s + ny * wl * ws))
            right.append((x * s - nx * wr * ws, y * s - ny * wr * ws))
        allx = [p[0] for p in left + right]
        ally = [p[1] for p in left + right]
        ox, oy = self.margin - min(allx), self.margin - min(ally)
        sh = lambda pts: [(round(x + ox, 4), round(y + oy, 4))
                          for x, y in pts]
        self.center, self.left, self.right = sh(center), sh(left), sh(right)
        self.tangent = tang
        self.width = round(max(allx) - min(allx) + 2 * self.margin, 3)
        self.height = round(max(ally) - min(ally) + 2 * self.margin, 3)
        self.cum_s = [0.0]
        for i in range(1, n):
            self.cum_s.append(self.cum_s[-1] + math.dist(
                self.center[i - 1], self.center[i]))
        self.length = self.cum_s[-1] + math.dist(self.center[-1],
                                                 self.center[0])
        self.track_width = [(wl + wr) * ws for _, _, wr, wl in raw]
        # Noise streams are seeded per scene like a maze seed.
        self.seed = zlib.crc32(f"{name}:{self.scale}".encode()) & 0xFFFF
        tx0, ty0 = self.tangent[0]
        self.start_pose = (self.center[0][0], self.center[0][1],
                           math.atan2(ty0, tx0) % (2 * math.pi))
        self.start_line = (self.left[0], self.right[0])
        self._segments = None

    @staticmethod
    def _load(name):
        path = os.path.join(TRACK_DIR, f"{name}.csv")
        rows = []
        with open(path) as f:
            for r in csv.reader(f):
                if not r or r[0].startswith("#"):
                    continue
                rows.append(tuple(float(v) for v in r[:4]))
        if len(rows) < 8:
            raise ValueError(f"track {name!r}: too few points")
        return rows

    # -- Maze surface ------------------------------------------------------

    def cell_center(self, cell):
        return self.center[0]

    def escaped(self, x, y):
        return False

    def segments(self):
        """Both edges as closed polylines: the barriers."""
        if self._segments is None:
            segs = []
            for loop in (self.left, self.right):
                for i in range(len(loop)):
                    x1, y1 = loop[i]
                    x2, y2 = loop[(i + 1) % len(loop)]
                    segs.append((x1, y1, x2, y2))
            self._segments = segs
        return self._segments

    def hash(self):
        canon = repr((self.name, self.scale, self.width_scale,
                      self.checkpoints, self.center[:8],
                      len(self.center))).encode()
        return hashlib.sha256(canon).hexdigest()[:16]

    def to_dict(self):
        return {
            "kind": "track",
            "name": self.name,
            "scale": self.scale,
            "width": self.width,
            "height": self.height,
            "cell_size": self.cell_size,
            "length_m": round(self.length, 3),
            "track_width_mean_m": round(
                sum(self.track_width) / len(self.track_width), 3),
            "checkpoints": self.checkpoints,
            "center": [list(p) for p in self.center],
            "left": [list(p) for p in self.left],
            "right": [list(p) for p in self.right],
            "start_line": [list(self.start_line[0]),
                           list(self.start_line[1])],
            "start_pose": [round(v, 4) for v in self.start_pose],
            "segments": [list(s) for s in self.segments()],
            "start_cell": list(self.start_cell),
            "goal_cell": list(self.goal_cell),
            "has_exit": False,
            "locked": False,
            "seed": self.seed,
            "hash": self.hash(),
        }

    # -- lap geometry ------------------------------------------------------

    def nearest_index(self, x, y, hint=None, window=40):
        """Index of the nearest centerline point; a hint makes the
        search local (the car cannot jump), None searches everything."""
        n = len(self.center)
        if hint is None:
            rng = range(n)
        else:
            rng = ((hint + k) % n for k in range(-window, window + 1))
        best, bi = None, 0
        for i in rng:
            cx, cy = self.center[i]
            d = (x - cx) ** 2 + (y - cy) ** 2
            if best is None or d < best:
                best, bi = d, i
        return bi

    def sector(self, idx):
        return idx * self.checkpoints // len(self.center)

    def side_of_start(self, x, y):
        """Signed distance along the start tangent from the start line
        (negative = behind the line, positive = past it)."""
        sx, sy = self.center[0]
        tx, ty = self.tangent[0]
        return (x - sx) * tx + (y - sy) * ty

    def crossed_start(self, x0, y0, x1, y1):
        """+1 if the move (x0,y0)->(x1,y1) crossed the start line
        forward, -1 backward, 0 otherwise.  The crossing must happen
        within the line's span (between the two edges)."""
        a, b = self.side_of_start(x0, y0), self.side_of_start(x1, y1)
        if (a < 0.0) == (b < 0.0) or a == b:
            return 0
        t = a / (a - b)
        px, py = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
        (lx, ly), (rx, ry) = self.start_line
        seg = math.hypot(rx - lx, ry - ly)
        if seg <= 0:
            return 0
        u = ((px - lx) * (rx - lx) + (py - ly) * (ry - ly)) / (seg * seg)
        if u < -0.05 or u > 1.05:
            return 0
        return 1 if b > a else -1
