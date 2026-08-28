"""Procedural maze generation.

Grid maze, iterative depth-first spanning tree (perfect maze), optional
"braiding" that opens a fraction of dead-ends into loops.  Cells are indexed
(cx, cy) with the world origin at the south-west corner; cell (cx, cy) spans
x in [cx*cs, (cx+1)*cs), y in [cy*cs, (cy+1)*cs).

Walls live on grid lines and are returned as line segments in meters.
Generation is fully determined by (seed, width, height, braid), so a reset
with the same parameters reproduces the identical maze.  "Same seed family"
perturbations derive a new effective seed as seed + 1000 * family_index.
"""

import hashlib
import math
import random
from collections import deque


def _pt_seg_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 <= 0 else max(0.0, min(1.0, ((px - x1) * dx
                                               + (py - y1) * dy) / l2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


class Maze:
    def __init__(self, seed, width, height, cell_size=0.5, braid=0.0,
                 family_index=0, style="grid", curviness=1.0,
                 robot_radius=0.09, locked=False, duo=False,
                 goal_chamber=False):
        self.base_seed = seed
        self.family_index = family_index
        self.seed = seed + 1000 * family_index
        self.width = width
        self.height = height
        self.cell_size = cell_size
        self.braid = braid
        self.style = style
        self.curviness = curviness
        self.robot_radius = robot_radius
        self.has_exit = style == "organic"
        self.locked = locked and self.has_exit
        self.goal_chamber = goal_chamber and self.has_exit
        self.exit_wall = None
        self.door_segments = None   # closes the exit until unlocked
        self.key_pos = None         # world coords of the key
        self.key_cell = None
        self._organic_segments = None
        # h_walls[cy][cx]: wall along y = cy*cs, under cell column cx.
        #   cy in [0, height]; boundary rows are cy=0 and cy=height.
        # v_walls[cx][cy]: wall along x = cx*cs, beside cell row cy.
        self.h_walls = [[True] * width for _ in range(height + 1)]
        self.v_walls = [[True] * height for _ in range(width + 1)]
        self._generate()
        self.start_cell = (0, 0)
        if self.has_exit:
            # The goal is an opening in the outer boundary on the far
            # side: reaching it means escaping, which is unmistakable.
            self.goal_cell = self._farthest_boundary_cell(self.start_cell)
            self._open_exit()
        else:
            self.goal_cell = self._farthest_cell(self.start_cell)
        # Duo: a second spawn, deep in the map but away from both the
        # first spawn and the exit, so neither bot starts near the goal.
        self.spawn_b_cell = None
        if duo:
            d_start = self._bfs_dist(self.start_cell)
            d_goal = self._bfs_dist(self.goal_cell)
            self.spawn_b_cell = max(
                d_start,
                key=lambda c: (min(d_start.get(c, 0), d_goal.get(c, 0)),
                               d_start.get(c, 0)))
        if self.style == "organic":
            if self.locked:
                self._place_key()
            self._organicize()
            if self.goal_chamber:
                self._add_goal_chamber()

    # -- generation ---------------------------------------------------------

    def _neighbors(self, cx, cy):
        if cy + 1 < self.height:
            yield cx, cy + 1, ("h", cy + 1, cx)
        if cy - 1 >= 0:
            yield cx, cy - 1, ("h", cy, cx)
        if cx + 1 < self.width:
            yield cx + 1, cy, ("v", cx + 1, cy)
        if cx - 1 >= 0:
            yield cx - 1, cy, ("v", cx, cy)

    def _wall_open(self, wall):
        kind, a, b = wall
        return not (self.h_walls[a][b] if kind == "h" else self.v_walls[a][b])

    def _remove_wall(self, wall):
        kind, a, b = wall
        if kind == "h":
            self.h_walls[a][b] = False
        else:
            self.v_walls[a][b] = False

    def _generate(self):
        rng = random.Random(self.seed)
        visited = [[False] * self.height for _ in range(self.width)]
        stack = [(0, 0)]
        visited[0][0] = True
        while stack:
            cx, cy = stack[-1]
            options = [(nx, ny, w) for nx, ny, w in self._neighbors(cx, cy)
                       if not visited[nx][ny]]
            if not options:
                stack.pop()
                continue
            nx, ny, wall = rng.choice(options)
            self._remove_wall(wall)
            visited[nx][ny] = True
            stack.append((nx, ny))
        if self.braid > 0:
            self._braid(rng)

    def _cell_walls(self, cx, cy):
        return [("h", cy, cx), ("h", cy + 1, cx),
                ("v", cx, cy), ("v", cx + 1, cy)]

    def _is_boundary_wall(self, wall):
        kind, a, b = wall
        if kind == "h":
            return a == 0 or a == self.height
        return a == 0 or a == self.width

    def _braid(self, rng):
        for cx in range(self.width):
            for cy in range(self.height):
                closed = [w for w in self._cell_walls(cx, cy)
                          if not self._wall_open(w)]
                if len(closed) != 3:
                    continue  # not a dead end
                if rng.random() >= self.braid:
                    continue
                candidates = [w for w in closed
                              if not self._is_boundary_wall(w)]
                if candidates:
                    self._remove_wall(rng.choice(candidates))

    # -- queries ------------------------------------------------------------

    def open_neighbors(self, cx, cy):
        for nx, ny, wall in self._neighbors(cx, cy):
            if self._wall_open(wall):
                yield nx, ny

    def _farthest_cell(self, start):
        dist = {start: 0}
        q = deque([start])
        far, far_d = start, 0
        while q:
            c = q.popleft()
            for n in self.open_neighbors(*c):
                if n not in dist:
                    dist[n] = dist[c] + 1
                    if dist[n] > far_d:
                        far, far_d = n, dist[n]
                    q.append(n)
        return far

    def _bfs_dist(self, start):
        dist = {start: 0}
        q = deque([start])
        while q:
            c = q.popleft()
            for n in self.open_neighbors(*c):
                if n not in dist:
                    dist[n] = dist[c] + 1
                    q.append(n)
        return dist

    def _farthest_boundary_cell(self, start):
        dist = self._bfs_dist(start)
        boundary = [c for c in dist
                    if c[0] in (0, self.width - 1)
                    or c[1] in (0, self.height - 1)]
        return max(boundary, key=lambda c: dist[c])

    def _open_exit(self):
        """Remove one outer wall of the goal cell — the way out."""
        cx, cy = self.goal_cell
        options = []
        if cy == self.height - 1:
            options.append(("h", self.height, cx))
        if cy == 0:
            options.append(("h", 0, cx))
        if cx == self.width - 1:
            options.append(("v", self.width, cy))
        if cx == 0:
            options.append(("v", 0, cy))
        wall = options[0]
        self._remove_wall(wall)
        cs = self.cell_size
        kind, a, b = wall
        if kind == "h":
            self.exit_wall = (b * cs, a * cs, (b + 1) * cs, a * cs)
        else:
            self.exit_wall = (a * cs, b * cs, a * cs, (b + 1) * cs)

    # -- organic style ------------------------------------------------------

    def _lattice_walls(self):
        """Closed walls as lattice-node pairs ((i1,j1),(i2,j2))."""
        out = []
        for cy in range(self.height + 1):
            for cx in range(self.width):
                if self.h_walls[cy][cx]:
                    out.append(((cx, cy), (cx + 1, cy)))
        for cx in range(self.width + 1):
            for cy in range(self.height):
                if self.v_walls[cx][cy]:
                    out.append(((cx, cy), (cx, cy + 1)))
        return out

    def _organicize(self):
        """Replace straight lattice walls with wavy polylines: every
        lattice node is jittered (shared by all walls that meet there)
        and every wall bows with a smooth per-wall wave, so nothing is
        axis-aligned or straight.  Amplitudes keep the narrowest gap
        comfortably wider than the robot; if the seeded draw still
        pinches the start pose, the whole displacement field is damped
        and rebuilt (deterministically)."""
        cs = self.cell_size
        scale = max(0.0, min(1.0, self.curviness))
        for _attempt in range(6):
            rng = random.Random(self.seed * 7919 + 13)
            node_j = {}
            for i in range(self.width + 1):
                for j in range(self.height + 1):
                    a = rng.uniform(0, 2 * math.pi)
                    r = rng.uniform(0.02, 0.05) * cs / 0.5 * scale
                    node_j[(i, j)] = (r * math.cos(a), r * math.sin(a))
            segs = []
            for (n1, n2) in self._lattice_walls():
                j1, j2 = node_j[n1], node_j[n2]
                x1 = n1[0] * cs + j1[0]
                y1 = n1[1] * cs + j1[1]
                x2 = n2[0] * cs + j2[0]
                y2 = n2[1] * cs + j2[1]
                dx, dy = x2 - x1, y2 - y1
                L = math.hypot(dx, dy) or 1.0
                nx, ny = -dy / L, dx / L
                amp = rng.uniform(0.04, 0.07) * cs / 0.5 * scale
                phase = rng.uniform(0, 2 * math.pi)
                freq = rng.choice((1.0, 1.5, 2.0))
                k = 10
                pts = []
                for s in range(k + 1):
                    t = s / k
                    off = amp * math.sin(math.pi * t) \
                        * math.sin(2 * math.pi * freq * t + phase)
                    pts.append((x1 + dx * t + nx * off,
                                y1 + dy * t + ny * off))
                for p, q in zip(pts, pts[1:]):
                    segs.append((p[0], p[1], q[0], q[1]))
            # The door: one straight segment closing the exit gap
            # between the two jittered gap posts — the only flat wall
            # in the world.
            door = None
            if self.locked and self.exit_wall is not None:
                ex1, ey1, ex2, ey2 = self.exit_wall
                n1 = (round(ex1 / cs), round(ey1 / cs))
                n2 = (round(ex2 / cs), round(ey2 / cs))
                j1, j2 = node_j[n1], node_j[n2]
                door = [(ex1 + j1[0], ey1 + j1[1],
                         ex2 + j2[0], ey2 + j2[1])]
            sx, sy = self.cell_center(self.start_cell)
            probes = [(sx, sy)]
            if self.spawn_b_cell:
                probes.append(self.cell_center(self.spawn_b_cell))
            if self.key_pos:
                probes.append(self.key_pos)
            clear = min(_pt_seg_dist(px, py, *s)
                        for px, py in probes for s in segs)
            if clear >= self.robot_radius + 0.04:
                self._organic_segments = segs
                self.door_segments = door
                return
            scale *= 0.75
        self._organic_segments = segs  # heavily damped fallback
        self.door_segments = door

    def _add_goal_chamber(self):
        """Pen in the space beyond the exit: three straight walls form
        a small chamber attached to the outside of the opening.  An
        escaped robot stays at the goal (and can re-enter the maze);
        the gaps where the chamber meets the jittered boundary are far
        narrower than the robot, so there is no way out of the world.
        """
        x1, y1, x2, y2 = self.exit_wall
        m = 0.25   # widen beyond the opening posts
        depth = 0.7
        if abs(x1 - x2) < 1e-9:    # vertical wall: west or east edge
            out = -1.0 if x1 <= 1e-9 else 1.0
            lo, hi = min(y1, y2) - m, max(y1, y2) + m
            bx = x1 + out * depth
            segs = [(x1, lo, bx, lo), (bx, lo, bx, hi),
                    (bx, hi, x1, hi)]
        else:                       # horizontal wall: south or north
            out = -1.0 if y1 <= 1e-9 else 1.0
            lo, hi = min(x1, x2) - m, max(x1, x2) + m
            by = y1 + out * depth
            segs = [(lo, y1, lo, by), (lo, by, hi, by),
                    (hi, by, hi, y1)]
        self._chamber_segments = segs
        self._organic_segments.extend(segs)

    def _place_key(self):
        """The key sits in a dead-end far from the door (and from the
        start as a tie-break), so the door is usually found first."""
        d_goal = self._bfs_dist(self.goal_cell)
        d_start = self._bfs_dist(self.start_cell)
        options = [c for c in self.dead_ends()
                   if c not in (self.start_cell, self.goal_cell)]
        if not options:
            options = [c for c in d_goal
                       if c not in (self.start_cell, self.goal_cell)]
        self.key_cell = max(options,
                            key=lambda c: (min(d_goal.get(c, 0),
                                               d_start.get(c, 0)),
                                           d_goal.get(c, 0)
                                           + d_start.get(c, 0)))
        self.key_pos = self.cell_center(self.key_cell)

    def escaped(self, x, y):
        """True once (x, y) is outside the maze bounding box."""
        cs = self.cell_size
        return (x < 0 or y < 0
                or x > self.width * cs or y > self.height * cs)

    def solution_path(self):
        """BFS shortest path start -> goal as list of cells."""
        prev = {self.start_cell: None}
        q = deque([self.start_cell])
        while q:
            c = q.popleft()
            if c == self.goal_cell:
                break
            for n in self.open_neighbors(*c):
                if n not in prev:
                    prev[n] = c
                    q.append(n)
        path, c = [], self.goal_cell
        while c is not None:
            path.append(c)
            c = prev[c]
        return list(reversed(path))

    def dead_ends(self):
        out = []
        for cx in range(self.width):
            for cy in range(self.height):
                if len(list(self.open_neighbors(cx, cy))) == 1:
                    out.append((cx, cy))
        return out

    def cell_center(self, cell):
        cx, cy = cell
        return ((cx + 0.5) * self.cell_size, (cy + 0.5) * self.cell_size)

    def segments(self):
        """All wall segments [(x1, y1, x2, y2), ...] in meters."""
        if self._organic_segments is not None:
            return self._organic_segments
        cs = self.cell_size
        segs = []
        for cy in range(self.height + 1):
            for cx in range(self.width):
                if self.h_walls[cy][cx]:
                    segs.append((cx * cs, cy * cs, (cx + 1) * cs, cy * cs))
        for cx in range(self.width + 1):
            for cy in range(self.height):
                if self.v_walls[cx][cy]:
                    segs.append((cx * cs, cy * cs, cx * cs, (cy + 1) * cs))
        return segs

    def hash(self):
        canon = repr((self.width, self.height, self.cell_size, self.style,
                      self.locked, self.goal_chamber, self.key_cell,
                      [tuple(r) for r in self.h_walls],
                      [tuple(c) for c in self.v_walls],
                      [tuple(round(v, 6) for v in s)
                       for s in (self._organic_segments or [])])).encode()
        return hashlib.sha256(canon).hexdigest()[:16]

    def to_dict(self):
        return {
            "style": self.style,
            "has_exit": self.has_exit,
            "exit_wall": list(self.exit_wall) if self.exit_wall else None,
            "locked": self.locked,
            "door_segments": [list(s) for s in self.door_segments]
            if self.door_segments else None,
            "key_pos": list(self.key_pos) if self.key_pos else None,
            "key_cell": list(self.key_cell) if self.key_cell else None,
            "seed": self.base_seed,
            "family_index": self.family_index,
            "effective_seed": self.seed,
            "width": self.width,
            "height": self.height,
            "cell_size": self.cell_size,
            "braid": self.braid,
            "start_cell": list(self.start_cell),
            "spawn_b_cell": list(self.spawn_b_cell)
            if self.spawn_b_cell else None,
            "goal_cell": list(self.goal_cell),
            "segments": [list(s) for s in self.segments()],
            "dead_ends": [list(c) for c in self.dead_ends()],
            "solution_path": [list(c) for c in self.solution_path()],
            "hash": self.hash(),
        }

    def ascii(self):
        """Terse top-down render (row y=height-1 printed first)."""
        lines = []
        for cy in range(self.height - 1, -1, -1):
            top = ""
            mid = ""
            for cx in range(self.width):
                top += "+" + ("---" if self.h_walls[cy + 1][cx] else "   ")
                mid += ("|" if self.v_walls[cx][cy] else " ")
                mark = "   "
                if (cx, cy) == self.start_cell:
                    mark = " S "
                elif (cx, cy) == self.goal_cell:
                    mark = " G "
                mid += mark
            top += "+"
            mid += "|" if self.v_walls[self.width][cy] else " "
            lines.append(top)
            lines.append(mid)
        bottom = ""
        for cx in range(self.width):
            bottom += "+" + ("---" if self.h_walls[0][cx] else "   ")
        lines.append(bottom + "+")
        return "\n".join(lines)
