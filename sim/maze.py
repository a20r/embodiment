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
import random
from collections import deque


class Maze:
    def __init__(self, seed, width, height, cell_size=0.5, braid=0.0,
                 family_index=0):
        self.base_seed = seed
        self.family_index = family_index
        self.seed = seed + 1000 * family_index
        self.width = width
        self.height = height
        self.cell_size = cell_size
        self.braid = braid
        # h_walls[cy][cx]: wall along y = cy*cs, under cell column cx.
        #   cy in [0, height]; boundary rows are cy=0 and cy=height.
        # v_walls[cx][cy]: wall along x = cx*cs, beside cell row cy.
        self.h_walls = [[True] * width for _ in range(height + 1)]
        self.v_walls = [[True] * height for _ in range(width + 1)]
        self._generate()
        self.start_cell = (0, 0)
        self.goal_cell = self._farthest_cell(self.start_cell)

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
        canon = repr((self.width, self.height,
                      [tuple(r) for r in self.h_walls],
                      [tuple(c) for c in self.v_walls])).encode()
        return hashlib.sha256(canon).hexdigest()[:16]

    def to_dict(self):
        return {
            "seed": self.base_seed,
            "family_index": self.family_index,
            "effective_seed": self.seed,
            "width": self.width,
            "height": self.height,
            "cell_size": self.cell_size,
            "braid": self.braid,
            "start_cell": list(self.start_cell),
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
