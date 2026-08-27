"""Ground-truth world: differential-drive kinematics, collision, sensors.

The World owns the true state.  The device bridge calls `*_frame()` methods
to emit sensor readings (noise applied per emission) and `set_motor()` to
apply actuator writes.  The tick loop calls `step()` at a fixed rate.

All randomness comes from named, seeded streams so an episode is reproducible
given the same command timeline.  A single lock guards state: `step()` and
frame emissions are serialized.
"""

import math
import random
import threading
import zlib

TWO_PI = 2.0 * math.pi


def stable_seed(*parts):
    """Deterministic across processes (unlike hash(), which is salted)."""
    return zlib.crc32(repr(parts).encode())


class SegIndex:
    """Uniform-grid spatial index over wall segments.  Organic mazes
    carry thousands of small segments; queries return only those whose
    bounding boxes fall near a point, keeping collision checks and
    raycasts local."""

    CELL = 0.5

    def __init__(self, segments):
        self.segments = segments
        self.buckets = {}
        c = self.CELL
        for idx, (x1, y1, x2, y2) in enumerate(segments):
            i0 = int(min(x1, x2) // c)
            i1 = int(max(x1, x2) // c)
            j0 = int(min(y1, y2) // c)
            j1 = int(max(y1, y2) // c)
            for i in range(i0, i1 + 1):
                for j in range(j0, j1 + 1):
                    self.buckets.setdefault((i, j), []).append(idx)

    def near(self, x, y, r):
        c = self.CELL
        seen = set()
        out = []
        for i in range(int((x - r) // c), int((x + r) // c) + 1):
            for j in range(int((y - r) // c), int((y + r) // c) + 1):
                for idx in self.buckets.get((i, j), ()):
                    if idx not in seen:
                        seen.add(idx)
                        out.append(self.segments[idx])
        return out


def _seg_dist(px, py, x1, y1, x2, y2):
    """Distance from point to segment, and the closest point."""
    dx, dy = x2 - x1, y2 - y1
    seg_len2 = dx * dx + dy * dy
    if seg_len2 <= 0.0:
        t = 0.0
    else:
        t = ((px - x1) * dx + (py - y1) * dy) / seg_len2
        t = max(0.0, min(1.0, t))
    cx, cy = x1 + t * dx, y1 + t * dy
    return math.hypot(px - cx, py - cy), cx, cy


def _ray_seg(px, py, ddx, ddy, x1, y1, x2, y2):
    """Ray (origin p, unit dir d) vs segment; returns hit distance or None."""
    ex, ey = x2 - x1, y2 - y1
    denom = ddx * ey - ddy * ex
    if abs(denom) < 1e-12:
        return None
    ox, oy = x1 - px, y1 - py
    t = (ox * ey - oy * ex) / denom       # distance along ray
    u = (ox * ddy - oy * ddx) / denom     # position along segment
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


class World:
    def __init__(self, cfg, maze, episode_index=0, log_fn=None):
        self.cfg = cfg
        self.maze = maze
        self.noise = cfg["noise"]
        self.robot_cfg = cfg["robot"]
        self.lidar_cfg = cfg["lidar"]
        self.dt = 1.0 / cfg["sim"]["tick_hz"]
        self.log = log_fn or (lambda rec: None)
        self.episode_index = episode_index
        # RLock: the device bridge samples frames + tick atomically while
        # frame methods take the lock themselves.
        self.lock = threading.RLock()
        self._segments = maze.segments()
        self._index = SegIndex(self._segments)
        self.reset()

    def reset(self):
        with self.lock:
            # Reseed noise streams so a reset world replays identically.
            base = (self.maze.seed, self.episode_index)
            self.rng_slip = random.Random(stable_seed(*base, "slip"))
            self.rng_lidar = random.Random(stable_seed(*base, "lidar"))
            self.rng_heading = random.Random(
                stable_seed(*base, "heading"))
            self.rng_encoder = random.Random(
                stable_seed(*base, "encoder"))
            self.tick = 0
            sx, sy = self.maze.cell_center(self.maze.start_cell)
            self.x, self.y, self.theta = sx, sy, 0.0
            self.v = 0.0
            self.w = 0.0
            self.cmd = [0, 0]          # last written pwm [left, right]
            self.cmd_eff = [0, 0]      # pwm currently in effect (post latency)
            self.pending = []          # (apply_tick, side, pwm)
            self.enc = [0.0, 0.0]      # cumulative encoder ticks (float)
            self.heading_drift = 0.0   # degrees, random walk
            self.colliding = False
            self.bump = [False, False]  # front, rear
            self.collision_count = 0
            self.goal_reached = False
            self.goal_tick = None
            self.trail = []            # [(tick, x, y)]
            self.events = []           # experimenter-facing event tail
            self._event(dict(event="reset",
                             pose=[self.x, self.y, self.theta]))

    def _event(self, rec):
        rec["t"] = self.tick
        self.log(rec)
        self.events.append(rec)
        if len(self.events) > 200:
            del self.events[:100]

    # -- actuation ----------------------------------------------------------

    def set_motor(self, side, pwm):
        """side: 0=left, 1=right.  Applied after actuation latency."""
        pwm = max(-255, min(255, int(pwm)))
        with self.lock:
            self.cmd[side] = pwm
            latency = int(self.noise["actuation_latency_ticks"])
            self.pending.append((self.tick + latency, side, pwm))

    # -- physics ------------------------------------------------------------

    def _collides(self, x, y):
        r = self.robot_cfg["radius"]
        worst = None
        for seg in self._index.near(x, y, r + 0.05):
            d, cx, cy = _seg_dist(x, y, *seg)
            if d < r:
                if worst is None or d < worst[0]:
                    worst = (d, cx, cy)
        return worst

    def step(self):
        with self.lock:
            self.tick += 1
            # Apply matured commands.
            if self.pending:
                still = []
                for apply_tick, side, pwm in self.pending:
                    if apply_tick <= self.tick:
                        self.cmd_eff[side] = pwm
                    else:
                        still.append((apply_tick, side, pwm))
                self.pending = still

            n = self.noise
            gains = (n["motor_gain_left"], n["motor_gain_right"])
            max_speed = self.robot_cfg["max_speed"]
            wheel_v = []   # motor-shaft surface speed (what encoders see)
            ground_v = []  # ground-contact speed after slip (what moves us)
            for i in (0, 1):
                v = self.cmd_eff[i] / 255.0 * max_speed * gains[i]
                slip = 0.0
                if n["slip_sigma"] > 0 or n["slip_mu"] > 0:
                    slip = self.rng_slip.gauss(n["slip_mu"], n["slip_sigma"])
                    slip = max(0.0, min(0.5, slip))
                wheel_v.append(v)
                ground_v.append(v * (1.0 - slip))

            wr = self.robot_cfg["wheel_radius"]
            tpr = self.robot_cfg["encoder_ticks_per_rev"]
            for i in (0, 1):
                self.enc[i] += wheel_v[i] * self.dt / (TWO_PI * wr) * tpr

            self.v = (ground_v[0] + ground_v[1]) / 2.0
            self.w = (ground_v[1] - ground_v[0]) / self.robot_cfg["wheelbase"]
            self.theta = (self.theta + self.w * self.dt) % TWO_PI
            nx = self.x + self.v * math.cos(self.theta) * self.dt
            ny = self.y + self.v * math.sin(self.theta) * self.dt

            hit = self._collides(nx, ny)
            was_colliding = self.colliding
            self.colliding = False
            contact = None
            if hit is None:
                self.x, self.y = nx, ny
            else:
                # Slide: try each axis alone; else stay put.
                if self._collides(nx, self.y) is None:
                    self.x = nx
                elif self._collides(self.x, ny) is None:
                    self.y = ny
                self.colliding = True
                contact = self._collides(self.x, self.y) or hit

            # Bump switches from contact bearing relative to heading.
            self.bump = [False, False]
            near = self._collides(self.x, self.y)
            probe = near or (contact if self.colliding else None)
            if self.colliding and probe:
                _, cx, cy = probe
                bearing = math.atan2(cy - self.y, cx - self.x) - self.theta
                bearing = (bearing + math.pi) % TWO_PI - math.pi
                if abs(bearing) <= math.radians(60):
                    self.bump[0] = True
                elif abs(bearing) >= math.radians(120):
                    self.bump[1] = True
            if self.colliding and not was_colliding:
                self.collision_count += 1
                self._event(dict(event="collision",
                                 pose=[round(self.x, 4), round(self.y, 4),
                                       round(self.theta, 4)]))

            if self.noise["heading_drift_deg"] > 0:
                self.heading_drift += self.rng_heading.gauss(
                    0.0, self.noise["heading_drift_deg"])

            if not self.goal_reached:
                if self.maze.has_exit:
                    reached = self.maze.escaped(self.x, self.y)
                else:
                    gx, gy = self.maze.cell_center(self.maze.goal_cell)
                    reached = math.hypot(self.x - gx, self.y - gy) \
                        < 0.35 * self.maze.cell_size
                if reached:
                    self.goal_reached = True
                    self.goal_tick = self.tick
                    self._event(dict(event="goal_reached"))

            if self.tick % 5 == 0:
                self.trail.append((self.tick, round(self.x, 4),
                                   round(self.y, 4)))
                if len(self.trail) > 20000:
                    del self.trail[:5000]

            self.log({
                "t": self.tick,
                "pose": [round(self.x, 5), round(self.y, 5),
                         round(self.theta, 5)],
                "v": round(self.v, 5),
                "w": round(self.w, 5),
                "cmd": list(self.cmd),
                "cmd_eff": list(self.cmd_eff),
                "enc": [round(self.enc[0], 2), round(self.enc[1], 2)],
                "col": int(self.colliding),
                "bump": [int(self.bump[0]), int(self.bump[1])],
                "goal": int(self.goal_reached),
            })

    # -- sensor emission (called by device bridge on read) ------------------

    def _cast_ray(self, angle, candidates=None):
        d = math.cos(angle), math.sin(angle)
        best = self.lidar_cfg["max_range"]
        for seg in (candidates if candidates is not None
                    else self._segments):
            t = _ray_seg(self.x, self.y, d[0], d[1], *seg)
            if t is not None and t < best:
                best = t
        return best

    def _ray_candidates(self):
        return self._index.near(self.x, self.y,
                                self.lidar_cfg["max_range"] + 0.1)

    def ray_angles(self):
        n = self.lidar_cfg["rays"]
        fov = math.radians(self.lidar_cfg["fov_deg"])
        if abs(fov - TWO_PI) < 1e-9:
            return [k * fov / n for k in range(n)]
        return [-fov / 2 + k * fov / (n - 1) for k in range(n)] \
            if n > 1 else [0.0]

    def lidar_true(self):
        """Noise-free ranges (experimenter/dashboard use only)."""
        with self.lock:
            cand = self._ray_candidates()
            return [self._cast_ray(self.theta + a, cand)
                    for a in self.ray_angles()]

    def lidar_frame(self):
        n = self.noise
        with self.lock:
            cand = self._ray_candidates()
            vals = []
            for a in self.ray_angles():
                if n["lidar_dropout_p"] > 0 and \
                        self.rng_lidar.random() < n["lidar_dropout_p"]:
                    vals.append(-1.0)
                    continue
                r = self._cast_ray(self.theta + a, cand)
                if n["lidar_sigma_m"] > 0:
                    r += self.rng_lidar.gauss(0.0, n["lidar_sigma_m"])
                vals.append(max(0.0, min(self.lidar_cfg["max_range"], r)))
            return ",".join(f"{v:.3f}" for v in vals)

    def heading_frame(self):
        n = self.noise
        with self.lock:
            deg = math.degrees(self.theta) + self.heading_drift
            if n["heading_sigma_deg"] > 0:
                deg += self.rng_heading.gauss(0.0, n["heading_sigma_deg"])
            # Round, then wrap: 359.97 must read "0.0", never "360.0".
            return f"{round(deg % 360.0, 1) % 360.0:.1f}"

    def encoder_frame(self, side):
        n = self.noise
        with self.lock:
            val = self.enc[side]
            j = int(n["encoder_jitter_ticks"])
            if j > 0:
                val += self.rng_encoder.randint(-j, j)
            return str(int(val))

    def bump_frame(self, which):
        with self.lock:
            return "1" if self.bump[which] else "0"

    def status_frame(self):
        with self.lock:
            return f"tick={self.tick} goal={int(self.goal_reached)}"

    def snapshot(self, since_tick=0):
        """Experimenter-facing state for the dashboard (ground truth)."""
        with self.lock:
            return {
                "tick": self.tick,
                "sim_time_s": round(self.tick * self.dt, 3),
                "pose": [self.x, self.y, self.theta],
                "cmd": list(self.cmd),
                "cmd_eff": list(self.cmd_eff),
                "enc": [int(self.enc[0]), int(self.enc[1])],
                "colliding": self.colliding,
                "collision_count": self.collision_count,
                "bump": [self.bump[0], self.bump[1]],
                "goal_reached": self.goal_reached,
                "goal_tick": self.goal_tick,
                "trail": [p for p in self.trail if p[0] > since_tick],
                "events": self.events[-50:],
            }
