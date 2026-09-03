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
from collections import deque

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
    def __init__(self, cfg, maze, episode_index=0, log_fn=None,
                 bot_id="", spawn_cell=None, spawn_theta=0.0):
        self.cfg = cfg
        self.maze = maze
        self.noise = cfg["noise"]
        self.robot_cfg = cfg["robot"]
        # Duo: a second World may share this maze.  The peer shows up on
        # lidar (a moving octagon), blocks motion (disc-disc), and is
        # reachable over a proximity-gated serial link.
        self.bot_id = bot_id
        self.spawn_cell = spawn_cell
        self.spawn_theta = spawn_theta
        self.peer = None
        duo = cfg.get("duo", {})
        self.duo_range = float(duo.get("comms_range", 0.8))
        self.duo_max_bytes = int(duo.get("max_line_bytes", 256))
        self.duo_queue = int(duo.get("queue_depth", 64))
        # together: no individual goal latch — the daemon fires
        # set_joint_goal() on both worlds when both are in the goal
        # region with entries inside the together window.
        self.joint_goal = duo.get("objective", "solo") == "together"
        self.peer_signal_scale = float(duo.get("peer_signal_scale", 2.0))
        rate = float(duo.get("tx_rate_hz", 0) or 0)
        self.tx_min_ticks = (cfg["sim"]["tick_hz"] / rate) if rate > 0 \
            else 0
        self.model = cfg["robot"].get("model", "diffdrive")
        self.car_cfg = cfg["robot"].get("car", {})
        self.actuators = ["accel", "steer"] if self.model == "car" \
            else ["motor_left", "motor_right"]
        self.lidar_cfg = cfg["lidar"]
        self.lidar3d_cfg = cfg.get("lidar3d") or {}
        self.lidar3d_on = bool(self.lidar3d_cfg.get("enabled"))
        self.dt = 1.0 / cfg["sim"]["tick_hz"]
        self.log = log_fn or (lambda rec: None)
        self.episode_index = episode_index
        # RLock: the device bridge samples frames + tick atomically while
        # frame methods take the lock themselves.
        self.lock = threading.RLock()
        self._segments = maze.segments()
        self._index = SegIndex(self._segments)
        # Locked-exit scenario: door segments are collidable/visible
        # while locked; the key is a small lidar-visible post (never
        # collidable — rolling over it picks it up).
        self._door_segments = list(maze.door_segments or [])
        self._key_segments = []
        if maze.locked and maze.key_pos:
            kx, ky = maze.key_pos
            r = 0.035
            pts = [(kx + r * math.cos(a), ky + r * math.sin(a))
                   for a in [k * TWO_PI / 8 for k in range(9)]]
            self._key_segments = [
                (p[0], p[1], q[0], q[1]) for p, q in zip(pts, pts[1:])]
            dx1, dy1, dx2, dy2 = self._door_segments[0]
            self._door_center = ((dx1 + dx2) / 2, (dy1 + dy2) / 2)
        self.reset()

    def set_peer(self, other):
        """Wire in the other robot (duo mode).  Peer state is read
        without taking the peer's lock: attribute reads are atomic under
        the GIL and a one-tick-stale pose is harmless, while cross-lock
        acquisition between two worlds could deadlock."""
        self.peer = other

    def reset(self):
        with self.lock:
            # Reseed noise streams so a reset world replays identically.
            # bot_id keeps the two duo robots' noise streams distinct.
            base = (self.maze.seed, self.episode_index, self.bot_id)
            self.rng_slip = random.Random(stable_seed(*base, "slip"))
            self.rng_lidar = random.Random(stable_seed(*base, "lidar"))
            self.rng_heading = random.Random(
                stable_seed(*base, "heading"))
            self.rng_encoder = random.Random(
                stable_seed(*base, "encoder"))
            self.rng_beacon = random.Random(
                stable_seed(*base, "beacon"))
            self.key_carried = False
            self.door_open = False
            # Gyro bias: fixed magnitude from the noise dial, sign
            # seeded per episode, applied every tick.
            mag = self.noise.get("heading_bias_deg_per_min", 0.0)
            sign = 1.0 if random.Random(
                stable_seed(*base, "gyro_bias")).random() < 0.5 else -1.0
            self._heading_bias_per_tick = sign * mag * self.dt / 60.0
            self.tick = 0
            sx, sy = self.maze.cell_center(
                self.spawn_cell or self.maze.start_cell)
            self.x, self.y, self.theta = sx, sy, self.spawn_theta % TWO_PI
            self.serial_rx = deque(maxlen=self.duo_queue)
            self.comms = {"tx": 0, "tx_delivered": 0, "rx_read": 0,
                          "tx_rate_dropped": 0}
            self._last_tx_tick = None
            self.v = 0.0
            self.w = 0.0
            self.phi = 0.0             # car: current steering angle, rad
            self.cmd = {a: 0 for a in self.actuators}   # last written
            self.cmd_eff = {a: 0 for a in self.actuators}  # post latency
            self.pending = []          # (apply_tick, logical, value)
            self.enc = [0.0, 0.0]      # cumulative encoder ticks (float)
            self.heading_drift = 0.0   # degrees, random walk
            self.colliding = False
            self.bump = [False, False]  # front, rear
            self.collision_count = 0
            self.goal_reached = False
            self.goal_tick = None
            self.region_entry = None   # tick of current goal-region stay
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

    def set_actuator(self, logical, value):
        """Command one actuator channel; applied after latency."""
        value = max(-255, min(255, int(value)))
        with self.lock:
            if logical not in self.cmd:
                return
            self.cmd[logical] = value
            latency = int(self.noise["actuation_latency_ticks"])
            self.pending.append((self.tick + latency, logical, value))

    def set_motor(self, side, pwm):
        """Back-compat wrapper: side 0=left, 1=right (diffdrive)."""
        self.set_actuator(self.actuators[side], pwm)

    # -- physics ------------------------------------------------------------

    def _solid_extra(self):
        """Segments that are solid right now beyond the static walls."""
        if self._door_segments and not self.door_open:
            return self._door_segments
        return ()

    def _collides(self, x, y):
        r = self.robot_cfg["radius"]
        worst = None
        candidates = self._index.near(x, y, r + 0.05)
        extra = self._solid_extra()
        for seg in (candidates if not extra
                    else list(candidates) + list(extra)):
            d, cx, cy = _seg_dist(x, y, *seg)
            if d < r:
                if worst is None or d < worst[0]:
                    worst = (d, cx, cy)
        if self.peer is not None:
            px, py = self.peer.x, self.peer.y
            pr = self.peer.robot_cfg["radius"]
            dc = math.hypot(x - px, y - py)
            d = dc - pr   # our center to the peer's surface
            if d < r:
                if dc > 1e-9:
                    cx = px + pr * (x - px) / dc
                    cy = py + pr * (y - py) / dc
                else:
                    cx, cy = px, py
                if worst is None or d < worst[0]:
                    worst = (d, cx, cy)
        return worst

    def step(self):
        with self.lock:
            self.tick += 1
            # Apply matured commands.
            if self.pending:
                still = []
                for apply_tick, logical, value in self.pending:
                    if apply_tick <= self.tick:
                        self.cmd_eff[logical] = value
                    else:
                        still.append((apply_tick, logical, value))
                self.pending = still

            n = self.noise
            if self.model == "car":
                c = self.car_cfg
                slip = 0.0
                if n["slip_sigma"] > 0 or n["slip_mu"] > 0:
                    slip = self.rng_slip.gauss(n["slip_mu"],
                                               n["slip_sigma"])
                    slip = max(0.0, min(0.5, slip))
                a = self.cmd_eff["accel"] / 255.0 \
                    * c.get("accel_max", 0.4) * (1.0 - slip)
                self.v += (a - c.get("drag", 0.35) * self.v) * self.dt
                self.v = max(-c.get("v_rev_max", 0.15),
                             min(c.get("v_max", 0.5), self.v))
                phi_target = self.cmd_eff["steer"] / 255.0 \
                    * math.radians(c.get("steer_max_deg", 35.0))
                rate = math.radians(c.get("steer_rate_deg_s", 120.0)) \
                    * self.dt
                dphi = max(-rate, min(rate, phi_target - self.phi))
                self.phi += dphi
                L = c.get("wheelbase", 0.12)
                self.w = self.v / L * math.tan(self.phi)
                # keep the encoder accumulators moving for GT continuity
                wr = self.robot_cfg["wheel_radius"]
                tpr = self.robot_cfg["encoder_ticks_per_rev"]
                self.enc[0] += self.v * self.dt / (TWO_PI * wr) * tpr
                self.enc[1] = self.enc[0]
            else:
                gains = (n["motor_gain_left"], n["motor_gain_right"])
                max_speed = self.robot_cfg["max_speed"]
                wheel_v = []   # motor-shaft speed (what encoders see)
                ground_v = []  # after slip (what moves us)
                for logical, gain in zip(self.actuators, gains):
                    v = self.cmd_eff[logical] / 255.0 * max_speed * gain
                    slip = 0.0
                    if n["slip_sigma"] > 0 or n["slip_mu"] > 0:
                        slip = self.rng_slip.gauss(n["slip_mu"],
                                                   n["slip_sigma"])
                        slip = max(0.0, min(0.5, slip))
                    wheel_v.append(v)
                    ground_v.append(v * (1.0 - slip))
                wr = self.robot_cfg["wheel_radius"]
                tpr = self.robot_cfg["encoder_ticks_per_rev"]
                for i in (0, 1):
                    self.enc[i] += wheel_v[i] * self.dt \
                        / (TWO_PI * wr) * tpr
                self.v = (ground_v[0] + ground_v[1]) / 2.0
                self.w = (ground_v[1] - ground_v[0]) \
                    / self.robot_cfg["wheelbase"]

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
                slid = False
                if self._collides(nx, self.y) is None:
                    self.x = nx
                    slid = True
                elif self._collides(self.x, ny) is None:
                    self.y = ny
                    slid = True
                self.colliding = True
                contact = self._collides(self.x, self.y) or hit
                if self.model == "car":
                    # Momentum dies against a wall: a full block stops
                    # the car; a scrape scrubs speed.
                    self.v = self.v * 0.5 if slid else 0.0

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
            self.heading_drift += self._heading_bias_per_tick

            if self.maze.locked:
                if not self.key_carried and self.maze.key_pos:
                    kx, ky = self.maze.key_pos
                    if math.hypot(self.x - kx, self.y - ky) < 0.12:
                        self.key_carried = True
                        self._event(dict(event="key_pickup",
                                         pose=[round(self.x, 4),
                                               round(self.y, 4)]))
                if self.key_carried and not self.door_open:
                    dcx, dcy = self._door_center
                    if math.hypot(self.x - dcx, self.y - dcy) < 0.30:
                        self.door_open = True
                        self._event(dict(event="door_unlocked",
                                         pose=[round(self.x, 4),
                                               round(self.y, 4)]))

            if not self.goal_reached:
                if self.maze.has_exit:
                    reached = self.maze.escaped(self.x, self.y)
                else:
                    gx, gy = self.maze.cell_center(self.maze.goal_cell)
                    reached = math.hypot(self.x - gx, self.y - gy) \
                        < 0.35 * self.maze.cell_size
                if self.joint_goal:
                    # No solo completion: only track region occupancy;
                    # the daemon fires the joint latch on both worlds.
                    if reached and self.region_entry is None:
                        self.region_entry = self.tick
                        self._event(dict(event="region_enter"))
                    elif not reached and self.region_entry is not None:
                        self._event(dict(event="region_exit",
                                         entered=self.region_entry))
                        self.region_entry = None
                elif reached:
                    self.goal_reached = True
                    self.goal_tick = self.tick
                    # Power down: an escaped robot must not keep driving
                    # on its last command while its peer plays on.
                    for a in self.actuators:
                        self.cmd[a] = 0
                        self.cmd_eff[a] = 0
                    self.pending = []
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
                **({"phi": round(math.degrees(self.phi), 2)}
                   if self.model == "car" else {}),
                "cmd": dict(self.cmd),
                "cmd_eff": dict(self.cmd_eff),
                "enc": [round(self.enc[0], 2), round(self.enc[1], 2)],
                "col": int(self.colliding),
                "bump": [int(self.bump[0]), int(self.bump[1])],
                "goal": int(self.goal_reached),
                **({"key": int(self.key_carried),
                    "door": int(self.door_open)}
                   if self.maze.locked else {}),
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
        cand = list(self._index.near(self.x, self.y,
                                     self.lidar_cfg["max_range"] + 0.1))
        cand.extend(self._solid_extra())
        if self._key_segments and not self.key_carried:
            cand.extend(self._key_segments)
        if self.peer is not None:
            # The other robot is a real obstacle: a small moving octagon
            # on the scan, indistinguishable in kind from any other
            # surface until the agent notices it moves.
            px, py = self.peer.x, self.peer.y
            pr = self.peer.robot_cfg["radius"]
            pts = [(px + pr * math.cos(a), py + pr * math.sin(a))
                   for a in [k * TWO_PI / 8 for k in range(9)]]
            cand.extend((p[0], p[1], q[0], q[1])
                        for p, q in zip(pts, pts[1:]))
        return cand

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

    # -- 3D lidar -----------------------------------------------------------
    # The world is still 2D-kinematic; the point cloud comes from lifting
    # the 2D cast: every solid gets a height (walls, the peer, the key
    # post) and the floor is z=0.  A ring at elevation e reaching a face
    # at horizontal distance d meets it at z = h_s + d*tan(e); it returns
    # if 0 <= z <= face height, passes over it otherwise, and a downward
    # ring that reaches z=0 first returns the floor.

    def _ring_elevations(self):
        c = self.lidar3d_cfg
        n = int(c["rings"])
        vfov = math.radians(c["vfov_deg"])
        if n == 1:
            return [0.0]
        return [-vfov / 2 + k * vfov / (n - 1) for k in range(n)]

    def _azimuths3d(self):
        n = int(self.lidar3d_cfg["azimuths"])
        return [k * TWO_PI / n for k in range(n)]

    def _height_groups(self):
        """[(segments, height)] per solid class, built once per frame."""
        c = self.lidar3d_cfg
        walls = list(self._index.near(self.x, self.y, c["max_range"] + 0.1))
        walls.extend(self._solid_extra())
        groups = [(walls, c["wall_height"])]
        if self._key_segments and not self.key_carried:
            groups.append((self._key_segments, c["post_height"]))
        if self.peer is not None:
            px, py = self.peer.x, self.peer.y
            pr = self.peer.robot_cfg["radius"]
            pts = [(px + pr * math.cos(a), py + pr * math.sin(a))
                   for a in [k * TWO_PI / 8 for k in range(9)]]
            groups.append(([(p[0], p[1], q[0], q[1])
                            for p, q in zip(pts, pts[1:])],
                           c["robot_height"]))
        return groups

    def _ring_hits(self, angle, groups):
        """Range per ring along one azimuth; None = no return."""
        c = self.lidar3d_cfg
        mr = c["max_range"]
        hs = c["sensor_height"]
        dx, dy = math.cos(angle), math.sin(angle)
        faces = []
        for segs, h in groups:
            best = None
            for seg in segs:
                t = _ray_seg(self.x, self.y, dx, dy, *seg)
                if t is not None and t < mr and (best is None or t < best):
                    best = t
            if best is not None:
                faces.append((best, h))
        faces.sort()
        out = []
        for e in self._ring_elevations():
            ce, se = math.cos(e), math.sin(e)
            tan_e = se / ce
            d_floor = -hs / tan_e if se < -1e-9 else None
            r = None
            for d, h in faces:
                if d_floor is not None and d_floor < d:
                    break               # the floor comes first
                if 0.0 <= hs + d * tan_e <= h:
                    r = d / ce
                    break               # else: passes over this face
            if r is None and d_floor is not None:
                r = d_floor / ce
            out.append(r if r is not None and r <= mr else None)
        return out

    def lidar3d_points(self, noisy):
        """Sensor-frame points (x forward, y left, z up; origin at the
        sensor) for one frame.  Dropped points are omitted, as a real
        unit omits no-returns."""
        n = self.noise
        sigma = n.get("lidar3d_sigma_m", 0.0)
        drop = n.get("lidar3d_dropout_p", 0.0)
        pts = []
        with self.lock:
            groups = self._height_groups()
            elevs = self._ring_elevations()
            for a_rel in self._azimuths3d():
                hits = self._ring_hits(self.theta + a_rel, groups)
                ca, sa = math.cos(a_rel), math.sin(a_rel)
                for e, r in zip(elevs, hits):
                    if r is None:
                        continue
                    if noisy:
                        if drop > 0 and self.rng_lidar.random() < drop:
                            continue
                        if sigma > 0:
                            r = max(0.0, r + self.rng_lidar.gauss(0.0, sigma))
                    ce = math.cos(e)
                    pts.append((r * ce * ca, r * ce * sa, r * math.sin(e)))
        return pts

    def lidar3d_frame(self):
        return ";".join(f"{x:.3f},{y:.3f},{z:.3f}"
                        for x, y, z in self.lidar3d_points(noisy=True))

    def lidar3d_true(self):
        """Noise-free cloud in world coordinates (dashboard use only)."""
        with self.lock:
            ct, st = math.cos(self.theta), math.sin(self.theta)
            hs = self.lidar3d_cfg["sensor_height"]
            return [[round(self.x + x * ct - y * st, 3),
                     round(self.y + x * st + y * ct, 3),
                     round(hs + z, 3)]
                    for x, y, z in self.lidar3d_points(noisy=False)]

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

    def speed_frame(self):
        """Signed ground speed, m/s (the car's speedometer)."""
        n = self.noise
        with self.lock:
            v = self.v
            sigma = n.get("speed_sigma_ms", 0.0)
            if sigma > 0:
                v += self.rng_encoder.gauss(0.0, sigma)
            return f"{v:.3f}"

    def beacon_frame(self):
        """Signal-strength receiver for the key's transmitter.  Rises
        toward 1.0 as the robot nears the key (through walls); reads a
        saturated 9.999 once the key is carried."""
        n = self.noise
        with self.lock:
            if not self.maze.locked or not self.maze.key_pos:
                return "0.000"
            if self.key_carried:
                return "9.999"
            kx, ky = self.maze.key_pos
            d = math.hypot(self.x - kx, self.y - ky)
            s = 1.0 / (1.0 + (d / 0.8) ** 2)
            sigma = n.get("beacon_sigma", 0.0)
            if sigma > 0:
                s += self.rng_beacon.gauss(0.0, sigma)
            return f"{max(0.0, min(1.0, s)):.3f}"

    # -- serial link (duo) ---------------------------------------------------

    def send_serial(self, raw):
        """One line written to the TX port.  Delivered into the peer's
        RX queue only if the peer is currently within comms_range;
        otherwise it simply vanishes (radio semantics: no error, no
        buffering).  Lock-free toward the peer — deque.append is atomic
        and we never take the peer's lock (see set_peer)."""
        line = raw[:self.duo_max_bytes]
        # Duty cycle: excess lines vanish before the range gate, with
        # no error back to the writer.  Logged in aggregate — a spam
        # loop must not flood the ground-truth record.
        if self.tx_min_ticks:
            now = self.tick
            if self._last_tx_tick is not None and \
                    now - self._last_tx_tick < self.tx_min_ticks:
                self.comms["tx_rate_dropped"] += 1
                if self.comms["tx_rate_dropped"] % 500 == 1:
                    self._event(dict(
                        event="comms_rate_drop",
                        total=self.comms["tx_rate_dropped"]))
                return
            self._last_tx_tick = now
        peer = self.peer
        delivered = False
        dist = None
        if peer is not None:
            dist = math.hypot(self.x - peer.x, self.y - peer.y)
            if dist <= self.duo_range:
                peer.serial_rx.append(line)
                peer._event(dict(event="comms_rx", frm=self.bot_id,
                                 line=line))
                delivered = True
        self.comms["tx"] += 1
        if delivered:
            self.comms["tx_delivered"] += 1
        self._event(dict(event="comms_tx", line=line,
                         delivered=delivered,
                         dist=None if dist is None else round(dist, 3)))

    def set_joint_goal(self):
        """Both bots arrived together — latch the goal on this world.
        Called by the daemon on both worlds in the same tick."""
        with self.lock:
            if self.goal_reached:
                return
            self.goal_reached = True
            self.goal_tick = self.tick
            for a in self.actuators:
                self.cmd[a] = 0
                self.cmd_eff[a] = 0
            self.pending = []
            self._event(dict(event="goal_reached", joint=True))

    def peer_signal_frame(self):
        """Signal strength to the peer (yelling in a maze): rises as
        the other robot nears, passes through walls, long tail."""
        n = self.noise
        with self.lock:
            if self.peer is None:
                return "0.000"
            d = math.hypot(self.x - self.peer.x, self.y - self.peer.y)
            s = 1.0 / (1.0 + (d / self.peer_signal_scale) ** 2)
            sigma = n.get("peer_signal_sigma", 0.0)
            if sigma > 0:
                s += self.rng_beacon.gauss(0.0, sigma)
            return f"{max(0.0, min(1.0, s)):.3f}"

    def serial_rx_frame(self):
        """Oldest pending received line; an empty line means nothing
        waiting.  popleft is atomic, so the peer may append mid-read."""
        try:
            line = self.serial_rx.popleft()
        except IndexError:
            return ""
        self.comms["rx_read"] += 1
        return line

    def status_frame(self):
        with self.lock:
            line = f"tick={self.tick} goal={int(self.goal_reached)}"
            if self.joint_goal:
                # Per-bot arrival flag: the joint goal= cannot fire for
                # a solo arriver, so without this a bot standing in the
                # goal region has no instrument that says so (the duo5
                # GOALFOUND pathology).
                line += f" here={int(self.region_entry is not None)}"
            if self.maze.locked and self._door_segments:
                dcx, dcy = self._door_center
                if math.hypot(self.x - dcx, self.y - dcy) < 0.5:
                    line += " door=" + ("open" if self.door_open
                                        else "locked")
            return line

    def snapshot(self, since_tick=0):
        """Experimenter-facing state for the dashboard (ground truth)."""
        with self.lock:
            return {
                "tick": self.tick,
                "sim_time_s": round(self.tick * self.dt, 3),
                "bot_id": self.bot_id,
                "comms": dict(self.comms),
                "rx_pending": len(self.serial_rx),
                "pose": [self.x, self.y, self.theta],
                "cmd": list(self.cmd.values()),
                "cmd_eff": list(self.cmd_eff.values()),
                "v": round(self.v, 3),
                "phi_deg": round(math.degrees(self.phi), 1),
                "enc": [int(self.enc[0]), int(self.enc[1])],
                "colliding": self.colliding,
                "collision_count": self.collision_count,
                "bump": [self.bump[0], self.bump[1]],
                "goal_reached": self.goal_reached,
                "goal_tick": self.goal_tick,
                "in_region": self.region_entry is not None,
                "key_carried": self.key_carried,
                "door_open": self.door_open,
                "trail": [p for p in self.trail if p[0] > since_tick],
                "events": self.events[-50:],
            }
