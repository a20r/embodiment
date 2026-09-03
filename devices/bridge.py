"""FIFO device bridge: exposes sim I/O as files under /dev/robot.

The sim daemon (host) creates a directory of named pipes which is
bind-mounted into the bot container at /dev/robot.  FIFOs give real
blocking read()/write() file semantics across the container boundary with
zero code inside the container (the sanctioned fallback to FUSE; see
DECISIONS.md).

Semantics, per device kind:

  Sensor device (read side, agent POV):
    Opening the file for reading yields one complete measurement frame
    (a single text line) and then EOF, so `cat` returns a snapshot.
    A reader that holds the file open receives a fresh frame every
    FRAME_INTERVAL seconds (a stream).  A "dropped read" (noise dial)
    is served as an immediate EOF with no data.

  Actuator device (write side, agent POV):
    Each text line written is one command; e.g. `echo 120 > motor_left`.
    Values are clamped to [-255, 255]; garbage lines are ignored (logged).
    The command persists until replaced.

  Reading an actuator or writing a sensor blocks (they are one-way
  devices); in-container tooling should use timeouts.

Naming: labels=on uses documented names; labels=off names the files
d0..dN with a deterministic seeded permutation.  The sensor_remap
perturbation re-binds which physical sensor feeds which file; motor_swap
swaps the two actuator channels.  The file->binding table is written to
the run dir (host side only) as device_map.json.
"""

import json
import os
import random
import threading

from sim.config import SENSOR_DEVICES, ACTUATOR_DEVICES, ALL_DEVICES
from sim.world import stable_seed

FRAME_INTERVAL = 0.025  # seconds between frames for a held-open reader


def compute_bindings(seed, labels_on, remap_index=0, motor_swapped=False,
                     extra_sensors=(), exclude_sensors=(),
                     sensors=None, actuators=None):
    """Return {filename: logical_device}.

    labels=off: filenames are d0..dN, assignment is a seeded permutation.
    remap_index>0: sensor bindings are additionally permuted (a "wiring
    change") in both label modes; motor_swapped crosses the two channels.
    Pass explicit `sensors`/`actuators` lists for non-default vehicle
    models; otherwise the diffdrive defaults apply (with extra_sensors
    appended and exclude_sensors removed).
    """
    if sensors is None:
        sensors = [s for s in SENSOR_DEVICES
                   if s not in exclude_sensors] + list(extra_sensors)
    base_sensors = list(sensors)
    sensors = base_sensors[:]
    # Walk the remap chain so each remap step observably differs from
    # BOTH the identity wiring and the wiring it replaces.
    for i in range(1, remap_index + 1):
        rng = random.Random(stable_seed(seed, "sensor_remap", i))
        prev = sensors[:]
        while True:
            shuffled = prev[:]
            rng.shuffle(shuffled)
            if shuffled != prev and shuffled != base_sensors:
                break
        sensors = shuffled
    canonical_act = list(actuators if actuators is not None
                         else ACTUATOR_DEVICES)
    actuators = canonical_act[:]
    if motor_swapped and len(actuators) == 2:
        actuators = [actuators[1], actuators[0]]

    logical = sensors + actuators  # physical device behind slot i
    n = len(logical)
    if labels_on:
        filenames = base_sensors + canonical_act
    else:
        rng = random.Random(stable_seed(seed, "labels_off", n))
        order = list(range(n))
        rng.shuffle(order)
        filenames = [f"d{order[i]}" for i in range(n)]
    return {filenames[i]: logical[i] for i in range(n)}


class DeviceBridge:
    def __init__(self, devfs_dir, world, bindings, log_fn=None,
                 actuators=None):
        self.dir = devfs_dir
        self.world = world
        self.bindings = bindings
        self.actuator_set = set(actuators if actuators is not None
                                else ACTUATOR_DEVICES)
        self.log = log_fn or (lambda rec: None)
        self.rng_drop = random.Random(
            stable_seed(world.maze.seed, world.episode_index,
                        getattr(world, "bot_id", ""), "drops"))
        self.running = True
        self.threads = []
        self.read_counts = {}   # filename -> served frames
        self.write_counts = {}
        self.drop_counts = {}

    def start(self):
        os.makedirs(self.dir, exist_ok=True)
        for name in os.listdir(self.dir):
            os.unlink(os.path.join(self.dir, name))
        for filename, logical in self.bindings.items():
            os.mkfifo(os.path.join(self.dir, filename))
            self._spawn(filename, logical)
        t = threading.Thread(target=self._watchdog_loop, daemon=True)
        t.start()
        self.threads.append(t)

    def _spawn(self, filename, logical):
        path = os.path.join(self.dir, filename)
        target = self._actuator_loop if logical in self.actuator_set \
            else self._sensor_loop
        t = threading.Thread(target=target,
                             args=(path, filename, logical), daemon=True)
        t.start()
        self.threads.append(t)

    def _watchdog_loop(self):
        """The bus re-enumerates: a device file the agent deleted comes
        back with a fresh serving thread (the old thread is parked on
        the orphaned inode; capped so an rm loop can't grow us
        unboundedly)."""
        import time
        heals = {}
        while self.running:
            time.sleep(0.5)
            for filename, logical in self.bindings.items():
                path = os.path.join(self.dir, filename)
                if os.path.exists(path):
                    continue
                heals[filename] = heals.get(filename, 0) + 1
                if heals[filename] > 20:
                    continue
                try:
                    os.mkfifo(path)
                except OSError:
                    continue
                self.log(dict(event="device_recreated", dev=filename,
                              physical=logical, t=self.world.tick))
                self._spawn(filename, logical)

    def stop(self):
        self.running = False
        # Unblock sensor threads stuck in open() by briefly opening the
        # read end; wake actuator threads (blocked in read) with a
        # newline they will ignore.
        for filename, logical in self.bindings.items():
            path = os.path.join(self.dir, filename)
            try:
                if logical in self.actuator_set:
                    fd = os.open(path, os.O_WRONLY | os.O_NONBLOCK)
                    os.write(fd, b"\n")
                else:
                    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                os.close(fd)
            except OSError:
                pass

    def _heal(self, path):
        """Backoff for transient open failures; if our file is gone the
        watchdog recreates it with a fresh thread, so this thread ends."""
        import time
        time.sleep(0.2)
        return os.path.exists(path)

    # -- frames -------------------------------------------------------------

    def _emit(self, logical):
        w = self.world
        if logical == "lidar":
            return w.lidar_frame()
        if logical == "heading":
            return w.heading_frame()
        if logical == "encoder_left":
            return w.encoder_frame(0)
        if logical == "encoder_right":
            return w.encoder_frame(1)
        if logical == "bump_front":
            return w.bump_frame(0)
        if logical == "bump_rear":
            return w.bump_frame(1)
        if logical == "status":
            return w.status_frame()
        if logical == "beacon":
            return w.beacon_frame()
        if logical == "speed":
            return w.speed_frame()
        if logical == "serial_rx":
            return w.serial_rx_frame()
        if logical == "peer_signal":
            return w.peer_signal_frame()
        if logical == "lidar3d":
            return w.lidar3d_frame()
        raise ValueError(logical)

    # Ground truth records the exact frame served, but a point cloud is
    # tens of kB per read; log a digest instead (count + hash), which
    # still proves what was served without bloating the record.
    DIGEST_OVER = 512

    def _frame_record(self, frame):
        if len(frame) <= self.DIGEST_OVER:
            return frame
        import hashlib
        return (f"<{len(frame)}B {frame.count(';') + 1}pts "
                f"sha1={hashlib.sha1(frame.encode()).hexdigest()[:12]}>")

    def _interval(self, logical):
        """Seconds between frames for a held-open reader."""
        if logical == "lidar3d":
            hz = float(self.world.lidar3d_cfg.get("stream_hz", 10) or 10)
            return 1.0 / hz
        return FRAME_INTERVAL

    def _sensor_loop(self, path, filename, logical):
        import time
        drop_p = self.world.noise["dropped_read_p"]
        while self.running:
            try:
                # Blocks until a reader opens the file.
                fd = os.open(path, os.O_WRONLY)
            except OSError:
                if self._heal(path):
                    continue
                return  # file gone; the watchdog spawns a successor
            if not self.running:
                os.close(fd)
                return
            try:
                if drop_p > 0 and self.rng_drop.random() < drop_p:
                    self.drop_counts[filename] = \
                        self.drop_counts.get(filename, 0) + 1
                    self.log(dict(event="read_dropped", dev=filename,
                                  physical=logical, t=self.world.tick))
                else:
                    # Frame and tick sampled atomically (RLock) so the
                    # logged t is the tick the values describe.
                    with self.world.lock:
                        frame = self._emit(logical)
                        tick = self.world.tick
                    os.write(fd, (frame + "\n").encode())
                    self.read_counts[filename] = \
                        self.read_counts.get(filename, 0) + 1
                    self.log(dict(event="read", dev=filename,
                                  physical=logical,
                                  value=self._frame_record(frame),
                                  t=tick))
            except (BrokenPipeError, OSError):
                pass
            finally:
                try:
                    os.close(fd)
                except OSError:
                    pass
            # Give one-shot readers (cat) time to see EOF before the next
            # frame; a held-open reader gets a ~40 Hz stream (slower for
            # the big point-cloud frames).
            time.sleep(self._interval(logical))

    def _actuator_loop(self, path, filename, logical):
        while self.running:
            try:
                # O_RDWR: our own write end keeps the FIFO alive, so a
                # writer closing never delivers EOF — there is no
                # close/reopen window in which a fresh writer's command
                # could be discarded.
                fd = os.open(path, os.O_RDWR)
                f = os.fdopen(fd, "r")
            except OSError:
                if self._heal(path):
                    continue
                return  # file gone; the watchdog spawns a successor
            with f:
                for line in f:
                    if not self.running:
                        return
                    line = line.strip()
                    if not line:
                        continue
                    if logical == "serial_tx":
                        # Transceiver: raw text lines, not integers.
                        # Delivery (range gate, byte cap, GT comms
                        # events) is the world's business.
                        self.world.send_serial(line)
                        self.write_counts[filename] = \
                            self.write_counts.get(filename, 0) + 1
                        continue
                    try:
                        val = int(float(line))
                    except (ValueError, OverflowError):
                        self.log(dict(event="write_invalid", dev=filename,
                                      physical=logical, raw=line[:100],
                                      t=self.world.tick))
                        continue
                    with self.world.lock:
                        self.world.set_actuator(logical, val)
                        tick = self.world.tick
                    self.write_counts[filename] = \
                        self.write_counts.get(filename, 0) + 1
                    self.log(dict(event="write", dev=filename,
                                  physical=logical, value=val,
                                  t=tick))

    def stats(self):
        return {
            "reads": dict(self.read_counts),
            "writes": dict(self.write_counts),
            "drops": dict(self.drop_counts),
        }


def write_device_map(run_dir, bindings, labels_on, remap_index,
                     motor_swapped):
    with open(os.path.join(run_dir, "device_map.json"), "w") as f:
        json.dump({
            "labels": "on" if labels_on else "off",
            "remap_index": remap_index,
            "motor_swapped": motor_swapped,
            "file_to_physical": bindings,
        }, f, indent=2, sort_keys=True)
