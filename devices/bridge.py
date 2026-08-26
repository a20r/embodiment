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


def compute_bindings(seed, labels_on, remap_index=0, motor_swapped=False):
    """Return {filename: logical_device}.

    labels=off: filenames are d0..dN, assignment is a seeded permutation.
    remap_index>0: sensor bindings are additionally permuted (a "wiring
    change") in both label modes; motor_swapped crosses the two channels.
    """
    sensors = list(SENSOR_DEVICES)
    if remap_index > 0:
        rng = random.Random(stable_seed(seed, "sensor_remap", remap_index))
        while True:
            shuffled = sensors[:]
            rng.shuffle(shuffled)
            if shuffled != sensors:
                break
        sensors = shuffled
    actuators = list(ACTUATOR_DEVICES)
    if motor_swapped:
        actuators = [actuators[1], actuators[0]]

    logical = sensors + actuators  # physical device behind slot i
    if labels_on:
        filenames = SENSOR_DEVICES + ACTUATOR_DEVICES
    else:
        rng = random.Random(stable_seed(seed, "labels_off"))
        order = list(range(len(ALL_DEVICES)))
        rng.shuffle(order)
        filenames = [None] * len(ALL_DEVICES)
        for slot, pos in enumerate(order):
            filenames[slot] = f"d{pos}"
    return {filenames[i]: logical[i] for i in range(len(logical))}


class DeviceBridge:
    def __init__(self, devfs_dir, world, bindings, log_fn=None):
        self.dir = devfs_dir
        self.world = world
        self.bindings = bindings
        self.log = log_fn or (lambda rec: None)
        self.rng_drop = random.Random(
            stable_seed(world.maze.seed, world.episode_index, "drops"))
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
            path = os.path.join(self.dir, filename)
            os.mkfifo(path)
            if logical in ACTUATOR_DEVICES:
                t = threading.Thread(target=self._actuator_loop,
                                     args=(path, filename, logical),
                                     daemon=True)
            else:
                t = threading.Thread(target=self._sensor_loop,
                                     args=(path, filename, logical),
                                     daemon=True)
            t.start()
            self.threads.append(t)

    def stop(self):
        self.running = False
        # Unblock threads stuck in open() by briefly opening the other end.
        for filename, logical in self.bindings.items():
            path = os.path.join(self.dir, filename)
            try:
                if logical in ACTUATOR_DEVICES:
                    fd = os.open(path, os.O_WRONLY | os.O_NONBLOCK)
                else:
                    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                os.close(fd)
            except OSError:
                pass

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
        raise ValueError(logical)

    def _sensor_loop(self, path, filename, logical):
        import time
        drop_p = self.world.noise["dropped_read_p"]
        while self.running:
            try:
                # Blocks until a reader opens the file.
                fd = os.open(path, os.O_WRONLY)
            except OSError:
                continue
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
                    frame = self._emit(logical)
                    os.write(fd, (frame + "\n").encode())
                    self.read_counts[filename] = \
                        self.read_counts.get(filename, 0) + 1
                    self.log(dict(event="read", dev=filename,
                                  physical=logical, value=frame,
                                  t=self.world.tick))
            except (BrokenPipeError, OSError):
                pass
            finally:
                try:
                    os.close(fd)
                except OSError:
                    pass
            # Give one-shot readers (cat) time to see EOF before the next
            # frame; a held-open reader gets a ~40 Hz stream.
            time.sleep(FRAME_INTERVAL)

    def _actuator_loop(self, path, filename, logical):
        side = ACTUATOR_DEVICES.index(logical)
        while self.running:
            try:
                f = open(path, "r")  # blocks until a writer opens
            except OSError:
                continue
            if not self.running:
                f.close()
                return
            with f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        val = int(float(line))
                    except ValueError:
                        self.log(dict(event="write_invalid", dev=filename,
                                      physical=logical, raw=line[:100],
                                      t=self.world.tick))
                        continue
                    self.world.set_motor(side, val)
                    self.write_counts[filename] = \
                        self.write_counts.get(filename, 0) + 1
                    self.log(dict(event="write", dev=filename,
                                  physical=logical, value=val,
                                  t=self.world.tick))

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
