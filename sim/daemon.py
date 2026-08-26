"""Sim daemon: owns ground truth, backs the device files, serves the API.

Run as its own process per episode:

    python -m sim.daemon --config <resolved.json> --run-dir <dir> \
        --devfs <dir> [--episode-index N] [--port P]

On startup it writes maze.json, device_map.json and resolved_config.json to
the run dir, opens the append-only ground_truth.jsonl, creates the FIFO
device tree, then ticks the world at tick_hz scaled by realtime_factor.
Prints "READY <port>" on stdout when the API is up.
"""

import argparse
import json
import os
import signal
import sys
import threading
import time

from devices.bridge import DeviceBridge, compute_bindings, write_device_map
from sim import config as simconfig
from sim.api import serve
from sim.maze import Maze
from sim.world import World


class GroundTruthLog:
    """Append-only JSONL; the agent can never see this file."""

    def __init__(self, path):
        self.f = open(path, "a", buffering=1024 * 64)
        self.lock = threading.Lock()
        self.count = 0
        self.closed = False

    def write(self, rec):
        with self.lock:
            if self.closed:
                return
            self.f.write(json.dumps(rec, separators=(",", ":")) + "\n")
            self.count += 1
            if self.count % 200 == 0:
                self.f.flush()

    def close(self):
        with self.lock:
            self.closed = True
            self.f.flush()
            self.f.close()


class Daemon:
    def __init__(self, cfg, run_dir, devfs_dir, episode_index=0):
        self.cfg = cfg
        self.run_dir = run_dir
        os.makedirs(run_dir, exist_ok=True)
        self.gt = GroundTruthLog(os.path.join(run_dir, "ground_truth.jsonl"))

        m = cfg["maze"]
        pert = cfg.get("perturb_state", {})
        self.maze = Maze(m["seed"], m["width"], m["height"],
                         cell_size=m["cell_size"], braid=m["braid"],
                         family_index=pert.get("family_index", 0))
        self.world = World(cfg, self.maze, episode_index=episode_index,
                           log_fn=self.gt.write)

        labels_on = cfg["labels"] == "on"
        bindings = compute_bindings(
            m["seed"], labels_on,
            remap_index=pert.get("remap_index", 0),
            motor_swapped=pert.get("motor_swapped", False))
        self.bridge = DeviceBridge(devfs_dir, self.world, bindings,
                                   log_fn=self.gt.write)

        write_device_map(run_dir, bindings, labels_on,
                         pert.get("remap_index", 0),
                         pert.get("motor_swapped", False))
        with open(os.path.join(run_dir, "maze.json"), "w") as f:
            json.dump(self.maze.to_dict(), f)
        simconfig.dump_resolved(
            cfg, os.path.join(run_dir, "resolved_config.json"))
        self.gt.write(dict(event="daemon_start", maze_hash=self.maze.hash(),
                           episode_index=episode_index,
                           perturb_state=pert, t=0))

        self.paused = False
        self.rtf = float(cfg["sim"]["realtime_factor"])
        self.tick_hz = cfg["sim"]["tick_hz"]
        self.running = True
        self._server = None

    def set_rtf(self, factor):
        self.rtf = max(0.0, factor)

    def run(self):
        self.bridge.start()
        self._server = serve(self, self.cfg["sim"]["api_host"],
                             self.cfg["sim"]["api_port"])
        print(f"READY {self.cfg['sim']['api_port']}", flush=True)
        next_t = time.perf_counter()
        while self.running:
            if self.paused:
                time.sleep(0.05)
                next_t = time.perf_counter()
                continue
            self.world.step()
            if self.rtf > 0:
                period = 1.0 / (self.tick_hz * self.rtf)
                next_t += period
                delay = next_t - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                elif delay < -1.0:
                    next_t = time.perf_counter()  # fell behind; don't spiral
        self._teardown()

    def shutdown(self):
        self.running = False

    def _teardown(self):
        self.bridge.stop()
        if self._server:
            self._server.shutdown()
        self.gt.write(dict(event="daemon_stop", t=self.world.tick,
                           device_stats=self.bridge.stats()))
        self.gt.close()


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--config", required=True,
                   help="path to resolved config JSON")
    p.add_argument("--run-dir", required=True)
    p.add_argument("--devfs", required=True)
    p.add_argument("--episode-index", type=int, default=0)
    p.add_argument("--port", type=int, default=None)
    args = p.parse_args(argv)

    cfg = simconfig.load_resolved(args.config)
    if args.port is not None:
        cfg["sim"]["api_port"] = args.port

    daemon = Daemon(cfg, args.run_dir, args.devfs,
                    episode_index=args.episode_index)
    signal.signal(signal.SIGTERM, lambda *a: daemon.shutdown())
    signal.signal(signal.SIGINT, lambda *a: daemon.shutdown())
    daemon.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
