"""Sim daemon subprocess management + its HTTP API, from the host side."""

import json
import os
import subprocess
import sys
import time
import urllib.request

from sim import config as simconfig


def api_get(port, path, host="127.0.0.1"):
    with urllib.request.urlopen(f"http://{host}:{port}{path}",
                                timeout=10) as r:
        return json.loads(r.read())


def api_post(port, path, body=None, host="127.0.0.1"):
    data = json.dumps(body or {}).encode()
    req = urllib.request.Request(f"http://{host}:{port}{path}", data=data,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read())


class SimDaemonProc:
    def __init__(self, cfg, run_dir, devfs_dir, episode_index=0,
                 repo_root=None):
        self.cfg = cfg
        self.run_dir = os.path.abspath(run_dir)
        self.devfs_dir = os.path.abspath(devfs_dir)
        self.episode_index = episode_index
        self.repo_root = repo_root or os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))
        self.port = cfg["sim"]["api_port"]
        self.proc = None

    def start(self, timeout_s=15):
        os.makedirs(self.run_dir, exist_ok=True)
        os.makedirs(self.devfs_dir, exist_ok=True)
        cfg_path = os.path.join(self.run_dir, "daemon_config.json")
        simconfig.dump_resolved(self.cfg, cfg_path)
        log = open(os.path.join(self.run_dir, "daemon.log"), "ab")
        self.proc = subprocess.Popen(
            [sys.executable, "-m", "sim.daemon",
             "--config", cfg_path,
             "--run-dir", self.run_dir,
             "--devfs", self.devfs_dir,
             "--episode-index", str(self.episode_index),
             "--port", str(self.port)],
            cwd=self.repo_root, stdout=log, stderr=log)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError(
                    f"sim daemon exited early; see {self.run_dir}/daemon.log")
            try:
                if api_get(self.port, "/health").get("ok"):
                    return self
            except OSError:
                time.sleep(0.1)
        raise RuntimeError("sim daemon did not become healthy")

    def get(self, path):
        return api_get(self.port, path)

    def post(self, path, body=None):
        return api_post(self.port, path, body)

    def stop(self, timeout_s=10):
        if not self.proc:
            return
        try:
            self.post("/shutdown")
        except OSError:
            pass
        try:
            self.proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None
