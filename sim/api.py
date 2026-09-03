"""Experimenter-facing HTTP API for the sim daemon (localhost only).

Ground truth flows out of here for the dashboard and the harness; nothing
served on this port is reachable from inside the bot container
(network=none).
"""

import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs


def make_handler(daemon):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            if u.path == "/health":
                # run_dir + pid let a client verify it reached THIS
                # daemon, not a stale one holding the same port.
                self._json({"ok": True, "tick": daemon.world.tick,
                            "run_dir": daemon.run_dir,
                            "pid": os.getpid()})
            elif u.path == "/state":
                since = int(q.get("since", ["0"])[0])
                snap = daemon.world.snapshot(since_tick=since)
                snap["paused"] = daemon.paused
                snap["realtime_factor"] = daemon.rtf
                l3 = daemon.world.lidar3d_on
                if not l3:
                    # The beam scan only exists when the point cloud
                    # does not; the dashboard draws whatever is here.
                    snap["lidar_true"] = daemon.world.lidar_true()
                    snap["ray_angles"] = daemon.world.ray_angles()
                snap["device_stats"] = daemon.bridge.stats()
                # The 3D cloud is ~3k points per robot; cast it only
                # when a viewer asks (cloud=1), once per world.
                cloud = q.get("cloud", ["0"])[0] == "1" and l3
                duo = len(daemon.worlds) > 1
                if cloud:
                    snap["lidar3d_cfg"] = daemon.world.lidar3d_cfg
                    if not duo:
                        snap["lidar3d_true"] = daemon.world.lidar3d_true()
                if duo:
                    bots = []
                    for world, bridge in zip(daemon.worlds,
                                             daemon.bridges):
                        b = world.snapshot(since_tick=since)
                        if not l3:
                            b["lidar_true"] = world.lidar_true()
                        if cloud:
                            b["lidar3d_true"] = world.lidar3d_true()
                        b["device_stats"] = bridge.stats()
                        bots.append(b)
                    snap["bots"] = bots
                self._json(snap)
            elif u.path == "/maze":
                self._json(daemon.maze.to_dict())
            elif u.path == "/config":
                self._json(daemon.cfg)
            else:
                self._json({"error": "not found"}, 404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            body = {}
            if n:
                try:
                    body = json.loads(self.rfile.read(n) or b"{}")
                except json.JSONDecodeError:
                    body = {}
            if self.path == "/reset":
                for world in daemon.worlds:
                    world.reset()
                self._json({"ok": True})
            elif self.path == "/pause":
                daemon.paused = True
                self._json({"ok": True})
            elif self.path == "/resume":
                daemon.paused = False
                self._json({"ok": True})
            elif self.path == "/rtf":
                daemon.set_rtf(float(body.get("factor", 1.0)))
                self._json({"ok": True, "realtime_factor": daemon.rtf})
            elif self.path == "/shutdown":
                self._json({"ok": True})
                threading.Thread(target=daemon.shutdown,
                                 daemon=True).start()
            else:
                self._json({"error": "not found"}, 404)

    return Handler


def serve(daemon, host, port):
    server = ThreadingHTTPServer((host, port), make_handler(daemon))
    server.daemon_threads = True
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    return server
