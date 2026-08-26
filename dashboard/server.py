"""Mazebot dashboard: plain HTML/JS served by the harness, no build step.

    python3 -m dashboard.server [--config config.yaml] [--port 8080]

Serves dashboard/static/ plus a JSON API over the runs/ directory, and
proxies live state from the sim daemon (localhost only).  Everything here
is experimenter-facing ground truth; none of it is reachable from the bot
container.
"""

import argparse
import difflib
import json
import os
import sys
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATIC = os.path.join(REPO, "dashboard", "static")

MIME = {".html": "text/html", ".js": "text/javascript",
        ".css": "text/css", ".svg": "image/svg+xml",
        ".json": "application/json"}


def safe_join(base, *parts):
    """Join and refuse to escape base (path traversal guard)."""
    path = os.path.realpath(os.path.join(base, *parts))
    if not path.startswith(os.path.realpath(base) + os.sep) \
            and path != os.path.realpath(base):
        raise PermissionError("path escapes base")
    return path


class Api:
    def __init__(self, runs_dir, daemon_port):
        self.runs = runs_dir
        self.daemon_port = daemon_port

    # -- series / episodes ------------------------------------------------

    def series_list(self):
        out = []
        if not os.path.isdir(self.runs):
            return out
        for name in sorted(os.listdir(self.runs)):
            sdir = os.path.join(self.runs, name)
            if not os.path.isdir(sdir):
                continue
            eps = [e for e in sorted(os.listdir(sdir))
                   if e.startswith("ep_")]
            meta = {}
            sj = os.path.join(sdir, "series.json")
            if os.path.exists(sj):
                with open(sj) as f:
                    c = json.load(f).get("config", {})
                meta = {k: c.get(k) for k in
                        ("arm", "labels", "model", "noise_profile")}
            if eps or meta:
                out.append(dict(name=name, episodes=len(eps), **meta))
        return out

    def episodes(self, series):
        sdir = safe_join(self.runs, series)
        out = []
        for name in sorted(os.listdir(sdir)):
            if not name.startswith("ep_"):
                continue
            n = int(name.split("_")[1])
            summary_path = os.path.join(sdir, name, "summary.json")
            if os.path.exists(summary_path):
                with open(summary_path) as f:
                    out.append(dict(json.load(f), running=False))
            else:
                out.append(dict(episode=n, running=True))
        return out

    def ep_dir(self, series, ep):
        return safe_join(self.runs, series, f"ep_{int(ep):03d}")

    def maze(self, series, ep):
        with open(os.path.join(self.ep_dir(series, ep), "maze.json")) as f:
            return json.load(f)

    def transcript(self, series, ep, after=0, limit=500):
        path = os.path.join(self.ep_dir(series, ep), "transcript.jsonl")
        events = []
        next_index = after
        if os.path.exists(path):
            with open(path) as f:
                for i, line in enumerate(f):
                    if i < after or len(events) >= limit:
                        if len(events) >= limit:
                            break
                        continue
                    try:
                        events.append(json.loads(line))
                    except json.JSONDecodeError:
                        continue
                    next_index = i + 1
        return {"events": events, "next": next_index}

    def gt_trail(self, series, ep, max_points=3000):
        """Downsampled pose trail + events from the ground-truth log,
        for replaying finished episodes."""
        path = os.path.join(self.ep_dir(series, ep), "ground_truth.jsonl")
        poses, events = [], []
        if os.path.exists(path):
            with open(path) as f:
                for line in f:
                    try:
                        r = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if "event" in r:
                        if r["event"] in ("collision", "goal_reached",
                                          "reset"):
                            events.append({"event": r["event"],
                                           "t": r.get("t")})
                    elif "pose" in r:
                        poses.append([r["t"]] + r["pose"] +
                                     [r.get("col", 0)])
        step = max(1, len(poses) // max_points)
        return {"poses": poses[::step], "events": events,
                "total_ticks": poses[-1][0] if poses else 0}

    # -- memory -----------------------------------------------------------

    def memory_tree(self, series, snapshot=None):
        base = os.path.join(safe_join(self.runs, series), "memory") \
            if not snapshot else \
            os.path.join(self.ep_dir(series, snapshot), "memory_snapshot")
        out = []
        for root, _dirs, files in os.walk(base):
            for name in sorted(files):
                full = os.path.join(root, name)
                out.append({"path": os.path.relpath(full, base),
                            "size": os.path.getsize(full)})
        return sorted(out, key=lambda x: x["path"])

    def memory_file(self, series, path, snapshot=None, cap=200_000):
        base = os.path.join(safe_join(self.runs, series), "memory") \
            if not snapshot else \
            os.path.join(self.ep_dir(series, snapshot), "memory_snapshot")
        full = safe_join(base, path)
        with open(full, "rb") as f:
            data = f.read(cap + 1)
        truncated = len(data) > cap
        return {"path": path, "truncated": truncated,
                "content": data[:cap].decode(errors="replace")}

    def memory_diff(self, series, ep):
        """Unified diff of /memory between episode ep-1 and ep snapshots."""
        ep = int(ep)
        new_base = os.path.join(self.ep_dir(series, ep), "memory_snapshot")
        if ep > 1:
            old_base = os.path.join(self.ep_dir(series, ep - 1),
                                    "memory_snapshot")
        else:
            old_base = None  # first episode: everything is new
        files = set()
        for base in (old_base, new_base):
            if base and os.path.isdir(base):
                for root, _d, fs in os.walk(base):
                    for name in fs:
                        files.add(os.path.relpath(
                            os.path.join(root, name), base))

        def read_lines(base, rel):
            if not base:
                return []
            full = os.path.join(base, rel)
            if not os.path.exists(full):
                return []
            try:
                with open(full, errors="replace") as f:
                    return f.readlines()
            except OSError:
                return []

        diffs = []
        for rel in sorted(files):
            old = read_lines(old_base, rel)
            new = read_lines(new_base, rel)
            if old == new:
                continue
            udiff = list(difflib.unified_diff(
                old, new, fromfile=f"ep{ep - 1}/{rel}" if ep > 1
                else "/dev/null",
                tofile=f"ep{ep}/{rel}", n=3))
            diffs.append({"path": rel, "diff": "".join(udiff)})
        return {"episode": ep, "diffs": diffs}

    # -- metrics ----------------------------------------------------------

    def metrics(self, series):
        eps = [e for e in self.episodes(series) if not e.get("running")]
        curve = []
        for e in eps:
            curve.append({
                "episode": e["episode"],
                "solved": e.get("solved", False),
                "goal_tick": e.get("goal_tick"),
                "sim_time_to_solve_s": None,
                "wall_s": e.get("wall_s"),
                "tokens_out": (e.get("tokens") or {}).get("output"),
                "turns": e.get("turns"),
                "collisions": e.get("collisions"),
                "end_reason": e.get("end_reason"),
                "restarts": e.get("restarts"),
            })
            if e.get("goal_tick"):
                curve[-1]["sim_time_to_solve_s"] = round(
                    e["goal_tick"] / 50.0, 1)
        evals = {}
        eval_dir = os.path.join(safe_join(self.runs, series), "evals")
        if os.path.isdir(eval_dir):
            for name in sorted(os.listdir(eval_dir)):
                if name.endswith(".json"):
                    try:
                        with open(os.path.join(eval_dir, name)) as f:
                            evals[name[:-5]] = json.load(f)
                    except (OSError, json.JSONDecodeError):
                        pass
        return {"learning_curve": curve, "evals": evals}

    # -- live proxy -------------------------------------------------------

    def live(self, path, method="GET", body=None):
        url = f"http://127.0.0.1:{self.daemon_port}{path}"
        data = json.dumps(body or {}).encode() if method == "POST" else None
        req = urllib.request.Request(url, data=data, method=method)
        with urllib.request.urlopen(req, timeout=3) as r:
            return json.loads(r.read())


def make_handler(api):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _static(self, rel):
            if rel in ("", "/"):
                rel = "index.html"
            try:
                path = safe_join(STATIC, rel.lstrip("/"))
                with open(path, "rb") as f:
                    body = f.read()
            except (OSError, PermissionError):
                self._json({"error": "not found"}, 404)
                return
            ext = os.path.splitext(path)[1]
            self.send_response(200)
            self.send_header("Content-Type",
                             MIME.get(ext, "application/octet-stream"))
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            q = {k: v[0] for k, v in parse_qs(u.query).items()}
            p = u.path
            try:
                if not p.startswith("/api/"):
                    self._static(p)
                elif p == "/api/series":
                    self._json(api.series_list())
                elif p == "/api/episodes":
                    self._json(api.episodes(q["series"]))
                elif p == "/api/maze":
                    self._json(api.maze(q["series"], q["ep"]))
                elif p == "/api/transcript":
                    self._json(api.transcript(q["series"], q["ep"],
                                              after=int(q.get("after", 0))))
                elif p == "/api/gt_trail":
                    self._json(api.gt_trail(q["series"], q["ep"]))
                elif p == "/api/memory/tree":
                    self._json(api.memory_tree(q["series"],
                                               q.get("snapshot")))
                elif p == "/api/memory/file":
                    self._json(api.memory_file(q["series"], q["path"],
                                               q.get("snapshot")))
                elif p == "/api/memory/diff":
                    self._json(api.memory_diff(q["series"], q["ep"]))
                elif p == "/api/metrics":
                    self._json(api.metrics(q["series"]))
                elif p == "/api/live/state":
                    since = q.get("since", "0")
                    self._json(api.live(f"/state?since={since}"))
                elif p == "/api/live/maze":
                    self._json(api.live("/maze"))
                else:
                    self._json({"error": "not found"}, 404)
            except (FileNotFoundError, KeyError):
                self._json({"error": "not found"}, 404)
            except PermissionError:
                self._json({"error": "forbidden"}, 403)
            except OSError as e:
                self._json({"error": f"unavailable: {e}"}, 502)

        def do_POST(self):
            u = urlparse(self.path)
            if u.path in ("/api/live/pause", "/api/live/resume",
                          "/api/live/rtf", "/api/live/reset"):
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}") if n else {}
                try:
                    self._json(api.live(
                        u.path.replace("/api/live", ""), "POST", body))
                except OSError as e:
                    self._json({"error": f"unavailable: {e}"}, 502)
            else:
                self._json({"error": "not found"}, 404)

    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(REPO, "config.yaml"))
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--host", default="127.0.0.1")
    args = ap.parse_args(argv)

    sys.path.insert(0, REPO)
    from sim import config as simconfig
    cfg = simconfig.resolve(args.config)
    port = args.port or cfg["dashboard"]["port"]
    runs_dir = os.path.join(REPO, cfg.get("runs_dir", "runs"))

    api = Api(runs_dir, cfg["sim"]["api_port"])
    server = ThreadingHTTPServer((args.host, port), make_handler(api))
    server.daemon_threads = True
    print(f"dashboard: http://{args.host}:{port}/  "
          f"(runs: {runs_dir}, daemon proxy: {cfg['sim']['api_port']})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
