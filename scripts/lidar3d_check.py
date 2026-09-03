"""Host-side validation of the 3D lidar (point-cloud) sensor, no docker.

World-level geometry (floor plane, wall faces, beams clearing wall tops,
the peer as a cylinder, agreement with the 2D scan), the config swap,
frame format and determinism, then a throwaway daemon on port 8796 to
prove the device is served over a FIFO, ground truth records a digest
rather than 50 kB frames, and /state?cloud=1 carries the true cloud.

Run from the repo root:  python scripts/lidar3d_check.py
"""

import json
import math
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

from sim import config as simconfig      # noqa: E402
from sim.maze import Maze                # noqa: E402
from sim.world import World              # noqa: E402

PORT = 8796
FAILS = []


def check(name, ok, detail=""):
    print(f"  {'ok  ' if ok else 'FAIL'} {name}" +
          (f"  ({detail})" if detail else ""))
    if not ok:
        FAILS.append(name)


def cfg_for(**over):
    o = {"lidar3d": {"enabled": True}, "noise_profile": "clean"}
    o.update(over)
    return simconfig.resolve(None, overrides=o)


def parse(frame):
    return [tuple(float(v) for v in p.split(","))
            for p in frame.split(";") if p]


def geometry():
    print("== geometry ==")
    cfg = cfg_for()
    c = cfg["lidar3d"]
    m = Maze(7, 6, 6, cell_size=0.5)
    w = World(cfg, m)
    sensors, _ = simconfig.device_sets(cfg)
    check("device set swaps lidar for lidar3d",
          "lidar3d" in sensors and "lidar" not in sensors, ",".join(sensors))
    off, _ = simconfig.device_sets(simconfig.resolve(None))
    check("default config keeps the 2D lidar",
          "lidar" in off and "lidar3d" not in off)

    t0 = time.time()
    pts = w.lidar3d_points(noisy=False)
    dt = (time.time() - t0) * 1000
    check("cloud is dense", len(pts) > 0.8 * c["rings"] * c["azimuths"],
          f"{len(pts)} of {c['rings'] * c['azimuths']}")
    check("a frame casts in well under a tick", dt < 15, f"{dt:.1f} ms")
    hs = c["sensor_height"]
    floor = [p for p in pts if abs(p[2] + hs) < 1e-3]
    check("floor plane sits at -sensor_height", len(floor) > 50,
          f"{len(floor)} floor points at z={-hs}")
    check("floor points all on downward rings",
          all(p[2] < 0 for p in floor))
    zmax = max(p[2] for p in pts) + hs
    check("no point above the wall top",
          zmax <= c["wall_height"] + 1e-6, f"max world z {zmax:.3f}")
    # Beams that clear the wall top return nothing: the top ring has
    # fewer returns than the horizontal-most ring.
    elevs = w._ring_elevations()
    by_ring = {}
    for x, y, z in pts:
        e = math.atan2(z, math.hypot(x, y))
        k = min(range(len(elevs)), key=lambda i: abs(elevs[i] - e))
        by_ring[k] = by_ring.get(k, 0) + 1
    top, mid = by_ring.get(len(elevs) - 1, 0), by_ring[len(elevs) // 2]
    check("top ring loses returns over the wall tops", top < mid,
          f"top={top} mid={mid}")
    # Near-horizontal ring agrees with the 2D scan at every azimuth.
    e1 = min((e for e in elevs if e > 0), key=abs)
    w.lidar_cfg = dict(w.lidar_cfg, rays=c["azimuths"])
    r2d = w.lidar_true()
    err = []
    for x, y, z in pts:
        if abs(math.atan2(z, math.hypot(x, y)) - e1) > 1e-6:
            continue
        a = math.atan2(y, x) % (2 * math.pi)
        k = round(a / (2 * math.pi / c["azimuths"])) % c["azimuths"]
        err.append(abs(math.hypot(x, y) - r2d[k]))
    check("near-horizontal ring reproduces the 2D scan",
          err and max(err) < 1e-6, f"n={len(err)} max err={max(err):.2e}")
    # Facing a wall at a known distance: hit range and height check.
    frame = w.lidar3d_frame()
    p2 = parse(frame)
    check("frame parses as x,y,z triples", len(p2) == len(pts),
          f"{len(frame)} bytes")
    w.reset()
    check("frame is deterministic after reset", w.lidar3d_frame() == frame)
    w2 = World(cfg_for(noise_profile="default_noisy"), m)
    n2 = len(parse(w2.lidar3d_frame()))
    check("noisy profile drops about 1% of points",
          0.95 * len(pts) < n2 < len(pts), f"{n2} vs {len(pts)}")
    # True cloud is in world coordinates: floor points at z = 0.
    tw = w.lidar3d_true()
    check("true cloud is world-frame with the floor at z=0",
          any(abs(p[2]) < 1e-3 for p in tw)
          and all(0 <= p[0] <= 3.6 and 0 <= p[1] <= 3.6 for p in tw))
    # Duo: the peer is seen as a robot_height cylinder, nothing taller.
    dcfg = cfg_for(duo={"enabled": True})
    wa = World(dcfg, m, bot_id="a")
    wb = World(dcfg, m, bot_id="b",
               spawn_cell=(m.start_cell[0] + 1, m.start_cell[1]))
    wa.set_peer(wb)
    wb.set_peer(wa)
    rh = dcfg["lidar3d"]["robot_height"]
    d = math.hypot(wa.x - wb.x, wa.y - wb.y)
    bearing = math.atan2(wb.y - wa.y, wb.x - wa.x) - wa.theta
    # Points on the peer: at its bearing, at its near face's range.
    peer_pts = [p for p in wa.lidar3d_points(noisy=False)
                if abs((math.atan2(p[1], p[0]) - bearing + math.pi)
                       % (2 * math.pi) - math.pi) < 0.15
                and abs(math.hypot(p[0], p[1]) - (d - 0.09)) < 0.03]
    check("peer returns present at its bearing and distance",
          len(peer_pts) > 5, f"{len(peer_pts)} pts at d={d:.2f}")
    check("peer returns never exceed robot_height",
          all(p[2] + hs <= rh + 1e-6 for p in peer_pts),
          f"max z {max(p[2] + hs for p in peer_pts):.3f}" if peer_pts
          else "")
    ep = [p for p in peer_pts if p[2] + hs > 0.9 * rh]
    check("peer top edge is painted by upward rings", len(ep) >= 1,
          f"{len(ep)} pts near the top")


def daemon():
    print("== end-to-end daemon (port %d) ==" % PORT)
    import shutil
    import urllib.request
    scratch = os.environ.get("LIDAR3D_CHECK_DIR", "/tmp/lidar3d_check")
    shutil.rmtree(scratch, ignore_errors=True)
    run_dir = os.path.join(scratch, "run")
    devfs = os.path.join(scratch, "devfs")
    os.makedirs(run_dir)
    cfg = cfg_for(labels="on", sim={"api_port": PORT})
    cfg_path = os.path.join(run_dir, "daemon_config.json")
    simconfig.dump_resolved(cfg, cfg_path)
    proc = subprocess.Popen(
        [sys.executable, "-m", "sim.daemon", "--config", cfg_path,
         "--run-dir", run_dir, "--devfs", devfs, "--port", str(PORT)],
        cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    try:
        state = None
        for _ in range(100):
            try:
                with urllib.request.urlopen(
                        f"http://127.0.0.1:{PORT}/state", timeout=2) as r:
                    state = json.loads(r.read())
                break
            except OSError:
                time.sleep(0.1)
        check("daemon up", state is not None)
        check("/state without cloud=1 carries no cloud",
              "lidar3d_true" not in state)
        names = sorted(os.listdir(devfs))
        check("lidar3d port exists, lidar does not",
              "lidar3d" in names and "lidar" not in names, ",".join(names))
        r = subprocess.run(["timeout", "2", "cat",
                            os.path.join(devfs, "lidar3d")],
                           capture_output=True)
        frame = r.stdout.decode().strip()
        pts = parse(frame) if frame else []
        check("one cat returns one point-cloud frame", len(pts) > 1000,
              f"{len(pts)} pts, {len(frame)} bytes")
        with urllib.request.urlopen(
                f"http://127.0.0.1:{PORT}/state?cloud=1", timeout=5) as rr:
            st = json.loads(rr.read())
        check("/state?cloud=1 carries the true cloud and its config",
              len(st.get("lidar3d_true", [])) > 1000
              and st.get("lidar3d_cfg", {}).get("wall_height") == 0.4)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
    # The daemon flushes ground truth on shutdown; read it afterwards.
    gt = os.path.join(run_dir, "ground_truth.jsonl")
    reads = [json.loads(l) for l in open(gt) if '"read"' in l]
    l3 = [x for x in reads if x.get("event") == "read"
          and x.get("physical") == "lidar3d"]
    check("ground truth logs the lidar3d read", len(l3) >= 1)
    check("ground truth stores a digest, not the frame",
          bool(l3) and all(x["value"].startswith("<") and "sha1=" in x["value"]
                           and len(x["value"]) < 80 for x in l3),
          (l3[0]["value"] if l3 else ""))


def main():
    geometry()
    daemon()
    print("PASS" if not FAILS else f"FAILED: {FAILS}")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
