"""Host-side validation of the race-track scene (scene: track).

In-process: circuit geometry from the bundled centerline, config
guards, the car's friction-circle grip (off = the original kinematics),
the IMU and status frames, lap timing with the sector rule (no
shortcuts, no wrong-way laps), completion after warm-up + timed laps,
determinism.  Then a real daemon on an alternate port serves the track
over FIFOs.  Set TRACK_DEMO_DIR to also write a small demo episode
(maze.json, ground_truth.jsonl, transcript.jsonl, summary.json) for the
renderers.

Run from the repo root:  python scripts/track_check.py
"""

import json
import math
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

from sim import config as simconfig       # noqa: E402
from sim.track import Track               # noqa: E402
from sim.world import World               # noqa: E402

PORT = 8795
FAILS = []
TWO_PI = 2 * math.pi


def check(name, ok, detail=""):
    print(f"  {'ok  ' if ok else 'FAIL'} {name}" +
          (f"  ({detail})" if detail else ""))
    if not ok:
        FAILS.append(name)


def race_cfg(**over):
    base = {
        "scene": "track", "labels": "off", "prompt_variant": "race",
        "readme_variant": "race",
        "robot": {"model": "car",
                  "car": {"a_grip": 1.5, "v_max": 2.0, "accel_max": 1.5}},
        "lidar": {"max_range": 6.0},
        "track": {"laps_warmup": 1, "laps_timed": 2},
        "sim": {"api_port": PORT},
    }
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            base[k] = {**base[k], **v}
        else:
            base[k] = v
    cfg = simconfig.resolve(None, overrides=base)
    cfg["noise"] = dict(simconfig.NOISE_PROFILES["clean"])
    return cfg


def curvature(tr, j):
    n = len(tr.center)
    p0, p1, p2 = tr.center[(j - 1) % n], tr.center[j], tr.center[(j + 1) % n]
    a, b, c = math.dist(p0, p1), math.dist(p1, p2), math.dist(p0, p2)
    s = (a + b + c) / 2
    area = max(s * (s - a) * (s - b) * (s - c), 0.0) ** 0.5
    return 4 * area / (a * b * c) if area > 1e-9 else 0.0


class PurePursuit:
    """A competent scripted driver: aims at a lookahead point on the
    centerline and holds the grip-limited speed for the curvature
    ahead.  Used to exercise laps, not to set records."""

    def __init__(self, tr, a_grip, v_max, lookahead=0.8, margin=0.85):
        self.tr, self.n = tr, len(tr.center)
        self.a_grip, self.v_max = a_grip, v_max
        self.lookahead, self.margin = lookahead, margin
        self.idx = None

    def act(self, w):
        tr = self.tr
        self.idx = tr.nearest_index(w.x, w.y, self.idx)
        j, dist = self.idx, 0.0
        while dist < self.lookahead:
            dist += math.dist(tr.center[j], tr.center[(j + 1) % self.n])
            j = (j + 1) % self.n
        tx, ty = tr.center[j]
        ang = (math.atan2(ty - w.y, tx - w.x) - w.theta + math.pi) \
            % TWO_PI - math.pi
        steer = max(-255, min(255, int(ang / math.radians(35) * 255 * 1.5)))
        k = max(curvature(tr, (j + q) % self.n) for q in range(0, 30, 5))
        vt = min(self.v_max, math.sqrt(self.a_grip / k) * self.margin) \
            if k > 0 else self.v_max
        w.set_actuator("steer", steer)
        w.set_actuator("accel", 255 if w.v < vt else -255)


def in_process():
    print("== track geometry ==")
    tr = Track("austin", scale=0.07)
    n = len(tr.center)
    check("bundled centerline loads", n > 1000, f"{n} points")
    check("length is COTA scaled", abs(tr.length - 5508 * 0.07) < 3.0,
          f"{tr.length:.1f} m")
    segs = tr.segments()
    check("two closed edge loops", len(segs) == 2 * n)
    closed = all(math.dist(loop[0], loop[-1]) < 2.0
                 for loop in (tr.left, tr.right))
    check("edge loops close", closed)
    wl = math.dist(tr.left[0], tr.center[0])
    wr = math.dist(tr.right[0], tr.center[0])
    check("edges sit at the scaled widths",
          abs(wl - 7.361 * 0.07) < 0.01 and abs(wr - 7.565 * 0.07) < 0.01,
          f"wl={wl:.3f} wr={wr:.3f}")
    d = tr.to_dict()
    check("to_dict carries the render data",
          d["kind"] == "track" and len(d["left"]) == n
          and len(d["right"]) == n and len(d["start_line"]) == 2
          and d["length_m"] > 0 and d["width"] > 0 and d["height"] > 0)
    check("hash is stable", tr.hash() == Track("austin", scale=0.07).hash()
          and tr.hash() != Track("austin", scale=0.05).hash())
    check("start pose is on the line facing forward",
          math.dist(tr.start_pose[:2], tr.center[0]) < 1e-6
          and abs(tr.side_of_start(*tr.start_pose[:2])) < 1e-9)
    check("nearest_index with a hint agrees with the global search",
          all(tr.nearest_index(*tr.center[i], hint=(i - 3) % n)
              == tr.nearest_index(*tr.center[i]) for i in (0, 5, 400, n - 1)))
    tx, ty = tr.tangent[0]
    check("forward crossing detected, backward ignored, off-line ignored",
          tr.crossed_start(tr.center[0][0] - tx * 0.2, tr.center[0][1] - ty * 0.2,
                           tr.center[0][0] + tx * 0.2, tr.center[0][1] + ty * 0.2)
          == 1
          and tr.crossed_start(tr.center[0][0] + tx * 0.2,
                               tr.center[0][1] + ty * 0.2,
                               tr.center[0][0] - tx * 0.2,
                               tr.center[0][1] - ty * 0.2) == -1
          and tr.crossed_start(tr.center[0][0] - tx * 0.2 + 5 * (-ty),
                               tr.center[0][1] - ty * 0.2 + 5 * tx,
                               tr.center[0][0] + tx * 0.2 + 5 * (-ty),
                               tr.center[0][1] + ty * 0.2 + 5 * tx) == 0)

    print("== config guards ==")
    cfg = race_cfg()
    check("track scene resolves with the car", cfg["scene"] == "track")
    sensors, actuators = simconfig.device_sets(cfg)
    check("race car carries an IMU", "imu" in sensors
          and actuators == ["accel", "steer"], str(sensors))
    plain = simconfig.resolve(None, overrides={"robot": {"model": "car"}})
    check("maze car has no IMU", "imu" not in simconfig.device_sets(plain)[0])
    for name, over in [
            ("track needs the car", {"scene": "track"}),
            ("track is solo only",
             {"scene": "track", "robot": {"model": "car"},
              "duo": {"enabled": True}}),
            ("negative grip rejected",
             {"robot": {"model": "car", "car": {"a_grip": -1}}}),
            ("laps_timed >= 1",
             {"scene": "track", "robot": {"model": "car"},
              "track": {"laps_timed": 0}}),
            ("track with labels on rejected",
             {"scene": "track", "robot": {"model": "car"},
              "labels": "on", "readme_variant": "race"}),
            ("track without a README variant rejected",
             {"scene": "track", "robot": {"model": "car"},
              "labels": "off"}),
            ("maze_regen on a track rejected",
             {"scene": "track", "robot": {"model": "car"},
              "labels": "off", "readme_variant": "race",
              "perturbations": [{"at_episode": 2, "name": "maze_regen"}]}),
            ("width_scale 0 rejected",
             {"scene": "track", "robot": {"model": "car"},
              "labels": "off", "readme_variant": "race",
              "track": {"width_scale": 0}}),
            ("custom image without its dockerfile rejected",
             {"container": {"image": "mazebot-bot-np"}})]:
        try:
            simconfig.resolve(None, overrides=over)
            check(name, False)
        except ValueError:
            check(name, True)
    check("prompt variant race accepted",
          simconfig.resolve(None, overrides={"prompt_variant": "race"})
          ["prompt_variant"] == "race")
    try:
        Track("austin", scale=0.07, checkpoints=5000)
        check("checkpoints beyond the point count rejected", False)
    except ValueError:
        check("checkpoints beyond the point count rejected", True)
    # README lap counts come from the config.
    import shutil as _sh
    import tempfile as _tf
    from harness.episode import prepare_bot_dir
    d = _tf.mkdtemp(prefix="race_readme_")
    prepare_bot_dir(race_cfg(track={"laps_warmup": 2, "laps_timed": 3}), d)
    txt = open(os.path.join(d, "README.md")).read()
    _sh.rmtree(d, ignore_errors=True)
    check("README states the configured lap counts",
          "two laps of warm-up" in txt and "three laps that are timed" in txt
          and "{laps" not in txt, txt[txt.find("You get"):][:80])

    print("== grip ==")
    # a_grip 0 must be the pre-grip car exactly: replay its kinematics
    # (integrate v, clamp, then w from the new v) tick by tick during
    # the transient, combined throttle and steer, no saturation.
    cfg0 = race_cfg(robot={"car": {"a_grip": 0.0, "v_max": 2.0,
                                    "accel_max": 1.5}})
    w0 = World(cfg0, tr, spawn_theta=tr.start_pose[2])
    w0._collides = lambda *a, **k: None
    c = w0.car_cfg
    v = phi = 0.0
    same = True
    w0.set_actuator("accel", 200)
    w0.set_actuator("steer", 150)
    for _ in range(40):
        w0.step()
        # reference: the committed pre-grip kinematics
        a = w0.cmd_eff["accel"] / 255.0 * c["accel_max"]
        v += (a - c["drag"] * v) * w0.dt
        v = max(-c["v_rev_max"], min(c["v_max"], v))
        target = w0.cmd_eff["steer"] / 255.0 * math.radians(c["steer_max_deg"])
        rate = math.radians(c["steer_rate_deg_s"]) * w0.dt
        phi += max(-rate, min(rate, target - phi))
        w_ref = v / c["wheelbase"] * math.tan(phi)
        if abs(w0.v - v) > 1e-12 or abs(w0.w - w_ref) > 1e-12:
            same = False
    check("grip off: pre-grip kinematics reproduced tick for tick", same,
          f"v={w0.v:.5f} ref={v:.5f} w={w0.w:.5f} ref={w_ref:.5f}")
    check("grip off: no slip bookkeeping",
          w0.slide_ticks == 0 and not w0.slipping)

    w1 = World(race_cfg(), tr, spawn_theta=tr.start_pose[2])
    w0 = World(cfg0, tr, spawn_theta=tr.start_pose[2])
    for w in (w0, w1):
        # Physics only: a grip-limited car at full lock would otherwise
        # meet the track edge and the wall stop would zero its speed.
        w._collides = lambda *a, **k: None
        w.set_actuator("accel", 250)
        for _ in range(100):
            w.step()
    check("launch under the limit is grip-free (1.47 < 1.5 m/s^2)",
          abs(w0.v - w1.v) < 1e-9 and not w1.slipping and w1.slide_ticks == 0,
          f"v0={w0.v:.3f} v1={w1.v:.3f}")
    L = w1.car_cfg["wheelbase"]
    for w in (w0, w1):
        w.set_actuator("steer", 255)
        for _ in range(60):
            w.step()
    check("grip on: lateral acceleration capped, car marked sliding",
          abs(w1.v * w1.w) <= 1.5 + 1e-6 and w1.slipping
          and w1.slide_ticks > 0
          and abs(w1.w) < abs(w1.v / L * math.tan(w1.phi)),
          f"a_lat={w1.v * w1.w:.3f} w={w1.w:.3f} "
          f"w_kin={w1.v / L * math.tan(w1.phi):.3f}")
    check("sliding scrubs speed", w1.v < w0.v, f"{w1.v:.3f} < {w0.v:.3f}")
    ax, ay, wz = (float(x) for x in w1.imu_frame().split(","))
    check("IMU reports achieved lateral accel and yaw rate",
          abs(ay - w1.v * w1.w) < 1e-3 and abs(wz - w1.w) < 1e-3
          and abs(ay) <= 1.5 + 1e-3, f"{ax},{ay},{wz}")
    # IMU longitudinal tracks the speedometer, scrub included.
    vb = w1.v
    w1.step()
    ax, _, _ = (float(x) for x in w1.imu_frame().split(","))
    check("IMU longitudinal equals the speed change (scrub included)",
          abs(ax - (w1.v - vb) / w1.dt) < 1e-3,
          f"ax={ax:.3f} dv/dt={(w1.v - vb) / w1.dt:.3f}")
    check("status frame shows lap timing fields",
          w1.status_frame().startswith("tick=") and " lap=0 " in
          w1.status_frame() + " " and "last=0.0" in w1.status_frame()
          and "best=0.0" in w1.status_frame(), w1.status_frame())
    glog = []
    wg = World(race_cfg(), tr, log_fn=glog.append,
               spawn_theta=tr.start_pose[2])
    wg._collides = lambda *a, **k: None
    wg.set_actuator("accel", 255)
    for _ in range(100):
        wg.step()
    straight = glog[-1].get("slip")
    wg.set_actuator("steer", 255)
    for _ in range(40):
        wg.step()
    check("GT tick record carries slip 0 straight, 1 while sliding",
          straight == 0 and glog[-1].get("slip") == 1,
          f"straight={straight} sliding={glog[-1].get('slip')}")
    # Low grip, high throttle: the launch still accelerates at about
    # the limit, monotonically, and forward/reverse are symmetric.
    lo = race_cfg(robot={"car": {"a_grip": 0.3, "accel_max": 1.5,
                                  "v_max": 2.0, "v_rev_max": 2.0}})
    speeds = {}
    for cmd in (255, -255):
        w = World(lo, tr, spawn_theta=tr.start_pose[2])
        w._collides = lambda *a, **k: None
        w.set_actuator("accel", cmd)
        vs = []
        for _ in range(100):
            w.step()
            vs.append(w.v)
        speeds[cmd] = vs
    fwd, rev = speeds[255], speeds[-255]
    check("wheelspin launch is monotonic and near the grip limit",
          all(b >= a for a, b in zip(fwd, fwd[1:])) and fwd[-1] > 0.3
          and fwd[-1] < 0.3 * 2.0 * 1.2, f"v after 2 s = {fwd[-1]:.3f}")
    check("forward and reverse launches are symmetric",
          all(abs(a + b) < 1e-9 for a, b in zip(fwd, rev)),
          f"{fwd[-1]:.4f} vs {rev[-1]:.4f}")

    print("== laps ==")
    cfg = race_cfg()
    log = []
    w = World(cfg, tr, log_fn=log.append, spawn_theta=tr.start_pose[2])
    drv = PurePursuit(tr, 1.5, 2.0)
    t0 = time.time()
    for _ in range(50 * 900):
        drv.act(w)
        w.step()
        if w.goal_reached:
            break
    laps = [e for e in log if e.get("event") == "lap"]
    check("pure pursuit completes warm-up + 2 timed laps",
          w.goal_reached and w.lap == 3 and len(w.laps) == 2,
          f"lap={w.lap} laps={w.laps} collisions={w.collision_count}")
    check("lap events carry times and the timed flag",
          [e["timed"] for e in laps] == [False, True, True]
          and all(60 < e["time_s"] < 600 for e in laps), str(laps))
    check("best lap is the minimum timed lap",
          w.best_lap == min(w.laps) and w.last_lap == w.laps[-1])
    check("status frame reports last/best",
          f"lap=3 last={w.laps[-1]:.1f} best={min(w.laps):.1f}"
          in w.status_frame(), w.status_frame())
    check("completion powers the car down",
          all(v == 0 for v in w.cmd_eff.values()) and w.goal_tick is not None)
    check("no wall contact for a centerline driver", w.collision_count == 0,
          str(w.collision_count))
    check("summary fields in the snapshot",
          w.snapshot()["laps"] == w.laps and w.snapshot()["lap"] == 3
          and "slide_ticks" in w.snapshot())
    check("in-process pace", time.time() - t0 < 60,
          f"{time.time() - t0:.1f}s for {w.tick} ticks")
    if os.environ.get("TRACK_DEMO_DIR"):
        _write_demo(os.environ["TRACK_DEMO_DIR"], tr, cfg, log, w)

    # Shortcut: teleport across the line without visiting the sectors.
    w2 = World(cfg, tr, spawn_theta=tr.start_pose[2])
    tx, ty = tr.tangent[0]
    w2.x, w2.y = tr.center[0][0] - tx * 0.3, tr.center[0][1] - ty * 0.3
    w2._track_idx = tr.nearest_index(w2.x, w2.y)
    w2.set_actuator("accel", 255)
    for _ in range(80):
        w2.step()
    check("standing-start crossing counts nothing, silently",
          w2.lap == 0 and not any(e["event"] in ("lap", "lap_rejected")
                                  for e in w2.events))
    # A real shortcut: the last third of the circuit, then the line.
    w2b = World(cfg, tr, spawn_theta=tr.start_pose[2])
    w2b._collides = lambda *a, **k: None
    n = len(tr.center)
    # Teleports must stay within the local search window of the
    # nearest-point hint, as a real car does.
    w2b._track_idx = int(n * 0.7) - 1
    for i in range(int(n * 0.7), n, 3):
        w2b.x, w2b.y = tr.center[i]
        w2b.step()
    # ...then drive across the line (a crossing is a move, not a jump).
    w2b.theta = tr.start_pose[2]
    w2b.set_actuator("accel", 255)
    for _ in range(80):
        w2b.step()
    rej = [e for e in w2b.events if e["event"] == "lap_rejected"]
    check("crossing without the sectors is rejected",
          w2b.lap == 0 and len(rej) == 1 and 3 < rej[0]["sectors"] < 18,
          str(rej))
    # Wrong way: reverse over the line.
    w3 = World(cfg, tr, spawn_theta=tr.start_pose[2])
    w3.x, w3.y = tr.center[0][0] + tx * 0.3, tr.center[0][1] + ty * 0.3
    w3._track_idx = tr.nearest_index(w3.x, w3.y)
    w3.set_actuator("accel", -255)
    for _ in range(150):
        w3.step()
    check("backward crossing counts nothing",
          w3.lap == 0 and not any(e["event"] in ("lap", "lap_rejected")
                                  for e in w3.events)
          and tr.side_of_start(w3.x, w3.y) < 0, f"side={tr.side_of_start(w3.x, w3.y):.2f}")
    # Wrong-way tour: walk the whole circuit in reverse order (every
    # sector visited), then cross forward.  Directional progress must
    # leave it uncounted.
    w4 = World(cfg, tr, spawn_theta=tr.start_pose[2])
    w4._collides = lambda *a, **k: None
    n = len(tr.center)
    for i in range(n - 1, 0, -3):
        w4.x, w4.y = tr.center[i]
        w4.step()
    w4.x, w4.y = tr.center[0][0] - tx * 0.2, tr.center[0][1] - ty * 0.2
    w4.step()
    w4.x, w4.y = tr.center[0][0] + tx * 0.2, tr.center[0][1] + ty * 0.2
    w4.step()
    check("wrong-way tour then forward crossing is not a lap",
          w4.lap == 0 and len(w4._cp_seen) <= 3,
          f"lap={w4.lap} sectors={len(w4._cp_seen)}")
    check("standing-start line dancing emits no rejection",
          not any(e["event"] == "lap_rejected" for e in w4.events))

    print("== determinism ==")
    outs = []
    for _ in range(2):
        cfg_n = race_cfg()
        cfg_n["noise"] = dict(simconfig.NOISE_PROFILES["default_noisy"])
        wd = World(cfg_n, tr, spawn_theta=tr.start_pose[2])
        d2 = PurePursuit(tr, 1.5, 2.0)
        for _ in range(50 * 30):
            d2.act(wd)
            wd.step()
        outs.append((round(wd.x, 6), round(wd.y, 6), wd.slide_ticks,
                     wd.imu_frame(), wd.lidar_frame()))
    check("noisy runs replay identically", outs[0] == outs[1])


def _write_demo(out, tr, cfg, log, w):
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "maze.json"), "w") as f:
        json.dump(tr.to_dict(), f)
    with open(os.path.join(out, "ground_truth.jsonl"), "w") as f:
        for r in log:
            f.write(json.dumps(r) + "\n")
    simconfig.dump_resolved(cfg, os.path.join(out, "resolved_config.json"))
    with open(os.path.join(out, "transcript.jsonl"), "w") as f:
        f.write(json.dumps(dict(type="meta", arm="A", labels="off",
                                model="mock:pure-pursuit",
                                maze_hash=tr.hash(),
                                noise_profile="clean", ts=0)) + "\n")
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(dict(solved=w.goal_reached, goal_tick=w.goal_tick,
                       end_reason="solved", lap=w.lap, laps=w.laps,
                       best_lap_s=w.best_lap, slide_ticks=w.slide_ticks,
                       collisions=w.collision_count, turns=0,
                       tokens={"output": 0}), f)
    print(f"  demo episode written to {out}")


def end_to_end():
    print("== end-to-end daemon (port %d) ==" % PORT)
    import shutil
    import urllib.request
    scratch = os.environ.get("TRACK_CHECK_DIR", "/tmp/track_check")
    shutil.rmtree(scratch, ignore_errors=True)
    run_dir = os.path.join(scratch, "run")
    devfs = os.path.join(scratch, "devfs")
    os.makedirs(run_dir)
    cfg = race_cfg(sim={"api_port": PORT, "realtime_factor": 4.0})
    cfg_path = os.path.join(run_dir, "daemon_config.json")
    simconfig.dump_resolved(cfg, cfg_path)
    proc = subprocess.Popen(
        [sys.executable, "-m", "sim.daemon", "--config", cfg_path,
         "--run-dir", run_dir, "--devfs", devfs, "--port", str(PORT)],
        cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    def get(path):
        with urllib.request.urlopen(
                f"http://127.0.0.1:{PORT}{path}", timeout=2) as r:
            return json.loads(r.read())

    try:
        st = None
        for _ in range(100):
            try:
                st = get("/state")
                break
            except OSError:
                time.sleep(0.1)
        check("daemon up", st is not None)
        mz = get("/maze")
        check("/maze serves the track", mz.get("kind") == "track"
              and len(mz.get("left", [])) > 1000)
        with open(os.path.join(run_dir, "maze.json")) as f:
            check("maze.json is the track", json.load(f).get("kind") == "track")
        with open(os.path.join(run_dir, "device_map.json")) as f:
            m = json.load(f)["file_to_physical"]
        check("IMU and speed ports mapped, no encoders",
              "imu" in m.values() and "speed" in m.values()
              and "encoder_left" not in m.values(), str(sorted(m.values())))
        status = next(k for k, v in m.items() if v == "status")
        imu = next(k for k, v in m.items() if v == "imu")
        accel = next(k for k, v in m.items() if v == "accel")
        r = subprocess.run(["timeout", "2", "cat",
                            os.path.join(devfs, status)],
                           capture_output=True)
        line = r.stdout.decode().strip()
        check("status FIFO carries lap timing", "lap=0" in line
              and "best=0.0" in line, repr(line))
        pose0 = st["pose"]
        with open(os.path.join(devfs, accel), "w") as f:
            f.write("255\n")
        time.sleep(1.5)
        st = get("/state")
        moved = math.dist(pose0[:2], st["pose"][:2])
        check("accel over the FIFO moves the car", moved > 0.2,
              f"{moved:.2f} m")
        with open(os.path.join(devfs, accel), "w") as f:
            f.write("255\n")
        steer = next(k for k, v in m.items() if v == "steer")
        with open(os.path.join(devfs, steer), "w") as f:
            f.write("200\n")
        time.sleep(0.4)
        vals = None
        for _ in range(8):
            r = subprocess.run(["timeout", "2", "cat",
                                os.path.join(devfs, imu)],
                               capture_output=True)
            try:
                vals = [float(x) for x in
                        r.stdout.decode().strip().split(",")]
            except ValueError:
                vals = None
            if vals and len(vals) == 3 and (abs(vals[0]) > 0.05
                                             or abs(vals[2]) > 0.05):
                break
            time.sleep(0.1)
        check("IMU FIFO serves live accel/yaw during a steered launch",
              vals is not None and len(vals) == 3
              and (abs(vals[0]) > 0.05 or abs(vals[2]) > 0.05),
              str(vals))
        check("/state carries lap fields",
              st.get("lap") == 0 and st.get("laps") == []
              and "slide_ticks" in st)
        check("start pose faces the track direction",
              abs((pose0[2] - mz["start_pose"][2] + math.pi) % TWO_PI
                  - math.pi) < 1e-3)
    finally:
        proc.terminate()
        proc.wait(timeout=10)


if __name__ == "__main__":
    in_process()
    end_to_end()
    print("PASS" if not FAILS else f"FAILED: {FAILS}")
    sys.exit(1 if FAILS else 0)
