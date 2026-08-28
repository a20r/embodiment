"""End-to-end smoke test: the scripted controller solves a maze through
the device files, from inside the airgapped container.

Variants (all run by default):

  labeled_clean    labels on,  clean profile     + determinism/reset checks
  labeled_noisy    labels on,  default_noisy     + noise-engaged checks
  unlabeled_clean  labels off, clean profile     + in-container discovery
  perturbations    motor_swap / maze_regen / sensor_remap plumbing checks

Run from the repo root:  python3 -m scripts.smoke [--variant NAME]
"""

import argparse
import json
import os
import sys
import time

from harness.container import BotContainer, build_image, image_exists
from harness.simproc import SimDaemonProc
from sim import config as simconfig

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SMOKE_OVERRIDES = {
    "maze": {"seed": 11, "width": 6, "height": 6, "braid": 0.0},
    "sim": {"realtime_factor": 4.0, "api_port": 8791},
}


def resolve(variant_overrides):
    o = simconfig.deep_merge(SMOKE_OVERRIDES, variant_overrides)
    return simconfig.resolve(overrides=o)


class Check:
    def __init__(self):
        self.failures = []

    def ok(self, cond, label):
        print(f"    {'PASS' if cond else 'FAIL'}  {label}")
        if not cond:
            self.failures.append(label)


def start_stack(name, cfg, perturb_state=None):
    import shutil
    run_dir = os.path.join(REPO, "runs", "smoke", name)
    if os.path.exists(run_dir):
        shutil.rmtree(run_dir)  # stale runs must not contaminate the log
    os.makedirs(run_dir)
    if perturb_state:
        cfg = dict(cfg, perturb_state=perturb_state)
    devfs = os.path.join(run_dir, "devfs")
    daemon = SimDaemonProc(cfg, run_dir, devfs, repo_root=REPO,
                           start_paused=True).start()
    box = BotContainer(f"mazebot-smoke-{name}",
                       {devfs: "/dev/robot"}, workdir="/")
    box.start()
    box.exec("mkdir -p /smoke", timeout_s=10)
    for script in ("wall_follower.py", "probe.py"):
        box.cp_in(os.path.join(REPO, "scripts", script), f"/smoke/{script}")
    daemon.resume()
    return daemon, box, run_dir


def parse_gt(run_dir):
    ticks, events, reads, writes = 0, [], {}, {}
    max_t = 0
    with open(os.path.join(run_dir, "ground_truth.jsonl")) as f:
        for line in f:
            rec = json.loads(line)
            if "pose" in rec and "event" not in rec:
                ticks += 1
                max_t = max(max_t, rec["t"])
            elif "event" in rec:
                events.append(rec)
                if rec["event"] == "read":
                    reads[rec["dev"]] = reads.get(rec["dev"], 0) + 1
                elif rec["event"] == "write":
                    writes[rec["dev"]] = writes.get(rec["dev"], 0) + 1
    return ticks, max_t, events, reads, writes


def run_follower(box, mapping=None, timeout_s=170):
    map_arg = json.dumps(mapping or {})
    cmd = (f"python3 /smoke/wall_follower.py --max-wall-s {timeout_s - 20} "
           f"--map '{map_arg}'")
    rc, out = box.exec(cmd, timeout_s=timeout_s)
    return rc, out.decode(errors="replace")


def variant_labeled(check, noisy):
    name = "labeled_noisy" if noisy else "labeled_clean"
    cfg = resolve({"labels": "on",
                   "noise_profile": "default_noisy" if noisy else "clean"})
    daemon, box, run_dir = start_stack(name, cfg)
    try:
        t0 = time.time()
        rc, out = run_follower(box)
        wall = time.time() - t0
        print(out.strip().splitlines()[-1] if out.strip() else "(no output)")
        state = daemon.get("/state")
        check.ok(rc == 0, f"{name}: follower exited 0 ({wall:.0f}s wall)")
        check.ok(state["goal_reached"], f"{name}: sim reports goal reached")
        stats = state["device_stats"]
        for dev in simconfig.SENSOR_DEVICES:
            check.ok(stats["reads"].get(dev, 0) > 0,
                     f"{name}: device {dev} served reads")
        for dev in simconfig.ACTUATOR_DEVICES:
            check.ok(stats["writes"].get(dev, 0) > 0,
                     f"{name}: device {dev} accepted writes")
        if noisy:
            drops = sum(stats["drops"].values())
            check.ok(drops > 0, f"{name}: dropped reads occurred ({drops})")
            check.ok(cfg["noise"]["motor_gain_right"] != 1.0,
                     f"{name}: motor asymmetry engaged")
            check.ok(cfg["noise"]["actuation_latency_ticks"] > 0,
                     f"{name}: actuation latency engaged")
        else:
            # Deterministic reset: same pose as start, goal cleared.
            daemon.post("/reset")
            st = daemon.get("/state")
            sx, sy = 0.25, 0.25  # start cell center for 0.5m cells
            check.ok(abs(st["pose"][0] - sx) < 1e-9
                     and abs(st["pose"][1] - sy) < 1e-9,
                     f"{name}: reset restores exact start pose")
            check.ok(not st["goal_reached"], f"{name}: reset clears goal")
    finally:
        box.stop()
        daemon.stop()

    if not noisy:
        # Determinism across daemon restarts: identical maze.
        h1 = maze_hash_for(cfg, "det_a")
        h2 = maze_hash_for(cfg, "det_b")
        check.ok(h1 == h2, f"{name}: same seed => identical maze ({h1})")
        # Ground-truth log integrity.
        ticks, max_t, events, reads, writes = parse_gt(
            os.path.join(REPO, "runs", "smoke", name))
        # A mid-run /reset restarts tick numbering, so records >= max tick.
        check.ok(ticks >= max_t and max_t > 0,
                 f"{name}: gt log has per-tick records ({ticks})")
        check.ok(any(e["event"] == "goal_reached" for e in events),
                 f"{name}: gt log contains goal_reached event")
        check.ok(len(reads) == len(simconfig.SENSOR_DEVICES)
                 and len(writes) == len(simconfig.ACTUATOR_DEVICES),
                 f"{name}: gt log captured reads+writes for all devices")


def maze_hash_for(cfg, name):
    run_dir = os.path.join(REPO, "runs", "smoke", name)
    daemon = SimDaemonProc(cfg, run_dir,
                           os.path.join(run_dir, "devfs"),
                           repo_root=REPO).start()
    try:
        return daemon.get("/maze")["hash"]
    finally:
        daemon.stop()


def variant_unlabeled(check):
    name = "unlabeled_clean"
    cfg = resolve({"labels": "off", "noise_profile": "clean"})
    daemon, box, run_dir = start_stack(name, cfg)
    try:
        rc, out = box.exec("python3 /smoke/probe.py /dev/robot",
                           timeout_s=90)
        stderr_info = "\n".join(
            ln for ln in out.decode(errors="replace").splitlines()
            if ln.startswith("#"))
        print(stderr_info)
        check.ok(rc == 0, f"{name}: probe completed")
        if rc != 0:
            return
        mapping = json.loads(
            [ln for ln in out.decode().splitlines()
             if ln.startswith("{")][-1])
        with open(os.path.join(run_dir, "device_map.json")) as f:
            truth = f.read()
        truth = json.loads(truth)["file_to_physical"]
        for logical in ("motor_left", "motor_right", "lidar", "status",
                        "heading", "encoder_left", "encoder_right"):
            found = mapping.get(logical)
            check.ok(found is not None and truth.get(found) == logical,
                     f"{name}: probe identified {logical} "
                     f"(-> {found})")
        rc, out = run_follower(box, mapping=mapping)
        print(out.strip().splitlines()[-1] if out.strip() else "")
        check.ok(rc == 0, f"{name}: follower solved via discovered map")
        check.ok(daemon.get("/state")["goal_reached"],
                 f"{name}: sim reports goal reached")
        files = sorted(truth.keys())
        check.ok(all(f.startswith("d") and f[1:].isdigit() for f in files),
                 f"{name}: device files are anonymous ({files[:4]}...)")
    finally:
        box.stop()
        daemon.stop()


def variant_perturbations(check):
    base = resolve({"labels": "on", "noise_profile": "clean"})

    # maze_regen: same seed family, different maze.
    h0 = maze_hash_for(base, "pert_maze0")
    h1 = maze_hash_for(dict(base, perturb_state={"family_index": 1}),
                       "pert_maze1")
    check.ok(h0 != h1, f"perturb: maze_regen changes maze ({h0} -> {h1})")

    # sensor_remap: binding table actually re-wires sensors.
    from devices.bridge import compute_bindings
    b0 = compute_bindings(base["maze"]["seed"], True, remap_index=0)
    b1 = compute_bindings(base["maze"]["seed"], True, remap_index=1)
    check.ok(b0 != b1, "perturb: sensor_remap changes device bindings")
    check.ok(all(b1[f] in simconfig.SENSOR_DEVICES
                 for f in simconfig.SENSOR_DEVICES),
             "perturb: sensor_remap only re-wires sensors")

    # motor_swap: writing the left motor file drives the right wheel.
    cfg = dict(base)
    daemon, box, run_dir = start_stack(
        "pert_swap", cfg, perturb_state={"motor_swapped": True,
                                         "family_index": 0,
                                         "remap_index": 0})
    try:
        script = (
            "e_r0=$(head -1 /dev/robot/encoder_right); "
            "e_l0=$(head -1 /dev/robot/encoder_left); "
            "echo 120 > /dev/robot/motor_left; sleep 1.2; "
            "echo 0 > /dev/robot/motor_left; "
            "e_r1=$(head -1 /dev/robot/encoder_right); "
            "e_l1=$(head -1 /dev/robot/encoder_left); "
            "echo RESULT $((e_r1 - e_r0)) $((e_l1 - e_l0))")
        rc, out = box.exec(script, timeout_s=30)
        line = [ln for ln in out.decode().splitlines()
                if ln.startswith("RESULT")]
        dr, dl = (int(x) for x in line[0].split()[1:3]) if line else (0, 0)
        check.ok(rc == 0 and dr > 50 and abs(dl) < 10,
                 f"perturb: motor_swap crosses channels "
                 f"(right enc moved {dr}, left {dl})")
    finally:
        box.stop()
        daemon.stop()


VARIANTS = {
    "labeled_clean": lambda c: variant_labeled(c, noisy=False),
    "labeled_noisy": lambda c: variant_labeled(c, noisy=True),
    "unlabeled_clean": variant_unlabeled,
    "perturbations": variant_perturbations,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variant", choices=sorted(VARIANTS), default=None)
    args = ap.parse_args()

    if not image_exists():
        print("building bot image...")
        build_image(REPO)

    check = Check()
    names = [args.variant] if args.variant else list(VARIANTS)
    for name in names:
        print(f"\n== smoke variant: {name} ==")
        try:
            VARIANTS[name](check)
        except Exception as e:
            check.ok(False, f"{name}: crashed: {e!r}")

    print(f"\n{'=' * 60}")
    if check.failures:
        print(f"SMOKE FAILED: {len(check.failures)} failure(s)")
        for f in check.failures:
            print(f"  - {f}")
        return 1
    print("SMOKE PASSED: the simulated world works end-to-end "
          "from inside the container.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
