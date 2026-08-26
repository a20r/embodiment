"""Ablation runner: how much is a given memory file worth?

Snapshots the series' /memory, deletes or corrupts the target file, runs
one episode against it, and compares to a control episode run with the
intact memory.  Both runs happen in throwaway series directories; the
real series is never touched.

    python3 -m evals.ablate --series <name> --file notes.md
        [--mode delete|corrupt] [--model mock:wall-follower]

Writes runs/<series>/evals/ablation_<file>.json.
"""

import argparse
import json
import os
import random
import re
import shutil

from evals import common
from sim import perturb as perturb_mod


def _prep_series(src_series, tag, transform=None):
    """Clone memory (+ perturb state) into a throwaway series dir."""
    src = common.series_dir(src_series)
    dst = common.series_dir(f"{src_series}__{tag}")
    if os.path.exists(dst):
        shutil.rmtree(dst)
    os.makedirs(dst)
    mem_src = os.path.join(src, "memory")
    mem_dst = os.path.join(dst, "memory")
    if os.path.isdir(mem_src):
        shutil.copytree(mem_src, mem_dst)
    else:
        os.makedirs(mem_dst)
    state = perturb_mod.load_state(src)
    perturb_mod.save_state(dst, state)
    if transform:
        transform(mem_dst)
    return dst, state


def _run_one(cfg, series_name, state):
    from harness.episode import run_episode
    epcfg = dict(cfg)
    epcfg = json.loads(json.dumps(epcfg))  # deep copy
    epcfg["series"] = dict(epcfg.get("series", {}), name=series_name)
    epcfg["perturb_state"] = {k: state[k] for k in
                              ("family_index", "remap_index",
                               "motor_swapped")}
    return run_episode(epcfg, common.series_dir(series_name), 1)


def run(series, target_file, mode="delete", model=None):
    sj = common.load_json(os.path.join(common.series_dir(series),
                                       "series.json"))
    cfg = sj["config"]
    if model:
        cfg = dict(cfg, model=model)

    def ablation(mem_dir):
        path = os.path.join(mem_dir, target_file)
        if not os.path.exists(path):
            raise SystemExit(
                f"memory file {target_file!r} does not exist in "
                f"{series}/memory")
        if mode == "delete":
            os.unlink(path)
        else:  # corrupt: deterministic line shuffle
            with open(path, errors="replace") as f:
                lines = f.readlines()
            random.Random(1337).shuffle(lines)
            with open(path, "w") as f:
                f.writelines(lines)

    safe = re.sub(r"[^\w.-]", "_", target_file)
    print(f"ablation control run (intact memory)...")
    ctl_dir, state = _prep_series(series, "ablate_control")
    ctl = _run_one(cfg, f"{series}__ablate_control", state)
    print(f"ablation treatment run ({mode} {target_file})...")
    trt_dir, state2 = _prep_series(series, f"ablate_{safe}",
                                   transform=ablation)
    trt = _run_one(cfg, f"{series}__ablate_{safe}", state2)

    def t(s):
        return round(s["goal_tick"] / 50.0, 1) if s.get("goal_tick") \
            else None
    rows = [
        {"run": "control", "solved": ctl["solved"],
         "sim_s_to_solve": t(ctl), "turns": ctl["turns"],
         "collisions": ctl["collisions"],
         "end_reason": ctl["end_reason"]},
        {"run": f"{mode}:{target_file}", "solved": trt["solved"],
         "sim_s_to_solve": t(trt), "turns": trt["turns"],
         "collisions": trt["collisions"],
         "end_reason": trt["end_reason"]},
    ]
    regression = None
    if t(ctl) is not None and t(trt) is not None:
        regression = round(t(trt) - t(ctl), 1)
    summary = {"file": target_file, "mode": mode,
               "regression_sim_s": regression,
               "control_solved": ctl["solved"],
               "ablated_solved": trt["solved"]}
    common.write_eval(series, f"ablation_{safe}", rows, summary)
    print(json.dumps(summary, indent=2))
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--series", required=True)
    ap.add_argument("--file", required=True,
                    help="path relative to /memory")
    ap.add_argument("--mode", choices=["delete", "corrupt"],
                    default="delete")
    ap.add_argument("--model", default=None,
                    help="override model for the ablation episodes")
    args = ap.parse_args()
    run(args.series, args.file, args.mode, args.model)


if __name__ == "__main__":
    main()
