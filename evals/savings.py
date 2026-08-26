"""Savings test: after a perturbation, how fast does the agent re-learn
compared to its first exposure?

Clones the series (memory + perturb state) into a throwaway series,
applies the perturbation, runs K episodes, and compares the
time-to-solve trajectory against the original series' first K episodes.

    python3 -m evals.savings --series <name> --perturb motor_swap
        [--episodes 3] [--model ...]

Writes runs/<series>/evals/savings_<perturb>.json.
"""

import argparse
import json
import os

from evals import common
from evals.ablate import _prep_series, _run_one
from sim import perturb as perturb_mod


def run(series, perturbation, k=3, model=None):
    sj = common.load_json(os.path.join(common.series_dir(series),
                                       "series.json"))
    cfg = sj["config"]
    if model:
        cfg = dict(cfg, model=model)

    baseline = common.episodes(series)[:k]
    if not baseline:
        raise SystemExit(f"series {series!r} has no finished episodes")

    tag = f"savings_{perturbation}"
    tmp_name = f"{series}__{tag}"
    _tmp_dir, state = _prep_series(series, tag)
    perturb_mod.apply(state, perturbation, episode_index=0)
    perturb_mod.save_state(common.series_dir(tmp_name), state)

    def t(s):
        return round(s["goal_tick"] / 50.0, 1) if s.get("goal_tick") \
            else None

    after = []
    for ep in range(1, k + 1):
        print(f"savings run: episode {ep}/{k} after {perturbation}...")
        from harness.episode import run_episode
        epcfg = json.loads(json.dumps(cfg))
        epcfg["series"] = dict(epcfg.get("series", {}), name=tmp_name)
        epcfg["perturb_state"] = {kk: state[kk] for kk in
                                  ("family_index", "remap_index",
                                   "motor_swapped")}
        after.append(run_episode(epcfg, common.series_dir(tmp_name), ep))

    rows = []
    for i in range(k):
        base = baseline[i][1] if i < len(baseline) else None
        aft = after[i] if i < len(after) else None
        rows.append({
            "episode": i + 1,
            "first_exposure_sim_s": t(base) if base else None,
            "first_exposure_solved": base.get("solved") if base else None,
            "after_perturb_sim_s": t(aft) if aft else None,
            "after_perturb_solved": aft.get("solved") if aft else None,
        })
    base_total = sum(r["first_exposure_sim_s"] or 0 for r in rows)
    after_total = sum(r["after_perturb_sim_s"] or 0 for r in rows)
    savings_pct = None
    if base_total > 0 and all(r["after_perturb_solved"] for r in rows):
        savings_pct = round(100 * (base_total - after_total) / base_total,
                            1)
    summary = {"perturbation": perturbation, "episodes": k,
               "first_exposure_total_sim_s": round(base_total, 1),
               "after_perturb_total_sim_s": round(after_total, 1),
               "savings_pct": savings_pct}
    common.write_eval(series, f"savings_{perturbation}", rows, summary)

    pts_base = [(r["episode"], r["first_exposure_sim_s"]) for r in rows
                if r["first_exposure_sim_s"] is not None]
    pts_after = [(r["episode"], r["after_perturb_sim_s"]) for r in rows
                 if r["after_perturb_sim_s"] is not None]
    common.svg_line_chart(
        {"first exposure": pts_base, f"after {perturbation}": pts_after},
        os.path.join(common.eval_dir(series),
                     f"savings_{perturbation}.svg"),
        title=f"Savings — {perturbation}",
        y_label="sim seconds to solve")
    print(json.dumps(summary, indent=2))
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--series", required=True)
    ap.add_argument("--perturb", required=True,
                    choices=list(perturb_mod.PERTURBATIONS))
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--model", default=None)
    args = ap.parse_args()
    run(args.series, args.perturb, args.episodes, args.model)


if __name__ == "__main__":
    main()
