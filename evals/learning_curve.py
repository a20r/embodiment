"""Learning curve: time-to-solve per episode across a series.

    python3 -m evals.learning_curve --series <name>

Emits runs/<series>/evals/learning_curve.{json,csv,svg}.
"""

import argparse
import csv
import os

from evals import common


def run(series):
    eps = common.episodes(series)
    if not eps:
        print(f"no finished episodes in series {series!r}")
        return None
    rows = []
    for n, s, _dir in eps:
        tick_hz = 50
        rows.append({
            "episode": n,
            "solved": s.get("solved", False),
            "end_reason": s.get("end_reason"),
            "ticks_to_solve": s.get("goal_tick"),
            "sim_s_to_solve": round(s["goal_tick"] / tick_hz, 1)
            if s.get("goal_tick") else None,
            "wall_s": s.get("wall_s"),
            "turns": s.get("turns"),
            "execs": s.get("execs"),
            "restarts": s.get("restarts"),
            "collisions": s.get("collisions"),
            "tokens_out": (s.get("tokens") or {}).get("output"),
            "maze_hash": s.get("maze_hash"),
        })

    out_dir = common.eval_dir(series)
    with open(os.path.join(out_dir, "learning_curve.csv"), "w",
              newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    solved_pts = [(r["episode"], r["sim_s_to_solve"]) for r in rows
                  if r["sim_s_to_solve"] is not None]
    dnf = [(r["episode"], "DNF") for r in rows
           if r["sim_s_to_solve"] is None]
    common.svg_line_chart({"time to solve": solved_pts},
                          os.path.join(out_dir, "learning_curve.svg"),
                          title=f"Learning curve — {series}",
                          y_label="sim seconds to solve",
                          dnf_marks=dnf)

    solved = [r for r in rows if r["solved"]]
    summary = {
        "episodes": len(rows),
        "solved": len(solved),
        "solve_rate": round(len(solved) / len(rows), 2),
        "first_solve_sim_s": solved_pts[0][1] if solved_pts else None,
        "best_sim_s": min((p[1] for p in solved_pts), default=None),
        "last_sim_s": solved_pts[-1][1] if solved_pts else None,
        "improvement_first_to_last":
            round(solved_pts[0][1] - solved_pts[-1][1], 1)
            if len(solved_pts) >= 2 else None,
    }
    common.write_eval(series, "learning_curve", rows, summary)
    print(f"learning_curve: {len(rows)} episodes, "
          f"{summary['solve_rate'] * 100:.0f}% solved "
          f"-> {out_dir}/learning_curve.{{json,csv,svg}}")
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--series", required=True)
    args = ap.parse_args()
    run(args.series)


if __name__ == "__main__":
    main()
