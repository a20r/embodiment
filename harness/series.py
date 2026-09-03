"""Series runner: N episodes with persistent /memory and perturbations."""

import json
import os

from harness.episode import run_episode
from harness.container import image_exists, build_image
from sim import perturb

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def series_dir_for(cfg):
    return os.path.join(REPO, cfg.get("runs_dir", "runs"),
                        cfg["series"]["name"])


def completed_episodes(series_dir):
    done = []
    if not os.path.isdir(series_dir):
        return done
    for name in sorted(os.listdir(series_dir)):
        if name.startswith("ep_"):
            if os.path.exists(os.path.join(series_dir, name,
                                           "summary.json")):
                done.append(int(name.split("_")[1]))
    return done


def run_series(cfg, episodes=None, fresh=False):
    box = cfg.get("container") or {}
    image = box.get("image", "mazebot-bot")
    if not image_exists(image):
        print(f"building bot image {image}...")
        build_image(REPO, image, box.get("dockerfile", "Dockerfile.bot"))
    series_dir = series_dir_for(cfg)
    os.makedirs(series_dir, exist_ok=True)
    with open(os.path.join(series_dir, "series.json"), "w") as f:
        json.dump({"config": cfg}, f, indent=2)

    if fresh:
        import shutil
        for name in list(os.listdir(series_dir)):
            if name.startswith("ep_") or name in ("memory", "state.json"):
                path = os.path.join(series_dir, name)
                shutil.rmtree(path) if os.path.isdir(path) \
                    else os.unlink(path)

    total = episodes or cfg["series"]["episodes"]
    done = completed_episodes(series_dir)
    start = (max(done) + 1) if done else 1
    state = perturb.load_state(series_dir)

    if start > total:
        print(f"series already has {len(done)} episode(s); nothing to do "
              f"(use --fresh to start over)")
        return []

    summaries = []
    for ep in range(start, total + 1):
        due = perturb.due_for_episode(cfg, ep) \
            + perturb.take_pending(series_dir)
        for name in due:
            print(f"applying perturbation before episode {ep}: {name}")
            perturb.apply(state, name, episode_index=ep)
        perturb.save_state(series_dir, state)

        epcfg = dict(cfg)
        epcfg["perturb_state"] = {k: state[k] for k in
                                  ("family_index", "remap_index",
                                   "motor_swapped")}
        print(f"=== episode {ep}/{total} (arm {cfg['arm']}, "
              f"labels {cfg['labels']}, {cfg['noise_profile']}) ===")
        if cfg.get("duo", {}).get("enabled"):
            from harness.duo import run_duo_episode
            summary = run_duo_episode(epcfg, series_dir, ep)
            print(json.dumps(
                {bid: {k: s.get(k) for k in ("solved", "end_reason",
                                             "goal_tick", "turns")}
                 for bid, s in summary["bots"].items()}, indent=None))
        else:
            summary = run_episode(epcfg, series_dir, ep)
            print(json.dumps({k: summary[k] for k in
                              ("solved", "end_reason", "goal_tick",
                               "wall_s", "turns", "restarts")},
                             indent=None))
        summaries.append(summary)
    return summaries
