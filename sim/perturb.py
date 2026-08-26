"""Perturbations: one-line config changes that alter the robot's world.

Perturbation state is cumulative across a series and stored in the series
directory (state.json).  Each perturbation is applied at an episode
boundary, either scheduled in config.yaml:

    perturbations:
      - {at_episode: 5, name: motor_swap}

or on demand via `botctl perturb <name>` (queued for the next episode).

  motor_swap    swap the wiring of the two motor channels
  maze_regen    new maze from the same seed family (family_index += 1)
  sensor_remap  re-wire which physical sensor feeds which device file
"""

import json
import os

PERTURBATIONS = ("motor_swap", "maze_regen", "sensor_remap")


def initial_state():
    return {
        "family_index": 0,
        "remap_index": 0,
        "motor_swapped": False,
        "applied": [],  # [(episode, name)]
    }


def state_path(series_dir):
    return os.path.join(series_dir, "state.json")


def load_state(series_dir):
    path = state_path(series_dir)
    if os.path.exists(path):
        with open(path) as f:
            return json.load(f)
    return initial_state()


def save_state(series_dir, state):
    with open(state_path(series_dir), "w") as f:
        json.dump(state, f, indent=2)


def apply(state, name, episode_index=None):
    if name not in PERTURBATIONS:
        raise ValueError(
            f"unknown perturbation {name!r}; known: {PERTURBATIONS}")
    if name == "motor_swap":
        state["motor_swapped"] = not state["motor_swapped"]
    elif name == "maze_regen":
        state["family_index"] += 1
    elif name == "sensor_remap":
        state["remap_index"] += 1
    state["applied"].append([episode_index, name])
    return state


def pending_path(series_dir):
    return os.path.join(series_dir, "pending_perturbations.json")


def queue(series_dir, name):
    """Queue a perturbation for the next episode (botctl perturb)."""
    if name not in PERTURBATIONS:
        raise ValueError(
            f"unknown perturbation {name!r}; known: {PERTURBATIONS}")
    path = pending_path(series_dir)
    pending = []
    if os.path.exists(path):
        with open(path) as f:
            pending = json.load(f)
    pending.append(name)
    os.makedirs(series_dir, exist_ok=True)
    with open(path, "w") as f:
        json.dump(pending, f)


def take_pending(series_dir):
    path = pending_path(series_dir)
    if not os.path.exists(path):
        return []
    with open(path) as f:
        pending = json.load(f)
    os.unlink(path)
    return pending


def due_for_episode(cfg, episode_index):
    """Perturbations scheduled in config for this episode (1-based)."""
    return [p["name"] for p in cfg.get("perturbations", [])
            if p.get("at_episode") == episode_index]
