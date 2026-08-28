"""Configuration loading and resolution for the Mazebot platform.

A single config.yaml drives everything.  This module owns the defaults, the
named noise profiles, and the deep-merge resolution order:

    defaults  <-  named noise profile  <-  config.yaml  <-  CLI overrides

The fully resolved config is a plain dict; the daemon receives it as JSON so
there is exactly one resolution point (here) for every process.
"""

import copy
import json
import os

import yaml

DEFAULTS = {
    "model": "claude-fable-5",
    "arm": "A",              # A = scratch /memory, B = pre-seeded memory system
    "labels": "on",          # on = documented device names, off = d0..dN
    # standard = walled-maze framing | lost = "you are lost; the goal
    # will be obvious" (no maze, no robot morphology)
    "prompt_variant": "standard",
    # None = derived from labels (labeled/unlabeled); "minimal" = a
    # README that only says it's a robot with ports under /dev/robot;
    # "minimal_duo" additionally mentions the transceiver mechanics.
    "readme_variant": None,
    # Two robots in one world, each running its own agent, with a
    # proximity-gated serial link (TX/RX port pair per robot).
    "duo": {
        "enabled": False,
        "comms_range": 0.8,       # meters; out of range, TX vanishes
        "max_line_bytes": 256,    # per transmitted line
        "queue_depth": 64,        # undelivered RX lines kept
    },
    "noise_profile": "default_noisy",
    "maze": {
        "seed": 7,
        "width": 9,
        "height": 9,
        "cell_size": 0.5,    # meters
        "braid": 0.0,        # fraction of dead-ends opened into loops
        # grid = straight lattice walls, goal is an interior cell.
        # organic = wavy non-convex walls, nothing axis-aligned, and the
        #   goal is an opening in the far boundary (escape to solve).
        "style": "grid",
        "curviness": 1.0,    # organic only: 0..1 wall waviness
        # organic only: the exit is closed by a door that unlocks only
        # when the robot arrives carrying the key.  Adds a key object
        # (lidar-visible post, picked up by rolling over it) and an
        # anonymous signal-strength port that rises near the key.
        "locked": False,
    },
    "robot": {
        "radius": 0.09,             # disc radius, m
        "wheelbase": 0.16,          # wheel separation, m
        "wheel_radius": 0.03,       # m
        "max_speed": 0.35,          # wheel surface speed at |pwm|=255, m/s
        "encoder_ticks_per_rev": 360,
        # false = the robot has no wheel encoders at all (the ports do
        # not exist): state estimation must come from lidar/heading.
        "encoders": True,
        # Vehicle class.  "diffdrive" = two motor channels.  "car" =
        # kinematic bicycle: an accel/brake channel and a slew-limited
        # steering-angle channel; momentum + drag; encoders are
        # replaced by a signed speedometer port.
        "model": "diffdrive",
        "car": {
            "wheelbase": 0.12,        # m (front-rear axle distance)
            "steer_max_deg": 35.0,    # |steering angle| limit
            "steer_rate_deg_s": 120.0,  # steering actuator slew rate
            "accel_max": 0.4,         # m/s^2 at |cmd|=255
            "drag": 0.35,             # 1/s velocity decay (coasting)
            "v_max": 0.5,             # m/s forward
            "v_rev_max": 0.15,        # m/s reverse
        },
    },
    "lidar": {
        "rays": 16,
        "fov_deg": 360.0,
        "max_range": 3.0,    # m
    },
    "sim": {
        "tick_hz": 50,
        "realtime_factor": 1.0,   # sim-seconds per wall-second; 0 = unthrottled
        "api_host": "127.0.0.1",
        "api_port": 8787,
    },
    # Noise values here override the named profile (partial override allowed).
    "noise": {},
    "budget": {
        "max_context_tokens": 160000,       # end/restart episode past this
        "max_total_output_tokens": 120000,  # cumulative model output per episode
        "max_turns": 400,
        "max_wallclock_s": 1800,
        "on_context_full": "end",           # end | restart (bare restart)
        "exec_timeout_s": 60,               # per bash tool call
        "output_truncate_bytes": 20000,
    },
    "series": {
        "name": "dev",
        "episodes": 1,
    },
    # List of {"at_episode": N, "name": "motor_swap"|"maze_regen"|"sensor_remap"}
    "perturbations": [],
    "dashboard": {
        "host": "127.0.0.1",
        "port": 8080,
    },
    "runs_dir": "runs",
}

# All-off is the definition of "clean"; default_noisy is the shipped profile.
NOISE_PROFILES = {
    "clean": {
        "lidar_sigma_m": 0.0,          # gaussian range noise
        "lidar_dropout_p": 0.0,        # per-ray invalid return (-1.0)
        "heading_sigma_deg": 0.0,      # per-read gaussian
        "heading_drift_deg": 0.0,      # random-walk step std per tick
        "encoder_jitter_ticks": 0,     # +/- uniform jitter per read
        "slip_mu": 0.0,                # mean wheel slip fraction
        "slip_sigma": 0.0,             # slip std per wheel per tick
        "motor_gain_left": 1.0,        # asymmetry: effective motor strength
        "motor_gain_right": 1.0,
        "actuation_latency_ticks": 0,  # command takes effect N ticks later
        "dropped_read_p": 0.0,         # per device read: empty read served
        "beacon_sigma": 0.0,           # key signal-strength noise
        # constant gyro bias, degrees per minute of sim time; sign is
        # seeded per episode.  Integrated heading walks away over time.
        "heading_bias_deg_per_min": 0.0,
        "speed_sigma_ms": 0.0,         # speedometer noise (car model)
    },
    "default_noisy": {
        "lidar_sigma_m": 0.01,
        "lidar_dropout_p": 0.01,
        "heading_sigma_deg": 2.0,
        "heading_drift_deg": 0.002,
        "encoder_jitter_ticks": 1,
        "slip_mu": 0.03,
        "slip_sigma": 0.04,
        "motor_gain_left": 1.0,
        "motor_gain_right": 0.94,
        "actuation_latency_ticks": 3,
        "dropped_read_p": 0.02,
        "beacon_sigma": 0.008,
        "heading_bias_deg_per_min": 0.0,
        "speed_sigma_ms": 0.01,
    },
}

# Logical device set.  Sensors emit frames on read; actuators consume writes.
SENSOR_DEVICES = [
    "lidar",
    "heading",
    "encoder_left",
    "encoder_right",
    "bump_front",
    "bump_rear",
    "status",
]
ACTUATOR_DEVICES = ["motor_left", "motor_right"]
ALL_DEVICES = SENSOR_DEVICES + ACTUATOR_DEVICES


def deep_merge(base, override):
    out = copy.deepcopy(base)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def resolve(config_path=None, overrides=None):
    """Resolve config.yaml + overrides into a single flat dict."""
    user_cfg = {}
    if config_path and os.path.exists(config_path):
        with open(config_path) as f:
            user_cfg = yaml.safe_load(f) or {}
    cfg = deep_merge(DEFAULTS, user_cfg)
    if overrides:
        cfg = deep_merge(cfg, overrides)

    profile_name = cfg.get("noise_profile", "default_noisy")
    if profile_name not in NOISE_PROFILES:
        raise ValueError(
            f"unknown noise_profile {profile_name!r}; "
            f"known: {sorted(NOISE_PROFILES)}"
        )
    noise = deep_merge(NOISE_PROFILES[profile_name], cfg.get("noise", {}))
    cfg["noise"] = noise

    if cfg["labels"] not in ("on", "off", True, False):
        raise ValueError("labels must be 'on' or 'off'")
    cfg["labels"] = "on" if cfg["labels"] in ("on", True) else "off"
    if cfg["arm"] not in ("A", "B"):
        raise ValueError("arm must be 'A' or 'B'")
    if cfg["maze"].get("style", "grid") not in ("grid", "organic"):
        raise ValueError("maze.style must be 'grid' or 'organic'")
    if cfg["maze"].get("locked") and \
            cfg["maze"].get("style") != "organic":
        raise ValueError("maze.locked requires maze.style: organic")
    if cfg["prompt_variant"] not in ("standard", "lost"):
        raise ValueError("prompt_variant must be 'standard' or 'lost'")
    if cfg["robot"].get("model", "diffdrive") not in ("diffdrive", "car"):
        raise ValueError("robot.model must be 'diffdrive' or 'car'")
    return cfg


def device_sets(cfg):
    """(sensors, actuators) logical device lists for this config."""
    model = cfg["robot"].get("model", "diffdrive")
    if model == "car":
        sensors = ["lidar", "heading", "speed",
                   "bump_front", "bump_rear", "status"]
        actuators = ["accel", "steer"]
    else:
        sensors = list(SENSOR_DEVICES)
        if not cfg["robot"].get("encoders", True):
            sensors = [s for s in sensors
                       if s not in ("encoder_left", "encoder_right")]
        actuators = list(ACTUATOR_DEVICES)
    if cfg["maze"].get("locked"):
        sensors.append("beacon")
    if cfg.get("duo", {}).get("enabled"):
        sensors.append("serial_rx")
        actuators = list(actuators) + ["serial_tx"]
    return sensors, actuators


def load_resolved(path):
    with open(path) as f:
        return json.load(f)


def dump_resolved(cfg, path):
    with open(path, "w") as f:
        json.dump(cfg, f, indent=2, sort_keys=True)
