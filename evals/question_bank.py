"""Quiz questions generated from run artifacts, with mechanical checkers.

Each question is {"id", "text", "expected", "checker", "tol"?}.  Checkers
are tolerant of phrasing but strict about facts; every expected answer
comes from ground truth the agent never saw directly (device_map.json,
maze.json, resolved_config.json).
"""

import re


def _num(answer):
    m = re.search(r"-?\d+\.?\d*", answer.replace(",", ""))
    return float(m.group()) if m else None


def check(question, answer):
    answer = (answer or "").strip()
    if not answer or answer.upper().startswith("UNKNOWN"):
        return False
    kind = question["checker"]
    exp = question["expected"]
    low = answer.lower()
    if kind == "token":
        # devices may be cited with or without the /dev/robot/ prefix
        token = exp.lower()
        return bool(re.search(
            rf"(^|[^a-z0-9_]){re.escape(token)}($|[^a-z0-9_])", low))
    if kind == "number":
        val = _num(answer)
        return val is not None and abs(val - exp) <= question["tol"]
    if kind == "dimensions":
        nums = re.findall(r"\d+", answer)
        return len(nums) >= 2 and sorted(map(int, nums[:2])) == \
            sorted(list(exp))
    if kind == "cell":
        nums = re.findall(r"-?\d+", answer)
        if len(nums) < 2:
            return False
        return [int(nums[0]), int(nums[1])] in exp
    if kind == "keyword":
        return any(k.lower() in low for k in exp)
    raise ValueError(f"unknown checker {kind}")


def build(device_map, maze, cfg):
    """device_map: file_to_physical dict; maze: maze.json dict."""
    phys_to_file = {v: k for k, v in
                    device_map["file_to_physical"].items()}
    robot = cfg["robot"]
    q = []

    def add(qid, text, checker, expected, tol=None):
        item = {"id": qid, "text": text, "checker": checker,
                "expected": expected}
        if tol is not None:
            item["tol"] = tol
        q.append(item)

    for logical, label in [
            ("motor_left", "the LEFT motor"),
            ("motor_right", "the RIGHT motor"),
            ("lidar", "the range/lidar sensor"),
            ("heading", "the heading/orientation sensor"),
            ("status", "the status readout that reports goal progress")]:
        add(f"dev_{logical}",
            f"Which device file under /dev/robot is {label}? "
            f"(give the file name)",
            "token", phys_to_file[logical])

    add("lidar_beams",
        "How many range beams does the lidar return per reading?",
        "number", cfg["lidar"]["rays"], tol=0)
    add("beam0_dir",
        "Which direction does the first lidar beam point, relative to "
        "the robot?",
        "keyword", ["forward", "front", "ahead", "straight"])
    add("pwm_sign",
        "Does a positive motor PWM value drive a wheel forward or "
        "backward?",
        "keyword", ["forward", "ahead"])
    add("goal_signal",
        "How can you tell, from the devices, that the robot has reached "
        "the goal?",
        "keyword", ["goal=1", "goal = 1",
                    phys_to_file["status"], "goal flag", "goal field"])
    add("maze_dims",
        "How many cells wide and tall is the maze?",
        "dimensions", (maze["width"], maze["height"]))
    add("corridor_width",
        "Approximately how wide are the maze corridors, in meters?",
        "number", maze["cell_size"], tol=0.3 * maze["cell_size"])
    if maze.get("dead_ends"):
        add("dead_end",
            "Using a grid where the robot's starting cell is (0,0) and "
            "+x is the robot's initial heading, give the (col,row) of "
            "one dead-end cell in the maze.",
            "cell", maze["dead_ends"])
    add("wheelbase",
        "Approximately how far apart are the two wheels (track width), "
        "in meters?",
        "number", robot["wheelbase"], tol=0.5 * robot["wheelbase"])
    add("enc_ticks",
        "Approximately how many encoder ticks correspond to one wheel "
        "revolution?",
        "number", robot["encoder_ticks_per_rev"],
        tol=0.2 * robot["encoder_ticks_per_rev"])
    add("max_speed",
        "Approximately what is the robot's top wheel speed in m/s "
        "(at full PWM)?",
        "number", robot["max_speed"], tol=0.4 * robot["max_speed"])
    return q
