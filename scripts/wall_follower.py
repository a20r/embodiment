"""Scripted right-hand wall follower — the non-LLM smoke controller.

Runs INSIDE the bot container, stdlib only, and touches nothing but the
device files.  Proves the whole simulated world end-to-end: kinematics,
sensors, noise handling, maze solvability, goal signaling.

Usage:
    python3 wall_follower.py [--dev /dev/robot] [--map '{"lidar":"d3",...}']
                             [--max-wall-s 180] [--verbose]

--map maps logical names -> device file names (identity by default), so the
same controller drives labeled and unlabeled modes (the probe supplies the
map in hard mode).  Exit 0 = goal reached; 2 = timeout.
"""

import argparse
import json
import os
import sys
import time

MAX_RANGE = 3.0


def read_device(path, timeout_s=1.0):
    """One-shot read: returns the latest frame line, or None on timeout.

    Handles dropped reads (EOF with no data) by retrying within budget.
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            time.sleep(0.01)
            continue
        buf = b""
        try:
            inner = time.time() + 0.25
            while time.time() < inner:
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    time.sleep(0.003)
                    continue
                if chunk:
                    buf += chunk
                    if b"\n" in buf:
                        return buf.split(b"\n")[0].decode(errors="replace")
                else:
                    if buf:
                        return buf.decode(errors="replace")
                    time.sleep(0.003)
        finally:
            os.close(fd)
    return None


class Robot:
    def __init__(self, dev_dir, mapping):
        self.dev = dev_dir
        self.map = mapping
        self.motor_fds = {}
        self.last_scan = [MAX_RANGE] * 16

    def path(self, logical):
        return os.path.join(self.dev, self.map.get(logical, logical))

    def motors(self, left, right):
        for name, val in (("motor_left", left), ("motor_right", right)):
            p = self.path(name)
            if name not in self.motor_fds:
                self.motor_fds[name] = os.open(p, os.O_WRONLY)
            os.write(self.motor_fds[name], f"{int(val)}\n".encode())

    def stop(self):
        try:
            self.motors(0, 0)
        except OSError:
            pass

    def lidar(self):
        line = read_device(self.path("lidar"))
        if not line:
            return self.last_scan
        vals = []
        for i, tok in enumerate(line.split(",")):
            try:
                v = float(tok)
            except ValueError:
                v = -1.0
            if v < 0:  # dropout: reuse last good value
                v = self.last_scan[i] if i < len(self.last_scan) \
                    else MAX_RANGE
            vals.append(v)
        if len(vals) >= 8:
            self.last_scan = vals
        return self.last_scan

    def goal_reached(self):
        line = read_device(self.path("status"))
        return line is not None and "goal=1" in line

    def bump_front(self):
        line = read_device(self.path("bump_front"), timeout_s=0.4)
        return line is not None and line.strip() == "1"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/robot")
    ap.add_argument("--map", default="{}")
    ap.add_argument("--max-wall-s", type=float, default=180.0)
    ap.add_argument("--use-bump", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    bot = Robot(args.dev, json.loads(args.map))
    n_rays = len(bot.lidar())

    def ray(scan, frac):
        """Ray nearest to `frac` turns CCW from forward."""
        return scan[round(frac * n_rays) % n_rays]

    BASE = 90
    TURN = 100
    FRONT_MIN = 0.21
    FRONT_CLEAR = 0.32
    RIGHT_OPEN = 0.42
    TARGET_RIGHT = 0.17

    start = time.time()
    cycles = 0
    state = "follow"
    exercised = False
    while time.time() - start < args.max_wall_s:
        cycles += 1
        if cycles % 4 == 1 and bot.goal_reached():
            bot.stop()
            print(f"GOAL after {cycles} cycles, "
                  f"{time.time() - start:.1f}s wall")
            return 0
        if not exercised and cycles > 5:
            # Touch every documented sensor once (smoke coverage).
            for dev in ("heading", "encoder_left", "encoder_right",
                        "bump_front", "bump_rear"):
                read_device(bot.path(dev), timeout_s=0.5)
            exercised = True

        scan = bot.lidar()
        front = min(ray(scan, 15 / 16), ray(scan, 0.0), ray(scan, 1 / 16))
        right = ray(scan, 12 / 16)
        front_right = ray(scan, 14 / 16)

        bumped = args.use_bump and cycles % 3 == 0 and bot.bump_front()
        if bumped:
            bot.motors(-BASE, -BASE)
            time.sleep(0.25)
            bot.motors(-TURN, TURN)
            time.sleep(0.15)
            state = "follow"
            continue

        if front < FRONT_MIN:
            state = "turn_left"
        elif state == "turn_left" and front >= FRONT_CLEAR:
            state = "follow"

        if state == "turn_left":
            bot.motors(-TURN, TURN)  # rotate CCW in place
        elif right > RIGHT_OPEN and front_right > 0.30:
            bot.motors(BASE, int(BASE * 0.25))  # arc right to hug wall
        else:
            err = right - TARGET_RIGHT
            steer = max(-35, min(35, int(240 * err)))
            bot.motors(BASE + steer, BASE - steer)

        if args.verbose and cycles % 25 == 0:
            print(f"cyc={cycles} state={state} f={front:.2f} "
                  f"r={right:.2f} fr={front_right:.2f}", flush=True)
        time.sleep(0.02)

    bot.stop()
    print(f"TIMEOUT after {cycles} cycles")
    return 2


if __name__ == "__main__":
    sys.exit(main())
