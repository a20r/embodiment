"""Hard-mode device discovery — runs inside the container, labels off.

Given only a directory of anonymous device files (d0..dN), empirically
determines which is which, exactly the way the LLM agent must:

  1. Poll-read every file: files that produce data are sensors; files
     that never produce data are actuator candidates.
  2. Classify sensors by content shape: many CSV floats = lidar,
     'key=value' text = status, single decimal float = heading,
     bare integers = encoders/bumpers (disambiguated by actuation).
  3. Command the two actuator candidates in opposition (+120/-120): the
     robot spins in place.  The sign of the heading change identifies
     which channel is the right motor (heading is CCW-positive, so a
     CCW turn means the forward-driven wheel is the right one).
     Integer sensors that moved during the spin are encoders (the one
     counting up belongs to the forward-driven wheel); integer sensors
     that stayed in {0,1} are bump switches (front/rear left unresolved
     — the follower doesn't need them).
  4. Counter-spin to roughly restore heading, stop motors.

Prints a JSON mapping {logical_name: filename} on stdout.
Exit 0 on success, 3 if discovery failed.
"""

import json
import os
import sys
import time


def sample_lines(path, duration_s=0.45, max_lines=4):
    """Nonblocking poll-read; returns complete lines seen (may be [])."""
    lines = []
    deadline = time.time() + duration_s
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return lines
    buf = b""
    try:
        while time.time() < deadline and len(lines) < max_lines:
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                time.sleep(0.005)
                continue
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    lines.append(line.decode(errors="replace"))
            else:
                time.sleep(0.005)
    finally:
        os.close(fd)
    return lines


def read_one(path, timeout_s=1.5):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        lines = sample_lines(path, duration_s=0.3, max_lines=1)
        if lines:
            return lines[0]
    return None


def classify_shape(line):
    if "=" in line:
        return "status"
    if "," in line:
        toks = line.split(",")
        try:
            [float(t) for t in toks]
            if len(toks) >= 4:
                return "lidar"
        except ValueError:
            pass
        return "unknown"
    try:
        int(line)
        return "int"
    except ValueError:
        pass
    try:
        float(line)
        return "float"
    except ValueError:
        return "unknown"


def unwrapped_heading_delta(samples):
    """Total signed change across a heading sample list (degrees)."""
    total = 0.0
    for a, b in zip(samples, samples[1:]):
        d = (b - a + 180.0) % 360.0 - 180.0
        total += d
    return total


def main():
    dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/robot"
    files = sorted(os.listdir(dev))
    print(f"# probing {len(files)} devices in {dev}", file=sys.stderr)

    sensors = {}
    writables = []
    for name in files:
        path = os.path.join(dev, name)
        lines = sample_lines(path)
        if lines:
            sensors[name] = lines
        else:
            writables.append(name)
    print(f"# sensors={sorted(sensors)} writables={writables}",
          file=sys.stderr)
    if len(writables) != 2:
        print(f"# expected 2 writable devices, found {writables}",
              file=sys.stderr)
        return 3

    mapping = {}
    ints = []
    floats = []
    for name, lines in sensors.items():
        shape = classify_shape(lines[-1])
        if shape == "status":
            mapping["status"] = name
        elif shape == "lidar":
            mapping["lidar"] = name
        elif shape == "int":
            ints.append(name)
        elif shape == "float":
            floats.append(name)
    if len(floats) == 1:
        mapping["heading"] = floats[0]
    if "lidar" not in mapping or "status" not in mapping \
            or "heading" not in mapping:
        print(f"# classification incomplete: {mapping}", file=sys.stderr)
        return 3

    heading_path = os.path.join(dev, mapping["heading"])
    w0, w1 = (os.path.join(dev, w) for w in writables)

    def command(path, val):
        fd = os.open(path, os.O_WRONLY)
        os.write(fd, f"{val}\n".encode())
        os.close(fd)

    def snap_ints():
        out = {}
        for n in ints:
            line = read_one(os.path.join(dev, n))
            out[n] = int(line) if line is not None else 0
        return out

    before_ints = snap_ints()
    headings = []
    h0 = read_one(heading_path)
    if h0 is not None:
        headings.append(float(h0))

    # Spin: w0 forward, w1 backward.  Equal magnitudes -> rotation in
    # place, so this is safe in a corridor.
    command(w0, 120)
    command(w1, -120)
    t_end = time.time() + 0.9
    while time.time() < t_end:
        h = read_one(heading_path, timeout_s=0.4)
        if h is not None:
            headings.append(float(h))
    command(w0, 0)
    command(w1, 0)
    after_ints = snap_ints()

    # Counter-spin to roughly restore heading.
    command(w0, -120)
    command(w1, 120)
    time.sleep(0.9)
    command(w0, 0)
    command(w1, 0)

    dh = unwrapped_heading_delta(headings)
    print(f"# spin heading delta = {dh:.1f} deg "
          f"({len(headings)} samples)", file=sys.stderr)
    if abs(dh) < 10.0:
        print("# spin produced no clear heading change", file=sys.stderr)
        return 3
    if dh > 0:  # CCW: forward-driven wheel (w0) is the right wheel
        mapping["motor_right"], mapping["motor_left"] = writables
    else:
        mapping["motor_left"], mapping["motor_right"] = writables

    moved = {n: after_ints[n] - before_ints[n] for n in ints}
    encoders = [n for n in ints if abs(moved[n]) > 20]
    bumps = [n for n in ints if abs(moved[n]) <= 20]
    if len(encoders) == 2:
        # The encoder that counted up belongs to the forward-driven
        # wheel (w0), which we just identified as right or left.
        up = max(encoders, key=lambda n: moved[n])
        down = min(encoders, key=lambda n: moved[n])
        if dh > 0:
            mapping["encoder_right"], mapping["encoder_left"] = up, down
        else:
            mapping["encoder_left"], mapping["encoder_right"] = up, down
    for i, n in enumerate(bumps):
        mapping[f"bump_unknown_{i}"] = n

    print(json.dumps(mapping))
    return 0


if __name__ == "__main__":
    sys.exit(main())
