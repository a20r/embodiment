"""Host-side validation of the duo (two-bot) extension.

Part 1 exercises World/Maze in-process: spawn placement + clearance,
peer visibility on lidar, disc-disc collision, serial range gating,
byte cap, queue depth, and reset semantics.  Part 2 boots a real duo
daemon on an alternate port and pushes a line through the actual FIFO
transceiver a -> b.

Run from the repo root:  python scripts/duo_check.py
"""

import json
import math
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

from sim import config as simconfig       # noqa: E402
from sim.maze import Maze                 # noqa: E402
from sim.world import World, _seg_dist    # noqa: E402

PORT = 8798
FAILS = []


def check(name, ok, detail=""):
    print(f"  {'ok  ' if ok else 'FAIL'} {name}" +
          (f"  ({detail})" if detail else ""))
    if not ok:
        FAILS.append(name)


def duo_cfg(**over):
    cfg = simconfig.resolve(None, overrides={
        "labels": "off", "prompt_variant": "lost",
        "readme_variant": "minimal_duo",
        "duo": {"enabled": True},
        "maze": {"style": "organic", "seed": 41, "width": 9,
                 "height": 9, "curviness": 1.0},
        "sim": {"api_port": PORT},
        **over,
    })
    return cfg


def in_process():
    print("== in-process ==")
    cfg = duo_cfg()
    m = cfg["maze"]
    maze = Maze(m["seed"], m["width"], m["height"],
                cell_size=m["cell_size"], braid=m["braid"],
                style="organic", curviness=m["curviness"],
                robot_radius=cfg["robot"]["radius"], duo=True)
    check("spawn_b exists", maze.spawn_b_cell is not None)
    check("spawn_b differs from start",
          maze.spawn_b_cell != maze.start_cell)
    check("spawn_b differs from goal",
          maze.spawn_b_cell != maze.goal_cell)
    bx, by = maze.cell_center(maze.spawn_b_cell)
    clear = min(_seg_dist(bx, by, *s)[0] for s in maze.segments())
    check("spawn_b clearance", clear >= cfg["robot"]["radius"] + 0.04,
          f"clear={clear:.3f}")

    cfg["noise"] = dict(simconfig.NOISE_PROFILES["clean"])
    wa = World(cfg, maze, bot_id="a", spawn_cell=maze.start_cell)
    wb = World(cfg, maze, bot_id="b", spawn_cell=maze.spawn_b_cell,
               spawn_theta=math.pi)
    wa.set_peer(wb)
    wb.set_peer(wa)

    # Peer on lidar: park B 0.30 m dead ahead of A and read ray 0.
    wa.theta = 0.0
    wb.x, wb.y = wa.x + 0.30, wa.y
    r0 = wa.lidar_true()[0]
    expected = 0.30 - cfg["robot"]["radius"]
    check("peer visible on lidar", abs(r0 - expected) < 0.03,
          f"ray0={r0:.3f} expected~{expected:.3f}")
    # ...and gone from the scan once the peer moves away.
    far = maze.cell_center(maze.spawn_b_cell)
    wb.x, wb.y = far
    r0b = wa.lidar_true()[0]
    check("peer blip moves with peer", abs(r0b - r0) > 0.05,
          f"ray0 now {r0b:.3f}")

    # Disc-disc collision: drive A straight at B, expect a stop with
    # centers no closer than the two radii.
    wb.x, wb.y = wa.x + 0.30, wa.y
    wa.set_actuator("motor_left", 255)
    wa.set_actuator("motor_right", 255)
    for _ in range(400):
        wa.step()
    gap = math.hypot(wa.x - wb.x, wa.y - wb.y)
    check("robot-robot collision stops motion",
          gap >= 2 * cfg["robot"]["radius"] - 0.005, f"gap={gap:.3f}")
    check("collision registered", wa.collision_count >= 1)
    check("front bump on peer contact", wa.bump[0])
    wa.set_actuator("motor_left", 0)
    wa.set_actuator("motor_right", 0)

    # Serial: in range -> delivered; out of range -> vanishes.
    wb.x, wb.y = wa.x + 0.5, wa.y      # 0.5 < 0.8 comms_range
    wa.send_serial("hello?")
    check("in-range line delivered", wb.serial_rx_frame() == "hello?")
    check("rx then empty", wb.serial_rx_frame() == "")
    wb.x, wb.y = wa.x + 5.0, wa.y
    wa.send_serial("anyone there")
    check("out-of-range line vanishes", wb.serial_rx_frame() == "")
    check("tx counted", wa.comms["tx"] == 2
          and wa.comms["tx_delivered"] == 1, str(wa.comms))

    # Byte cap + queue depth.
    wb.x, wb.y = wa.x + 0.5, wa.y
    wa.send_serial("x" * 1000)
    got = wb.serial_rx_frame()
    check("line capped to max_line_bytes",
          len(got) == cfg["duo"]["max_line_bytes"], f"len={len(got)}")
    for i in range(100):
        wa.send_serial(f"m{i}")
    depth = len(wb.serial_rx)
    check("queue depth capped", depth == cfg["duo"]["queue_depth"],
          f"depth={depth}")
    check("oldest dropped first", wb.serial_rx_frame() ==
          f"m{100 - cfg['duo']['queue_depth']}")
    wb.reset()
    check("reset clears rx queue", len(wb.serial_rx) == 0)


def mission_mode():
    print("== mission mode (objective: together, peer_signal) ==")
    cfg = duo_cfg(duo={"enabled": True, "objective": "together",
                       "together_window_s": 60, "peer_signal": True})
    cfg["noise"] = dict(simconfig.NOISE_PROFILES["clean"])
    sensors, actuators = simconfig.device_sets(cfg)
    check("peer_signal in sensor set", "peer_signal" in sensors)
    m = cfg["maze"]
    maze = Maze(m["seed"], m["width"], m["height"],
                cell_size=m["cell_size"], style="organic",
                curviness=m["curviness"],
                robot_radius=cfg["robot"]["radius"], duo=True)
    wa = World(cfg, maze, bot_id="a", spawn_cell=maze.start_cell)
    wb = World(cfg, maze, bot_id="b", spawn_cell=maze.spawn_b_cell)
    wa.set_peer(wb)
    wb.set_peer(wa)

    # Signal strength: strictly decreasing with distance, in (0, 1].
    vals = []
    for d in (0.5, 1.0, 2.0, 4.0):
        wb.x, wb.y = wa.x + d, wa.y
        vals.append(float(wa.peer_signal_frame()))
    check("peer signal falls with distance",
          all(a > b for a, b in zip(vals, vals[1:])), str(vals))
    check("peer signal in range",
          all(0.0 < v <= 1.0 for v in vals))

    # No solo latch: a bot alone in the goal region does not complete.
    out_x = -0.5   # outside the maze bounding box
    wa.x, wa.y = out_x, 1.0
    wa.step()
    check("region entry tracked", wa.region_entry is not None)
    check("no solo completion", not wa.goal_reached)
    check("status shows here=1 in the goal region",
          "here=1" in wa.status_frame(), wa.status_frame())
    # Leaving the region clears the entry (a lapsed arrival must
    # re-cross).
    wa.x, wa.y = 1.0, 1.0
    wa.step()
    check("region exit clears entry", wa.region_entry is None)
    check("here clears on exit", "here=0" in wa.status_frame())
    solo_cfg = simconfig.resolve(None, overrides={
        "maze": {"style": "organic", "seed": 41}})
    solo_cfg["noise"] = dict(simconfig.NOISE_PROFILES["clean"])
    solo_w = World(solo_cfg, maze, bot_id="s")
    check("solo status has no here field",
          "here=" not in solo_w.status_frame())

    # The daemon's joint predicate: entries within the window fire the
    # latch on both; outside the window they do not.
    window = int(cfg["duo"]["together_window_s"]
                 * cfg["sim"]["tick_hz"])
    wa.x, wa.y = out_x, 1.0
    wa.step()
    for _ in range(window + 100):
        wb.tick += 1   # advance B's clock past the window
    wb.x, wb.y = out_x, 2.0
    wb.step()
    gap = abs(wa.region_entry - wb.region_entry)
    check("stale entries do not fire", gap > window, f"gap={gap}")
    # A re-crosses: exit, then re-enter close to B's entry time.
    wa.x, wa.y = 1.0, 1.0
    wa.step()
    wa.tick = wb.tick
    wa.x, wa.y = out_x, 1.0
    wa.step()
    gap = abs(wa.region_entry - wb.region_entry)
    check("re-entry refreshes the window", gap <= window, f"gap={gap}")
    if gap <= window:
        wa.set_joint_goal()
        wb.set_joint_goal()
    check("joint latch on both", wa.goal_reached and wb.goal_reached)
    check("joint latch zeroes motors",
          all(v == 0 for v in wa.cmd_eff.values())
          and all(v == 0 for v in wb.cmd_eff.values()))
    check("goal ticks recorded",
          wa.goal_tick is not None and wb.goal_tick is not None)

    # TX duty cycle: excess lines vanish silently; the window re-arms.
    cfg_r = duo_cfg(duo={"enabled": True, "tx_rate_hz": 1.0})
    cfg_r["noise"] = dict(simconfig.NOISE_PROFILES["clean"])
    wr = World(cfg_r, maze, bot_id="a", spawn_cell=maze.start_cell)
    ws = World(cfg_r, maze, bot_id="b", spawn_cell=maze.spawn_b_cell)
    wr.set_peer(ws)
    ws.set_peer(wr)
    ws.x, ws.y = wr.x + 0.5, wr.y
    for i in range(5):
        wr.send_serial(f"burst{i}")
    check("rate cap accepts first line only", wr.comms["tx"] == 1
          and wr.comms["tx_rate_dropped"] == 4, str(wr.comms))
    check("capped lines never reach the peer", len(ws.serial_rx) == 1)
    wr.tick += 60   # past the 50-tick window at 1 Hz
    wr.send_serial("after window")
    check("window re-arms", wr.comms["tx"] == 2, str(wr.comms))

    # Goal chamber: the space beyond the exit is walled in.
    mc = Maze(m["seed"], m["width"], m["height"],
              cell_size=m["cell_size"], style="organic",
              curviness=m["curviness"],
              robot_radius=cfg["robot"]["radius"], duo=True,
              goal_chamber=True)
    check("chamber adds 3 walls",
          len(mc._chamber_segments) == 3)
    x1, y1, x2, y2 = mc.exit_wall
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    if abs(x1 - x2) < 1e-9:
        ox, oy = (-1, 0) if x1 <= 1e-9 else (1, 0)
    else:
        ox, oy = (0, -1) if y1 <= 1e-9 else (0, 1)
    inside = (cx + ox * 0.35, cy + oy * 0.35)
    check("chamber interior is the goal region", mc.escaped(*inside))
    clear = min(_seg_dist(inside[0], inside[1], *s)[0]
                for s in mc.segments())
    check("chamber fits the robot",
          clear > cfg["robot"]["radius"] + 0.04, f"clear={clear:.3f}")
    behind = (cx + ox * 1.0, cy + oy * 1.0)
    near = min(_seg_dist(behind[0], behind[1], *s)[0]
               for s in mc.segments())
    check("back wall pens the robot in", near < 0.35,
          f"nearest={near:.3f}")
    check("chamber changes maze hash", mc.hash() != maze.hash())

    # Corner exit (7x7 seed 58 puts the exit at cell (0,6)): the chamber
    # must not overhang the maze corner, or its side wall meets nothing.
    mk = Maze(58, 7, 7, cell_size=0.5, style="organic", curviness=0.9,
              robot_radius=cfg["robot"]["radius"], duo=True,
              goal_chamber=True)
    # The chamber protrudes outward along the exit normal by design; the
    # clamp applies along the exit wall's own axis.
    span = 7 * 0.5
    ex1, ey1, ex2, ey2 = mk.exit_wall
    along = (1, 3) if abs(ex1 - ex2) < 1e-9 else (0, 2)
    inside_span = all(
        -1e-9 <= seg[i] <= span + 1e-9 for seg in mk._chamber_segments
        for i in along)
    check("corner-exit chamber stays within the maze span", inside_span,
          str(mk._chamber_segments))
    # The chamber's attach points must coincide with maze wall ends: the
    # nearest maze segment to each attach point is (nearly) touching.
    attach = [(mk._chamber_segments[0][0], mk._chamber_segments[0][1]),
              (mk._chamber_segments[2][2], mk._chamber_segments[2][3])]
    maze_segs = [s for s in mk.segments() if s not in mk._chamber_segments]
    gaps = [min(_seg_dist(px, py, *s)[0] for s in maze_segs)
            for px, py in attach]
    check("corner-exit chamber is sealed (attach gaps < robot diameter)",
          all(g < 2 * cfg["robot"]["radius"] for g in gaps),
          f"gaps={[round(g, 3) for g in gaps]}")


def end_to_end():
    print("== end-to-end daemon (port %d) ==" % PORT)
    import shutil
    import urllib.request
    scratch = os.environ.get("DUO_CHECK_DIR", "/tmp/duo_check")
    shutil.rmtree(scratch, ignore_errors=True)
    run_dir = os.path.join(scratch, "run")
    devfs = os.path.join(scratch, "devfs")
    os.makedirs(run_dir)
    # Huge comms range so the far-apart spawns still deliver.
    cfg = duo_cfg(duo={"enabled": True, "comms_range": 100.0})
    cfg_path = os.path.join(run_dir, "daemon_config.json")
    simconfig.dump_resolved(cfg, cfg_path)
    proc = subprocess.Popen(
        [sys.executable, "-m", "sim.daemon", "--config", cfg_path,
         "--run-dir", run_dir, "--devfs", devfs, "--port", str(PORT)],
        cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    try:
        state = None
        for _ in range(100):
            try:
                with urllib.request.urlopen(
                        f"http://127.0.0.1:{PORT}/state", timeout=2) as r:
                    state = json.loads(r.read())
                break
            except OSError:
                time.sleep(0.1)
        check("daemon up", state is not None)
        check("two bots in /state", len(state.get("bots", [])) == 2)
        pa = state["bots"][0]["pose"]
        pb = state["bots"][1]["pose"]
        check("distinct spawns",
              math.hypot(pa[0] - pb[0], pa[1] - pb[1]) > 1.0,
              f"d={math.hypot(pa[0] - pb[0], pa[1] - pb[1]):.2f}")

        names_a = sorted(os.listdir(os.path.join(devfs, "a")))
        names_b = sorted(os.listdir(os.path.join(devfs, "b")))
        check("same port names on both bots", names_a == names_b,
              ",".join(names_a))
        check("11 ports (8 sensors + 3 actuators)", len(names_a) == 11)
        with open(os.path.join(run_dir, "device_map.json")) as f:
            fmap = f.read()
        m = json.loads(fmap)["file_to_physical"]
        tx = next(k for k, v in m.items() if v == "serial_tx")
        rx = next(k for k, v in m.items() if v == "serial_rx")
        check("anonymous names", tx.startswith("d") and rx.startswith("d"),
              f"tx={tx} rx={rx}")

        with open(os.path.join(devfs, "a", tx), "w") as f:
            f.write("ping over fifo\n")
        got = ""
        deadline = time.time() + 5
        while time.time() < deadline and "ping" not in got:
            r = subprocess.run(
                ["timeout", "1", "cat", os.path.join(devfs, "b", rx)],
                capture_output=True)
            got = r.stdout.decode().strip()
            if not got:
                time.sleep(0.1)
        check("line crosses a->b over FIFOs", got == "ping over fifo",
              repr(got))
        with urllib.request.urlopen(
                f"http://127.0.0.1:{PORT}/state", timeout=2) as r:
            state = json.loads(r.read())
        ca = state["bots"][0]["comms"]
        cb = state["bots"][1]["comms"]
        check("comms accounting", ca["tx"] == 1
              and ca["tx_delivered"] == 1 and cb["rx_read"] >= 1,
              f"a={ca} b={cb}")
        gt_b = os.path.join(run_dir, "ground_truth_b.jsonl")
        check("per-bot GT logs", os.path.exists(gt_b))
    finally:
        proc.terminate()
        proc.wait(timeout=10)


if __name__ == "__main__":
    in_process()
    mission_mode()
    end_to_end()
    print("PASS" if not FAILS else f"FAILED: {FAILS}")
    sys.exit(1 if FAILS else 0)
