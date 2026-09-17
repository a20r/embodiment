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

    # TX status (duo.tx_status): the status port reports each write's
    # fate; the counter advances on busy writes too.
    cfg_t = duo_cfg(duo={"enabled": True, "tx_rate_hz": 1.0,
                         "tx_status": True})
    cfg_t["noise"] = dict(simconfig.NOISE_PROFILES["clean"])
    ta = World(cfg_t, maze, bot_id="a", spawn_cell=maze.start_cell)
    tb = World(cfg_t, maze, bot_id="b", spawn_cell=maze.spawn_b_cell)
    ta.set_peer(tb)
    tb.set_peer(ta)
    events = []
    ta.log = events.append
    check("tx=0:idle before any write", "tx=0:idle" in ta.status_frame(),
          ta.status_frame())
    tb.x, tb.y = ta.x + 0.5, ta.y
    ta.send_serial("hi")
    check("in range -> tx=1:ok", "tx=1:ok" in ta.status_frame())
    check("ok line reaches the peer", tb.serial_rx_frame() == "hi")
    check("receiver counts rx_received", tb.comms["rx_received"] == 1,
          str(tb.comms))
    ta.tick += 60
    tb.x, tb.y = ta.x + 5.0, ta.y
    ta.send_serial("far")
    check("out of range -> tx=2:lost", "tx=2:lost" in ta.status_frame())
    check("lost line is not queued", len(tb.serial_rx) == 0)
    ta.send_serial("too soon")          # inside the 50-tick window
    check("rate-capped -> tx=3:busy", "tx=3:busy" in ta.status_frame())
    check("busy counts as a rate drop, not a tx",
          ta.comms["tx_rate_dropped"] == 1 and ta.comms["tx"] == 2,
          str(ta.comms))
    ta.tick += 60
    tb.x, tb.y = ta.x + 0.5, ta.y
    ta.send_serial("again")
    check("re-armed write -> tx=4:ok", "tx=4:ok" in ta.status_frame())
    check("re-armed line delivered", tb.serial_rx_frame() == "again")
    tx_ev = [e for e in events if e.get("event") == "comms_tx"]
    check("comms_tx carries seq and outcome",
          [(e.get("seq"), e.get("outcome")) for e in tx_ev]
          == [(1, "ok"), (2, "lost"), (4, "ok")], str(tx_ev))
    check("busy writes are not comms_tx events", len(tx_ev) == 3)
    check("snapshot exposes tx_last", ta.snapshot()["tx_last"] == [4, "ok"])
    plain = []
    wr.log = plain.append
    wr.tick += 60
    wr.send_serial("plain")
    check("flag off: status has no tx field",
          "tx=" not in wr.status_frame(), wr.status_frame())
    check("flag off: comms_tx has no seq/outcome",
          plain and plain[-1]["event"] == "comms_tx"
          and "seq" not in plain[-1] and "outcome" not in plain[-1],
          str(plain[-1:]))
    check("solo status has no tx field", "tx=" not in solo_w.status_frame())

    # Blocking-RX primitives: delivery raises the event, peek does not
    # consume, commit does.
    tb.rx_event.clear()
    ta.tick += 60
    ta.send_serial("peekme")
    check("delivery sets the receiver's rx_event", tb.rx_event.is_set())
    check("peek returns the line without consuming it",
          tb.serial_rx_peek() == "peekme"
          and tb.serial_rx_peek() == "peekme" and len(tb.serial_rx) == 1)
    tb.serial_rx_commit()
    check("commit consumes and counts the read",
          tb.serial_rx_peek() is None and tb.comms["rx_read"] == 3,
          str(tb.comms))
    tb.serial_rx_commit()
    check("commit on an empty queue is a no-op",
          tb.comms["rx_read"] == 3)
    tb.serial_rx.append("served")
    tb.serial_rx.append("newer")
    tb.serial_rx_commit("evicted")     # not the head: burst evicted it
    check("commit of an evicted line pops nothing",
          list(tb.serial_rx) == ["served", "newer"]
          and tb.comms["rx_read"] == 3)
    tb.serial_rx_commit(tb.serial_rx_peek())
    check("commit of the served head pops exactly it",
          list(tb.serial_rx) == ["newer"] and tb.comms["rx_read"] == 4)
    tb.serial_rx.clear()

    # Config guards for the link-layer knobs.
    for k in ("tx_status", "rx_blocking", "rx_wakes_agent"):
        try:
            simconfig.resolve(None, overrides={"duo": {k: True}})
            check(f"duo.{k} without duo.enabled is rejected", False)
        except ValueError:
            check(f"duo.{k} without duo.enabled is rejected", True)
    try:
        simconfig.resolve(None, overrides={
            "duo": {"enabled": True, "together_window_s": 0}})
        check("together_window_s must be positive", False)
    except ValueError:
        check("together_window_s must be positive", True)

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


def _cat(path, timeout_s=1):
    """One read of a FIFO by an external reader, like the agent's
    `timeout N cat`; returns (exit code, stripped stdout)."""
    r = subprocess.run(["timeout", str(timeout_s), "cat", path],
                       capture_output=True)
    return r.returncode, r.stdout.decode().strip()


def _cat_until(path, pred, deadline_s=6):
    """Re-read (dropped-read noise is on) until pred(text) or timeout."""
    got = ""
    deadline = time.time() + deadline_s
    while time.time() < deadline:
        _, got = _cat(path, 1)
        if pred(got):
            return got
        time.sleep(0.1)
    return got


def end_to_end_flags():
    port = PORT + 1
    print("== end-to-end daemon, tx_status + rx_blocking (port %d) =="
          % port)
    import shutil
    import urllib.request
    scratch = os.environ.get("DUO_CHECK_DIR", "/tmp/duo_check") + "_flags"
    shutil.rmtree(scratch, ignore_errors=True)
    run_dir = os.path.join(scratch, "run")
    devfs = os.path.join(scratch, "devfs")
    os.makedirs(run_dir)
    rtf = 2.0
    cfg = duo_cfg(duo={"enabled": True, "comms_range": 100.0,
                       "objective": "together", "tx_status": True,
                       "rx_blocking": True},
                  sim={"api_port": port, "realtime_factor": rtf})
    cfg_path = os.path.join(run_dir, "daemon_config.json")
    simconfig.dump_resolved(cfg, cfg_path)
    proc = subprocess.Popen(
        [sys.executable, "-m", "sim.daemon", "--config", cfg_path,
         "--run-dir", run_dir, "--devfs", devfs, "--port", str(port)],
        cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    def state():
        with urllib.request.urlopen(
                f"http://127.0.0.1:{port}/state", timeout=2) as r:
            return json.loads(r.read())

    try:
        st = None
        for _ in range(100):
            try:
                st = state()
                break
            except OSError:
                time.sleep(0.1)
        check("flags daemon up", st is not None)
        want = int(cfg["duo"]["together_window_s"] * cfg["sim"]["tick_hz"]
                   * rtf)
        check("together window is wall seconds (scaled by rtf)",
              st.get("joint_window_ticks") == want,
              f"{st.get('joint_window_ticks')} vs {want}")
        with open(os.path.join(run_dir, "device_map.json")) as f:
            m = json.load(f)["file_to_physical"]
        tx = next(k for k, v in m.items() if v == "serial_tx")
        rx = next(k for k, v in m.items() if v == "serial_rx")
        status = next(k for k, v in m.items() if v == "status")
        a_tx = os.path.join(devfs, "a", tx)
        a_st = os.path.join(devfs, "a", status)
        b_rx = os.path.join(devfs, "b", rx)

        got = _cat_until(a_st, lambda s: "tx=" in s)
        check("status shows tx=0:idle before any write", "tx=0:idle" in got,
              repr(got))
        code, out = _cat(b_rx, 1)
        check("blocking RX: empty queue blocks the reader (timeout, "
              "no output)", code == 124 and out == "", f"{code} {out!r}")

        with open(a_tx, "w") as f:
            f.write("ping over fifo\n")
        got = _cat_until(a_st, lambda s: "tx=1:" in s)
        check("status reports tx=1:ok after the write", "tx=1:ok" in got,
              repr(got))
        got = _cat_until(b_rx, lambda s: "ping" in s)
        check("blocking RX yields the line once it arrives",
              got == "ping over fifo", repr(got))
        code, out = _cat(b_rx, 1)
        check("blocking RX: drained queue blocks again",
              code == 124 and out == "", f"{code} {out!r}")

        # A reader that gave up consumed nothing: the next line waits
        # for the next reader.
        with open(a_tx, "w") as f:
            f.write("second\n")
        time.sleep(0.5)
        got = _cat_until(b_rx, lambda s: s == "second")
        check("line written with no reader waits for the next reader",
              got == "second", repr(got))
        st = state()
        ca = st["bots"][0]
        cb = st["bots"][1]["comms"]
        check("blocking RX accounting: rx_read == lines returned",
              cb["rx_read"] == 2 and cb["rx_received"] == 2, str(cb))
        check("snapshot tx_last follows the writes",
              ca["tx_last"] == [2, "ok"] and ca["comms"]["tx"] == 2,
              str(ca["tx_last"]))
        # The blocking thread may be parked in open(); teardown must
        # not hang on it, and the GT log is complete only after stop.
        t0 = time.time()
        proc.terminate()
        proc.wait(timeout=10)
        check("teardown with a blocking RX thread is prompt",
              time.time() - t0 < 5, f"{time.time() - t0:.1f}s")
        gt_b = os.path.join(run_dir, "ground_truth_b.jsonl")
        with open(gt_b) as f:
            reads = [json.loads(l) for l in f if '"event":"read"' in l
                     and '"physical":"serial_rx"' in l]
        check("blocking RX reads are ground-truth logged",
              [r["value"] for r in reads] == ["ping over fifo", "second"],
              str([r.get("value") for r in reads]))
    finally:
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=10)


def wake_loop():
    """duo.rx_wakes_agent in harness/duo.py's per-bot loop, driven by a
    stub model that always ends its turn and a stub daemon whose
    rx_received counter can be advanced; no docker, no tokens."""
    print("== harness: rx_wakes_agent loop (stub model/daemon) ==")
    import shutil
    import tempfile
    from harness import duo as hd
    from harness import llm

    class StubModel:
        def create(self, system, messages, max_tokens=16000):
            return llm.Response(
                content=[llm.Block(type="text", text="done")],
                stop_reason="end_turn",
                usage=llm.Usage(input_tokens=10, output_tokens=5))

        @staticmethod
        def serialize_content(content):
            return [{"type": "text", "text": b.text} for b in content]

    class StubDaemon:
        def __init__(self, rx_after_s=None):
            self.t0 = time.time()
            self.rx_after = rx_after_s

        def get(self, path):
            if path == "/maze":
                return {"hash": "stub"}
            rx = int(self.rx_after is not None
                     and time.time() - self.t0 > self.rx_after)
            bot = {"goal_reached": False, "goal_tick": None, "tick": 1,
                   "sim_time_s": 0.0, "collision_count": 0,
                   "comms": {"tx": 0, "tx_delivered": 0, "rx_read": 0,
                             "tx_rate_dropped": 0, "rx_received": rx}}
            return {"tick": 1, "bots": [bot, dict(bot)]}

    class StubBox:
        def exec(self, command, timeout_s=60):
            return 0, ""

    real_make = hd.llm.make_model
    hd.llm.make_model = lambda m, r: StubModel()
    try:
        def run(duo_over, budget_over, rx_after):
            cfg = duo_cfg(duo={"enabled": True, **duo_over},
                          budget=budget_over)
            ep = tempfile.mkdtemp(prefix="wake_check_")
            try:
                return hd._run_bot(cfg, StubDaemon(rx_after), StubBox(),
                                   ep, "a", 0)
            finally:
                shutil.rmtree(ep, ignore_errors=True)

        # Flag off: three blind nudges, then agent_stopped (unchanged).
        s = run({}, {"max_wallclock_s": 60}, None)
        check("flag off: 3 nudges then agent_stopped",
              s["end_reason"] == "agent_stopped" and s["nudges"] == 4
              and s["wakes"] == 0 and s["turns"] == 4,
              f"{s['end_reason']} nudges={s['nudges']} turns={s['turns']}")
        # Flag on: the first pause ends when a line is delivered (wake,
        # no nudge counted); later pauses time out into the nudge path.
        t0 = time.time()
        s = run({"rx_wakes_agent": True, "rx_wake_timeout_s": 2},
                {"max_wallclock_s": 60}, 1.0)
        check("flag on: delivered line wakes the agent without a nudge",
              s["wakes"] == 1 and s["nudges"] == 4
              and s["end_reason"] == "agent_stopped" and s["turns"] == 5,
              f"wakes={s['wakes']} nudges={s['nudges']} "
              f"turns={s['turns']} {s['end_reason']}")
        check("wake waits for the line, timeouts bound the rest",
              8.0 < time.time() - t0 < 20.0, f"{time.time() - t0:.1f}s")
        # Wallclock inside a pause ends the episode as wallclock, not
        # as a stop, and costs no nudge.
        s = run({"rx_wakes_agent": True, "rx_wake_timeout_s": 60},
                {"max_wallclock_s": 3}, None)
        check("wallclock during a pause ends as wallclock",
              s["end_reason"] == "wallclock" and s["nudges"] == 0
              and s["wakes"] == 0, f"{s['end_reason']} {s['nudges']}")
    finally:
        hd.llm.make_model = real_make


if __name__ == "__main__":
    in_process()
    mission_mode()
    wake_loop()
    end_to_end()
    end_to_end_flags()
    print("PASS" if not FAILS else f"FAILED: {FAILS}")
    sys.exit(1 if FAILS else 0)
