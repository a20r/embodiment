"""Generate a self-contained replay page for a duo (two-bot) episode.

Usage: python scripts/make_duo_replay.py <series> <ep_dir> <out.html> [title]
"""
import json
import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ep_dir = os.path.join(REPO, "runs", sys.argv[1], sys.argv[2])
out_path = sys.argv[3]
title = sys.argv[4] if len(sys.argv) > 4 else "Mazebot Duo"

maze = json.load(open(os.path.join(ep_dir, "maze.json")))
try:
    rc = json.load(open(os.path.join(ep_dir, "resolved_config.json")))
    comms_range = rc.get("duo", {}).get("comms_range", 0.8)
except OSError:
    comms_range = 0.8

BOTS = ("a", "b")
bots = {}
comms = []
for bid in BOTS:
    poses, colls, goal_tick = {}, [], None
    path = os.path.join(ep_dir, f"ground_truth_{bid}.jsonl")
    for line in open(path):
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            continue
        if "event" in r:
            ev = r["event"]
            if ev == "collision":
                colls.append(r.get("t"))
            elif ev == "goal_reached":
                goal_tick = r.get("t")
            elif ev == "comms_tx":
                comms.append({"t": r.get("t") or 0, "from": bid,
                              "line": (r.get("line") or "")[:200],
                              "ok": bool(r.get("delivered")),
                              "dist": r.get("dist")})
        elif "pose" in r:
            poses[r["t"]] = [round(r["pose"][0], 3),
                             round(r["pose"][1], 3),
                             round(r["pose"][2], 3), r.get("col", 0)]
    bots[bid] = {"poses": poses, "collisions": colls,
                 "goal_tick": goal_tick}

ticks = sorted(set(bots["a"]["poses"]) & set(bots["b"]["poses"]))
step = max(1, len(ticks) // 4000)
frames = [[t] + bots["a"]["poses"][t] + bots["b"]["poses"][t]
          for t in ticks[::step]]
comms.sort(key=lambda c: c["t"])

# A chatty controller can radiate tens of thousands of lines; the page
# collapses consecutive repeats and keeps every worded line, then
# downsamples the numeric chatter to keep the DOM sane.  Totals are
# preserved separately for the header chips.
tx_total = len(comms)
tx_delivered = sum(1 for c in comms if c["ok"])
runs = []
for c in comms:
    prev = runs[-1] if runs else None
    if prev and prev["from"] == c["from"] and prev["line"] == c["line"] \
            and prev["ok"] == c["ok"]:
        prev["n"] += 1
    else:
        runs.append(dict(c, n=1))
for r in runs:
    r["w"] = bool(re.search(r"[A-Za-z]{2,}", r["line"]))
CAP = 1400
worded = [r for r in runs if r["w"]]
numeric = [r for r in runs if not r["w"]]
keep_n = max(0, CAP - len(worded))
if len(numeric) > keep_n and keep_n > 0:
    stride = len(numeric) / keep_n
    numeric = [numeric[int(i * stride)] for i in range(keep_n)]
elif keep_n == 0:
    numeric = []
comms = sorted(worded + numeric, key=lambda c: c["t"])
for c in comms:
    c.pop("w", None)


def load_transcript(bid):
    items, meta, turns, usage_out = [], {}, 0, 0
    path = os.path.join(ep_dir, f"transcript_{bid}.jsonl")
    if not os.path.exists(path):
        return items, meta, turns, usage_out
    for line in open(path):
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            continue
        t = r.get("type")
        if t == "meta":
            meta = r
        elif t == "assistant":
            turns += 1
            usage_out += r.get("usage", {}).get("output", 0)
            for b in r.get("content", []):
                if not isinstance(b, dict):
                    continue
                if b.get("type") == "text" and b.get("text"):
                    items.append({"k": "agent", "t": b["text"][:1200]})
                elif b.get("type") == "tool_use":
                    items.append({"k": "cmd", "t": (b.get("input") or {})
                                  .get("command", "")[:600]})
        elif t == "exec_result":
            items.append({"k": "out", "t": r.get("output", "")[:700]})
        elif t == "note":
            items.append({"k": "note", "t": f"{r.get('kind')}" +
                          (f": {r.get('reason')}"
                           if r.get("reason") else "")})
    return items, meta, turns, usage_out


def load_memory(bid):
    memory = {}
    snap = os.path.join(ep_dir, f"memory_snapshot_{bid}")
    live = os.path.join(os.path.dirname(ep_dir), f"memory_{bid}")
    src = snap if os.path.isdir(snap) and os.listdir(snap) else live
    if os.path.isdir(src):
        for root, _d, files in os.walk(src):
            for name in sorted(files):
                rel = os.path.relpath(os.path.join(root, name), src)
                with open(os.path.join(root, name),
                          errors="replace") as f:
                    memory[rel] = f.read(20000)
    return memory


summary = {}
spath = os.path.join(ep_dir, "summary.json")
if os.path.exists(spath):
    summary = json.load(open(spath))

meta = {}
data_bots = {}
for bid in BOTS:
    items, m, turns, usage_out = load_transcript(bid)
    meta = m or meta
    s = (summary.get("bots") or {}).get(bid, {})
    data_bots[bid] = {
        "collisions": bots[bid]["collisions"],
        "goal_tick": bots[bid]["goal_tick"],
        "transcript": items,
        "memory": load_memory(bid),
        "stats": {"turns": turns,
                  "tokens_out": s.get("tokens", {}).get("output",
                                                        usage_out),
                  "end_reason": s.get("end_reason"),
                  "solved": bool(s.get("solved")
                                 or bots[bid]["goal_tick"])},
    }

first_contact = next((c["t"] for c in comms if c["ok"]), None)
data = {
    "tx_total": tx_total,
    "tx_delivered": tx_delivered,
    "maze": {k: maze.get(k) for k in
             ("width", "height", "cell_size", "segments", "start_cell",
              "spawn_b_cell", "goal_cell", "hash")},
    "frames": frames,
    "bots": data_bots,
    "comms": comms,
    "range": comms_range,
    "first_contact": first_contact,
    "meta": {"model": meta.get("model"), "arm": meta.get("arm"),
             "labels": meta.get("labels"),
             "noise": meta.get("noise_profile"),
             "maze_hash": meta.get("maze_hash")},
    "live": not bool(summary),
}

tpl = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "duo_replay_template.html")).read()
tpl = tpl.replace("__TITLE__", title)
html = tpl.replace("__DATA__", json.dumps(data, separators=(",", ":"))
                   .replace("</", "<\\/"))
open(out_path, "w").write(html)
n_deliv = sum(1 for c in comms if c["ok"])
print(f"wrote {out_path} ({len(html)//1024} KB, {len(frames)} frames, "
      f"{len(comms)} tx / {n_deliv} delivered)")
