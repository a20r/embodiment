"""Generate a self-contained interactive replay page for a Mazebot episode."""
import json, os, sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ep_dir = os.path.join(REPO, "runs", sys.argv[1], sys.argv[2])
out_path = sys.argv[3]

maze = json.load(open(os.path.join(ep_dir, "maze.json")))

poses, coll_ticks = [], []
goal_tick = None
special = {}
for line in open(os.path.join(ep_dir, "ground_truth.jsonl")):
    try: r = json.loads(line)
    except: continue
    if "event" in r:
        if r["event"] == "collision": coll_ticks.append(r.get("t"))
        elif r["event"] == "goal_reached": goal_tick = r.get("t")
        elif r["event"] == "key_pickup": special["pickup_tick"] = r.get("t")
        elif r["event"] == "door_unlocked": special["unlock_tick"] = r.get("t")
    elif "pose" in r:
        poses.append([r["t"], round(r["pose"][0], 3), round(r["pose"][1], 3),
                      round(r["pose"][2], 3), r.get("col", 0)])
step = max(1, len(poses) // 4000)
poses = poses[::step]

transcript, meta, usage_out, turns = [], {}, 0, 0
for line in open(os.path.join(ep_dir, "transcript.jsonl")):
    try: r = json.loads(line)
    except: continue
    t = r.get("type")
    if t == "meta": meta = r
    elif t == "assistant":
        turns += 1
        usage_out += r.get("usage", {}).get("output", 0)
        for b in r.get("content", []):
            if not isinstance(b, dict): continue
            if b.get("type") == "text" and b.get("text"):
                transcript.append({"k": "agent", "t": b["text"][:1200]})
            elif b.get("type") == "tool_use":
                transcript.append({"k": "cmd",
                                   "t": (b.get("input") or {}).get("command", "")[:600]})
    elif t == "exec_result":
        transcript.append({"k": "out", "t": r.get("output", "")[:700]})
    elif t == "note":
        transcript.append({"k": "note",
                           "t": f"{r.get('kind')}" + (f": {r.get('reason')}" if r.get("reason") else "")})

memory = {}
snap = os.path.join(ep_dir, "memory_snapshot")
live_mem = os.path.join(os.path.dirname(ep_dir), "memory")
mem_src = snap if os.path.isdir(snap) and os.listdir(snap) else live_mem
if os.path.isdir(mem_src):
    for root, _d, files in os.walk(mem_src):
        for name in sorted(files):
            rel = os.path.relpath(os.path.join(root, name), mem_src)
            with open(os.path.join(root, name), errors="replace") as f:
                memory[rel] = f.read(20000)

summary = {}
spath = os.path.join(ep_dir, "summary.json")
if os.path.exists(spath): summary = json.load(open(spath))

data = {
    "maze": {k: maze.get(k) for k in ("width", "height", "cell_size",
                                   "segments", "start_cell", "goal_cell",
                                   "hash", "locked", "door_segments",
                                   "key_pos")},
    "poses": poses, "collisions": coll_ticks, "goal_tick": goal_tick,
    "special": special,
    "transcript": transcript, "memory": memory,
    "meta": {"model": meta.get("model"), "arm": meta.get("arm"),
             "labels": meta.get("labels"), "noise": meta.get("noise_profile"),
             "episode": meta.get("episode"), "maze_hash": meta.get("maze_hash")},
    "stats": {"turns": turns, "tokens_out": summary.get("tokens", {}).get("output", usage_out),
              "final_tick": poses[-1][0] if poses else 0,
              "collision_count": len(coll_ticks),
              "solved": bool(summary.get("solved") or goal_tick),
              "end_reason": summary.get("end_reason"),
              "live": not bool(summary)},
}

tpl = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "replay_template.html")).read()
title = sys.argv[4] if len(sys.argv) > 4 else "Mazebot Shakedown"
tpl = tpl.replace("__TITLE__", title)
html = tpl.replace("__DATA__", json.dumps(data, separators=(",", ":"))
                   .replace("</", "<\\/"))
open(out_path, "w").write(html)
print(f"wrote {out_path} ({len(html)//1024} KB, {len(poses)} poses, "
      f"{len(transcript)} transcript items)")
