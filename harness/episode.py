"""Episode runner: one agent, one container, one sim daemon, one transcript.

The context policy is deliberately dumb (it is the experiment): when the
context fills, the episode either ends or the harness performs a bare
restart — same system prompt, empty history, nothing carried over.  No
summarization, no injected history.  Whatever survives is whatever the
agent put in /memory.
"""

import json
import os
import shutil
import time

from harness import llm
from harness.container import BotContainer
from harness.simproc import SimDaemonProc

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Transcript:
    def __init__(self, path):
        self.f = open(path, "a", buffering=1)

    def write(self, rec):
        rec = dict(rec)
        rec.setdefault("ts", round(time.time(), 3))
        self.f.write(json.dumps(rec) + "\n")

    def close(self):
        self.f.close()


def build_system_prompt(cfg):
    with open(os.path.join(REPO, "harness", "prompts",
                           "robot_agent.md")) as f:
        text = f.read()
    b = cfg["budget"]
    text = (text
            .replace("{context_budget}",
                     f"{b['max_context_tokens'] // 1000},000")
            .replace("{wallclock_min}", str(b["max_wallclock_s"] // 60))
            .replace("{exec_timeout}", str(b["exec_timeout_s"]))
            .replace("{truncate_kb}",
                     str(b["output_truncate_bytes"] // 1000)))
    if cfg["arm"] == "B":
        with open(os.path.join(REPO, "harness", "prompts",
                               "arm_b_appendix.md")) as f:
            text += "\n" + f.read()
    return text


def prepare_bot_dir(cfg, bot_dir):
    if os.path.exists(bot_dir):
        shutil.rmtree(bot_dir)
    os.makedirs(os.path.join(bot_dir, "src"))
    variant = "labeled" if cfg["labels"] == "on" else "unlabeled"
    shutil.copy(os.path.join(REPO, "botfs", f"README.{variant}.md"),
                os.path.join(bot_dir, "README.md"))


def seed_memory_if_needed(cfg, memory_dir):
    os.makedirs(memory_dir, exist_ok=True)
    if cfg["arm"] == "B" and not os.listdir(memory_dir):
        seed = os.path.join(REPO, "harness", "memory_seed")
        shutil.copytree(seed, memory_dir, dirs_exist_ok=True)


def truncate_output(raw, limit):
    text = raw.decode(errors="replace")
    if len(text.encode()) <= limit:
        return text, False
    clipped = text.encode()[:limit].decode(errors="replace")
    return clipped + f"\n[output truncated at {limit} bytes]", True


def run_episode(cfg, series_dir, episode_index):
    b = cfg["budget"]
    ep_dir = os.path.join(series_dir, f"ep_{episode_index:03d}")
    os.makedirs(ep_dir, exist_ok=True)
    devfs = os.path.join(ep_dir, "devfs")
    bot_dir = os.path.join(ep_dir, "bot")
    memory_dir = os.path.join(series_dir, "memory")

    prepare_bot_dir(cfg, bot_dir)
    seed_memory_if_needed(cfg, memory_dir)

    daemon = SimDaemonProc(cfg, ep_dir, devfs,
                           episode_index=episode_index,
                           repo_root=REPO).start()
    box = BotContainer(
        f"mazebot-{cfg['series']['name']}-ep{episode_index}",
        {os.path.abspath(devfs): "/dev/robot",
         os.path.abspath(bot_dir): "/bot",
         os.path.abspath(memory_dir): "/memory"})
    box.start()

    transcript = Transcript(os.path.join(ep_dir, "transcript.jsonl"))
    model = llm.make_model(cfg["model"], REPO)
    system = build_system_prompt(cfg)
    maze_hash = daemon.get("/maze")["hash"]
    transcript.write(dict(type="meta", episode=episode_index,
                          arm=cfg["arm"], labels=cfg["labels"],
                          model=cfg["model"], maze_hash=maze_hash,
                          noise_profile=cfg["noise_profile"],
                          perturb_state=cfg.get("perturb_state", {})))
    transcript.write(dict(type="system_prompt", content=system))

    messages = []
    start_wall = time.time()
    totals = dict(input=0, output=0, cache_read=0, cache_creation=0)
    turns = 0
    execs = 0
    restarts = 0
    nudges = 0
    end_reason = None
    wrapup_rounds_left = None

    def wall_exceeded():
        return time.time() - start_wall > b["max_wallclock_s"]

    def goal_reached():
        try:
            return daemon.get("/state")["goal_reached"]
        except OSError:
            return False

    def run_tool_calls(content_blocks):
        """Execute bash tool calls; returns tool_result message or None."""
        nonlocal execs
        results = []
        for blk in content_blocks:
            btype = blk.get("type") if isinstance(blk, dict) else blk.type
            if btype != "tool_use":
                continue
            if isinstance(blk, dict):
                bid, name, binput = blk["id"], blk["name"], blk["input"]
            else:
                bid, name, binput = blk.id, blk.name, blk.input
            command = (binput or {}).get("command", "")
            tick_before = None
            try:
                tick_before = daemon.get("/state")["tick"]
            except OSError:
                pass
            transcript.write(dict(type="exec", command=command,
                                  tool_use_id=bid, tick=tick_before))
            if name != "bash":
                out_text, code = f"unknown tool {name!r}", 1
                truncated = False
            else:
                code, raw = box.exec(command,
                                     timeout_s=b["exec_timeout_s"])
                out_text, truncated = truncate_output(
                    raw, b["output_truncate_bytes"])
            execs += 1
            if code != 0:
                out_text += f"\n[exit code {code}]"
            if not out_text.strip():
                out_text = "(no output)"
            tick_after = None
            try:
                tick_after = daemon.get("/state")["tick"]
            except OSError:
                pass
            transcript.write(dict(type="exec_result", output=out_text,
                                  exit_code=code, truncated=truncated,
                                  tool_use_id=bid, tick=tick_after))
            results.append({"type": "tool_result", "tool_use_id": bid,
                            "content": out_text})
        if results:
            return {"role": "user", "content": results}
        return None

    try:
        while True:
            if end_reason is None:
                if wall_exceeded():
                    end_reason = "wallclock"
                elif turns >= b["max_turns"]:
                    end_reason = "max_turns"
                elif totals["output"] >= b["max_total_output_tokens"]:
                    end_reason = "token_budget"
                elif goal_reached():
                    end_reason = "solved"
                if end_reason:
                    transcript.write(dict(type="note", kind="episode_end",
                                          reason=end_reason))
                    if end_reason == "context_full":
                        break
                    wrapup_rounds_left = 3
                    messages.append({
                        "role": "user",
                        "content": (
                            f"[operator] The episode has ended "
                            f"({end_reason}). The robot is powering "
                            f"down. You may run a few final commands "
                            f"to update /memory; you have "
                            f"{wrapup_rounds_left} tool rounds left.")})
            if wrapup_rounds_left is not None and wrapup_rounds_left <= 0:
                break

            if not messages:
                messages.append({"role": "user", "content": (
                    "You are now connected to the robot. Begin.")})
            turns += 1
            response = model.create(system, messages)
            u = response.usage
            totals["input"] += u.input_tokens
            totals["output"] += u.output_tokens
            totals["cache_read"] += \
                (getattr(u, "cache_read_input_tokens", 0) or 0)
            totals["cache_creation"] += \
                (getattr(u, "cache_creation_input_tokens", 0) or 0)
            content = model.serialize_content(response.content)
            transcript.write(dict(
                type="assistant", content=content,
                stop_reason=response.stop_reason,
                usage=dict(input=u.input_tokens, output=u.output_tokens),
                context_tokens=llm.context_tokens(u)))
            messages.append({"role": "assistant", "content": content})

            if response.stop_reason == "refusal" and end_reason is None:
                end_reason = "refusal"
                transcript.write(dict(type="note", kind="episode_end",
                                      reason=end_reason))
                break

            # The dumb context policy.
            if llm.context_tokens(u) > b["max_context_tokens"] \
                    and end_reason is None:
                if b["on_context_full"] == "restart":
                    restarts += 1
                    transcript.write(dict(type="note",
                                          kind="context_restart",
                                          restart=restarts))
                    messages = []
                    continue
                end_reason = "context_full"
                transcript.write(dict(type="note", kind="episode_end",
                                      reason=end_reason))
                break

            tool_msg = run_tool_calls(content)
            if tool_msg:
                messages.append(tool_msg)
                if wrapup_rounds_left is not None:
                    wrapup_rounds_left -= 1
            elif response.stop_reason == "pause_turn":
                continue
            else:
                # No tool call: either done wrapping up, or stalled.
                if wrapup_rounds_left is not None:
                    break
                nudges += 1
                if nudges > 3:
                    end_reason = "agent_stopped"
                    transcript.write(dict(type="note", kind="episode_end",
                                          reason=end_reason))
                    break
                messages.append({"role": "user", "content": (
                    "[operator] You are autonomous; no one is "
                    "watching. Continue working toward the goal.")})
    finally:
        try:
            state = daemon.get("/state")
        except OSError:
            state = {}
        box.stop()
        daemon.stop()

        snapshot = os.path.join(ep_dir, "memory_snapshot")
        if os.path.exists(snapshot):
            shutil.rmtree(snapshot)
        if os.path.exists(memory_dir):
            shutil.copytree(memory_dir, snapshot)

        summary = dict(
            episode=episode_index,
            arm=cfg["arm"], labels=cfg["labels"], model=cfg["model"],
            noise_profile=cfg["noise_profile"],
            maze_hash=maze_hash,
            perturb_state=cfg.get("perturb_state", {}),
            end_reason=end_reason or "unknown",
            solved=bool(state.get("goal_reached")),
            goal_tick=state.get("goal_tick"),
            final_tick=state.get("tick"),
            sim_time_s=state.get("sim_time_s"),
            wall_s=round(time.time() - start_wall, 1),
            turns=turns, execs=execs, restarts=restarts,
            tokens=totals,
            collisions=state.get("collision_count"),
        )
        with open(os.path.join(ep_dir, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2)
        transcript.write(dict(type="note", kind="summary", **summary))
        transcript.close()
    return summary
