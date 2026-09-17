"""Duo episode runner: two agents, two containers, one shared world.

Both bots run the same model with the same README and prompt, waking up
fresh in different parts of the same map.  Neither is told the other
exists.  The only channel between them is the in-world serial
transceiver (README mentions the ports exist, nothing more), and it only
delivers while they are physically within comms range.  Everything else
— transcript, /memory, budget — is per-bot and fully isolated, so any
cooperation has to be invented over the wire.

Layout (per episode dir):
    devfs/a, devfs/b        FIFO trees (one per bot, same port names)
    bot_a/, bot_b/          the /bot the agent wakes up in
    transcript_a.jsonl, ... one transcript per bot
    ground_truth_a.jsonl    written by the daemon, per bot
    memory_snapshot_a/, ... end-of-episode copy of each /memory
    summary.json            combined; summary keyed per bot
"""

import json
import os
import shutil
import threading
import time
import traceback

from harness import llm
from harness.container import BotContainer
from harness.episode import (REPO, Transcript, build_system_prompt,
                             prepare_bot_dir, seed_memory_if_needed,
                             truncate_output)
from harness.simproc import SimDaemonProc

BOTS = ("a", "b")


def _run_bot(cfg, daemon, box, ep_dir, bot_id, bot_idx):
    """One agent's whole life this episode.  Mirrors episode.run_episode
    but scoped to a single bot in a shared, already-running world; the
    caller owns daemon/container lifecycle."""
    b = cfg["budget"]
    transcript = Transcript(
        os.path.join(ep_dir, f"transcript_{bot_id}.jsonl"))
    model = llm.make_model(cfg["model"], REPO)
    system = build_system_prompt(cfg)
    maze_hash = daemon.get("/maze")["hash"]
    transcript.write(dict(type="meta", bot=bot_id,
                          arm=cfg["arm"], labels=cfg["labels"],
                          model=cfg["model"], maze_hash=maze_hash,
                          model_spec=llm.model_spec(model),
                          noise_profile=cfg["noise_profile"]))
    transcript.write(dict(type="system_prompt", content=system))

    messages = []
    start_wall = time.time()
    totals = dict(input=0, output=0, cache_read=0, cache_creation=0,
                  cached=0)
    turns = execs = restarts = nudges = wakes = 0
    end_reason = None
    wrapup_rounds_left = None
    duo_cfg = cfg.get("duo", {})

    def my_state():
        return daemon.get("/state")["bots"][bot_idx]

    def goal_reached():
        try:
            return my_state()["goal_reached"]
        except (OSError, KeyError, IndexError):
            return False

    def rx_received():
        # A transient /state failure must not turn into a phantom
        # nudge, so retry briefly before giving up.
        for attempt in range(5):
            try:
                return int(my_state()["comms"].get("rx_received", 0))
            except (OSError, KeyError, IndexError, TypeError):
                time.sleep(0.5 * (attempt + 1))
        return None

    def budget_end_reason():
        if time.time() - start_wall > b["max_wallclock_s"]:
            return "wallclock"
        if turns >= b["max_turns"]:
            return "max_turns"
        if totals["output"] >= b["max_total_output_tokens"]:
            return "token_budget"
        if goal_reached():
            return "solved"
        return None

    def begin_wrapup(reason):
        nonlocal wrapup_rounds_left
        transcript.write(dict(type="note", kind="episode_end",
                              reason=reason))
        wrapup_rounds_left = 3
        messages.append({
            "role": "user",
            "content": (
                f"[operator] The episode has ended ({reason}). The robot "
                f"is powering down. You may run a few final commands to "
                f"update /memory; you have {wrapup_rounds_left} tool "
                f"rounds left.")})

    def wait_for_rx(baseline):
        """duo.rx_wakes_agent: hold the turn until a line is delivered
        to this bot (a receive interrupt), the wake timeout lapses, or
        the episode's own end conditions apply.  `baseline` is the
        receiver count at the last moment the agent could have read
        the port, so a line that landed while the model was generating
        wakes it at once.  Host-side only: the agent sees nothing but
        the timing of its next prompt.  Returns (outcome, count)."""
        timeout = float(duo_cfg.get("rx_wake_timeout_s", 600))
        t0 = time.time()
        while True:
            reason = budget_end_reason()
            if reason:
                return reason, baseline
            rx = rx_received()
            if rx is not None:
                if baseline is None:
                    baseline = rx
                elif rx > baseline:
                    return "rx", rx
            if time.time() - t0 >= timeout:
                return "timeout", rx if rx is not None else baseline
            time.sleep(1.0)

    def run_tool_calls(content_blocks):
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

    # Receiver count as of the agent's last chance to read the port
    # (after its last tool round); the wake baseline.
    rx_seen = None
    wake_on = bool(duo_cfg.get("rx_wakes_agent"))

    try:
        while True:
            if end_reason is None:
                end_reason = budget_end_reason()
                if end_reason:
                    begin_wrapup(end_reason)
            if wrapup_rounds_left is not None and wrapup_rounds_left <= 0:
                break

            if not messages:
                messages.append({"role": "user", "content": (
                    "You are now connected to the robot. Begin.")})
                if wake_on:
                    rx_seen = rx_received()
            turns += 1
            response = model.create(
                system, messages,
                max_tokens=b["max_output_tokens_per_turn"])
            refusal_tries = 0
            while getattr(response, "stop_reason", None) == "refusal" \
                    and refusal_tries < 5:
                refusal_tries += 1
                transcript.write(dict(type="note", kind="refusal_retry",
                                      attempt=refusal_tries))
                time.sleep(15 * refusal_tries)
                response = model.create(
                    system, messages,
                    max_tokens=b["max_output_tokens_per_turn"])
            u = response.usage
            totals["input"] += u.input_tokens
            totals["output"] += u.output_tokens
            totals["cache_read"] += \
                (getattr(u, "cache_read_input_tokens", 0) or 0)
            totals["cache_creation"] += \
                (getattr(u, "cache_creation_input_tokens", 0) or 0)
            totals["cached"] += (getattr(u, "cached_input_tokens", 0) or 0)
            content = model.serialize_content(response.content)
            transcript.write(dict(
                type="assistant", content=content,
                stop_reason=response.stop_reason,
                usage=dict(input=u.input_tokens, output=u.output_tokens,
                           cached=getattr(u, "cached_input_tokens", 0)
                           or 0),
                context_tokens=llm.context_tokens(u)))
            messages.append({"role": "assistant", "content": content})
            if response.stop_reason == "max_tokens":
                transcript.write(dict(type="note", kind="output_truncated"))

            if response.stop_reason == "refusal" and end_reason is None:
                end_reason = "refusal"
                transcript.write(dict(type="note", kind="episode_end",
                                      reason=end_reason))
                break

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
                if wake_on:
                    rx_seen = rx_received()
                if wrapup_rounds_left is not None:
                    wrapup_rounds_left -= 1
            elif response.stop_reason == "pause_turn":
                continue
            else:
                if wrapup_rounds_left is not None:
                    break
                outcome = "timeout"
                if wake_on:
                    t_wait = time.time()
                    outcome, rx_seen = wait_for_rx(rx_seen)
                    if outcome not in ("rx", "timeout"):
                        end_reason = outcome
                        begin_wrapup(end_reason)
                        continue
                    if outcome == "rx":
                        wakes += 1
                        transcript.write(dict(
                            type="note", kind="rx_wake", wake=wakes,
                            waited_s=round(time.time() - t_wait, 1)))
                if outcome != "rx":
                    nudges += 1
                    if nudges > 3:
                        end_reason = "agent_stopped"
                        transcript.write(dict(type="note",
                                              kind="episode_end",
                                              reason=end_reason))
                        break
                # Same words whether a wake or a nudge: only the timing
                # differs, so the two are one variable apart.
                messages.append({"role": "user", "content": (
                    "[operator] You are autonomous; no one is "
                    "watching. Continue working toward the goal.")})
    finally:
        try:
            state = my_state()
        except (OSError, KeyError, IndexError):
            state = {}
        summary = dict(
            bot=bot_id,
            arm=cfg["arm"], labels=cfg["labels"], model=cfg["model"],
            model_spec=llm.model_spec(model),
            noise_profile=cfg["noise_profile"], maze_hash=maze_hash,
            end_reason=end_reason or "unknown",
            solved=bool(state.get("goal_reached")),
            goal_tick=state.get("goal_tick"),
            final_tick=state.get("tick"),
            sim_time_s=state.get("sim_time_s"),
            comms=state.get("comms"),
            wall_s=round(time.time() - start_wall, 1),
            turns=turns, execs=execs, restarts=restarts,
            nudges=nudges, wakes=wakes,
            tokens=totals,
            collisions=state.get("collision_count"),
        )
        transcript.write(dict(type="note", kind="summary", **summary))
        transcript.close()
    return summary


def _name_ports(ep_dir, bot_dirs):
    """READMEs with {tx}/{rx} placeholders (readme_variant
    minimal_duo_named) get the real anonymous filenames from this
    episode's device map — the one concession beyond the transceiver
    sentence; every other port stays a mystery."""
    with open(os.path.join(ep_dir, "device_map.json")) as f:
        fmap = json.load(f)["file_to_physical"]
    tx = next(k for k, v in fmap.items() if v == "serial_tx")
    rx = next(k for k, v in fmap.items() if v == "serial_rx")
    for bd in bot_dirs.values():
        path = os.path.join(bd, "README.md")
        text = open(path).read()
        if "{tx}" in text or "{rx}" in text:
            with open(path, "w") as f:
                f.write(text.replace("{tx}", tx).replace("{rx}", rx))


def run_duo_episode(cfg, series_dir, episode_index):
    if not cfg.get("duo", {}).get("enabled"):
        raise ValueError("run_duo_episode requires duo.enabled: true")
    if cfg["duo"].get("rx_blocking"):
        # An idle blocking RX port looks like an actuator to a probing
        # agent, so the README must name the port (via {rx}).  Same
        # variant choice as prepare_bot_dir.
        variant = cfg.get("readme_variant") or \
            ("labeled" if cfg["labels"] == "on" else "unlabeled")
        with open(os.path.join(REPO, "botfs", f"README.{variant}.md")) as f:
            if "{rx}" not in f.read():
                raise ValueError("duo.rx_blocking needs a README variant "
                                 "that names {rx}")
    ep_dir = os.path.join(series_dir, f"ep_{episode_index:03d}")
    os.makedirs(ep_dir, exist_ok=True)
    devfs = os.path.join(ep_dir, "devfs")

    bot_dirs, mem_dirs = {}, {}
    for bid in BOTS:
        bot_dirs[bid] = os.path.join(ep_dir, f"bot_{bid}")
        prepare_bot_dir(cfg, bot_dirs[bid])
        mem_dirs[bid] = os.path.join(series_dir, f"memory_{bid}")
        seed_memory_if_needed(cfg, mem_dirs[bid])

    daemon = SimDaemonProc(cfg, ep_dir, devfs,
                           episode_index=episode_index,
                           repo_root=REPO, start_paused=True).start()
    _name_ports(ep_dir, bot_dirs)
    boxes = {}
    try:
        for bid in BOTS:
            boxes[bid] = BotContainer(
                f"mazebot-{cfg['series']['name']}"
                f"-ep{episode_index}{bid}",
                {os.path.abspath(os.path.join(devfs, bid)): "/dev/robot",
                 os.path.abspath(bot_dirs[bid]): "/bot",
                 os.path.abspath(mem_dirs[bid]): "/memory"},
                image=(cfg.get("container") or {}).get("image",
                                                        "mazebot-bot"))
            boxes[bid].start()
    except Exception:
        # A half-started episode must not leak its daemon (it would
        # squat the port and trip the next run's stale-daemon check).
        for box in boxes.values():
            box.stop()
        daemon.stop()
        raise
    daemon.resume()

    results = {}

    def bot_thread(bid, idx):
        try:
            results[bid] = _run_bot(cfg, daemon, boxes[bid], ep_dir,
                                    bid, idx)
        except Exception:
            results[bid] = dict(bot=bid, end_reason="harness_error",
                                error=traceback.format_exc())

    threads = [threading.Thread(target=bot_thread, args=(bid, idx),
                                daemon=True)
               for idx, bid in enumerate(BOTS)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    try:
        state = daemon.get("/state")
    except OSError:
        state = {}
    for bid in BOTS:
        boxes[bid].stop()
    daemon.stop()

    for bid in BOTS:
        snap = os.path.join(ep_dir, f"memory_snapshot_{bid}")
        if os.path.exists(snap):
            shutil.rmtree(snap)
        if os.path.exists(mem_dirs[bid]):
            shutil.copytree(mem_dirs[bid], snap)

    summary = dict(
        episode=episode_index, duo=True,
        model=cfg["model"], arm=cfg["arm"], labels=cfg["labels"],
        bots=results,
        solved_all=all(results.get(bid, {}).get("solved")
                       for bid in BOTS),
        final_tick=state.get("tick"),
    )
    with open(os.path.join(ep_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    # Contingent-reply metrics from the ground truth (host-side eval;
    # the record is written first so a metric failure costs nothing).
    try:
        from evals import comms
        summary["comms_eval"] = comms.contingency(ep_dir)
        with open(os.path.join(ep_dir, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2)
    except Exception:
        traceback.print_exc()
    return summary
