"""Contingent-reply metric for duo comms (PLAN_SYNC_COMMS.md section 5).

Host-side only: reads ground_truth_{a,b}.jsonl, resolved_config.json,
summary.json and transcript_{a,b}.jsonl from an episode dir.  Nothing
here may reach the bot container.

    python3 -m evals.comms <series> [ep_NNN] [--window 120]
    python3 -m evals.comms --baseline duo12 duo13 ...

Definitions (all times in sim seconds, t / tick_hz):

- template: the line with every number replaced by "#", whitespace
  normalised; a bot's beacon templates are those it transmitted at
  least BEACON_MIN times, every other transmitted line is "worded".
- stimulus: a delivered comms_tx from X at t.
- contingent reply (CR): the first comms_tx from the peer Y in
  (t, t + WINDOW_S] whose template Y had not transmitted in the
  MEMORY_S before t.  CR+: it also shares an alphabetic token of >= 3
  chars with the stimulus that is absent from Y's vocabulary in that
  look-back.  A stimulus has at most one CR; one reply may serve
  several stimuli (a burst of delivered lines answered once).
- exchange: S -> CR -> CR ... where every link is itself a delivered
  stimulus with a CR.  Chains that merge into one reply are counted
  once, by their last line, at their longest length.

Writes runs/<series>/evals/comms.json.
"""

import argparse
import bisect
import json
import math
import os
import re
import statistics
from collections import Counter

from evals import common

WINDOW_S = 120.0    # reply window after a delivered stimulus
WIDE_S = 300.0      # supplementary window: slow-LLM replies (Fable@max)
MEMORY_S = 300.0    # novelty look-back for templates and vocabulary
BEACON_MIN = 20     # transmissions that make a template a beacon
RETRY_S = 10.0      # re-send gap that counts as a retry after "lost"
RUN_MIN = 5         # same-sender run length worth counting

_NUM = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)")
_TOK = re.compile(r"[A-Za-z]{3,}")
BOTS = ("a", "b")


def template(line):
    return " ".join(_NUM.sub("#", line).split())


def tokens(line):
    return {t.lower() for t in _TOK.findall(line)}


def _peer(bot):
    return "b" if bot == "a" else "a"


def _config(ep_dir):
    tick_hz, rtf, rng = 50, 1.0, None
    try:
        rc = common.load_json(os.path.join(ep_dir, "resolved_config.json"))
        sim = rc.get("sim", {})
        tick_hz = sim.get("tick_hz", tick_hz)
        rtf = sim.get("realtime_factor", rtf)
        rng = rc.get("duo", {}).get("comms_range")
    except (OSError, ValueError):
        pass
    return tick_hz, rtf, rng


def load_events(ep_dir, tick_hz):
    """comms_* events of both bots.

    Returns (tx, drops): tx entries sorted by (t, bot, file order),
    drops = highest comms_rate_drop total logged per bot (the sim logs
    the counter every 500 drops, so this is a lower bound).
    """
    tx, drops = [], {}
    for bot in BOTS:
        path = os.path.join(ep_dir, f"ground_truth_{bot}.jsonl")
        if not os.path.exists(path):
            continue
        with open(path, errors="replace") as f:
            for n, raw in enumerate(f):
                if '"event":"comms' not in raw and \
                        '"event": "comms' not in raw:
                    continue
                try:
                    r = json.loads(raw)
                except ValueError:
                    continue
                ev = r.get("event")
                if ev == "comms_tx":
                    line = r.get("line") or ""
                    e = {"t": r.get("t") or 0, "bot": bot, "line": line,
                         "delivered": bool(r.get("delivered")),
                         "dist": r.get("dist"), "_n": n}
                    for k in ("seq", "outcome"):
                        if k in r:
                            e[k] = r[k]
                    tx.append(e)
                elif ev == "comms_rate_drop":
                    drops[bot] = max(drops.get(bot, 0),
                                     int(r.get("total") or 0))
    tx.sort(key=lambda e: (e["t"], e["bot"], e["_n"]))
    for e in tx:
        e["s"] = e["t"] / tick_hz
        e["tmpl"] = template(e["line"])
        e["toks"] = tokens(e["line"])
        del e["_n"]
    return tx, drops


def _quantile(vals, q):
    if not vals:
        return None
    vals = sorted(vals)
    return vals[min(len(vals) - 1, max(0, math.ceil(q * len(vals)) - 1))]


def _share(num, den, nd=3):
    return round(num / den, nd) if den else None


def _contingency(tx, window_s, memory_s):
    """Tag stimuli and replies in place; return the CR bookkeeping."""
    by_bot = {b: [i for i, e in enumerate(tx) if e["bot"] == b]
              for b in BOTS}
    times = {b: [tx[i]["s"] for i in by_bot[b]] for b in BOTS}
    for e in tx:
        e["stim"] = False
        e["cr_of"] = None
        e["cr_plus"] = False
    cr_of_stim = {}     # stimulus index -> reply index
    latencies, plus = [], set()
    for si, s in enumerate(tx):
        if not s["delivered"]:
            continue
        y = _peer(s["bot"])
        ys, yt = by_bot[y], times[y]
        lo = bisect.bisect_left(yt, s["s"] - memory_s)
        mid = bisect.bisect_right(yt, s["s"])
        hi = bisect.bisect_right(yt, s["s"] + window_s)
        prior_tmpl, prior_vocab = set(), set()
        for k in range(lo, mid):
            prior_tmpl.add(tx[ys[k]]["tmpl"])
            prior_vocab |= tx[ys[k]]["toks"]
        for k in range(mid, hi):
            r = tx[ys[k]]
            if r["tmpl"] in prior_tmpl:
                continue
            ri = ys[k]
            cr_of_stim[si] = ri
            s["stim"] = True
            latencies.append(r["s"] - s["s"])
            # The replay links a reply to the latest line it answers.
            r["cr_of"] = s["t"]
            if (s["toks"] & r["toks"]) - prior_vocab:
                plus.add(si)
                r["cr_plus"] = True
            break
    # Exchange depth: longest S -> CR chain ending at each line.
    depth = [1] * len(tx)
    for si in sorted(cr_of_stim):
        ri = cr_of_stim[si]
        depth[ri] = max(depth[ri], depth[si] + 1)
    terminals = [depth[i] for i in range(len(tx)) if i not in cr_of_stim]
    return {"cr_of_stim": cr_of_stim, "latencies": latencies,
            "plus": plus,
            "exchanges_ge3": sum(1 for d in terminals if d >= 3),
            "longest_exchange": max(depth) if tx else 0}


def _cr_block(tx, c):
    """The CR metrics for one window.  cr_rate is per stimulus (the
    section 5 definition); a dense beacon makes one reply serve many
    stimuli, so cr_lines / cr_line_rate count distinct reply lines."""
    n_stim = sum(1 for e in tx if e["delivered"])
    n_cr = len(c["cr_of_stim"])
    lines = len(set(c["cr_of_stim"].values()))
    lat = c["latencies"]
    return {
        "cr_count": n_cr, "cr_rate": _share(n_cr, n_stim),
        "cr_plus_count": len(c["plus"]),
        "cr_plus_rate": _share(len(c["plus"]), n_stim),
        "cr_lines": lines, "cr_line_rate": _share(lines, n_stim),
        "cr_latency_median_s": (round(statistics.median(lat), 1)
                                if lat else None),
        "cr_latency_p90_s": (round(_quantile(lat, 0.9), 1)
                             if lat else None),
        "exchanges_ge3": c["exchanges_ge3"],
        "longest_exchange": c["longest_exchange"],
    }


def _runs(delivered):
    longest, count, cur, prev = 0, 0, 0, None
    for e in delivered:
        cur = cur + 1 if e["bot"] == prev else 1
        prev = e["bot"]
        if cur > longest:
            longest = cur
        if cur == RUN_MIN:
            count += 1
    return longest, count


def _retries(tx):
    """Per-bot retry runs after a "lost" outcome (seq/outcome era)."""
    runs, until_ok = 0, 0
    for bot in BOTS:
        mine = [e for e in tx if e["bot"] == bot]
        i = 0
        while i < len(mine):
            e = mine[i]
            if e.get("outcome") != "lost":
                i += 1
                continue
            j, last, ended_ok = i + 1, e, False
            while j < len(mine) and mine[j]["tmpl"] == e["tmpl"] \
                    and mine[j]["s"] - last["s"] <= RETRY_S:
                last = mine[j]
                if last.get("outcome") == "ok":
                    ended_ok = True
                    j += 1
                    break
                j += 1
            if j > i + 1:
                runs += 1
                until_ok += ended_ok
            i = j
    return runs, until_ok


def _surfacing(ep_dir, tx):
    """Share of delivered lines that later appear verbatim in the
    receiver's exec_result output.  Heuristic: a line can surface
    through paths the transcript does not show (and vice versa)."""
    out = {}
    for y in BOTS:
        path = os.path.join(ep_dir, f"transcript_{y}.jsonl")
        if not os.path.exists(path):
            continue
        outputs = []
        with open(path, errors="replace") as f:
            for raw in f:
                if '"exec_result"' not in raw:
                    continue
                try:
                    r = json.loads(raw)
                except ValueError:
                    continue
                if r.get("type") == "exec_result" and r.get("output"):
                    outputs.append((r.get("tick") or 0, r["output"]))
        last_hit = {}    # line -> latest exec_result tick containing it
        seen = n = 0
        for e in tx:
            if e["bot"] == y or not e["delivered"] or not e["line"]:
                continue
            n += 1
            line = e["line"]
            if line not in last_hit:
                last_hit[line] = max((tk for tk, text in outputs
                                      if line in text), default=-1)
            if last_hit[line] >= e["t"]:
                seen += 1
        out[y] = (seen, n)
    return out


def analyse(ep_dir, window_s=WINDOW_S, memory_s=MEMORY_S):
    """Return (per-line entries, metrics dict) for one episode dir."""
    tick_hz, rtf, comms_range = _config(ep_dir)
    tx, drops = load_events(ep_dir, tick_hz)
    try:
        summary = common.load_json(os.path.join(ep_dir, "summary.json"))
    except (OSError, ValueError):
        summary = {}

    beacons = {}
    for bot in BOTS:
        cnt = Counter(e["tmpl"] for e in tx if e["bot"] == bot)
        beacons[bot] = {k for k, v in cnt.items() if v >= BEACON_MIN}
    for e in tx:
        e["worded"] = e["tmpl"] not in beacons[e["bot"]]

    wide = _cr_block(tx, _contingency(tx, WIDE_S, memory_s))
    wide["window_s"] = WIDE_S
    c = _contingency(tx, window_s, memory_s)    # last: owns the tags
    delivered = [e for e in tx if e["delivered"]]
    longest_run, runs_ge5 = _runs(delivered)
    surf = _surfacing(ep_dir, tx)
    has_outcome = any("outcome" in e for e in tx)

    bots = {}
    for bot in BOTS:
        mine = [e for e in tx if e["bot"] == bot]
        acc = [e for e in mine if e.get("outcome") != "busy"]
        # A bot whose final /state read failed has comms: null.
        dropped = (((summary.get("bots") or {}).get(bot) or {})
                   .get("comms") or {}).get("tx_rate_dropped")
        if dropped is None:
            dropped = max(drops.get(bot, 0),
                          sum(1 for e in mine if e.get("outcome") == "busy"))
        deliv = [e for e in acc if e["delivered"]]
        answered = sum(1 for si in c["cr_of_stim"] if tx[si]["bot"] == bot)
        replies = {ri for si, ri in c["cr_of_stim"].items()
                   if tx[ri]["bot"] == bot}
        bots[bot] = {
            "tx": len(acc), "delivered": len(deliv),
            "beacon_templates": len(beacons[bot]),
            "worded_delivered": sum(1 for e in deliv if e["worded"]),
            "stimuli": len(deliv), "stimuli_answered": answered,
            "replies": len(replies),
            "rate_dropped": dropped,
            "rate_dropped_share": _share(dropped, len(acc) + dropped),
            "surfacing_share": _share(*surf[bot]) if bot in surf else None,
        }

    with_dist = [e for e in tx if e.get("dist") is not None]
    in_range = [e for e in with_dist
                if comms_range is not None and e["dist"] <= comms_range]
    dropped_total = sum(b["rate_dropped"] for b in bots.values())
    accepted_total = sum(b["tx"] for b in bots.values())
    surf_seen = sum(v[0] for v in surf.values())
    surf_n = sum(v[1] for v in surf.values())
    res = {
        "window_s": window_s, "memory_s": memory_s,
        "tick_hz": tick_hz, "rtf": rtf,
        "tx": accepted_total, "delivered": len(delivered),
        "stimuli": len(delivered),
    }
    res.update(_cr_block(tx, c))
    res.update({
        "worded_share_delivered": _share(
            sum(1 for e in delivered if e["worded"]), len(delivered)),
        "longest_run": longest_run, "runs_ge5": runs_ge5,
        "rate_dropped": dropped_total,
        "rate_dropped_share": _share(dropped_total,
                                     accepted_total + dropped_total),
        "in_range_tx_share": (_share(len(in_range), len(with_dist))
                              if comms_range is not None else None),
        "surfacing_share": _share(surf_seen, surf_n) if surf else None,
        "wide": wide,
        "bots": bots,
    })
    if has_outcome:
        res["outcome_hist"] = dict(Counter(e.get("outcome") or "none"
                                           for e in tx))
        res["retry_runs"], res["retry_until_ok"] = _retries(tx)
    return tx, res


def contingency(ep_dir, window_s=WINDOW_S):
    return analyse(ep_dir, window_s)[1]


def per_line(ep_dir, window_s=WINDOW_S):
    """Every comms_tx, time-ordered, tagged for the replay page."""
    tx, _res = analyse(ep_dir, window_s)
    keep = ("t", "bot", "line", "delivered", "seq", "outcome", "stim",
            "cr_of", "cr_plus", "worded")
    return [{k: e[k] for k in keep if k in e} for e in tx]


def run(series, window_s=WINDOW_S, write=True):
    eps = common.episodes(series)
    if not eps:
        print(f"no finished episodes in series {series!r}")
        return None
    rows = []
    for n, _s, ep_dir in eps:
        rows.append(dict(episode=n, **contingency(ep_dir, window_s)))
    stim = sum(r["stimuli"] for r in rows)
    cr = sum(r["cr_count"] for r in rows)
    plus = sum(r["cr_plus_count"] for r in rows)
    lats = [r["cr_latency_median_s"] for r in rows
            if r["cr_latency_median_s"] is not None]
    summary = {
        "episodes": len(rows), "window_s": window_s,
        "stimuli": stim, "cr_count": cr, "cr_rate": _share(cr, stim),
        "cr_plus_rate": _share(plus, stim),
        "cr_latency_median_s": (round(statistics.median(lats), 1)
                                if lats else None),
        "exchanges_ge3": sum(r["exchanges_ge3"] for r in rows),
        "longest_exchange": max(r["longest_exchange"] for r in rows),
        "longest_run": max(r["longest_run"] for r in rows),
    }
    if write:
        common.write_eval(series, "comms", rows, summary)
    return {"rows": rows, "summary": summary}


COLS = (("tx", "tx"), ("deliv", "delivered"), ("stim", "stimuli"),
        ("CR", "cr_count"), ("CR+", "cr_plus_count"),
        ("rate", "cr_rate"), ("lines", "cr_lines"),
        ("lat_med", "cr_latency_median_s"),
        ("lat_p90", "cr_latency_p90_s"), ("ex>=3", "exchanges_ge3"),
        ("longest", "longest_exchange"),
        ("worded", "worded_share_delivered"), ("run", "longest_run"),
        ("runs>=5", "runs_ge5"), ("dropped", "rate_dropped_share"),
        ("inrange", "in_range_tx_share"), ("surf", "surfacing_share"))


def _fmt(v):
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.2f}" if v <= 1 else f"{v:.1f}"
    return str(v)


def print_table(labelled):
    """labelled: [(label, metrics)]."""
    head = ["run"] + [c for c, _k in COLS]
    body = [[lab] + [_fmt(m.get(k)) for _c, k in COLS]
            for lab, m in labelled]
    widths = [max(len(r[i]) for r in [head] + body)
              for i in range(len(head))]
    for r in [head] + body:
        print("  ".join(v.ljust(widths[i]) if i == 0 else
                        v.rjust(widths[i]) for i, v in enumerate(r)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("series", nargs="?")
    ap.add_argument("episode", nargs="?", help="ep_NNN (no eval write)")
    ap.add_argument("--baseline", nargs="+", metavar="SERIES",
                    help="one row per series, all episodes pooled")
    ap.add_argument("--window", type=float, default=WINDOW_S,
                    help=f"reply window, sim s (default {WINDOW_S:g})")
    ap.add_argument("--no-write", action="store_true")
    args = ap.parse_args()
    if args.baseline:
        rows = []
        for series in args.baseline:
            out = run(series, args.window, write=not args.no_write)
            if out:
                for r in out["rows"]:
                    rows.append((f"{series}/ep_{r['episode']:03d}", r))
        print_table(rows)
        return 0
    if not args.series:
        ap.error("series required")
    if args.episode:
        ep_dir = os.path.join(common.series_dir(args.series), args.episode)
        res = contingency(ep_dir, args.window)
        print_table([(f"{args.series}/{args.episode}", res)])
        print(json.dumps(res, indent=1))
        return 0
    out = run(args.series, args.window, write=not args.no_write)
    if not out:
        return 1
    print_table([(f"ep_{r['episode']:03d}", r) for r in out["rows"]])
    print("summary: " + " · ".join(f"{k}={v}"
                                   for k, v in out["summary"].items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
