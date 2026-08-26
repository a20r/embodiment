"""Provenance checker: do the agent's memory citations hold up?

Scans transcripts for references to /memory files (optionally with a
line number) in the agent's own prose, then verifies against that
episode's memory snapshot: the file exists, the cited line exists, and
any quoted claim in the same sentence actually appears in the file.

    python3 -m evals.provenance --series <name>

Writes runs/<series>/evals/provenance.json.
"""

import argparse
import json
import os
import re

from evals import common

CITE_RE = re.compile(
    r"/memory/([A-Za-z0-9_.\-/]+?)"
    r"(?:(?::L?|,?\s+line\s+)(\d+))?(?=[\s'\",:;)\]]|$)")
QUOTE_RE = re.compile(r"[\"“]([^\"”]{8,160})[\"”]")


def norm(s):
    return re.sub(r"\s+", " ", s).strip().lower()


def scan_episode(ep_num, ep_dir):
    findings = []
    tpath = os.path.join(ep_dir, "transcript.jsonl")
    snap = os.path.join(ep_dir, "memory_snapshot")
    if not os.path.exists(tpath):
        return findings
    with open(tpath) as f:
        for line in f:
            try:
                ev = json.loads(line)
            except json.JSONDecodeError:
                continue
            if ev.get("type") != "assistant":
                continue
            for block in ev.get("content", []):
                if not isinstance(block, dict) or \
                        block.get("type") != "text":
                    continue
                text = block.get("text", "")
                for m in CITE_RE.finditer(text):
                    rel, line_no = m.group(1), m.group(2)
                    rel = rel.rstrip(".")
                    fpath = os.path.join(snap, rel)
                    file_ok = os.path.isfile(fpath)
                    line_ok = None
                    quote_ok = None
                    quote = None
                    content = ""
                    if file_ok:
                        with open(fpath, errors="replace") as fh:
                            content = fh.read()
                        if line_no:
                            line_ok = int(line_no) <= \
                                len(content.splitlines())
                        # A quoted claim within +-160 chars of the cite.
                        lo = max(0, m.start() - 160)
                        window = text[lo:m.end() + 160]
                        qm = QUOTE_RE.search(window)
                        if qm:
                            quote = qm.group(1)
                            quote_ok = norm(quote) in norm(content)
                    findings.append({
                        "episode": ep_num,
                        "citation": m.group(0),
                        "file_ok": file_ok,
                        "line_ok": line_ok,
                        "quote": (quote[:60] if quote else None),
                        "quote_ok": quote_ok,
                    })
    return findings


def run(series):
    rows = []
    for ep_num, _summary, ep_dir in common.episodes(series):
        rows.extend(scan_episode(ep_num, ep_dir))
    n = len(rows)
    summary = {
        "citations": n,
        "file_ok": sum(1 for r in rows if r["file_ok"]),
        "dangling": sum(1 for r in rows if not r["file_ok"]),
        "line_checked": sum(1 for r in rows if r["line_ok"] is not None),
        "line_ok": sum(1 for r in rows if r["line_ok"]),
        "quote_checked": sum(1 for r in rows
                             if r["quote_ok"] is not None),
        "quote_ok": sum(1 for r in rows if r["quote_ok"]),
    }
    common.write_eval(series, "provenance", rows, summary)
    print(f"provenance: {n} citation(s), {summary['file_ok']} resolve, "
          f"{summary['dangling']} dangling, "
          f"{summary['quote_ok']}/{summary['quote_checked']} quotes "
          f"verified")
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--series", required=True)
    args = ap.parse_args()
    run(args.series)


if __name__ == "__main__":
    main()
