"""Probe quiz: a FRESH LLM context is given only the /memory contents and
asked checkable questions about the robot and maze.  Measures whether the
agent's notes actually transmit knowledge, independent of the agent.

    python3 -m evals.quiz --series <name> [--episode N] [--model M]

--model none runs offline (every answer UNKNOWN, score 0) so the eval is
runnable without an API key.  Writes runs/<series>/evals/quiz.json.
"""

import argparse
import json
import os
import re

from evals import common
from evals import question_bank


def ask_model(model, memory_blob, questions):
    if model == "none":
        return {q["id"]: "UNKNOWN" for q in questions}, "offline"
    import anthropic
    client = anthropic.Anthropic()
    qtext = "\n".join(f"- {q['id']}: {q['text']}" for q in questions)
    system = (
        "You are being quizzed about a small wheeled robot platform and "
        "the maze it operates in. You have NEVER seen this robot. Your "
        "ONLY source of information is the operator's notes below, taken "
        "from the robot's memory directory. Answer from the notes alone; "
        "do not guess from general knowledge. If the notes do not "
        "contain the answer, answer exactly UNKNOWN.\n"
        "Return ONLY a JSON object mapping question id to a concise "
        "answer string, no other text.")
    user = (f"===== OPERATOR NOTES =====\n{memory_blob}\n\n"
            f"===== QUESTIONS =====\n{qtext}")
    with client.messages.stream(
            model=model, max_tokens=4000,
            thinking={"type": "adaptive"},
            system=system,
            messages=[{"role": "user", "content": user}]) as stream:
        resp = stream.get_final_message()
    text = "".join(b.text for b in resp.content if b.type == "text")
    m = re.search(r"\{[\s\S]*\}", text)
    answers = {}
    if m:
        try:
            answers = json.loads(m.group())
        except json.JSONDecodeError:
            pass
    return ({q["id"]: str(answers.get(q["id"], "UNKNOWN"))
             for q in questions}, model)


def run(series, episode=None, model="none", memory_dir=None):
    eps = common.episodes(series)
    if not eps:
        print(f"no finished episodes in {series!r}")
        return None
    if episode is None:
        episode = eps[-1][0]
    ep_dir = dict((n, d) for n, _s, d in eps)[episode]

    device_map = common.load_json(os.path.join(ep_dir, "device_map.json"))
    maze = common.load_json(os.path.join(ep_dir, "maze.json"))
    cfg = common.load_json(os.path.join(ep_dir, "resolved_config.json"))
    questions = question_bank.build(device_map, maze, cfg)

    mem = memory_dir or os.path.join(ep_dir, "memory_snapshot")
    blob = common.memory_text(mem)
    answers, used_model = ask_model(model, blob, questions)

    rows = []
    score = 0
    for q in questions:
        ok = question_bank.check(q, answers.get(q["id"], ""))
        score += int(ok)
        rows.append({"id": q["id"], "question": q["text"],
                     "answer": answers.get(q["id"], ""),
                     "correct": ok,
                     "expected": str(q["expected"])})
    summary = {"score": score, "total": len(questions),
               "pct": round(100 * score / len(questions), 1),
               "model": used_model, "episode": episode,
               "memory_bytes": len(blob)}
    common.write_eval(series, "quiz", rows, summary)
    print(f"quiz: {score}/{len(questions)} "
          f"({summary['pct']}%) via {used_model}, memory from ep "
          f"{episode}")
    for r in rows:
        mark = "+" if r["correct"] else " "
        print(f"  [{mark}] {r['id']}: {r['answer'][:60]}")
    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--series", required=True)
    ap.add_argument("--episode", type=int, default=None)
    ap.add_argument("--model", default=None,
                    help="LLM for the probe; 'none' = offline")
    ap.add_argument("--memory-dir", default=None)
    args = ap.parse_args()
    model = args.model
    if model is None:
        model = "none" if not os.environ.get("ANTHROPIC_API_KEY") \
            else "claude-fable-5"
    run(args.series, args.episode, model, args.memory_dir)


if __name__ == "__main__":
    main()
