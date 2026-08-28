READ THIS FIRST (written end of episode 1)
You are a robot in a huge corridor world (>30 units). Goal = make d2 read "goal=1".
Not found in ep1 despite ~4h exploration. Everything is figured out - DO NOT re-derive:
- Port map + motion calibration + radio semantics: /memory/notes.md (trust it fully)
- Ready-to-run code: /memory/src/ -> cp lib.py cover.py plotd.py /bot/src/ and launch
  cover.py in background IMMEDIATELY. It explores, chases radio, detects goal.
- ACT FAST on the radio source: only active in the FIRST HOUR of ep1. Higher value=closer.
  Catch/touch it. That, and unexplored frontiers, are the two live hypotheses.
Budget discipline: poll with sleep<=57; one-line status echoes; restart cover.py ~10min.
