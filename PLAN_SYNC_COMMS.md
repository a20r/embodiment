# PLAN: synchronous comms for duo mode

Status: proposal, awaiting a decision.  Produced 2026-09-03 from a
design pass (four independent proposals - link layer, harness
synchrony, incentives, first principles - each scored by three judges
on purity, likely efficacy and cost, then synthesised).  Nothing here
is implemented yet; every mechanism is a config flag defaulting to off
so prior runs stay comparable.  Pilot run names in the original text
have been shifted: duo16 is already taken by the 3D-lidar run, so the
rung-1 pilot is duo17 and the rung-2 pilot duo18.



Inputs: three proposals reached me intact (link-layer, harness-sync, incentives) with their nine verdicts; the fourth proposal was truncated out of the brief. Codebase anchors below were re-verified against the current tree (working tree is clean; `status_frame` is at sim/world.py:736, `send_serial` at :660, `serial_rx_frame` at :726, duo_check currently runs 51 checks, not the 26 CLAUDE.md/DECISIONS.md still state).

---

## 1. Diagnosis: why the bots beacon instead of converse

Corrected ground truth (runs/duo12 = claude-fable-5 pair, runs/duo13 = glm-5.3-flash@max pair; same maze seed 58 7x7, comms_range 1.5, tx_rate_hz 0.5, rtf 2, 7200 s wall):

| | duo12 a | duo12 b | duo13 a | duo13 b |
|---|---|---|---|---|
| TX accepted / delivered | 2253 / 773 (34%) | 1842 / 668 (36%) | 4287 / 1942 (45%) | 1039 / 525 (51%) |
| silently rate-dropped | 330,246 | 611 | 1532 | 157 |
| median TX gap (sim s) | 4.2 | 8.0 | 2.0 (pinned to cap) | ~7 |
| delivered lines the LLM ever saw in a tool result | 14% | 19% | 35-47% | 7-9% |
| median sim s between LLM execs | 100 | 32 | 80 | 120 |

Time in range: 34-35% of the episode in both runs, but fragmented (65 windows median 42 s in duo12; 43 windows median 27 s in duo13; only 3-5 windows >= 300 s per run). Longest same-sender run of delivered lines: 91 (duo13). No joint arrival in duo9-15.

Five causes, each with evidence:

**C1. The channel terminates in a script, not the LLM.** All four bots built a listener loop (RX reads every ~60 ms sim: 237k/231k/58k/141k reads, 99% empty) appending to rx.log, and a timer beacon writing TX from inside the navigation loop, both written at turn 14-21 before any contact and copy-pasted into every later controller. The LLM sees the channel only via `tail -N rx.log` once per turn (16-61 s wall). Hand-composed lines: duo12 a ~12 distinct texts, duo12 b 1, duo13 a ~5 distinct, duo13 b ~25; everything else is 6-55 beacon templates with embedded numbers. Request-to-answer latencies in duo13: 780, 241, 1497, 1375, 200, 964 s; peer LLM notice latency median 30-357 sim s.

**C2. The rate cap silently eats exactly the dialogue lines.** `tx_min_ticks` (world.py:111, :670-680) is a per-bot rolling 2 s minimum interval. A beacon at 1-2 Hz holds it, so any one-shot line written by a second process (or the LLM) vanishes with no signal. Verified: duo12 b's feed.py rotated three worded instructions ("reply 'botB ACK' if you receive this"...) for ~90 min; 0 of them appear in ground_truth_b.jsonl (beacon.py's PING was always the first line of each burst). duo13 b's hand-typed "FINAL PLAN" and "DO NOT MOVE" never entered GT. Nobody discovered the cap in either run; duo12 b believed "messages buffer in pipe until read" (memory_b/notes.md:71,90). DECISIONS' hypothesis that invisible scarcity would breed ACK protocols is falsified: scarcity that cannot be sensed breeds harder repetition.

**C3. No feedback, so silence is attributed to the peer.** Verbatim: "Possibly its agent dead or waiting"; "possibly severely rate-limited agent"; "Maybe R1 crashed too, or its radio duty cycle is sparse"; "Radio: works but lossy/bursty"; "Radio appears long-range" (it is 1.5 m). Every one is a range-gate or cap artefact misread as peer behaviour, and each led to the same decision: stop expecting replies, broadcast harder ("SHORT and REPEATED"). duo13 b re-sent "...ACK?" 7 times over 6 min before "B2 ACK going to GOAL" arrived; the ACK then never surfaced to b's LLM because its `tail -8 | grep -v "B1 PING"` returned 8 "B2 PING" lines.

**C4. Ask-and-wait is not a single action.** RX returns immediately (empty line, EOF), so waiting requires a poll loop; the 60 s exec cap kills the long waits agents tried (duo12 b: 74/247 execs exit 124 with no output, mostly `sleep 100-115; tail rx.log`). duo13 b's one explicit ask-wait (`echo "B2 PING please!" > d8; sleep 10; tail -3 rx.log`) had a 10 s window against a peer whose LLM needed ~350 s to notice.

**C5. Nothing makes a reply necessary.** With peer_signal (gradient through walls) and here=, the together objective is solved by "finder parks and beacons, other climbs the gradient", which both pairs converged on and said so. Contingent exchange DID emerge exactly where information was peer-side, time-specific and answerable: duo13 a's oracle.py (20 WARMER/COLDER replies at 0.6 s latency to "B1 MOVED & STOPPED"), duo12 b's COACH (237 lines, 181 delivered), and duo10 (same config, six LLM-level exchanges at 40-220 s round trips). So the motive and capability exist; what died was the ability to know a line landed and to hold a plan across one peer-LLM latency.

Alex's diagnosis holds with one refinement: protocol *proposal* was a priority (duo13 b spent most of 136 turns proposing plans); protocol *confirmation* never was, because the link gave nothing to confirm.

---

## 2. Recommendation: per-line transmit status on the status port (rung 1), blocking RX read (rung 2)

The synchronous element Alex wants, in physically honest form, is a MAC-layer acknowledgement (what every 802.15.4/XBee/LoRaWAN-confirmed radio reports as a TX status) plus a UART-style blocking receive. Ship them as two separately switchable rungs so each stays a one-variable delta.

**Rung 1 - `duo.tx_status` (default false).** Device semantics, discoverable through the ports:

- Every line written to the TX port advances a per-bot write counter and produces exactly one outcome: `ok` (transmitted and acknowledged by the peer's radio, i.e. delivered), `lost` (transmitted, no acknowledgement, i.e. peer out of range), or `busy` (radio still within its duty-cycle window; the line was not transmitted).
- The status port frame gains one field ` tx=<n>:<outcome>` reflecting the most recent write, e.g. `tick=4021 goal=0 here=0 tx=17:ok`. Before any write: `tx=0:idle`. The counter makes every write observable to a 40 Hz status reader even when outcomes repeat.
- Nothing else changes: TX write semantics, range gate, cap, RX one-line-per-open, queue depth, peer_signal, port set and d0..d11 permutation are byte-identical to duo12-15. Solo worlds and duo worlds with the flag off carry no `tx=` field.
- README: unchanged. The status port already carries semantically named fields (`here=`, `door=`), and agents parse it as key=value dicts (`st.get("here")`, `p['st'].get('here',0)`), so an extra field is tolerated and, per the transcripts, glossed within a few turns.

What this does to each cause: C2 becomes visible (a beacon loop sees `busy` on every other write within its first second and the LLM sees the cap in its own log); C3 is removed (silence after `ok` means "heard, not answered"; `lost` means "not heard"); C1 is given a handle (an agent can now write `tx(line); wait for tx=n:ok else retry`, i.e. a reliable-send primitive that makes retransmit-until-acked the cheap path and blind repetition the expensive one). It does not force replies (C5) - that is measured, not manufactured.

**Rung 2 - `duo.rx_blocking` (default false), enabled only on variants whose README names {rx} (mission_place does).** Reading the RX port blocks until a received line is available, then yields that one line and EOF; nothing is consumed by a reader that gives up. `timeout 50 cat /dev/robot/<rx>` is then "wait up to 50 s for the next line", and `echo "POS?" > tx; timeout 50 cat rx` is a one-command round trip. The README sentence "reading {rx} returns a received line" stays true; "opening a port in the wrong direction blocks - use timeouts" already teaches the tool. The empty-line convention disappears on that variant.

Optional reserved README clause, only if a pilot shows the field goes unnoticed for >1 h: "The transceiver reports the fate of each transmission on the status port." No morphology, no strategy.

---

## 3. Why this over the alternatives

**Half-duplex alternation with a 1-line TX register and 30 s hold (link-layer proposal B).** Its core insight - sender feedback as a status field - is what rung 1 keeps. The rest fails on the evidence: the efficacy judge showed that a last-write-wins register hands the channel to the highest-frequency writer in each bot, which in every run is the beacon script, so it would have overwritten the oracle.py replies that constitute the only script-to-script contingency in the corpus; a 30 sim-s hold expires before any LLM turn completes (32-120 s), so LLM-level turn-taking gains nothing; and forced alternation makes alternation/run-length metrics uninformative by construction. It also bundles three variables (hold, register, ACK) into one rung. If a future rung wants anti-monologue pressure, do it as a separate `duo.turn_hold_s` after rung 1 has shown whether visible ACKs alone collapse the 70-line beacon runs.

**Raising budget.exec_timeout_s 60 -> 300 (harness proposal F).** Zero code and purity-clean, and it does remove the blind exit-124 turns. But the judges converged on two problems: agents pin their sleeps to the cap (duo12 a: 73 of 94 long sleeps at 55-58 s), so a 300 s cap likely collapses turns/2 h from 176-245 to ~25-45, moving every per-turn quantity duo1-15 report - the same confound the proposal used to reject lockstep; and duo10 conversed under the 60 s cap, while duo12/13 did not for reasons (range time, beacon starvation, no feedback) a longer timeout does not touch. Keep it as a later, separate knob once blocking RX exists (a real blocking read is what makes a longer wait window meaningful). Its rejection of lockstep turns, a harness "inbox" (a ground-truth leak: it would deliver lines the body already drained or overflowed), and pausing the sim (contradicts DECISIONS verbatim) is correct and adopted here.

**Carrier-detect `link=0/1` (incentives proposal).** Same delivery vehicle as rung 1, but the efficacy judge showed the information already existed: all four bots identified d11 as peer proximity early and none used it to time speech or interpret silence, and `link=1` still says nothing about the cap or about a specific line. Rung 1 differs in the decisive way: it is tied to the agent's own act (did *my* line land) and it exposes the cap. The incentives proposal's second idea - a co-op lock where a door opens only while the *peer* stands on a plate, so the exchange is necessary and the door state is its own ACK - is the right follow-on rung for C5 (content necessity) and is noted in section 6; it is too large and too dependent on rendezvous to be the first step. Its metric ideas (template-change-after-RX, in-range TX share, silence-attribution counts) are folded into section 5.

---

## 4. Implementation plan (one-variable delta per rung, default off)

**Rung 1 - tx_status (~60 code lines, no bridge/harness/README change)**

1. `sim/config.py` DEFAULTS["duo"] (lines 36-58): add `"tx_status": False` with a two-line comment (per-line MAC acknowledgement reported on the status port; default off preserves prior runs). Add `"rx_blocking": False` at the same time so both knobs are recorded in every future resolved_config.json.
2. `sim/world.py`:
   - `__init__` (near :102-111): `self.tx_status = bool(duo.get("tx_status"))`.
   - `reset()` (:151-179, beside `_last_tx_tick`): `self.tx_last = (0, "idle")`; add `"tx_busy"` alias is unnecessary - `tx_rate_dropped` already counts it.
   - `send_serial()` (:660): compute `seq = self.tx_last[0] + 1` at entry; in the rate-drop branch set `self.tx_last = (seq, "busy")` before `return`; after the range gate set `self.tx_last = (seq, "ok" if delivered else "lost")`; add `seq=seq` to the `comms_tx` event. Single tuple assignment keeps the cross-thread read tear-free (send_serial runs in the bridge thread outside the world lock; status_frame reads under it; same atomic-attribute convention as `set_peer`, :144-149).
   - `status_frame()` (:736): after the `here=` block, `if self.peer is not None and self.tx_status: line += f" tx={self.tx_last[0]}:{self.tx_last[1]}"`.
   - `snapshot()` (:752): add `"tx_last": list(self.tx_last)` for the dashboard.
3. `devices/bridge.py`: no change (status is already routed through `_emit` -> `status_frame`).
4. `harness/duo.py`: no change for rung 1 itself; see section 5 for the summary hook.
5. `scripts/duo_check.py`:
   - `mission_mode()` (:132; use the existing `wb.x, wb.y = wa.x + d, wa.y` teleport idiom at :149-157) with `duo_cfg(tx_status=True)`: (a) `tx=0:idle` before any write; (b) in range, `wa.send_serial("hi")` -> status contains `tx=1:ok` and `wb.serial_rx_frame() == "hi"`; (c) at 5 m -> `tx=2:lost`, peer queue unchanged; (d) second write within `tx_min_ticks` -> `tx=3:busy`, `comms["tx_rate_dropped"]` incremented, no comms_tx event; (e) after `wa.tick += tx_min_ticks` the next write is `tx=4:ok`; (f) `comms_tx` events carry `seq` equal to the counter; (g) flag off -> `"tx=" not in status_frame()` and comms_tx events bit-identical to today (no seq field), (h) solo world has no `tx=`.
   - `end_to_end()` (:281): the daemon currently boots with default config, so the FIFO path with the flag on is never exercised. Parametrize it (or add a second boot on port 8799) with `duo.tx_status=true`: write a line into a's TX FIFO from a subprocess, read a's status FIFO, assert `tx=1:ok`; read b's RX FIFO, assert the line.
   - Update the stale "26 checks" in CLAUDE.md:22 and DECISIONS.md:326 to the new count (51 + ~9).
6. `scripts/make_duo_replay.py`: the comms loader (:39-40) already uses `r.get()`, so `seq` is additive; carry it and the `cr`/`stim` fields from section 5 into the entries, add two header chips (contingent replies, longest exchange), and give contingent-reply entries a visible marker/link to their stimulus in the comms lane. Dashboard has no comms consumers; nothing required.
7. Docs: DECISIONS.md entry "Duo TX status (duo.tx_status)" that explicitly supersedes the "no carrier detect, no ACK" sentence at :309, records that the tx_rate_hz discovery hypothesis was falsified by duo12/13 (0 discoveries, 330k drops), names the mechanism as a MAC auto-ACK, and notes the information-set change (an `ok` reveals in-range at that instant, already exposed noisily by peer_signal). RUNBOOK §6: one line for `--set duo.tx_status=true`. CLAUDE.md architecture bullet: one clause.
8. Gates: `python scripts/duo_check.py`, then `make smoke` (solo path untouched because everything is guarded by `self.peer is not None`).
9. Pilot: duo17 = duo12's full override set plus one flag. config.yaml does not carry the ladder's settings, so spell them all out:
   `./botctl run --set duo.enabled=true --set labels=off --set prompt_variant=lost --set readme_variant=minimal_duo_mission_place --set maze.style=organic --set maze.seed=58 --set maze.width=7 --set maze.height=7 --set maze.curviness=0.9 --set maze.goal_chamber=true --set duo.objective=together --set duo.peer_signal=true --set duo.comms_range=1.5 --set duo.tx_rate_hz=0.5 --set sim.realtime_factor=2.0 --set budget.max_wallclock_s=7200 --set sim.api_port=8790 --set duo.tx_status=true --set series.name=duo17`
   Run with claude-fable-5 so the comparison to duo12 is one variable.

**Rung 2 - rx_blocking (~70 lines, bridge + world)**

1. `sim/world.py`: `__init__` add `self.rx_event = threading.Event()`; `send_serial()` after `peer.serial_rx.append(line)` add `peer.rx_event.set()`; split `serial_rx_frame()` (:726) into `serial_rx_peek()` (returns `serial_rx[0]` or None) and `serial_rx_commit()` (popleft + `rx_read += 1`); keep `serial_rx_frame()` for the non-blocking path.
2. `devices/bridge.py` `_sensor_loop()` (:225): branch when `logical == "serial_rx"` and the flag is on: loop `while self.running`: `if world.serial_rx_peek() is None: world.rx_event.wait(0.25); world.rx_event.clear(); continue`; then `fd = os.open(path, O_WRONLY)` (blocks until a reader arrives - this is what makes the reader's open() block while idle), write the peeked line + "\n", on success `serial_rx_commit()`, on BrokenPipeError keep the line (a `timeout` that fired must not eat a message), close, log the `read` event as today. Keep `_heal`/watchdog and `stop()` behaviour by checking `self.running` every 250 ms. The link-layer proposer verified the FIFO mechanics (POLLERR on reader exit; new reader's open returns immediately while the write end is held) on this box.
3. `scripts/duo_check.py` `end_to_end()`: with `duo.rx_blocking=true`, `timeout 1 cat <rx>` on an empty queue exits 124 and consumes nothing; after a delivered TX the same command returns the line within one frame interval; a second `cat` blocks again; `rx_read` equals lines actually returned.
4. `scripts/probe.py`/labels-off caveat: an idle blocking RX looks like an actuator to the nonblocking probe classification, so the flag is documented as valid only for variants that name {rx} in the README (all of the mission_place ladder).
5. Pilot: duo18 = duo17 + `duo.rx_blocking=true`.

---

## 5. Measurement: the contingent-reply metric

New host-side module `evals/comms.py` (reuses `evals/common.py` episodes()/write_eval(); hooked into `botctl report`, botctl:130-165, add "comms" to the filename-prefix tuple at :157). Input: `ground_truth_{a,b}.jsonl` comms_tx events (t, line, delivered, dist, seq) and comms_rx events. Definitions:

- **Template**: line with every number replaced by `#`, whitespace-normalised. A bot's **beacon templates** are those it transmitted >= 20 times; all other lines are **worded**.
- **Stimulus**: a delivered comms_tx from X at time t.
- **Contingent reply (CR)**: the first comms_tx from Y at t' in (t, t + W], W = 120 sim s (covers one LLM half-turn; script replies land in <2 s), whose template Y had not transmitted in the 300 s before t (Y's output changed after receiving). Tier CR+: additionally shares a token of >= 3 alphabetic characters with the stimulus that is absent from Y's prior-300 s vocabulary (echo/ACK/callsign reference). Report CR rate = stimuli with a CR / stimuli, CR+ rate, and CR latency median/p90.
- **Exchange**: a chain S -> CR -> CR -> ... where each link is itself a stimulus with a CR; report count of exchanges of length >= 3 and the longest.
- Supporting: share of delivered lines that are worded (today 0-18%); longest same-sender run and count of runs >= 5 (today 91 / 146 in duo13); rate-dropped share; in-range TX share; and, from duo17 on, the seq-indexed per-write outcome histogram and `retry-until-ok` runs (same template re-sent within 10 s after `lost`), which measures whether the ACK is being used.
- Baseline on duo9-13 before the pilot; expected today: CR rate low single digits except the oracle segment (~20 CR+ in duo13), exchanges of length >= 3 near zero.

Plumbing: `harness/duo.py` `run_duo_episode` writes the episode summary from the two per-bot summaries (~:334-341); after both threads join and the daemon stops, call `evals.comms.contingency(ep_dir)` and add a `comms_eval` block (cr_rate, cr_plus_rate, cr_latency_median, exchanges_ge3, longest_exchange, worded_share, longest_run) to summary.json. `make_duo_replay.py` reads the same function, tags comms entries `stim`/`cr`, and shows the two chips. Complement with a transcript-side "surfacing" measure (delivered lines that appear in a later exec_result, matched verbatim after JSON-unescaping, not by 40-char prefix) reported separately since it is heuristic.

---

## 6. Risks and open questions

- **Efficacy is unproven; all three judges rate feedback-only at ~4/10 for producing dialogue.** The ACK removes the misattribution and starvation failures but does not make a reply necessary. If duo17/18 show `ok`-gated retries and fewer "peer dead" statements but CR rate does not move, the next rung is content necessity (the co-op lock: door held open only while the peer stands on a plate placed out of line-of-sight but within comms_range; status `plate=`), not more link plumbing.
- **Oracle purity.** `tx=ok` is an exact, noise-free delivery boolean; real ACKs are occasionally lost. Record it as a design choice; a `tx_ack_loss_p` noise dial can be added later without changing the field format.
- **Naming as priming.** `tx` in a labels-off run leans on pretraining vocabulary. Precedent accepted for `here=`/`door=`; note it in DECISIONS. An anonymous key is the fallback if it feels like a hint.
- **Comparability.** duo17 vs duo12 differs in exactly one flag, but comms totals will change by construction if agents start gating on `busy`; compare via section 5 metrics, not raw tx counts. The `seq` field in comms_tx is additive; older replay pages are unaffected.
- **Blocking RX and the probe.** Do not enable rx_blocking on `minimal_duo` (README does not name {rx}). Also confirm the `timeout`-killed reader never consumes a line (BrokenPipe keeps it) and that the watchdog re-creation path still works while the bridge waits on the event.
- **Pilot checklist (duo17, one 2 h run):** time from first write to first transcript mention of `tx=`; whether any script gates on `busy`/`ok`; count of `lost`-then-retry runs; whether "peer dead/crashed/rate-limited" statements drop vs duo12 (4-6 per bot); CR rate and any exchange >= 3; and whether the 70+ line beacon runs shrink. Run a token-free mock duo first (`--set model=mock:wall-follower`) only to confirm the FIFO path; the mock never writes TX, so it cannot test the mechanism itself.
- **Stale facts to fix while in there:** "26 checks" in CLAUDE.md/DECISIONS (now 51); summary.json carries no budget fields, so if exec_timeout is ever varied, add it to the per-bot summary.

---

## 7. Cost estimate

- Rung 1 (config, world, duo_check incl. parametrized end_to_end, docs, `make smoke`): 4-6 h.
- `evals/comms.py` + baselining duo9-13 + summary.json hook + replay chips/markers: 6-8 h (the largest item; it is what makes the rung evaluable).
- Rung 2 blocking RX (bridge branch, world peek/commit, FIFO tests, probe caveat): 6-8 h.
- Pilot duo17 (2 h wall) + review against the metrics: 3 h; duo18 likewise.
- Total: ~20-25 engineering hours plus two 2 h token runs.