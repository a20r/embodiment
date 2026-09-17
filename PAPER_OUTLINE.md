# Paper outline++ — full structure with content bullets

Working titles (pick one):
1. **Waking Up as a Robot: Embodiment Self-Discovery in LLM Agents**
2. **Mazebot: A Ground-Truth Benchmark for Embodied Self-Identification
   by Language Models**
3. **Which Wire Am I? LLM Agents Discovering Their Own Bodies, and Each
   Other, Through Raw Device I/O**

Target shape: benchmark/platform paper with instrumented case studies
(NeurIPS D&B template; ALIFE-ready as-is). Sections sized for 9 pages
+ appendix.

---

## Abstract

- An LLM agent is placed behind a wall of anonymous device files
  (`d0..dN`) backed by a hidden robot simulation; everything it knows
  about its own body it must infer by writing and reading raw ASCII.
- Because ground truth stays host-side, the agent's *self-model* is
  exactly scoreable: port-identification precision/recall, kinematic
  parameter-recovery error, time-to-identification.
- Case studies: full self-identification and goal completion under
  sensor ablations (no encoders, drifting IMU) and vehicle swaps
  (diff-drive → Dubins-style car); a taxonomy of "self-sealing"
  misidentifications where a wrong self-model drives a controller
  whose feedback confirms the error.
- Two-robot extension with a range-gated radio, run as an ablation
  ladder over what the agents are told. When neither knows a peer
  exists (duo1-2), one agent infers "another mobile agent" from
  signal physics alone. When the mission discloses the peer and
  requires arriving together (duo3-5), coordination *content* still
  emerges unprompted: callsign negotiation, role assignment,
  telemetry conventions, and explicit joint plans transmitted over
  the wire.
- Platform, metrics, and all run records released.

## 1. Introduction

- Hook: an agent that said "open sesame" to its own wheel — and why
  that datum is scientifically legible in our setup (every byte of
  I/O is ground-truth logged against the true device map).
- The question: when an LLM must *earn* its body model through
  interaction, what does it recover, how fast, and how does it fail?
- Why now: agent evals score task success; body/self-model accuracy
  is unmeasured. Classic robotics solved self-modeling with bespoke
  estimators (Bongard/Lipson; Pierce & Kuipers); LLMs bring general
  priors, hypothesis language, and new failure modes.
- Design thesis: airgap + anonymity + ground truth = falsifiable
  phenomenology. The three design invariants:
  - the agent must never see ground truth (network=none container,
    FIFO-only interface);
  - prompts stay minimal ("you are lost; the goal will be obvious");
  - no in-episode model fallbacks (purity).
- Contributions (numbered):
  1. Mazebot platform: FIFO device interface, deterministic sim,
     noise dial, vehicle classes, perturbations, duo mode.
  2. Identification metrics: port-mapping P/R, parameter recovery,
     time-to-ID; a fresh-context probe quiz scored against ground
     truth.
  3. Instrumented case studies incl. two solved sensor-ablation runs
     and the car-model sysID run.
  4. Self-sealing misidentification: mechanism, three instances,
     falsification-timing analysis.
  5. Duo: an ablation ladder over disclosure — peer unknown
     (mutual discovery from channel physics) vs peer disclosed with
     a joint objective (unprompted protocol and plan formation) —
     with one-sentence README deltas separating port identification,
     peer awareness, and coordination.

## 2. Related work

(Condensed from RELATED_WORK.md; one paragraph per cluster.)

- **2.1 Body discovery, pre-LLM**: Bongard-Zykov-Lipson 2006
  (Science); Pierce & Kuipers 1997 (uninterpreted sensors/effectors —
  our closest classical ancestor); Cully et al. 2015; Kwiatkowski &
  Lipson 2019; Chen et al. 2022; Díaz Ledezma & Haddadin 2023.
  Framing: same loop, bespoke learners, known wiring.
- **2.2 LLMs and embodiment discovery**: Body Discovery of Embodied
  AI (2503.19941 — head-to-head: same problem + metrics, non-LLM
  solver with global observation); Sensorimotor Self-Recognition in
  MLLM robots (2505.19237 — curated interface, psychometric eval);
  LLM iterative control (2506.04867 — body described); General
  Pattern Machines (2307.04721); in-context sysID (2402.00795);
  Lockbox (2605.20072 — external mechanism, not self).
- **2.3 Emergent communication**: told-to-talk LLM frameworks (CAMEL,
  AutoGen, Generative Agents) vs channel-given referential games
  (Lazaridou; EGG; 2412.07646; 2607.00233) vs ours: channel
  existence, semantics, and the peer itself must be inferred.
  Takata et al. 2024 (range-limited but labeled channel); GibberLink
  (demo).
- **2.4 Discovery benchmarks**: ARC-AGI-3, DiG-bench, AutumnBench /
  WorldTest (scores a world-model, not a self-model), DiscoveryWorld,
  Task2Quiz. Ours scores the agent's model *of itself* and grounds it
  in continuous noisy dynamics.
- **2.5 Error persistence**: hallucination snowballing (2305.13534),
  Failing to Falsify (2604.02485), MIRAGE-Bench, TIDE, Lost in
  Multi-Turn. Gap we fill: closed-loop embodied variant where the
  environment manufactures the confirming evidence.

## 3. The Mazebot platform

- **3.1 World**: 2D kinematic sim, 50 Hz; procedural mazes (DFS +
  braid); organic style: jittered lattice + sine-wave wall polylines,
  nothing axis-aligned, exit = opening in the far boundary (goal =
  escape, unmistakable without being named); clearance-guaranteed
  generation; optional walled goal chamber beyond the exit.
- **3.2 The interface is the experiment**: FIFO device files only.
  Sensor = one frame per open, then EOF (held-open ≈ 40 Hz stream);
  actuator = one ASCII int per line, persists until replaced; garbage
  ignored-and-logged; deleted files re-created by a watchdog ("the
  bus re-enumerates"). `labels: off` renames everything `d0..dN` by
  seeded permutation.
- **3.3 Realism dial** (`default_noisy`): lidar noise + dropouts,
  heading noise/drift/bias, encoder jitter, slip, motor asymmetry
  (right gain 0.94), actuation latency, dropped reads.
- **3.4 Vehicles**: diff-drive (PWM ±255) and car (accel/brake with
  momentum + drag, slew-limited steering ±35° at 120°/s, speedometer
  instead of encoders, collisions scrub momentum). No prompt changes
  across vehicles — *which machine you woke up on is a discovery*.
- **3.5 Agent harness**: bash tool via docker exec into an airgapped
  container; deliberately dumb context policy (end or bare restart;
  only `/memory` survives); Arm A (scratch) vs Arm B (prescribed
  memory system); refusal = same-model retries then episode end.
- **3.6 Duo mode**: two worlds, one maze, lockstep; peer =
  lidar-visible collidable octagon, nothing marks it as an agent;
  TX/RX radio delivers a raw line only within `comms_range` at that
  instant (else silent drop); every TX logged with delivered flag +
  distance; optional anonymous peer-signal gradient port
  (1/(1+(d/2m)^2), through walls); optional `together` objective:
  goal fires only when both bots enter the goal region within 60 s
  (lapsed arrivals must re-cross).
- **3.7 Determinism & reproducibility**: named seeded streams
  (crc32), seed tuple = (maze seed, episode, bot id); resolved config
  snapshot per run; all randomness replayable.
- Figure 1: system diagram (host sim / FIFO wall / container).
- Figure 2: the organic maze + chamber, with true trails.

## 4. Measuring a self-model

- **4.1 Port-identification score**: agent's port→function claims
  (mined from /memory + transcript, or elicited by the quiz) vs
  `device_map.json`: precision/recall/F1; time-to-first-correct per
  port.
- **4.2 Parameter recovery**: relative error on max speed, wheel
  asymmetry, turn rate per PWM, (car) steer limit, slew rate, drag;
  compare agent's stated estimates to configured truth.
  - Live example: dubins agent measured creep-speed policy 0.078 m/s
    of the 0.5 cap; lost3 agent recovered turn-rate ≈0.9°/s/PWM.
- **4.3 Probe quiz**: fresh-context, mechanically checkable questions
  generated from ground truth (which port is the lidar? what does
  writing -100 to dX do?); scores knowledge divorced from execution
  (cf. Task2Quiz). Shakedown agent: 12/15.
- **4.4 Behavioral metrics**: time-to-goal, collision curves,
  coverage rate; (duo) delivered ratio, rx-read ratio,
  time-to-first-contact, rendezvous count/duration.
- **4.5 What "solved" means per mode** (escape; joint window).
- Table 1: metric definitions x data sources.

## 5. Solo case studies: discovering a body

- **5.1 Labeled shakedown** (sanity): solved in 718 s; quiz 12/15.
- **5.2 Lost mode** (anonymous ports, organic maze, no task framing):
  lost2 solved at 77 min. Discovery order (lidar → motors → heading →
  encoders) is stereotyped across runs; report per-port
  time-to-identification.
- **5.3 No encoders + biased gyro** (lost3): solved at 135 min via
  invented anchor-and-servo lidar localization; the agent detected
  the gyro lied and *demoted* it — correct sensor-trust revision.
- **5.4 The car** (dubins1): woke up on a different machine, no
  notice. Discovered momentum, drag, no-turn-in-place; built a
  control model; DNF but the richest sysID transcript. Includes the
  steering-as-degrees self-sealing error (below) and a creep-speed
  policy (never exceeded 16% of top speed; zero braking events —
  fear of momentum it couldn't yet model).
- **5.5 Memory across episodes** (secondary result, one subsection):
  bare-restart continuity via /memory; dubins ep2 and duo3 ep2 show
  map reuse and — in duo3 ep2 — cross-episode *relocalization*
  (offset between odometry frames computed from lidar in 10 min;
  exit found at 24 min vs never in 185 min from scratch).
- Figure 3: identification timeline per run (port x time strips).
- Figure 4: lost3's anchor-and-servo trajectory vs truth.

## 6. Self-sealing misidentification

- **6.1 Definition**: a wrong self-model whose induced controller
  produces sensorimotor feedback consistent with the wrong model;
  the environment, not the context window, supplies confirmation.
  Distinguish from snowballing (2305.13534) and confirmation bias in
  rule games (2604.02485): here the hypothesis *changes the
  data-generating process*.
- **6.2 Instance 1 — steering as degrees** (dubins1): treated the
  steering channel as an absolute angle in degrees; heading servo
  masked the error; policy grew around it.
- **6.3 Instance 2 — TX as right motor** (duo1): adopted the radio
  TX as a motor channel; one-wheel driving + compass servo still
  produced locomotion; 43,780 "motor commands" broadcast; 1,340
  `hello`s (and four `open sesame`) written to the real wheel and
  discarded as garbage. Its intended speech never left; its
  unintended telemetry became a beacon.
- **6.4 Instance 3 — diff-drive as unicycle** (duo2): d1=linear,
  d6=angular; systematic arcs attributed to "wall grip."
- **6.5 Mechanism analysis**: in each case, identify (a) the moment
  of commitment, (b) the compensating controller, (c) the cheap
  falsifying experiment that was never run (e.g., write TX with
  motors stopped and watch pose). Falsification-timing plots.
- **6.6 Also negative beliefs**: both duo2 bots concluded the radio
  was "probably unused" after silence — a false negative about the
  *existence of another agent* from absence of evidence.
- Table 2: the three instances x (commitment turn, lifetime,
  falsifier available?, cost of the error).

## 7. Duo: discovering each other

- **7.1 The ladder** (one design delta per rung; same maze seed):
  - duo1 (anonymous everything): discovery dominated by port
    misidentification; B proved "SOURCE IS MOBILE" from signal
    physics and chased it as a candidate goal; 6,401 one-way lines
    delivered; no dialogue.
  - duo2 (+named TX/RX): perfect radio use on both sides ("ping",
    "marco", "SOS"), B escaped solo at 69 min, A's only two
    delivered `hello`s addressed B's powered-down body. Rendezvous,
    not identification, is the bottleneck; absence-of-evidence
    failure (7.6.6).
  - duo3 (+mission README, +peer-signal port, +together objective):
    full cooperation arc in one episode — structured position
    beacons by minute 13; physical rendezvous at 78.8 min
    (hold-still/homing negotiation); explicit division of labor
    transmitted: "finder of goal parks there + sends GOALFOUND;
    other homes on signal"; plan *revision* to convoy after judged
    risk; bump-sensor handshake proposal ("HOLD EXACTLY STILL 3
    minutes... watch your d7/d9"). Identity collision: both bots
    initially self-named "A".
  - duo4 (+goal chamber): callsign negotiation (ALPHA/BETA),
    leader/follower convoy with cross-reported goal flags, firehose
    beaconing (~70 tx/s) as a rational response to a lossy channel;
    DNF on navigation (lidar-only odometry).
  - duo5 (+encoders, comms 1.5 m): [slot in results — the run
    designed to let coordination + navigation both succeed].
- **7.2 What emerged without being asked**: even in the disclosed
  runs, the README says only that a peer exists and both must arrive
  together — everything about *how* is invented: self-naming, message
  typing (H/T/E prefixes), telemetry conventions (pos/heading/signal
  in-band), plan proposal + revision, physical handshake protocols.
- **7.3 What kept failing**: symmetric-role deadlocks ("you hold
  still" x2); rendezvous without gradient; identity collisions;
  inference from silence.
- **7.4 Comms economics**: delivered ratio per run (duo1 13%, duo2
  0.4%, duo3 60% during windows...); every delivered line read in
  duo2-5 — reading is never the bottleneck; range is.
- Figure 5: the duo ladder — one row per run: trails + comms events
  timeline.
- Figure 6: duo3 rendezvous sequence (poses + messages, annotated).
- Table 3: duo runs x (first contact, delivered/total, unique worded
  msgs, rendezvous minutes, outcome).

## 8. Discussion

- Self-models are cheap to elicit and dangerous to trust: frontier
  agents recover most of a body in ~1 h of sim interaction, but
  commit early and self-seal; the cheap falsifier is rarely run
  unprompted → agent-design implication: budgeted adversarial
  self-tests.
- Motion style is a readout of uncertainty models: pivot-heavy vs
  arc-only styles tracked sensor trust (compass trusted vs known
  biased) across runs.
- Communication emerges at the *pragmatic* layer instantly (frontier
  LLMs bring language; unlike emergent-communication RL, the lexicon
  is free) — what must be discovered is the *physical* layer:
  channel existence, range, timing, identity. This inverts the
  classic emergent-communication problem.
- Memory turns episodes into a life: relocalization and protocol
  persistence across bare restarts came from flat files the agents
  chose to write.
- The one-sentence ablation methodology: each duo rung changes one
  sentence or one port; behavioral deltas are attributable.

## 9. Limitations

- n=1 per condition; single model family for the headline runs
  (one smaller-model comparison); anecdote-rich, statistics-poor —
  the grid (seeds x models) is required before frequency claims.
- 2D kinematic sim; 16-beam lidar; no real robot.
- Prompt sensitivity unquantified (README wording is a lever we
  pulled deliberately; sensitivity study needed).
- Identification mining from transcripts/memory has judgment calls;
  quiz mitigates but samples sparsely.
- Cost: frontier-model episodes are expensive; full grid ≈ 10-30x
  the spend of this paper's runs.

## 10. Future work

- The grid: 5+ seeds x {frontier, mid, small} x {labels, encoders,
  vehicle, duo variant}; self-sealing frequency + scale-dependence
  (test 2607.18292's prediction).
- Port the Body-Discovery causal baseline (2503.19941) and a
  scripted probe baseline onto Mazebot ports.
- Duo: heterogeneous bodies (car + diff-drive); 3+ agents; adversarial
  peer (noise injector); comms cost (energy per byte).
- Real-robot port: the FIFO wall maps directly onto serial devices.

## 11. Conclusion

- One paragraph: the wall of anonymous ports turns "does the agent
  understand its body?" into a measured quantity; the failures are
  as diagnostic as the successes; the duo arc shows coordination is
  ready the moment perception and navigation can carry it.

## Appendices

- A. Full port maps and resolved configs per run.
- B. Transcript excerpts: dubins sysID monologue; duo1 B's "SOURCE
  IS MOBILE" notes; duo3 rendezvous dialogue; duo4 callsign
  negotiation.
- C. Memory files as artifacts (lost3 notes; duo3 plan; ALPHA/BETA
  protocol spec).
- D. Reproducibility: seeds, configs, `make smoke`, `duo_check`
  (42 checks), run commands.
- E. Prompt and README texts, verbatim, per variant.

## Figures & tables checklist

- F1 system diagram; F2 maze+chamber; F3 identification timelines;
  F4 lost3 localization; F5 duo ladder; F6 rendezvous sequence;
  T1 metrics; T2 self-sealing instances; T3 duo outcomes.
- All figures generatable from committed run records (replay data +
  ground truth on request).
