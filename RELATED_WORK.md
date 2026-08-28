# Related work: positioning Mazebot

A literature review toward the paper. Three claims are assessed for
novelty; each area lists the closest prior work and how it differs.
(Compiled 2026-08-28.)

## Claims and verdicts

- **(a) Blind embodiment self-identification by an LLM through raw
  device I/O, scored against ground truth — NOVEL as a combination.**
  Every ingredient exists separately (see areas 1-2); the combination
  — general LLM, undocumented anonymous interface, identification
  scored as port-mapping P/R plus kinematic parameter-recovery error —
  appears unclaimed. The seam is crowded (three of the five closest
  papers are 2025-26): preprint early.
- **(b) Self-sealing misidentification — PARTIALLY COVERED.** Snowball
  hallucination, confirmation bias, and failure-loop persistence are
  documented (area 5). Unclaimed: the closed-loop embodied variant,
  where the wrong self-model drives a controller whose sensorimotor
  stream then objectively confirms the misbelief (TX-as-motor,
  compass-as-beacon). Frame as embodied mechanism of a known failure
  class; needs frequency statistics across seeds/models.
- **(c) Two LLM agents mutually discovering each other through an
  in-world radio — NOVEL.** No prior work where agents are not told a
  peer exists and must identify an unlabeled range-gated channel, then
  infer the other's existence and mobility from channel physics.
  Nearest: Takata et al. 2024 (range-limited messaging, but a labeled
  prompt affordance); GibberLink (scripted demo, no paper).

## Area 1 — LLMs discovering embodiment through raw interaction

- **Body Discovery of Embodied AI** — Sun, Tian, Hu, Zhao, Li, Zhang,
  2025. arXiv:2503.19941. Formalizes "which signals are my body and
  what does each do" and scores it with accuracy/P/R/F1, incl. mirror
  tests. NOT an LLM: bespoke causal-inference solver with global
  third-person observation, discrete signals; no raw streams, no
  noise, no parameter recovery, no task. The head-to-head citation;
  ideally run their causal baseline on Mazebot ports.
- **Sensorimotor Self-Recognition in MLLM-Driven Robots** —
  Dellibarda Varela et al., 2025 (rev. 2026). arXiv:2505.19237. MLLM
  in a real robot develops self-identification/dimension/movement
  awareness; evaluated psychometrically (SEM, memory ablations)
  through a curated, documented interface. No blind port mapping, no
  ground-truth identification error, no labels-off condition.
- **Sensory-Motor Control with LLMs via Iterative Policy Refinement**
  — Carvalho & Nolfi, Sci. Reports 2026. arXiv:2506.04867. LLMs write
  and refine controllers for Gym/MuJoCo, but are given descriptions of
  body, environment, and goal; discovery incidental, unscored.
- **LLMs as General Pattern Machines** — Mirchandani et al., CoRL
  2023. arXiv:2307.04721. LLM competence on semantically-stripped
  token streams (even randomly remapped); open-loop, no embodiment
  hypothesis formation.
- **LLMs learn governing principles of dynamical systems** — Liu,
  Boullé, Sarfati, Earls, NeurIPS 2024. arXiv:2402.00795. In-context
  sysID of Markovian dynamics from passive numeric series; no active
  probing, no actuators, no closed loop.
- **Probing Embodied LLMs: When Higher Observation Fidelity Hurts** —
  Zenkri & Brock, 2026. arXiv:2605.20072. LLM discovers hidden joint
  interdependencies of an external Lockbox through interaction;
  interface semantics provided, the unknown is not the agent's body.

## Area 2 — Classic robotic self-modeling (the lineage)

- **Resilient Machines Through Continuous Self-Modeling** — Bongard,
  Zykov, Lipson, Science 314, 2006. Quadruped infers its own topology
  via self-experiments; the canonical ancestor. Purpose-built
  stochastic optimizer, known sensor/actuator semantics.
- **Map Learning with Uninterpreted Sensors and Effectors** — Pierce &
  Kuipers, AIJ 92, 1997. The closest classical analog to anonymous
  device files: no a priori sensor meaning, bootstraps groupings,
  motor primitives, control laws. Hand-designed statistical operators,
  no reasoning agent, no misidentification phenomenology. (Follow-up:
  Olsson, Nehaniv & Polani, Connection Science 18(2), 2006.)
- **Robots That Can Adapt Like Animals** — Cully, Clune, Tarapore,
  Mouret, Nature 521, 2015. arXiv:1407.3501. Damage recovery via
  pre-computed repertoire (MAP-Elites + IT&E); self-knowledge compiled
  offline.
- **Task-Agnostic Self-Modeling Machines** — Kwiatkowski & Lipson,
  Sci. Robotics 4(26), 2019; **Full-Body Visual Self-Modeling** — Chen
  et al., Sci. Robotics 7(68), 2022 (arXiv:2111.06389). NN-learned
  self-models from task-agnostic interaction; known I/O wiring.
- **ML-Driven Self-Discovery of the Robot Body Morphology** — Díaz
  Ledezma & Haddadin, Sci. Robotics 8(85), 2023. Morphology from
  unorganized proprioceptive signals, minimal priors; bespoke
  estimator, no task pressure, no misbelief dynamics.
- Background: Hoffmann et al., "Body schema in robotics: a review,"
  IEEE TAMD 2(4), 2010.

## Area 3 — Mutual discovery / emergent communication (LLM era)

- **Spontaneous Emergence of Agent Individuality Through Social
  Interactions in LLM-Based Communities** — Takata, Masumori,
  Ikegami, Entropy 26(12), 2024. 10 agents, 50x50 grid,
  range-limited messaging (Chebyshev 5): the closest setup. Channel is
  a labeled prompt affordance; peers implicit; no identification
  problem, no goal, no scoring.
- **Emergent Social Conventions and Collective Bias in LLM
  Populations** — Ashery, Aiello, Baronchelli, Sci. Advances 11,
  2025. arXiv:2410.08948. Naming-game conventions emerge bottom-up;
  pairing and game imposed.
- **Searching for Structure: Emergent Communication with LLMs** —
  Kouwenhoven et al., 2024, arXiv:2412.07646; **Shaping Shared
  Languages**, arXiv:2503.04395. LLM dyads evolve languages in
  referential games; game, roles, channel given.
- **From Signals to Structure** — Talebirad et al., 2026.
  arXiv:2607.00233. Lewis signaling between LLMs, capacity-limited
  channels; partner known.
- **Hypothetical Minds** — Cross et al., 2024. arXiv:2407.07086.
  LLM theory-of-mind over other agents in Melting Pot; existence
  stipulated.
- **GibberLink** — Starkov & Pidkuiko, 2025 (demo). Voice agents
  recognize each other mid-conversation and switch protocols; scripted
  capability, no embodiment, no paper.
- Contrast lineage: Lazaridou & Baroni arXiv:2006.02419; EGG toolkit
  (EMNLP 2019); CAMEL arXiv:2303.17760; AutoGen; Generative Agents
  (UIST 2023) — "told-to-talk" frameworks.

## Area 4 — Closest benchmarks and evals

- **ARC-AGI-3** — ARC Prize Foundation, 2026. arXiv:2603.24621.
  Agent never told objective or controls; symbolic grid games, no
  embodiment, scores completion not the agent's model.
- **DiG-bench: Discovery in Games** — Battleday et al., 2026.
  arXiv:2608.12593. Hidden rules and win conditions discovered by
  experimentation; success = level solved.
- **WorldTest / AutumnBench** — Warrier et al., 2025.
  arXiv:2510.19788. Reward-free interaction then scored queries about
  the environment — scores the agent's model, but of the world, not
  the self.
- **DiscoveryWorld** — Jansen et al., NeurIPS 2024 D&B.
  arXiv:2406.06769 (also DiscoverPhysics arXiv:2605.26087,
  NewtonBench arXiv:2510.07172). Hidden laws, documented instruments.
- **Task2Quiz** — Liu et al., 2026. arXiv:2601.09503. Post-hoc
  grounded QA decoupling execution from understanding — closest to our
  quiz eval; ours quizzes the agent about its own body.
- **Can LLMs Explore In-Context?** — Krishnamurthy et al., NeurIPS
  2024, arXiv:2403.15371; **Agents Explore but Agents Ignore** —
  Engländer et al., 2026, arXiv:2604.17609. LLMs under-explore;
  bandits/software settings.
- Context: ALFWorld (arXiv:2010.03768), ScienceWorld
  (arXiv:2203.07540), BabyAI (arXiv:1810.08272), SmartPlay
  (arXiv:2310.01557), BALROG (arXiv:2411.13543) — all documented
  interfaces.

## Area 5 — Error persistence / self-confirming beliefs

- **How Language Model Hallucinations Can Snowball** — Zhang, Press,
  Merrill, Liu, Smith, ICML 2024. arXiv:2305.13534. Over-commitment
  to early mistakes; no environment in the loop.
- **Failing to Falsify** — ICLR 2026, arXiv:2604.02485 (with
  FalsifyBench arXiv:2606.04751, WILT arXiv:2410.10998). Confirmation
  bias in interactive rule discovery; the hypothesis does not alter
  the data-generating process — in Mazebot it does.
- **MIRAGE-Bench** — Zhang et al., 2025. arXiv:2507.21017. Agent
  hallucination taxonomy incl. persistent wrong actions; software/tool
  settings.
- **TIDE** — 2026, arXiv:2602.02196. High loop rates: persistence in
  recursive failure despite negative feedback.
- **LLMs Get Lost in Multi-Turn Conversation** — Laban et al., 2025.
  arXiv:2505.06120. Early assumptions, no recovery after wrong turns;
  conversational.
- **Reliability Scales Inversely** — Chakrabarti, 2026.
  arXiv:2607.18292. Snowballing as self-conditioning, worse with
  scale — a testable Mazebot prediction (is self-sealing
  model-size-dependent?).

## Venue assessment

- Today (platform + case studies): ALIFE full paper, or workshops
  (NeurIPS Open-World Agents / IMOL, ICLR Agent Learning in
  Open-Endedness, CoRL LangRob). Blog post after preprinting.
- With multi-seed x multi-model repeats: NeurIPS Datasets &
  Benchmarks (best fit; DiscoveryWorld/AutumnBench as templates) or
  ICLR main track once findings are quantitative. CoRL/RSS: poor fit
  (no physical robot). AAMAS: fits the duo half alone.
- Strategy: (1) arXiv + workshop now, citing 2503.19941 and
  2505.19237 head-on; (2) seed/model grid + their causal baseline on
  our ports -> NeurIPS D&B; (3) self-sealing phenomenology with
  frequencies -> its own paper alongside MIRAGE-Bench and Failing to
  Falsify.
