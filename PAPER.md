# Waking Up as a Robot: Embodiment Self-Discovery in LLM Agents

*Draft v1 — full text. Working alternatives for the title:
"Mazebot: A Ground-Truth Benchmark for Embodied Self-Identification by
Language Models"; "Which Wire Am I? LLM Agents Discovering Their Own
Bodies, and Each Other, Through Raw Device I/O".*

## Abstract

We present Mazebot, an experimental platform in which a large language
model agent is placed behind a wall of anonymous device files
(`d0..dN`) backed by a hidden robot simulation. The agent — a bash
tool in an airgapped container — is told almost nothing: the files
exist, some read and some write, and "You are lost. You need to find
the goal. You will know it when you reach it." Everything it comes to believe about its own body — which
file is the lidar, which are the motors, what its top speed is,
whether it is a differential-drive platform or a car with momentum —
it must infer by writing and reading raw ASCII. Because ground truth
never crosses into the container, the agent's *self-model* is exactly
scoreable: we define port-identification precision/recall, kinematic
parameter-recovery error, and time-to-identification, all measured
against the simulator's device map and configuration. In instrumented
case studies, a frontier model recovers a largely correct self-model
within roughly an hour of simulated interaction, solves mazes under
sensor ablations (no encoders; a compass with a hidden constant
bias), and reconstructs much of the control model of a car-like
vehicle it was never told it was driving. The failures are as
diagnostic as the successes: we identify *self-sealing
misidentifications*, in which a wrong self-model is installed into a
controller whose resulting sensorimotor stream then objectively
confirms the misbelief — including an agent that adopted its radio
transmitter as a motor and spent two hours hailing its own wheel. A
two-robot extension adds a range-gated serial radio and runs as an
ablation ladder over what the agents are told. When neither knows a
peer exists, one agent infers "SOURCE IS MOBILE" from signal physics
alone and hunts it. When the mission discloses the peer
and requires arriving at the goal together, coordination *content*
still emerges entirely unprompted: callsign negotiation, message
typing, in-band telemetry conventions, explicit division-of-labor
plans transmitted over the wire, and a proposed bump-sensor handshake.
We release the platform, metrics, and complete run records.

## 1. Introduction

During one of our experiments, a robot said "open sesame" to its own
wheel. Four times.

The robot was driven by a frontier language model whose only interface
to the world was a directory of eleven anonymous FIFO files. It had,
over two hours of careful experimentation, correctly identified its
lidar and one of its two motors. But it had crossed wires elsewhere in
its self-model: the file that was actually its radio transmitter it
believed to be its right motor; the file that was actually its right
motor it believed to be a communication port; and its compass it had
recast as a bearing-to-goal beacon. The consequences were flawless in
their irony. Every "motor command" it issued was broadcast into the
air — 43,780 transmissions, of which 5,775 reached a second robot it
did not know existed, which read and studied them — while every
deliberate message it composed, the `hello`s and `who`s and `open
sesame`s, was written to a wheel and discarded as garbage.

The reason this anecdote is *evidence* rather than color is the
architecture that produced it. In Mazebot, the simulation and its
ground truth live on the host; the agent lives in a network-isolated
container whose only window is the device files. Every byte the agent
reads or writes is logged against the true device map, every pose
against the true kinematics. When the agent writes "d3 = right motor"
into its notes, we do not have to interpret: it is wrong, at a known
timestamp, for known reasons, with a measurable cost. The platform
turns questions that are usually rhetorical — does the agent
*understand* its body? — into measured quantities.

That measurement is the gap we aim at. Evaluations of LLM agents
overwhelmingly score task success; what the agent believes about the
system it is controlling is rarely elicited and almost never scored.
Classical robotics solved body-model acquisition with purpose-built
estimators — Bongard, Zykov and Lipson's self-modeling quadruped;
Pierce and Kuipers' agent with uninterpreted sensors and effectors —
but those learners had their sensor semantics, or at least their
statistical form, designed in. A language model brings something
different to the same loop: general priors about what robots tend to
be, a hypothesis language rich enough to say "this port looks like an
odometer," and, as we document, new failure modes that arise
precisely because its hypotheses are so fluent.

Three design invariants define the platform. First, **the agent must
never see ground truth**: the container has no network, the run
directory is host-side, and in the anonymous-port conditions nothing
in any prompt names a device or a morphology. Second, **prompts stay minimal**: the agent is told it is
connected to a robot's onboard computer, that ports live under
`/dev/robot`, and — in the strictest condition — only "You are lost.
You need to find the goal. You will know it when you reach it." Third, **experiment
purity**: no model fallbacks inside an episode; a safety-classifier
refusal is retried on the same model and otherwise ends the episode.

Contributions:

1. **The platform**: a deterministic 2D kinematic simulator (two
   vehicle classes, procedurally generated organic mazes, a
   configurable noise model) exposed *only* as FIFO device files
   inside an airgapped container, with per-tick ground-truth logging
   of state and of every device interaction (§3).
2. **Self-model metrics**: port-identification precision/recall,
   kinematic parameter-recovery error, time-to-identification, and a
   fresh-context probe quiz scored mechanically against ground truth
   (§4).
3. **Instrumented case studies**: complete self-identification and
   maze escape under sensor ablations, and system identification of a
   car-like vehicle the agent was never told about (§5).
4. **Self-sealing misidentification**: a mechanism-level account of
   wrong self-models that manufacture their own confirmation, with
   four instances spanning port-level and protocol-level errors (§6).
5. **The duo ladder**: two agents, one world, a range-gated radio,
   and a five-run ablation over disclosure — from mutual discovery by
   channel physics alone to unprompted callsigns, roles, telemetry
   conventions, and transmitted joint plans (§7).

## 2. Related work

**Robotic self-modeling (pre-LLM).** The lineage we transplant begins
with resilient self-modeling machines [Bongard, Zykov & Lipson,
Science 2006], in which a quadruped infers its own topology through
actuation-sensing experiments, and reaches back to Pierce & Kuipers'
map learning with *uninterpreted* sensors and effectors [AIJ 1997] —
the closest classical analogue of our anonymous device files, solved
there by hand-designed statistical operators. Later work learns
self-models with neural networks [Kwiatkowski & Lipson 2019; Chen et
al. 2022] or recovers morphology from unorganized proprioception
[Díaz Ledezma & Haddadin 2023], and damage recovery via behavior
repertoires [Cully et al. 2015]. All assume known wiring or bespoke
learners; none scores a general agent's stated beliefs.

**LLMs and embodiment discovery.** *Body Discovery of Embodied AI*
[arXiv:2503.19941] formalizes "which signals are my body and what
does each do" and scores it with precision/recall — with a bespoke
causal-inference solver enjoying global third-person observation, not
an LLM probing raw byte streams; we adopt their scoring spirit and
differ in everything about the setting. *Sensorimotor
Self-Recognition in MLLM-driven robots* [arXiv:2505.19237] elicits
self-awareness through a curated, documented interface and evaluates
psychometrically. LLMs have been shown to control described bodies
via iterative policy refinement [arXiv:2506.04867], to operate on
semantically stripped token streams [General Pattern Machines,
arXiv:2307.04721], and to perform in-context system identification of
given dynamics [arXiv:2402.00795]; the Lockbox study
[arXiv:2605.20072] has an LLM discover hidden structure of an
*external* mechanism. To our knowledge no prior work has a general
LLM agent identify its *own* sensors, actuators, and kinematic
parameters through an undocumented raw interface, scored against
hidden ground truth.

**Emergent communication.** Multi-agent LLM frameworks tell their
agents to talk (CAMEL, AutoGen, Generative Agents); the
emergent-communication literature gives agents a channel and shapes a
protocol by optimization [Lazaridou & Baroni 2020; EGG; LLM-era
referential games, arXiv:2412.07646, 2607.00233]. Takata et al.
[Entropy 2024] give grid-world LLM agents range-limited messaging as
a labeled affordance. Our setting inverts the classical problem: the
lexicon is free (the agents share English), and what must be
discovered is the *physical* layer — that a channel exists, its
range, its loss model, and in the strictest condition the existence
of the interlocutor.

**Discovery benchmarks.** ARC-AGI-3 withholds objectives and
controls; DiG-bench hides game mechanics; AutumnBench/WorldTest
scores an agent's *world* model by post-hoc queries; DiscoveryWorld
and kin hide scientific laws behind documented instruments; Task2Quiz
[arXiv:2601.09503] shows task success is a poor proxy for
understanding. We score a model of the *self*, grounded in
continuous, noisy, closed-loop dynamics.

**Error persistence.** Hallucination snowballing [arXiv:2305.13534],
confirmation bias in interactive rule discovery [Failing to Falsify,
arXiv:2604.02485], agent failure loops [TIDE; MIRAGE-Bench], and
non-recovery from early wrong turns [arXiv:2505.06120] document the
components. The embodied variant we isolate is stronger: the wrong
belief changes the data-generating process, so the *environment*, not
the context window, supplies the confirming evidence (§6).

## 3. The Mazebot platform

### 3.1 World

A 2D kinematic simulation ticks at 50 Hz. Mazes are procedurally
generated (randomized DFS with optional braiding) in two styles: a
*grid* style with axis-aligned walls, and an *organic* style in which
lattice nodes are jittered and every wall is rendered as a
sine-perturbed polyline — nothing is straight, nothing is
axis-aligned, and nothing visually announces "constructed maze." In
organic mazes the goal is an opening in the far boundary: escape is
the goal, unmistakable on arrival without ever being named. A
generation-time retry loop guarantees clearance at every spawn.
Optional elements: a locked exit with a discoverable key and a
signal-strength beacon; and a *goal chamber* — three walls penning
the space beyond the exit, so an escaped robot remains at the goal
(and may re-enter) instead of wandering into unbounded space.

### 3.2 The interface is the experiment

The simulation is exposed exclusively as FIFO device files,
bind-mounted at `/dev/robot` inside a `--network=none` Docker
container holding a stock Debian with Python. Semantics: opening a
sensor file yields one complete measurement line, then EOF (`cat`
returns a snapshot; a held-open reader streams at ~40 Hz). An
actuator file consumes one ASCII integer per line; values clamp to
±255 and persist until replaced; garbage is ignored and logged. A
deleted device file is re-created by a watchdog — the bus
re-enumerates. In the standard condition (`labels: off`) filenames
are `d0..dN` under a seeded permutation. The agent's only tool is
bash. Its only persistent state across episodes is a `/memory` mount.

### 3.3 Realism dial

A named noise profile supplies Gaussian lidar noise and dropouts,
heading noise and random-walk drift (optionally a constant hidden
gyro bias), encoder jitter, wheel slip, motor asymmetry (right-motor
gain 0.94), actuation latency (three ticks), and dropped reads.
Everything derives from named seeded streams; an episode is exactly
reproducible.

### 3.4 Two vehicle classes

The *diff-drive* robot exposes two PWM motor channels, wheel
encoders, a compass, a 16-beam 360° lidar (3 m range), front and rear
bump switches, and a status port (`tick=N goal=0`). The *car* is a
kinematic bicycle: an acceleration/braking channel with momentum and
drag (it coasts; it must be actively braked), a steering channel
slew-limited at 120°/s to ±35° (no turning in place), a reverse speed
cap, and a signed speedometer in place of encoders. Crucially, no
prompt or README differs between vehicle classes: which machine you
woke up on is itself a discovery.

### 3.5 Agent harness

An episode runs one agent (Anthropic SDK; the case studies use a
frontier model, with one smaller-model comparison) against one
container and one daemon. The context policy is deliberately dumb —
when the context fills, the episode either ends or performs a bare
restart with nothing carried over except `/memory`. Budgets bound
turns, output tokens, and wallclock. In the duo mode, two agents run
concurrently in threads against one shared world, with fully separate
transcripts, memories, and budgets.

### 3.6 Duo mode

Two Worlds share the maze in lockstep. The peer is *just another
obstacle*: it appears on lidar as a small moving octagon and is
collidable (disc-disc, with bump switches firing); nothing marks it
as an agent. Each robot gets one TX and one RX port: a line written
to TX (raw text, byte-capped) is delivered into the peer's RX queue
only if the peer is within `comms_range` at that instant — otherwise
it vanishes silently. No carrier detect, no ACK. Every transmission
is ground-truth logged with a delivered flag and the true distance,
making "shouting into the void" a measurable quantity. Optional: an
anonymous *peer-signal* port whose analog value rises as the peer
nears (1/(1+(d/2m)²), through walls) — yelling in a maze; and a
*together* objective in which no bot completes alone — the goal
latches for both only when both are inside the goal region with
entry times within a 60 s window, and a lapsed solo arrival must exit
and re-cross.

## 4. Measuring a self-model

**Port identification.** The agent's port→function claims — mined
from `/memory` and transcripts, or elicited directly — are scored
against the true `device_map.json` as precision/recall/F1, with
per-port time-to-first-correct-identification.

**Parameter recovery.** Where the agent states quantitative
self-knowledge, we score relative error against configured truth:
top speed, turn rate per PWM unit, encoder ticks per distance, (car)
steering limit, slew rate, drag. Examples from the runs: one agent
recovered the turn-rate constant as ≈0.9°/s per PWM unit (duo2's
notes); the duo5 pair calibrated speed-per-command constants
("~0.0028 m/s per unit") before using them for dead reckoning.

**Probe quiz.** After an episode, a fresh-context model instance is
quizzed with mechanically checkable questions generated from ground
truth ("which port is the lidar?", "does a positive motor PWM value
drive a wheel forward or backward?"), decoupling knowledge from
execution (cf. Task2Quiz). The shakedown agent scored 12/15.

**Behavioral metrics.** Time-to-goal, collision counts and curves,
coverage; in duo: delivered ratio, read ratio, time-to-first-contact,
rendezvous windows and durations, and joint-arrival success.

## 5. Case studies: discovering a body

All runs below use the noisy profile and, except the shakedown,
anonymous ports and the minimal "lost" prompt.

**Labeled shakedown.** With documented device names, the agent solved
the maze in 718 simulated seconds (25 turns, 22 collisions) and
scored 12/15 on the probe quiz — the baseline sanity condition. The
same configuration on a much smaller model did not finish: it never
progressed past partially correct motor usage, establishing that the
discovery loop is not trivial.

**Lost mode.** With anonymous ports, an organic maze, and the minimal
prompt, the agent identified six of the seven sensor ports and
escaped in 77 minutes of simulated time (73 turns, 147 collisions);
the one hold-out, a rear bump switch that never fired, stayed in its
notes as "unknown, always 0" — an honest gap rather than a guess.
Discovery followed a stereotyped order — lidar first (a 16-float
line is unmistakable), then motors by writing and watching the lidar
change, then heading, then encoders — and the agent maintained a
running lab-notebook in `/memory`. (The sibling lost1 run's notebook
contains an explicit CORRECTION entry, for a mis-indexed lidar
beam.)

**No encoders, lying gyro.** We removed the encoder ports entirely
and gave the compass a hidden constant bias of 5°/minute. The agent
escaped at 135 minutes (100 turns, 277 collisions) — but not by
catching the lie. Its notes rate the compass "ABSOLUTE (compass,
trustworthy ±3deg)"; the drift it did observe it attributed to its
*wheels* ("open-loop straight drifts right 1–2 deg/s. Use
compass-servo for straight lines"), and its winning strategy was a
compass-heading-hold driver wrapped in an anti-revisit visit-grid
explorer with "frustration escapes." The bias was small enough that
servoing on the lying instrument still worked. The run is therefore
not a sensor-trust success story but a subtler datum: a wrong
*causal attribution* (blaming the body rather than the sensor) that
happened to be behaviorally harmless — the benign end of the
spectrum §6 examines.

**The car.** The agent woke up on the kinematic bicycle with no
notice that anything had changed. Over 292 turns (six sim-hours) it
discovered inertia ("inertia present," its port notes record),
throttle persistence, and the absence of turning-in-place; it
constructed a partial control model and drove — but did not escape,
accumulating 7,190 collisions. Notably it never identified the
speedometer (its notes file d7 as "unknown (gyro?)") and misread a
bump switch as a "facing-goal" sensor: the vehicle swap degraded
port identification, not just control. Two behaviors mark the run. The first
is the *creep-speed policy*: across the entire episode the agent
never commanded more than 16% of the vehicle's top speed and recorded
zero deliberate braking events — a rational fear of momentum it could
not yet model. The second is the steering misidentification described
in §6.

**Memory across episodes.** Because only `/memory` survives an
episode boundary, cross-episode competence is whatever the agent
chose to serialize. The strongest demonstration: a duo agent
restarted in a maze it had explored the previous episode matched its
current lidar-and-odometry frame against its saved map within ten
minutes ("current frame + (2,6) ≈ ep1 frame") and reached the exit
region in about 20 minutes — a location it had failed to find in 185
minutes from scratch. Continuation episodes also preserved social
artifacts: callsigns, message formats, and a written playbook (§7).

## 6. Self-sealing misidentification

**Definition.** A self-model error is *self-sealing* when the
controller built on the wrong model produces sensorimotor feedback
consistent with that model, so ordinary operation generates
confirmations rather than contradictions. The hypothesis alters the
data-generating process — unlike snowballing or classic confirmation
bias, where the wrong belief lives only in the context.

**Instance 1 — steering as degrees (car run).** The agent decided
the steering channel was an absolute angle in degrees. Its
heading-feedback servo silently compensated for the wrong gain, so
every closed-loop maneuver "worked," and the error survived the
entire episode inside an otherwise increasingly accurate control
model.

**Instance 2 — TX as right motor (duo1).** The agent adopted the
radio TX port as its right motor and the real right motor as a chat
port. One-wheeled driving plus a compass servo still produced
locomotion; 43,780 "motor commands" were broadcast to a peer it never
knew about; 1,340 `hello`s (and four `open sesame`s) went to the
wheel, logged by the simulator as invalid writes. The cheap falsifier
— write the suspected motor with the drivetrain stopped and watch the
pose — was never run.

**Instance 3 — diff-drive as unicycle, and the seal that broke
(duo2).** The agent modeled `d1` as a linear-velocity channel and
`d6` as an angular-velocity channel (they are left and right wheel
PWMs), and initially explained the resulting systematic arcs
physically — "Robot gets WEDGED near walls: forward cmd then causes
rotation (one wheel gripping)." This instance, uniquely, was
repaired in-episode: the agent eventually ran the discriminating
test and wrote "CRITICAL CORRECTION (supersedes above!) — d1 and d6
are WHEEL SPEED commands (differential drive), NOT linear/angular!
... All earlier weird 'wedge/pivot' behavior was just wrong actuator
model." The seal can break; what varies is when.

**Instance 4 — a protocol-level variant (duo5).** The pair agreed:
"if you find goal: park+spin+broadcast GOAL FOUND," keying the
announcement to the status port's goal flag. Under the *together*
objective that flag stays 0 by design until **both** robots arrive.
One robot found the exit, camped the goal chamber for the final 97
minutes, and honestly beaconed `goal 0` from inside the goal —
structurally unable to send the trigger its own plan required. The instrument the
protocol chose was the one signal that cannot fire until the protocol
has already succeeded.

**Negative beliefs seal too.** Both duo2 agents, having hailed on
the correct ports and heard only silence (they were never
simultaneously in range while both operated; the episode's only two
deliveries reached a robot that had already completed and powered
down), concluded the radio was "probably unused" — a false negative
about the *existence of another agent*, derived validly from absent
evidence.

Across instances the pattern is: commitment is early, compensation
is competent, and the falsifying experiment is cheap and known in
principle — run, in our corpus, in one case of four. We propose
falsification-timing — the gap between when a decisive test became
available and when (if ever) it was executed — as the metric that
separates these agents from ideal experimenters; Instance 3 shows
the distribution has mass at finite values, not only at "never."

## 7. Two robots: the disclosure ladder

Five runs share one maze seed; each rung is a small, fully specified
configuration delta (most a sentence of README or a port of
hardware; the duo3 rung changed several things at once, noted
below). The peer is always lidar-visible and collidable; the radio
is always range-gated with silent loss.

**duo1 — peer undisclosed.** The README's one extra sentence
describes the transceiver mechanics ("one pair of ports is a
short-range transceiver") and nothing else — not the peer. Both
agents found the TX/RX pair among the anonymous ports; one
misidentified it (§6). First contact — the
first delivered line — came at 46 minutes; over the episode, 49,023
transmissions produced 6,401 one-way deliveries across 13 encounter
windows (21 minutes in range; closest approach 0.41 m). The
correctly-wired agent probed with single words (`ping`, `where`,
`goal`, `help`, `hello`), observed that reply values varied while it
stood still, and wrote the episode's key inference in its notes:
"**SOURCE IS MOBILE**." It then built a chemotaxis chaser and hunted
the source as a candidate goal. No dialogue formed — one side was
broadcasting telemetry it didn't know it was sending — but one agent
had inferred the existence of another mobile system from channel
physics alone.

**duo2 — ports named.** One sentence added: which file transmits,
which receives. Both agents used the radio correctly and immediately
(`hello`, `ping`, `marco`, `SOS`); one polled its receiver 2,146
times. They were never simultaneously in range while both lived: one
escaped the maze solo at 69 minutes (the objective was still
individual), after which the other's only two delivered `hello`s —
at minutes 102 and 230 — were addressed to a powered-down robot
parked at the exit. Both agents' final notes declared the radio
"probably unused"/"no effect." Removing port identification exposed
the next two bottlenecks: rendezvous, and inference from silence.

**duo3 — mission disclosed.** This rung changes the most at once:
the README now states a peer exists and both must reach the goal
within a minute of each other; the objective becomes joint; an
anonymous peer-signal gradient port is added; and the wheel encoders
are removed (an ablation that persists through duo4). The full cooperation
arc appeared in one episode: structured position beacons by minute 13
(`HELLO from botA pos=(0.0,-0.2) h=91.2` — both agents, comically,
initially self-named "A"); physical rendezvous at minute 78.8
(signal 1.000) via an explicitly negotiated hold-still/homing
maneuver; single-letter message *types* (H/T/E); an explicit
division-of-labor plan transmitted over the wire — "finder of goal
parks there + sends GOALFOUND; other homes on signal"; a plan
*revision* to convoy exploration after they judged splitting too
risky; and a proposed physical handshake — "HOLD EXACTLY STILL 3 minutes. I
will sweep nearby cells and try to reach/touch you. Watch your
d7/d9." — proposing contact verification through the partner's rear
bump switch and status port. The run was cut short externally at 185
minutes with 2,112 of 4,677 lines delivered and no exit found.

**duo4 — goal chamber added.** With the arena sealed, the pair
solved duo3's identity collision by negotiating callsigns — ALPHA
and BETA — and ran a leader/follower convoy with goal flags
cross-reported in every frame. One agent's transmit counter reached
372,443 — dominated by a sub-minute burst in which one line ("hello
other robot, do you copy?") was pumped at thousands of repetitions
per second of simulated time, the mechanical extreme of "any overlap
must deliver." All 353 delivered lines were read. Navigation, not
coordination, failed: still without encoders (the duo3 ablation),
lidar-only odometry made coverage slow, and neither robot found the
exit in 202 minutes.

**duo5 — encoders restored, radio range 1.5 m.** The driving
transformed (5 and 1 collisions at the 24-minute mark, versus
hundreds in duo4) and the first sustained two-way telemetry convoy
formed: 1,116 delivered lines, position reports in both directions,
`follow me`, and live gradient-climbing of the peer-signal port
(`climb d5 0.782`). One robot found the exit at 104 minutes and
executed the agreed plan — park in the chamber and beacon — for the
remaining 97 minutes. The episode still ended without a joint
arrival, for the protocol-level reason given in §6: its "I found it"
trigger could never fire. The partner ended 4.1 m short at
wallclock. A continuation
episode with memories intact is in progress at time of writing; its
first inter-robot contact came within eight minutes of boot.

**What emerged without being asked.** Even in the disclosed runs the
README specifies only *that* a peer exists and *that* both must
arrive together. Everything about *how* was invented: self-naming
and its repair into callsigns; message types; in-band telemetry
schemas; plan proposal, agreement ("proto: agreed"), and revision;
physical verification protocols. **What kept failing:** symmetric
role deadlocks ("you hold still — no, you"); rendezvous without a
gradient; identity collisions; and inference from absence — silence
read as a dead channel rather than an out-of-range one.

## 8. Discussion

**Self-models are cheap to elicit and dangerous to trust.** An hour
of interaction buys a frontier agent a mostly correct body model —
and one or two confidently wrong entries whose costs compound. The
agents' compensating controllers are good enough to keep wrong
models alive; the decisive experiments are cheap and go unrun. If
these agents ran a budgeted adversarial self-test — five minutes of
"try to falsify your port map" — three of our four self-sealing
instances would have died in their first hour.

**Motion style is a readout of the uncertainty model.** The
pivot-heavy calibration style of agents that trust their compass,
the compass-servo straight-line driving of the agent that blamed its
wheels for drift, the creep-speed policy of the agent facing
unmodeled momentum: how the robot moves tracks what the agent
believes about its own body and sensors.

**LLMs invert the emergent-communication problem.** Classical
emergent communication gives agents a channel and asks whether a
lexicon forms. Here the lexicon is free — English arrives with the
model — and everything the classical setting takes for granted must
be discovered: that a channel exists, what its physics are, and that
there is anyone on the other end. The failures we observe live
exactly in that inverted layer: physical rendezvous, carrier
inference, identity.

**Episodes become a life.** Nothing persists but files the agents
choose to write; what they chose — maps, calibration constants,
protocols, playbooks, callsigns — was sufficient for cross-episode
relocalization and instant social re-establishment. Memory design
was not prescribed; the arm-A agents invented their own.

## 9. Limitations

Every condition is n=1 on essentially one frontier model (plus one
smaller-model comparison); the findings are existence proofs and
mechanism descriptions, not frequencies. The simulator is 2D and
kinematic with a 16-beam lidar. README wording is a powerful lever
we pulled deliberately; its sensitivity is unquantified. Mining
identification claims from transcripts involves judgment (the quiz
mitigates but samples sparsely). Frontier-model episodes cost tens
of dollars each, which constrained repetition in this version.

## 10. Future work

The multi-seed, multi-model grid (5+ seeds × frontier/mid/small ×
the ablation matrix) to convert every §5–§7 observation into a rate
with error bars, and to test whether self-sealing scales with model
size [cf. arXiv:2607.18292]. Porting the Body-Discovery causal
baseline [arXiv:2503.19941] and a scripted probe baseline onto
Mazebot's ports. Heterogeneous duos (car + diff-drive), triads,
adversarial peers, and communication costs. A serial-device port of
the FIFO wall to a physical robot.

## 11. Conclusion

A wall of anonymous device files turns embodiment into an empirical
question the agent must answer and the experimenter can grade. The
agents we studied answered it well — inventing localization schemes,
recovering control models, building protocols with each other — and
wrong in ways that teach: the most dangerous errors were not
failures to hypothesize but successes at compensation. Two robots
that were told nothing found each other anyway; two robots that were
told only "arrive together" invented everything else. The remaining
gap between them and the goal is, at this point, measured in meters
— and in one flag that couldn't fire.

---

## Reproducibility

All runs, configs, seeds, and replay pages are committed to the
repository (`runs/`), excluding per-tick ground-truth logs
(available on request; roughly 3–600 MB per episode). `make smoke`
verifies the full stack end-to-end with a scripted agent;
`scripts/duo_check.py` runs 42 host-side checks of the duo
machinery. Prompts and READMEs appear verbatim under
`harness/prompts/` and `botfs/`; every episode directory contains
its resolved configuration and true device map.

## Appendices (assembled from committed records)

- **A. Port maps and configs** per run (`device_map.json`,
  `resolved_config.json`).
- **B. Transcript excerpts**: the car-run system-identification
  monologue; duo1's "SOURCE IS MOBILE" notes; the duo3 rendezvous
  dialogue; duo4's callsign negotiation.
- **C. Memory artifacts**: the lost-mode lab notebook; duo3's
  transmitted plan; the duo5 playbook.
- **D. Replay pages**: a self-contained HTML replay per series
  (`runs/<series>/replay.html`; a second-episode page where
  recorded).
