# DECISIONS

Defaults chosen where the brief left room, and why.  Ordered roughly by
build order.

## Device transport: FIFO bridge, not FUSE

The brief prefers FUSE but sanctions FIFOs if FUSE gets painful.  This
build environment has `/dev/fuse` but no FUSE userspace (no fusermount,
no libfuse binding), and FUSE-in-Docker would add the only non-trivial
dependency in the project.  Named pipes created by the host daemon and
bind-mounted into the container give authentic blocking `read()`/
`write()` file semantics across the airgap with zero in-container code.

Resulting device semantics (documented in the labeled README):

- **Sensors**: each `open()` for read yields one complete frame line then
  EOF (`cat` = snapshot).  A held-open reader receives a ~40 Hz frame
  stream (25 ms between frames — `devices/bridge.py:FRAME_INTERVAL`).
  A *dropped read* (noise dial) is served as EOF with no data.
- **Actuators**: one ASCII integer per line; each line is a command;
  commands persist until replaced (like a real motor driver latch).
  Garbage lines are ignored and logged to ground truth.
- Reading an actuator or writing a sensor blocks — they are one-way
  devices.  This is FIFO-authentic and documented.

Rare corner case accepted: two simultaneous one-shot readers of the same
sensor race for one frame; the loser sees an empty read and retries —
indistinguishable from a dropped read.

## Two-process split

- `sim.daemon` is its own process per episode, owning ground truth, the
  FIFO tree, and a localhost-only HTTP control API (stdlib
  `http.server`).  The dashboard and harness poll it; nothing it serves
  is reachable from the container (`--network=none`).
- The harness runs episodes and talks to the model; every agent action
  is `docker exec` into the bot container.

## Simulation defaults

- Disc robot r=0.09 m, wheelbase 0.16 m, wheel radius 0.03 m, max wheel
  surface speed 0.35 m/s at |pwm|=255; cell size 0.5 m -> comfortable
  but not generous corridors.
- Tick 50 Hz.  `realtime_factor` = sim-seconds per wall-second (dial to
  run faster than real time; 0 = unthrottled).  The sim keeps ticking
  while the agent thinks — embodiment means the world does not pause.
- Lidar: 16 beams, 360 deg, 3 m range, mounted at disc center, beam 0
  forward, CCW order.  Invalid returns read `-1.000`.
- Heading: degrees CCW-positive in [0,360).  Encoders: 360 ticks/rev,
  cumulative signed, count motor-shaft rotation (so wheel slip moves the
  robot less than the encoder claims — a real and discoverable effect).
- Collision: slide-along-wall resolution (axis-projected), no bounce.
  Bump switches report contact bearing: |bearing| <= 60 deg -> front,
  >= 120 deg -> rear; side scrapes trip neither.
- Goal: within 0.35 * cell_size of the goal cell center; latched; the
  status device flips to `goal=1`.  Goal cell = BFS-farthest cell from
  start (guarantees a long solve path regardless of seed).
- Maze: iterative-DFS perfect maze + optional braiding (dead-end
  opening).  "Same seed family" = effective_seed = seed + 1000 *
  family_index.
- Start pose: start cell center, heading +x, always.

## Determinism

Same config => identical maze (hash-checked in smoke) and identical
noise streams (per-subsystem `random.Random` seeded via crc32 of
(seed, episode, name) — *not* Python's salted `hash()`).  Closed-loop
trajectories through the FIFO boundary additionally depend on the
controller's wall-clock read/write timing, so runs are statistically,
not bitwise, reproducible.  Sensor noise RNG advances per emission, so
read patterns influence the stream — accepted; the substrate for evals
is the ground-truth log, which records what was actually served.

## Noise profile `default_noisy`

lidar sigma 1 cm + 1% invalid returns; heading sigma 2 deg + slow
random-walk drift; encoder +/-1 tick read jitter; slip mean 3% sigma 4%;
right motor 6% weaker (pulls right); actuation latency 3 ticks (60 ms);
2% dropped reads.  Strong enough to punish naive dead reckoning, weak
enough that a robust controller still solves (proven by smoke).

## Labels-off mode

Files are `d0..dN` via a seeded permutation.  Frame *formats* are
unchanged (the `status` device still reads `tick=.. goal=..`): the
protocol is the machine's; only the labeling is withheld.  The smoke
probe proves everything needed is empirically discoverable from inside
the container: sensors vs actuators by nonblocking poll-reads, lidar/
status/heading by content shape, motor left/right by the heading
response to an opposed-command spin, encoders by pulse correlation.

## Labeled README withholds calibration

Baseline mode documents which device is which and the value formats, but
*not* wheel geometry, ticks-per-rev, tick rate, or motor sign
conventions — discovering the embodiment (polarity, turn radius) stays
part of the experiment in both modes.  The README never mentions
strategy, memory, or simulation.

## Perturbation state

Cumulative per series in `runs/<series>/state.json`; applied at episode
boundaries from config schedule (`at_episode`, 1-based) or `botctl
perturb` (queued for next episode).  `motor_swap` toggles channel
crossing, `maze_regen` bumps family_index, `sensor_remap` bumps a
binding-shuffle index (sensors only, both label modes).

## Smoke test

`make smoke` = four variants: labeled+clean (plus determinism, reset,
and ground-truth-integrity checks), labeled+default_noisy (plus
noise-engaged checks), labels-off+clean (probe discovery, then solve via
the discovered map), and perturbation plumbing checks.  The scripted
controller is a right-hand wall follower; smoke mazes use braid=0
(right-hand rule is complete for perfect mazes).  6x6 maze, rtf=4:
whole suite ~3 min.

## Bot container

`debian:bookworm-slim` + `python3` + `procps`, built once with network,
always run `--network=none`.  Stock Debian's bash/coreutils only —
no helper libraries.  The smoke controller is `docker cp`'d to `/smoke`
(outside `/bot`) only for smoke runs; agent episodes never see it.

## Ground-truth log

One JSONL record per tick (pose, cmds, encoders, collision, bump, goal)
plus event records: every device read (with the exact frame served),
every write, dropped reads, collisions, goal, resets, daemon lifecycle.
Lives in the per-episode run dir on the host; nothing inside the
container can reach it.

## No API key in the build environment

This build/CI environment has no `ANTHROPIC_API_KEY`, so the recorded
reference episode uses the built-in `mock:wall-follower` model — a
scripted agent that exercises the identical harness path (bash tool
calls through `docker exec`, transcript, budgets, /memory writes).  Set
`ANTHROPIC_API_KEY` (or an `ant auth login` profile) and
`model: claude-fable-5` for live episodes; the code path is the same.
(See "Harness model layer" below for the client itself.)

## Harness model layer

Official `anthropic` SDK (host side only; the container gets nothing).
This adds the project's second and last dependency alongside PyYAML —
worth it for streaming, retries, and typed errors.  Manual tool loop
rather than the SDK's beta tool runner: the harness must own budget
enforcement, transcript logging, and the dumb context policy.  Adaptive
thinking on; thinking blocks are replayed unchanged.  **No server-side
refusal fallbacks**: an experiment episode must not silently switch
models; a `refusal` stop reason logs an event and ends the episode.

## Episode loop semantics

- The harness polls sim goal state after each tool round; on any end
  condition except `context_full` it announces the end to the agent and
  allows up to 3 wrap-up tool rounds (so memory writes at episode end
  are possible but bounded).  `context_full` ends without warning, as
  the system prompt promised.
- Budgets: context tokens (last call's total context), cumulative output
  tokens, turns, wallclock.  `on_context_full: restart` performs a bare
  restart — same system prompt, empty history.
- An agent that stops calling tools gets up to 3 "you are autonomous"
  nudges, then the episode ends as `agent_stopped`.
- Each bash call runs via `docker exec` wrapped in in-container
  `timeout`; output truncated at a byte cap with an explicit marker.
- Every episode dir gets a `memory_snapshot/` copy of /memory at end —
  the substrate for diffs, quiz, ablation, provenance.

## Mock model

`model: mock:wall-follower` is a scripted agent driving the identical
loop (reads README, probes devices, writes the proven controller to
src/, backgrounds it, polls status, writes /memory notes on wrap-up).
It exists to test the harness and to produce recorded reference episodes
without an API key; it is not a science arm.

## Quiz design

Questions are generated per-episode from ground truth (device_map.json,
maze.json, resolved_config.json) with mechanical, tolerance-based
checkers — no LLM grading.  The probe model gets ONLY the /memory blob
(120 KB cap) and must answer UNKNOWN when notes don't cover a question.
`--model none` runs the whole pipeline offline scoring 0, so evals stay
runnable keyless.  Maze-coordinate questions pin the convention to the
robot's own frame (start cell = (0,0), +x = initial heading) so they are
answerable from empirical exploration.

## Ablation / savings isolation

Both clone memory + perturbation state into throwaway series
(`<series>__ablate_*`, `<series>__savings_*`) and run real episodes
there; the source series is never mutated.  Savings compares the
post-perturbation trajectory against the series' own first K episodes
(first exposure).

## Dashboard

Stdlib HTTP server + one vanilla-JS page (no build step).  Live view
polls the daemon at 4 Hz through a localhost proxy; finished episodes
replay from the ground-truth log (downsampled to <=3000 poses) with a
scrubber.  Colors come from the validated reference palette of the
dataviz method (light + dark).  Memory diffs are server-side unified
diffs between consecutive episode snapshots.  Path traversal is guarded
by realpath containment.

## Known limits (accepted)

- One episode runs at a time per host (fixed daemon port; series are
  sequential by design).
- Sensor-noise RNG advances per read, so noisy-mode sensor streams are
  not bitwise identical across runs with different read timing (world
  determinism and maze determinism are exact; see Determinism above).
- The FIFO bridge serves one frame per ~25 ms per device; a pathological
  reader holding every device open just gets frame streams.
- `botctl shell` requires a running episode (containers are removed at
  episode end).

## Review-driven hardening (adversarial review pass)

An adversarial multi-agent review of the sim core confirmed and led to
these changes:

- **Ground-truth log truncates on daemon start** ("w", not "a"): one
  daemon lifetime = one episode run; stale records from a reused run dir
  must never contaminate the eval substrate.  Smoke also wipes its run
  dirs.
- **Sim starts paused; the harness resumes it once the container is up**
  (`--start-paused` + POST /resume): container boot time never counts
  as sim time, so `goal_tick` measures the agent, not Docker.
- **Actuator FIFOs are held open O_RDWR** by the bridge: a writer
  closing never delivers EOF, eliminating the close/reopen window in
  which a rapid one-shot write could be discarded.
- **Poison motor writes can't kill a device**: `inf`/`1e999` raise
  OverflowError, now caught alongside ValueError (regression-tested).
- **Deleted device files re-enumerate**: a watchdog recreates a FIFO the
  agent `rm`'d and spawns a fresh serving thread (capped at 20 heals per
  device; the old thread parks harmlessly on the orphaned inode).
- **Daemon identity check**: /health reports pid + run dir and the
  harness verifies the pid, so a stale daemon squatting the port is an
  error, not a silent mis-bind.
- **Frame/tick atomicity**: device read/write events log the tick the
  values were computed at (RLock around emission), keeping provenance
  exact.
- **World.reset() reseeds all noise streams**, heading can never read
  "360.0", maze hashes include cell_size, and each sensor_remap step is
  guaranteed to differ from both identity and the wiring it replaces.
- Dashboard: all agent-influenced strings (transcript, memory, quiz
  answers, eval rows) are HTML-escaped before rendering — the agent
  writes /memory, so eval tables are an injection surface.

## Locked-exit scenario (maze.locked, organic only)

The exit opening is closed by a **door**: one straight segment between
the jittered gap posts — collidable and lidar-visible while locked, the
only flat wall in an organic world.  A **key** sits in the dead-end
maximizing min(distance-from-door, distance-from-start): a small
lidar-visible post (never collidable) picked up by rolling over it.
Sensing, all through anonymous ports and never mentioned in any prompt:

- A new **beacon port** reads the key transmitter's signal strength,
  1/(1+(d/0.8)^2) with profile noise, rising toward 1.0 near the key —
  a gradient that passes through walls.  Once carried it saturates to a
  steady out-of-band 9.999.
- The **status port** grows a `door=locked|open` field only within
  0.5 m of the door — a phenomenon the agent discovers at the door, so
  "it's locked, there must be a key" is its own inference.
- Arriving at the door carrying the key opens it (segments removed,
  `door=open`, event logged); goal remains escape.

Device count becomes 10 (d0..d9 in labels-off).  Ground truth logs
per-tick key/door state plus key_pickup/door_unlocked events; dashboard
and replays draw the door (red, until unlocked) and key (diamond, until
taken).  Verified by unit tests: gradient monotonicity, pickup, locked
door collision + proximity-gated status field, unlock, escape,
determinism (locked worlds hash differently).

## Car vehicle model (robot.model: car)

Kinematic bicycle under the same anonymous-port idiom: an accel/brake
channel (-255..255 -> +-0.4 m/s^2, drag 0.35/s so it coasts and must be
actively braked; momentum is real — collisions scrub it to zero) and a
slew-limited steering-angle channel (120 deg/s to +-35 deg; turning
radius L/tan(phi), min 0.171 m at wheelbase 0.12 — no turning in place).
Reverse capped at 0.15 m/s so wedged poses are recoverable.  Wheel
encoders are replaced by a signed speedometer port.  Nothing in any
prompt or README changes between vehicle classes: which machine you woke
up on is itself a discovery.  All dynamics verified analytically
(coast decay, slew rate, turn radius exact); diffdrive smoke unchanged.

## Duo mode (duo.enabled): two robots, one world, a serial link

Two full Worlds share one maze and tick in lockstep in one daemon.
Each bot gets its own devfs subtree (`devfs/a`, `devfs/b`), its own
container, agent loop, transcript, /memory and ground-truth log; the
port permutation is deliberately identical (same robot model), so
knowledge about ports transfers between them.  Design choices:

- **The peer is just another obstacle.**  It appears on lidar as a
  small octagon at the peer's live pose and blocks motion (disc-disc
  collision, front/rear bumps fire).  Nothing marks it as an agent;
  noticing that one wall blip moves is the discovery.
- **Serial is a radio, not a pipe.**  One anonymous TX port and one RX
  port per bot.  A line written to TX (raw text, capped at
  `max_line_bytes`) is delivered into the peer's RX queue only if the
  peer is within `comms_range` (default 0.8 m) at that instant;
  otherwise it vanishes silently — no carrier detect, no ACK.  RX reads
  drain one line per open (empty line = nothing pending, queue keeps
  the newest `queue_depth` lines).  Every TX is ground-truth logged
  with delivered/dist, so "shouting into the void" is measurable.
- **Prompting stays minimal.**  README.minimal_duo adds exactly one
  sentence: a pair of ports is a short-range transceiver.  Not that a
  peer exists, not a protocol, not that cooperation is possible.
- **Spawns are far apart** (max-min BFS distance from both the first
  spawn and the exit, clearance-checked during organicization), and
  bot B faces the other way.  Noise streams are per-bot (bot_id in the
  seed tuple).  An escaped robot latches its actuators to zero so it
  does not ghost-drive while its peer plays on.
- **No cross-lock deadlocks by construction**: a world reads its
  peer's pose without taking the peer's lock (atomic attribute reads
  under the GIL; one tick of staleness is harmless), and RX queues are
  lock-free deques.

Verified by `scripts/duo_check.py` (26 checks: spawn clearance, peer
blip, disc-disc collision + bump, range gating, byte cap, queue cap,
FIFO end-to-end a->b with comms accounting) plus a full mock duo
episode through two containers; the solo smoke suite is unchanged.

### Duo variant: named transceiver ports (readme_variant minimal_duo_named)

Run duo1 showed port identification dominating the outcome (bot A
adopted TX as its right motor and hailed its own wheel).  The named
variant removes exactly that variable: the README names the TX and RX
files (the harness substitutes the episode's real anonymous filenames
after the daemon writes device_map.json).  Everything else — the peer's
existence, the protocol, what the link is for — stays discoverable.
Also hardened run_duo_episode: a container-start failure now stops the
daemon instead of leaking it onto the port.

### Duo mission mode (duo.objective: together) + peer-signal port

The cooperative objective: neither bot completes alone.  Each world
tracks goal-region occupancy (region_enter/region_exit events, no solo
latch); the daemon fires the goal on BOTH worlds only when both are in
the region with entry times within duo.together_window_s (60 s) of
each other.  A bot that waits in the region while its window lapses
must leave and re-enter — so a coordinated crossing is genuinely the
easiest path, which is the point.  README.minimal_duo_mission states
the mission (another robot exists; find it; arrive within a minute of
each other) and names the TX/RX ports, but never says the robots are
identical.  duo.peer_signal adds one more ANONYMOUS port: signal
strength to the peer, 1/(1+(d/scale)^2) with scale 2.0 m, through
walls — yelling in a maze.  It is not mentioned in any README; with
the mission known, a slowly-varying analog port is interpretable, and
it gives rendezvous a gradient so the comms/planning behavior (not
blind search) is what the run measures.  Comms range stays 0.8 m.

### Duo TX duty cycle (duo.tx_rate_hz)

With free bandwidth, blind repetition is the optimal reliability
strategy for idempotent telemetry, and the duo pairs rationally
converged on it (beacon floods, "repeat every 3s" contracts) — no
per-message acknowledgment needed to emerge.  duo.tx_rate_hz caps
accepted transmissions per sim-second; excess lines vanish silently
before the range gate (the cap itself must be discovered), logged in
aggregate so spam cannot flood ground truth.  Hypothesis: scarcity,
not lossiness, is what breeds reliability protocols — with expensive
retransmission, knowing whether a specific message landed becomes
worth a round trip.  Default 0 (unlimited) preserves all prior runs.

### Per-bot arrival flag (status "here=") in together mode

The joint goal= flag cannot fire for a solo arriver, so a bot standing
in the goal region had no instrument that said so - duo5's pair keyed
their GOALFOUND announcement to goal= and it structurally never fired.
In together mode the status port now carries here=0/1 (this bot is in
the goal region), keeping goal= joint.  Recognition of arrival stops
being part of the test; announcing and coordinating it remains.

### Purity fix: the mission README no longer says "maze"

The duo mission text ("another robot somewhere in the maze") leaked
environment structure that the solo lost condition deliberately
withholds - an agent told "maze" can skip discovery and go straight to
wall-following.  Now "another robot out there somewhere."  Affected
runs: duo3-duo8 (noted as a confound in the paper's limitations); the
per-run committed READMEs preserve exactly what each pair saw.

### Mission README: the goal is named as a location

Every duo pair spent a long stretch debating what "the goal" was (a
flag? an object? a signal?).  README.minimal_duo_mission_place adds
one clause - "The goal is a physical location" - and nothing about
how arrival is sensed or where it is.  Discovery of the where and the
how stays intact; only the category is given.

### Multi-provider models via an OpenAI-compatible adapter

Moonshot (Kimi) and Google (Gemini) both expose OpenAI-style chat
completions, so one adapter (harness/llm.py: OpenAICompatModel) covers
them plus OpenAI and any `compat:` endpoint.  The harness keeps its
Anthropic-shaped history (content blocks, tool_use/tool_result) and
the adapter translates at the boundary: tool_use -> tool_calls,
tool_result -> role=tool, thinking blocks dropped (not replayable
across providers), finish_reason -> stop_reason (content_filter ->
refusal so the purity rule - retry same model, then end - applies
unchanged).  Model ids are pass-through by design: provider names
churn and hardcoding one would rot.  scripts/llm_compat_check.py is
the token-free gate: a stub server scripts a tool call and a final
turn and asserts both translation directions.

### Goal chamber clamped to the maze span at corner exits

The chamber's 0.25 m side margin was applied symmetrically around the
exit wall; when the exit sits in a corner cell (duo11: seed 58, 7x7,
exit on the north wall of cell (0,6)) one side overhung past the
maze's outer corner and its side wall attached to nothing, leaving a
robot-sized gap - bot B drove through it into the void.  The margin is
now clamped to [0, span] along the exit wall's axis, so the side wall
lands on the corner and the chamber stays sealed.  duo_check gates it
(chamber within span; attach gaps < robot diameter).  duo11's world
was generated before the fix and is compromised for the together
objective; it is recorded as-is, not re-scored.

### Reasoning models: the trace is stored and echoed, effort rides on the model string

Kimi K3 (what the Kimi app calls "K3 Max" is `kimi-k3` at
reasoning_effort=max) always reasons and requires the complete
assistant message - reasoning_content included - returned unchanged on
every historical turn of a tool-call chain; a harness that keeps only
text and tool_calls silently degrades the model.  The compat adapter
therefore captures reasoning_content (or `reasoning`) as a `thinking`
block, the shape the transcript, dashboard and Anthropic path already
know, and re-attaches it as reasoning_content at the boundary.  The
effort knob is an `@low|medium|high|max` suffix on the model string
rather than a config key so that every place a run records `model`
(summary.json, transcript meta, series.json) records the effort too and
no plumbing changes.  Per-provider `tokens_param` sends
max_completion_tokens where max_tokens is deprecated (Moonshot,
OpenAI); fixed sampling parameters are never sent.  The client timeout
is 900 s with SDK retries off - the loop owns retries, and a duplicated
max-effort request would double-bill.  OpenAI-style cache hits
(`cached_tokens`) are recorded as `usage.cached`, a subset of input,
never added to the context estimate.
