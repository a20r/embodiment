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

The harness's model client is stdlib `urllib` against the Anthropic
Messages API (keeps the dependency count at exactly one: PyYAML).  This
build/CI environment has no `ANTHROPIC_API_KEY`, so the recorded
reference episode uses the built-in `mock:wall-follower` model — a
scripted agent that exercises the identical harness path (bash tool
calls through `docker exec`, transcript, budgets, /memory writes).  Set
`ANTHROPIC_API_KEY` and `model: claude-fable-5` for live episodes; the
code path is the same.

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
