# CLAUDE.md

Mazebot: a research platform for embodied-LLM experiments. A host-side
sim daemon owns a 2D kinematic world (diff-drive or car robot in a
procedurally generated maze) and exposes it ONLY as FIFO device files,
bind-mounted at `/dev/robot` inside an airgapped Docker container
(`--network=none`). An LLM agent lives in that container with a bash
tool, a minimal README, and a persistent `/memory`; everything it knows
about its own body it must discover through the ports. Ground truth
stays on the host and feeds the evals, dashboard, and replay pages.

## Commands

- `make smoke` — build the bot image and run the full end-to-end smoke
  suite (`scripts/smoke.py`: labeled, unlabeled, unlabeled+probe,
  perturbations). The gate for any sim/devices/harness change.
- `./botctl run [--set KEY.PATH=VALUE ...]` — run an episode series
  per `config.yaml` plus overrides (e.g. `--set labels=off --set
  maze.style=organic`). Other subcommands: `reset`, `perturb`, `quiz`,
  `ablate`, `savings`, `report`, `shell`, `tail`, `dashboard`, `smoke`.
- `python scripts/duo_check.py` — host-side validation of duo mode
  (26 checks, no docker needed; boots a throwaway daemon on port 8798).
- `python scripts/make_replay.py <series> <ep_NNN> <out.html> [title]`
  — self-contained replay page for a solo episode;
  `scripts/make_duo_replay.py` likewise for duo episodes.
- `make dashboard` / `./botctl dashboard` — live experimenter dashboard
  (default 127.0.0.1:8080) against the running daemon.
- No API key? `--set model=mock:wall-follower` drives the identical
  harness path with a scripted agent.
- Other providers: `--set model=kimi:<id>[@effort]` / `zai:<id>[@effort]`
  / `deepseek:<id>[@effort]` / `gemini:<id>` / `openai:<id>` /
  `compat:<id>` (OpenAI-compatible adapter in `harness/llm.py`; keys
  from `MOONSHOT_API_KEY` / `ZAI_API_KEY` / `DEEPSEEK_API_KEY` /
  `GEMINI_API_KEY` / `OPENAI_API_KEY` / `LLM_BASE_URL`+`LLM_API_KEY`). Kimi "K3 Max" is `kimi:kimi-k3@max`;
  "Ox Alpha" is `zai:glm-5.3-flash`; reasoning traces are stored as
  thinking blocks and echoed back verbatim (Moonshot requires it, Z.ai
  keeps it with clear_thinking=false). Token-free check and the gate
  for any `harness/llm.py` change: `python scripts/llm_compat_check.py`
  (72 checks).

## Architecture

- `sim/config.py` is the single resolution point: DEFAULTS <- named
  noise profile <- config.yaml <- overrides; the resolved dict is passed
  to every process as JSON. `device_sets(cfg)` derives the logical
  sensor/actuator lists (vehicle model, encoders, locked maze, duo).
- `sim/daemon.py` (one process per episode) builds Maze + World(s) +
  DeviceBridge(s), writes `maze.json`, `device_map.json`,
  `resolved_config.json`, and the append-only `ground_truth*.jsonl`
  into the run dir, then ticks at `sim.tick_hz` x `realtime_factor`.
  Localhost HTTP API (`sim/api.py`, default port 8787): /health /state
  /maze /config /reset /pause /resume /rtf /shutdown.
- `devices/bridge.py`: sensors emit one frame line per open() then EOF
  (a held-open reader streams at ~40 Hz); actuators are held open
  O_RDWR and consume one ASCII int per line (garbage logged+ignored;
  `serial_tx` accepts raw text). `labels: off` names files d0..dN by a
  seeded permutation; a deleted file is re-created by a watchdog.
- `harness/episode.py`: the agent loop (Anthropic SDK, bash tool via
  `docker exec`). The context policy is deliberately dumb — on context
  full either end or a bare restart, nothing carried over except
  `/memory`. `harness/duo.py` runs two agents in threads against one
  shared world. `harness/series.py` chains episodes + perturbations.
- Duo mode (`duo.enabled`): two Worlds share the maze in lockstep; the
  peer is lidar-visible and collidable; one TX/RX port pair per bot
  delivers raw lines only within `duo.comms_range`, silently dropping
  the rest; every TX is ground-truth logged with delivered/dist.
- Determinism: all randomness flows from named seeded streams via
  `stable_seed()` (crc32, never `hash()`); the seed tuple includes
  maze seed, episode index, and (duo) bot id.

## Conventions and gotchas

- **The agent must never see ground truth.** Nothing from the run dir,
  the API, or this repo's host side may leak into `/bot`, `/memory`, or
  `/dev/robot`. The API key never enters the container or the repo
  (it lives in `/root/.mazebot_key` on this box, chmod 600).
- **Experiment purity**: no model fallbacks inside episodes (a refusal
  retries the same model, then ends the episode); prompts and READMEs
  stay minimal — labels-off runs must not name devices or morphology.
  The quiz eval is the one place server-side fallback is allowed.
- Run records are committed without ground truth or devfs:
  `find runs/<series> -type f ! -name 'ground_truth*.jsonl' -not -path
  '*/devfs/*' -exec git add -f {} +` (`runs/` is gitignored; exclude
  `__pycache__` too). Replay pages live at `runs/<series>/replay.html`.
- Ports: 8787 default daemon, 8790+ for concurrent runs, 8791 smoke,
  8798 duo_check, 8080 dashboard. A stale daemon on a port is detected
  by the /health pid check — kill it, don't reuse blindly.
- This box's dockerd dies periodically: kill stale containerd pids,
  `rm /var/run/docker.pid`, restart dockerd, `until docker info`.
  Avoid `pkill -f` matching your own command line (use `pgrep -x` or a
  `[.]` regex). Long-running experiments must be launched as tracked
  background tasks, not bare `nohup`.
- Style: PEP8-ish, 79-col, sparse comments that state invariants or
  non-obvious constraints only. DECISIONS.md records design rationale —
  add an entry when a default or mechanism changes; RUNBOOK.md records
  how to operate it.
