# Mazebot

A research platform for studying **emergent memory architecture in
embodied LLM agents**.  An agent wakes up on a simulated wheeled robot —
motors and sensors exposed only as device files inside an airgapped
container — and must discover its own embodiment, write its own control
code, and manage its own persistent `/memory` across episodes.  The
platform runs episode series, measures them, and lets you watch.

There is no hardware.  The 2D kinematic sim is the ground truth; from
inside the container it is indistinguishable from a real device bus.

## Quickstart

```bash
pip install -r requirements.txt   # host side: pyyaml + anthropic
make build                        # build the bot container image
make smoke                        # prove the whole simulated world (~3 min)

# record a no-API demo episode (scripted mock agent):
./botctl run --series demo --model mock:wall-follower \
    --set sim.realtime_factor=4

# live agent episodes (uses config.yaml; needs ANTHROPIC_API_KEY):
./botctl run

make dashboard                    # http://127.0.0.1:8080
make report
```

See `RUNBOOK.md` for the full 10-episode hard-mode recipe, and
`DECISIONS.md` for every default and why.

## Architecture

```
 host                                    │ bot container (docker,
                                         │  --network=none, plain
 ┌──────────────┐  owns ground truth     │  Debian + python3)
 │  sim daemon  │  ticks at 50 Hz        │
 │  (sim/)      │──── ground_truth.jsonl │
 │              │                        │
 │  FIFO bridge │══ bind mount ═════════▶│  /dev/robot/{lidar,motor_*,…}
 │  (devices/)  │                        │      ▲ the ONLY interface
 └──────┬───────┘                        │      │
        │ HTTP (localhost)               │  ┌───┴────┐
 ┌──────┴───────┐                        │  │ agent's│  /bot (README, src/)
 │  dashboard   │                        │  │ shell  │  /memory (persistent)
 │  (dashboard/)│                        │  └───▲────┘
 └──────────────┘                        │      │ docker exec
 ┌──────────────┐   Anthropic API        │      │
 │  harness     │────────────────────────┼──────┘
 │  (harness/)  │   bash tool loop       │
 └──────────────┘                        │
```

- **`sim/`** — maze generator (seeded, deterministic), differential-drive
  kinematics, raycast lidar, encoders, IMU heading, bump switches, the
  realism dial (noise profiles), perturbations, and the per-tick
  append-only ground-truth log.  The agent can never read any of it.
- **`devices/`** — the FIFO bridge that exposes sim I/O as files under
  `/dev/robot/` in the container.  `labels: off` renames everything to
  `d0..dN`.
- **`botfs/`** — the repo the agent wakes up in (README variants).
- **`harness/`** — episode/series runners, the bash-only tool loop, the
  dumb context policy, transcripts, Arm A/B memory handling.
- **`evals/`** — learning curve, fresh-context probe quiz, ablation,
  savings, provenance.
- **`dashboard/` + `botctl`** — live top-down view, transcript stream,
  memory browser with per-episode diffs, series metrics; CLI for all of
  the above.

## Where things live

```
runs/<series>/
  series.json            config echo
  state.json             cumulative perturbation state
  memory/                the agent's persistent /memory (live)
  evals/                 eval outputs (json/csv/svg)
  ep_003/
    ground_truth.jsonl   every tick + every device read/write (host only)
    transcript.jsonl     the full agent conversation
    summary.json         solved, ticks, tokens, end reason…
    maze.json            walls, start/goal, dead ends, solution path
    device_map.json      file → physical sensor binding (host only)
    memory_snapshot/     /memory as it stood at episode end
    bot/                 the agent's working directory
    devfs/               the FIFO device tree
```

## Adding a perturbation

1. Add a name + state change in `sim/perturb.py` (`apply()`).
2. Make the state matter: maze params in `sim/daemon.py`, device wiring
   in `devices/bridge.py:compute_bindings`, or physics in
   `sim/world.py`.
3. It is now usable as `./botctl perturb <name>`, in `config.yaml`
   under `perturbations:`, and in `evals/savings.py`.

## The two arms

- **Arm A**: `/memory` starts empty; zero guidance on how to use it.
- **Arm B**: `/memory` pre-seeded with a prescribed system (INDEX,
  topics/, CHANGELOG, conventions) and the system prompt explains it.

Compare arms by running two series and diffing their reports.

## Recorded live runs

`runs/{shakedown,haiku1,lost1,lost2,keyquest}` ship with this repo:
transcripts, summaries, maze data, device maps, and the agents' full
`/memory` — everything except the multi-GB-scale ground-truth tick logs
(regenerate those by re-running; the dashboard's replay view works from
what's committed only for trails already exported). Load any of them in
the dashboard, or export a shareable single-file replay page:

```bash
python3 scripts/make_replay.py <series> ep_001 out.html "Page Title"
```
