# RUNBOOK — launch a 10-episode Arm A series in hard mode and watch it

Everything below runs from the repo root on a machine with Docker and
Python 3.11+.

## 0. One-time setup

```bash
pip install -r requirements.txt
make build            # bot container image (needs network, once)
make smoke            # ~3 min; must end with SMOKE PASSED
export ANTHROPIC_API_KEY=...   # or `ant auth login`
```

## 1. Launch the series

Hard mode = anonymous devices (`labels: off`), shipped noise profile,
scratch memory (Arm A).  Ten episodes, one maze family, persistent
`/memory` across all ten:

```bash
./botctl run --series hard10 --episodes 10 \
    --arm A --labels off --noise default_noisy
```

That's it.  Episodes run sequentially; each prints its summary line.
The series survives interruption — rerunning the same command resumes
after the last finished episode (`--fresh` starts over).

Useful knobs (any config key):

```bash
./botctl run --series hard10 --episodes 10 --arm A --labels off \
    --set sim.realtime_factor=2 \
    --set budget.max_wallclock_s=1200 \
    --set maze.seed=42
```

## 2. Watch it

In a second terminal:

```bash
make dashboard        # http://127.0.0.1:8080
```

Pick series `hard10`.  The live episode shows the true pose, lidar rays,
breadcrumb trail and collision flashes over the maze; the Transcript tab
streams the agent's turns; the Memory tab shows `/memory` live (and
per-episode diffs once episodes finish); Metrics shows the learning
curve as it grows.  Finished episodes get a replay scrubber.

Terminal alternatives:

```bash
./botctl tail --series hard10 -f     # follow the transcript
./botctl shell --series hard10      # shell into the bot container
```

## 3. Mid-series interventions (optional)

```bash
./botctl perturb motor_swap --series hard10    # applies next episode
```

or schedule in `config.yaml` before launching:

```yaml
perturbations:
  - {at_episode: 6, name: motor_swap}
```

## 4. Measure

```bash
./botctl report --series hard10              # learning curve + summary
./botctl quiz --series hard10                # fresh-LLM probe of /memory
./botctl ablate topics/maze.md --series hard10
./botctl savings maze_regen --series hard10 --episodes 3
```

`report` regenerates `runs/hard10/evals/learning_curve.{json,csv,svg}`
and prints every eval summary it finds.  All evals also appear in the
dashboard's Metrics tab.

## 5. No API key? Dry-run everything

The scripted mock agent exercises the identical pipeline (container,
tool loop, transcripts, memory, evals, dashboard):

```bash
./botctl run --series demo --model mock:wall-follower \
    --set sim.realtime_factor=4
./botctl report --series demo
./botctl quiz --series demo --model none
```

## Troubleshooting

- `docker: command not found` / daemon not running — start Docker; the
  harness talks to it via the CLI.
- Episode hangs at start — check `runs/<series>/ep_NNN/daemon.log`.
- Ports 8787/8080 busy — `--set sim.api_port=…` /
  `./botctl dashboard --port …`.
- A wedged container: `docker rm -f $(docker ps -aq
  --filter name=mazebot-)`.

## 6. Duo (two bots, one world)

```bash
./botctl run --set duo.enabled=true --set labels=off \
  --set prompt_variant=lost --set readme_variant=minimal_duo \
  --set maze.style=organic --set series.name=duo1
```

Two containers come up (`...-ep1a`, `...-ep1b`), each with its own
/dev/robot, /bot and /memory; transcripts land as
`transcript_a.jsonl` / `transcript_b.jsonl`, ground truth as
`ground_truth_a.jsonl` / `ground_truth_b.jsonl`.  The bots can only
talk over the in-world transceiver, and only within
`duo.comms_range` meters.  Replay:

```bash
python scripts/make_duo_replay.py duo1 ep_001 duo1_replay.html "Duo"
```

Validate the machinery without spending tokens:
`python scripts/duo_check.py` (host-side, no docker needed).
