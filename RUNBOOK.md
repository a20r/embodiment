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

## 7. Other model providers

The agent model is selected by `model:` (config.yaml or `--set model=`).
Bare names are Anthropic (`claude-...`); `mock:wall-follower` is the
scripted agent.  Any OpenAI-compatible provider works via a prefix:

```bash
export MOONSHOT_API_KEY=...   # Kimi (Moonshot)
./botctl run --set model=kimi:<model-id> ...

export GEMINI_API_KEY=...     # Google Gemini (OpenAI-compatible endpoint)
./botctl run --set model=gemini:<model-id> ...

export OPENAI_API_KEY=...     # OpenAI
./botctl run --set model=openai:<model-id> ...

export LLM_BASE_URL=https://host/v1 LLM_API_KEY=...   # anything else
./botctl run --set model=compat:<model-id> ...
```

The `<model-id>` is passed through verbatim — take it from the
provider's model list (names change; nothing here hardcodes one).
An optional `@low|medium|high|max` suffix on the id becomes the
request's `reasoning_effort` (only for models that take one).
Keys live in the host environment only; they never enter the
container or the repo.  Validate the adapter without spending tokens:
`python scripts/llm_compat_check.py` (stub server on port 8797).
Caveats: adaptive thinking is Anthropic-only (omitted elsewhere);
`stop_reason: refusal` maps from `finish_reason: content_filter`; the
quiz eval's server-side-fallback option remains Anthropic-only.

### 7.1 Kimi K3 ("K3 Max")

The Kimi app's "K3 Max" is the API model `kimi-k3` at
`https://api.moonshot.ai/v1` run at `reasoning_effort=max` (the API
default; `low` and `high` also exist).  K3 always reasons: every reply
carries a `reasoning_content` trace, billed as output, which the
adapter stores as a `thinking` block and echoes back verbatim on every
historical assistant turn — Moonshot requires the complete assistant
message returned unchanged in tool-call chains, and dropping it
degrades the model.  Sampling parameters are fixed server-side and are
not sent; the output cap goes as `max_completion_tokens`.

```bash
# one-time: key file next to the Anthropic one (never in the repo)
install -m 600 /dev/null /root/.mazebot_kimi_key   # then paste the key in
export MOONSHOT_API_KEY="$(cat /root/.mazebot_kimi_key)"

./botctl run --set model=kimi:kimi-k3@max ...        # what the app calls K3 Max
./botctl run --set model=kimi:kimi-k3@low ...        # same model, cheaper
```

Pricing (Moonshot direct, Sept 2026): $3.00/M input, $0.30/M on cache
hit, $15.00/M output; 1M context; a $1 minimum top-up opens the API and
the cumulative top-up tier sets concurrency/RPM/TPM — a duo run needs
two concurrent long requests.  Transcripts record `usage.cached` per
turn so the cache-hit share is auditable.  Aggregators (OpenRouter,
Novita, DeepInfra) list `moonshotai/kimi-k3` at the same price but
differ in how they surface reasoning; use Moonshot direct.
