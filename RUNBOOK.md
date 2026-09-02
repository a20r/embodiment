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
An optional `@<effort>` suffix on the id becomes the request's
`reasoning_effort` (`@low|high|max` for kimi, `@low|medium|high|max`
elsewhere; only for models that take one).
Keys live in the host environment only; they never enter the
container, the daemon's environment, or the repo.  Validate the
adapter without spending tokens: `python scripts/llm_compat_check.py`
(stub server on port 8797; the gate for any `harness/llm.py` change).
Caveats: adaptive thinking is Anthropic-only (omitted elsewhere);
`stop_reason: refusal` maps from `finish_reason: content_filter` and
from Moonshot's HTTP 400 `type=content_filter` (same-model retry x5,
then the episode ends); the quiz eval's server-side-fallback option
remains Anthropic-only.  `budget.max_output_tokens_per_turn` (default
16000) is the per-call output cap; reasoning tokens count against it
and against `max_total_output_tokens`.

### 7.1 Kimi K3 ("K3 Max")

The Kimi app has no model called "K3 Max": its selector offers K3 with
a Low/High/Max thinking strength.  The API equivalent is the model
`kimi-k3` at `https://api.moonshot.ai/v1` with `reasoning_effort`
`low|high|max` (`max` is the server default and is what the adapter
sends when no suffix is given, so run records always state it; there
is no `medium`).  K3 always reasons: every reply carries a
`reasoning_content` trace, billed as output, which the adapter stores
as a `thinking` block and echoes back verbatim on every historical
assistant turn — Moonshot requires the complete assistant message
returned unchanged in tool-call chains (a tool-call turn without the
key is a 400; an empty trace is still echoed).  Sampling parameters are
fixed server-side and are not sent; the output cap goes as
`max_completion_tokens`; requests are streamed because Moonshot's
gateway kills non-streaming requests at 900 s, which a max-effort turn
over a long history can exceed.

```bash
# one-time: key file next to the Anthropic one (never in the repo)
install -m 600 /dev/null /root/.mazebot_kimi_key   # then paste the key in

# scope the key to the launcher rather than exporting it into the shell
MOONSHOT_API_KEY="$(cat /root/.mazebot_kimi_key)" \
  ./botctl run --set model=kimi:kimi-k3@max \
               --set budget.max_output_tokens_per_turn=65536 \
               --set budget.max_total_output_tokens=600000 ...
MOONSHOT_API_KEY="$(cat /root/.mazebot_kimi_key)" \
  ./botctl run --set model=kimi:kimi-k3@low ...      # same model, cheaper
```

Before spending: `python scripts/llm_compat_check.py`, then one
`kimi:kimi-k3@low` solo episode with `--set budget.max_turns=5` and
confirm `usage.cached > 0` from turn 2 and a `thinking` block on every
assistant record.  `botctl quiz` still uses the Anthropic client; pass
`--model claude-...` explicitly for kimi series.

Pricing (Moonshot direct, Sept 2026): $3.00/M input, $0.30/M on an
automatic prefix-cache hit, $15.00/M output (reasoning included); 1M
context; flat.  Rate limits follow the cumulative cash top-up tier:
Tier0 ($1) is concurrency 1 / 3 RPM / 1.5M tokens per day — it cannot
run a duo (the second bot 429s for the whole of the peer's turn) and
exhausts the day after a handful of 160k-context requests; **Tier1
($10)** gives 15 concurrent / 100 RPM / 2M TPM and is the minimum for
any real run; Tier2 ($20) only matters for two concurrent duo runs.
Limits are charged on `prompt_tokens + max_completion_tokens`, so the
per-turn cap above is part of the bill of each request.  Rough cost
per 150-turn episode at ~95% cache hits: $5 solo / $10 duo at `low`,
$7 / $14 at `high`, $12 / $24 at `max` (±2x on the reasoning-length
assumption); at `max` raise `max_total_output_tokens` or the episode
ends on `token_budget` after ~40 turns, and expect fewer turns per
wallclock hour than Claude.  Transcripts record `usage.cached` per turn
so the hit share is auditable.  Aggregators (OpenRouter, Novita,
DeepInfra) list `moonshotai/kimi-k3` at list price plus fees and
surface reasoning under other field names; use Moonshot direct.

### 7.2 Z.ai GLM-5.3-Flash ("Ox Alpha")

The stealth "Ox Alpha" of August 2026 is Z.ai's `glm-5.3-flash`: a
320B/18B MoE reasoning model, MIT weights, tool calling, 1M context,
128K max output, always thinking.  Z.ai's OpenAI-compatible endpoint
is `https://api.z.ai/api/paas/v4/`; the `zai:` provider sends
`thinking: {type: enabled, clear_thinking: false}` so the server keeps
the reasoning trace the adapter echoes (the API default `true` strips
it), and `reasoning_effort` from the `@low|medium|high|xhigh|max`
suffix (`max` by default; `none`/`minimal` are refused because 5.3-flash
cannot stop thinking).  Moderation arrives as `finish_reason:
sensitive` and maps to `refusal`.

```bash
install -m 600 /dev/null /root/.mazebot_zai_key      # then paste the key in
ZAI_API_KEY="$(cat /root/.mazebot_zai_key)" \
  ./botctl run --set model=zai:glm-5.3-flash@max ...
```

Pricing (Z.ai direct): $0.075/M input, $0.015/M cached input, $0.25/M
output as a launch promotion through 9 Sept 2026 (reported list price
after that: $0.15/$0.50) — roughly a fortieth of Kimi K3, so a
150-turn episode is well under $1 even at `max`.  OpenRouter
(`z-ai/glm-5.3-flash`), Novita and DeepInfra serve it at about the same
price.  Z.ai publishes no rate-limit tiers and there are public
reports of heavy throttling under load; run the same pre-spend probe
as for Kimi and treat the first duo as a concurrency pilot.  Plain
JSON requests (no streaming) for now.
