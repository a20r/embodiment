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

### 7.3 DeepSeek V4 and the cheap-tier landscape (Sept 2026)

`deepseek:deepseek-v4-flash` / `deepseek:deepseek-v4-pro` at
`https://api.deepseek.com` (`DEEPSEEK_API_KEY`).  Thinking is on by
default at effort `high` (`@low|high|max`); with tools present the API
requires reasoning_content passed back on every prior assistant turn
or returns 400, so the provider pads like Moonshot.  Direct prices are
time-of-day dependent (peak 01-04 and 06-10 UTC Mon-Fri doubles them):
off-peak flash $0.22/M input, $0.007/M cache hit, $0.66/M output; pro
$0.66 / $0.022 / $1.98.

Terminal-Bench 2.1 (the closest public proxy for a bash-tool agent
loop; llm-stats, 2 Sept 2026) against list prices:

| model | TB 2.1 | $/M in | $/M out |
|---|---|---|---|
| Gemini 3.8 Flash (`gemini:gemini-3.8-flash`) | 89.4 | 0.75 | 3.75 |
| Kimi K3 (`kimi:kimi-k3@max`) | 88.3 | 3.00 | 15.00 |
| GLM-5.3 (`zai:glm-5.3`) | 88.2 | 1.40 | 4.40 |
| DeepSeek V4 Pro (`deepseek:deepseek-v4-pro`) | 87.9 | 0.66 | 1.98 |
| GPT-5.6 Luna (`openai:gpt-5.6-luna`) | 84.7 | 0.20 | 1.20 |
| GLM-5.3-Flash (`zai:glm-5.3-flash`) | 84.3 | 0.15 | 0.50 |
| Claude Fable 5 | 84.3 | 10.00 | 50.00 |
| DeepSeek V4 Flash (`deepseek:deepseek-v4-flash`) | 82.7 | 0.22 | 0.66 |

GLM-5.3-Flash is the cheapest model on the board within five points
of the top; DeepSeek V4 Pro is the cheapest within two points.  For
an append-only harness the cache-hit price dominates input cost, so
compare output prices first.

### 7.4 Gemini 3.8 Flash

Getting a key (Google AI Studio, no Cloud console work needed):

1. Sign in at https://aistudio.google.com/apikey and click *Create
   API key*.  Every key belongs to a Google Cloud project; a default
   one is created for you on accepting the terms.
2. The key starts on the **Free** tier.  Free-tier prompts and
   responses "may be used to improve Google products" and are seen by
   human reviewers - do not run experiments on it.  Click *Set up
   billing* in AI Studio and attach a billing account: that is Tier 1
   (instant, $250/month cap; Tier 2 after $100 paid + 3 days).  Paid
   traffic is not used for training.  Exact RPM/TPM limits are shown
   per model in AI Studio rather than documented.
3. Store it like the others; the adapter reads `GEMINI_API_KEY`.

```bash
install -m 600 /dev/null /root/.mazebot_gemini_key   # then paste the key in
GEMINI_API_KEY="$(cat /root/.mazebot_gemini_key)" \
  ./botctl run --set model=gemini:gemini-3.8-flash@high ...
```

Model id `gemini-3.8-flash`, OpenAI-compatible base
`https://generativelanguage.googleapis.com/v1beta/openai/`.  The
`@minimal|low|medium|high` suffix maps onto Gemini's thinking level
(`high` is the 3.x default and is sent explicitly; thinking cannot be
switched off on 3.x models).  Pricing (paid tier): $0.75/M input,
$3.75/M output including thinking tokens, $0.075/M cached input,
through 31 Dec 2026; doubles on 1 Jan 2027.  Two caveats to settle on
the pre-spend probe: Google's OpenAI layer is "still in beta", and
thought summaries requested via `include_thoughts` may not surface
through chat completions at all ("no mechanism for out-of-band
transmission" per Google staff), in which case Gemini runs record no
reasoning trace - unlike Kimi, GLM and DeepSeek.  Explicit context
caching is a separate, opt-in API; the implicit cache is what the
`cached` usage field reflects, if the layer reports it.

## 8. 3D lidar (point cloud)

`--set lidar3d.enabled=true` replaces the 16-beam `lidar` port with a
`lidar3d` port: a spinning multi-ring unit (defaults: 16 rings over a
30 deg vertical field, 180 azimuths, 3 m range, mounted 0.12 m up)
returning one frame per `cat` as `x,y,z` triples in meters separated
by `;`, in the sensor frame (x forward, y left, z up, origin at the
sensor).  The world grows a floor at z=0, walls 0.40 m tall, a
0.15 m-tall peer and a 0.25 m key post, so rings paint the floor near
the robot, wall faces out to where the beam clears the wall top, and
nothing beyond; no-return points are omitted.  A frame is ~55 kB and
casts in under 10 ms; a held-open reader streams at `lidar3d.stream_hz`
(10) instead of 40.  Ground truth logs each read as a digest
(`<55513B 2845pts sha1=...>`) rather than the frame.  With labels on
the README variant `labeled_lidar3d` is selected automatically; with
labels off nothing changes and the agent must work out what a 55 kB
line of triples is.  The quiz asks for ring count, mount height and
the forward axis instead of beam count.  Gate: `python
scripts/lidar3d_check.py` (23 checks, no docker; boots a daemon on
port 8796).

The dashboard's left panel has a **3D** tab (three.js r128, vendored
under `dashboard/static/vendor/`, so it works offline): translucent
walls extruded to `wall_height`, floor, goal cell, each robot as a
cylinder with a heading mark, and the live ground-truth cloud coloured
by height when the run has `lidar3d`; orbit with the mouse, *follow*
keeps the camera on bot A.  The cloud is only cast when that tab is
showing (`/state?cloud=1`).  Headless render check (Playwright +
Chromium, against a running dashboard): `python
scripts/dashboard_render_check.py http://127.0.0.1:8080/ <series>`
screenshots both views and fails on any page error.
