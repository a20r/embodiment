"""Model layer for the episode loop.

Three implementations behind one tiny interface:

  AnthropicModel      real Messages API calls via the official SDK
  OpenAICompatModel   chat-completions providers (kimi:, gemini:, openai:,
                      compat:), translating at the boundary
  MockWallFollower    deterministic scripted agent (model: mock:wall-follower)

All return SDK-shaped objects (content blocks with .type/.text/.id/.name/
.input, .stop_reason, .usage) so `episode.py` has exactly one loop.

Deliberate deviations, documented in DECISIONS.md:
- No server-side refusal fallbacks: an experiment episode must not be
  silently served by a different model; a refusal is logged and ends the
  episode.
- Manual loop (not the beta tool runner): the harness must own budget
  enforcement, transcript logging, and the dumb context policy.
"""

import json
import os
import time
from dataclasses import dataclass, field

BASH_TOOL = {
    "name": "bash",
    "description": (
        "Run a bash command on the robot's onboard computer. Returns "
        "stdout and stderr. Commands have a time limit; run long-lived "
        "programs in the background."
    ),
    "input_schema": {
        "type": "object",
        "properties": {
            "command": {"type": "string",
                        "description": "The bash command to run."},
        },
        "required": ["command"],
    },
}


@dataclass
class Usage:
    input_tokens: int = 0
    output_tokens: int = 0
    cache_creation_input_tokens: int = 0
    cache_read_input_tokens: int = 0
    # OpenAI-style providers report cache hits as a subset of
    # input_tokens; kept for billing, never added by context_tokens().
    cached_input_tokens: int = 0


@dataclass
class Block:
    type: str
    text: str = ""
    id: str = ""
    name: str = ""
    input: dict = field(default_factory=dict)
    thinking: str = ""


@dataclass
class Response:
    content: list
    stop_reason: str
    usage: Usage


def context_tokens(usage):
    """Approximate context occupancy after a call."""
    return (usage.input_tokens
            + (getattr(usage, "cache_read_input_tokens", 0) or 0)
            + (getattr(usage, "cache_creation_input_tokens", 0) or 0)
            + usage.output_tokens)


def thinking_param(model):
    """Adaptive thinking exists on the 4.6+/5 families; older models
    (e.g. claude-haiku-4-5) reject it, so we omit the parameter there."""
    import re
    if re.match(r"claude-(fable|mythos|opus|sonnet)-(5|4-[678])", model):
        return {"type": "adaptive"}
    return None


class AnthropicModel:
    def __init__(self, model):
        import anthropic
        self.model = model
        self.client = anthropic.Anthropic()
        self._anthropic = anthropic

    def create(self, system, messages, max_tokens=16000):
        a = self._anthropic
        kwargs = {}
        think = thinking_param(self.model)
        if think:
            kwargs["thinking"] = think
        attempts = 0
        while True:
            attempts += 1
            try:
                with self.client.messages.stream(
                    model=self.model,
                    max_tokens=max_tokens,
                    system=system,
                    tools=[BASH_TOOL],
                    cache_control={"type": "ephemeral"},
                    messages=messages,
                    **kwargs,
                ) as stream:
                    return stream.get_final_message()
            except (a.RateLimitError, a.APIConnectionError) as e:
                if attempts >= 6:
                    raise
                time.sleep(min(60, 2 ** attempts))
            except a.APIStatusError as e:
                if e.status_code >= 500 and attempts < 6:
                    time.sleep(min(60, 2 ** attempts))
                    continue
                raise

    @staticmethod
    def serialize_content(content):
        """SDK content blocks -> JSON-safe list for the transcript and for
        replaying assistant turns (thinking blocks pass through unchanged)."""
        out = []
        for b in content:
            if hasattr(b, "to_dict"):
                out.append(b.to_dict())
            else:
                out.append(b)
        return out


class MockWallFollower:
    """Scripted agent used when model == 'mock:wall-follower'.

    Exercises the identical harness path (bash tool calls through docker
    exec, transcripts, budgets, /memory writes) without an API key.  It
    assumes labeled devices — it is a harness test, not a science run.
    """

    def __init__(self, repo_root):
        with open(os.path.join(repo_root, "scripts",
                               "wall_follower.py")) as f:
            self.controller_src = f.read()
        self.turn = 0
        self.poll_count = 0

    def _tool(self, text, command):
        self.turn += 1
        return Response(
            content=[
                Block(type="text", text=text),
                Block(type="tool_use", id=f"mock_{self.turn}",
                      name="bash", input={"command": command}),
            ],
            stop_reason="tool_use",
            usage=Usage(input_tokens=200 * self.turn, output_tokens=120),
        )

    def create(self, system, messages, max_tokens=16000):
        last = messages[-1] if messages else {}
        last_text = ""
        if isinstance(last.get("content"), str):
            last_text = last["content"]
        elif isinstance(last.get("content"), list):
            for b in last["content"]:
                if isinstance(b, dict) and b.get("type") == "tool_result":
                    c = b.get("content", "")
                    last_text = c if isinstance(c, str) else json.dumps(c)

        # Wrap-up phase: the harness announced the episode end.
        if "episode has ended" in last_text.lower():
            return self._tool(
                "Recording what I learned to /memory before shutdown.",
                "mkdir -p /memory && cat >> /memory/notes.md <<'EOF'\n"
                "# Episode notes (mock agent)\n"
                "- Devices under /dev/robot: lidar (CSV meters, beam 0 "
                "forward, CCW), heading (deg CCW), encoder_left/right "
                "(cumulative ticks), bump_front/rear (0/1), status "
                "(tick=N goal=0/1), motor_left/right (PWM -255..255, "
                "positive = forward).\n"
                "- Right-hand wall following solves the course; "
                "controller saved at src/drive.py.\n"
                "- See status device for goal signal.\n"
                "EOF\n"
                "echo saved && head -3 /memory/notes.md")
        if self.turn >= 40 or "saved" in last_text[:20]:
            return Response(
                content=[Block(type="text",
                               text="Goal reached and notes saved to "
                                    "/memory/notes.md. Episode complete.")],
                stop_reason="end_turn",
                usage=Usage(input_tokens=200 * self.turn, output_tokens=40))

        script = [
            ("Waking up on the robot. Reading the manual first.",
             "ls -la && cat README.md && cat /memory/* 2>/dev/null; "
             "ls /dev/robot"),
            ("Checking the status and taking a lidar snapshot.",
             "head -1 /dev/robot/status; head -1 /dev/robot/lidar; "
             "head -1 /dev/robot/heading"),
            ("Writing a right-hand wall-following controller to src/.",
             "mkdir -p src && cat > src/drive.py <<'MOCK_CONTROLLER_EOF'\n"
             + self.controller_src + "\nMOCK_CONTROLLER_EOF\n"
             "echo written $(wc -l < src/drive.py) lines"),
            ("Starting the controller in the background.",
             "nohup python3 src/drive.py --max-wall-s 600 "
             "> /tmp/drive.log 2>&1 & echo started"),
        ]
        if self.turn < len(script):
            return self._tool(*script[self.turn])

        # Poll until the harness sees the goal and ends the episode.
        self.poll_count += 1
        return self._tool(
            f"Polling for progress (check {self.poll_count}).",
            "sleep 5; head -1 /dev/robot/status; tail -2 /tmp/drive.log")

    @staticmethod
    def serialize_content(content):
        out = []
        for b in content:
            d = {"type": b.type}
            if b.type == "text":
                d["text"] = b.text
            elif b.type == "tool_use":
                d.update(id=b.id, name=b.name, input=b.input)
            out.append(d)
        return out


# OpenAI-compatible providers.  Model strings are
# "<provider>:<model-id>[@<reasoning_effort>]"; the id is passed through
# verbatim (check the provider's docs for the exact current names) and
# the optional "@low|high|max" suffix becomes the request's
# reasoning_effort (Kimi K3 always reasons; "max" is what the Kimi app
# calls K3 Max).  Keys come from the environment on the host and, like
# the Anthropic key, never enter the container or the repo.
# tokens_param: the output-cap parameter the endpoint wants (Moonshot
# and OpenAI deprecate max_tokens in favour of max_completion_tokens).
PROVIDERS = {
    "kimi": dict(base_url="https://api.moonshot.ai/v1",
                 key_env="MOONSHOT_API_KEY",
                 tokens_param="max_completion_tokens"),
    "gemini": dict(base_url="https://generativelanguage.googleapis.com/"
                            "v1beta/openai/",
                   key_env="GEMINI_API_KEY"),
    "openai": dict(base_url=None, key_env="OPENAI_API_KEY",
                   tokens_param="max_completion_tokens"),
    # Any OpenAI-compatible endpoint: LLM_BASE_URL + LLM_API_KEY.
    "compat": dict(base_url=os.environ.get("LLM_BASE_URL"),
                   key_env="LLM_API_KEY"),
}

# Reasoning models return their trace as reasoning_content and require
# it echoed back verbatim on every historical assistant turn (Moonshot:
# "return the complete assistant message unchanged in multi-turn
# conversations and tool calls"); dropping it degrades the model.  The
# trace is stored as a thinking block in the transcript (the dashboard
# already renders those) and re-attached at the boundary.
REASONING_FIELDS = ("reasoning_content", "reasoning")

OPENAI_BASH_TOOL = {
    "type": "function",
    "function": {
        "name": BASH_TOOL["name"],
        "description": BASH_TOOL["description"],
        "parameters": BASH_TOOL["input_schema"],
    },
}


def to_openai_messages(system, messages):
    """Anthropic-shaped history (as the harness stores it) -> OpenAI
    chat format.  Assistant tool_use blocks become tool_calls; user
    tool_result blocks become role=tool messages; thinking blocks go
    back as reasoning_content (the provider's own trace, verbatim)."""
    out = [{"role": "system", "content": system}]
    for m in messages:
        role, content = m["role"], m["content"]
        if isinstance(content, str):
            out.append({"role": role, "content": content})
            continue
        if role == "assistant":
            texts, calls, thinks = [], [], []
            for b in content:
                b = b if isinstance(b, dict) else _block_dict(b)
                if b.get("type") == "text" and b.get("text"):
                    texts.append(b["text"])
                elif b.get("type") == "tool_use":
                    calls.append({
                        "id": b["id"], "type": "function",
                        "function": {"name": b["name"],
                                     "arguments": json.dumps(
                                         b.get("input") or {})}})
                elif b.get("type") == "thinking" and b.get("thinking"):
                    thinks.append(b["thinking"])
            msg = {"role": "assistant", "content": "\n".join(texts) or None}
            if calls:
                msg["tool_calls"] = calls
            if thinks:
                msg["reasoning_content"] = "\n".join(thinks)
            out.append(msg)
        else:
            texts = []
            for b in content:
                if isinstance(b, dict) and b.get("type") == "tool_result":
                    c = b.get("content", "")
                    out.append({"role": "tool",
                                "tool_call_id": b["tool_use_id"],
                                "content": c if isinstance(c, str)
                                else json.dumps(c)})
                elif isinstance(b, dict) and b.get("type") == "text":
                    texts.append(b.get("text", ""))
            if texts:
                out.append({"role": "user", "content": "\n".join(texts)})
    return out


def _block_dict(b):
    d = {"type": b.type}
    if b.type == "text":
        d["text"] = b.text
    elif b.type == "tool_use":
        d.update(id=b.id, name=b.name, input=b.input)
    elif b.type == "thinking":
        d["thinking"] = b.thinking
    return d


def _reasoning_of(msg):
    """The provider's reasoning trace, whichever field carries it.  The
    openai SDK keeps unknown response fields in model_extra."""
    extra = getattr(msg, "model_extra", None) or {}
    for name in REASONING_FIELDS:
        v = getattr(msg, name, None) or extra.get(name)
        if isinstance(v, str) and v:
            return v
    return ""


class OpenAICompatModel:
    """Chat-completions adapter presenting the same interface as
    AnthropicModel (content blocks, stop_reason, usage)."""

    STOP_MAP = {"tool_calls": "tool_use", "stop": "end_turn",
                "length": "max_tokens", "content_filter": "refusal"}

    # A max-effort reasoning turn over a 100k-token history can run for
    # minutes; the SDK's own retries are off because the loop below owns
    # the retry policy (and a duplicate long request would double-bill).
    TIMEOUT_S = 900

    def __init__(self, provider, model, reasoning_effort=None):
        import openai
        spec = PROVIDERS[provider]
        key = os.environ.get(spec["key_env"])
        if not key:
            raise RuntimeError(
                f"{spec['key_env']} is not set (needed for {provider}:)")
        self.provider = provider
        self.model = model
        self.reasoning_effort = reasoning_effort
        self.tokens_param = spec.get("tokens_param", "max_tokens")
        self._openai = openai
        self.client = openai.OpenAI(api_key=key, base_url=spec["base_url"],
                                    timeout=self.TIMEOUT_S, max_retries=0)

    def create(self, system, messages, max_tokens=16000):
        o = self._openai
        kwargs = {self.tokens_param: max_tokens}
        if self.reasoning_effort:
            kwargs["reasoning_effort"] = self.reasoning_effort
        attempts = 0
        while True:
            attempts += 1
            try:
                r = self.client.chat.completions.create(
                    model=self.model,
                    messages=to_openai_messages(system, messages),
                    tools=[OPENAI_BASH_TOOL],
                    **kwargs,
                )
                return self._translate(r)
            except (o.RateLimitError, o.APIConnectionError,
                    o.APITimeoutError):
                if attempts >= 6:
                    raise
                time.sleep(min(60, 2 ** attempts))
            except o.APIStatusError as e:
                if e.status_code >= 500 and attempts < 6:
                    time.sleep(min(60, 2 ** attempts))
                    continue
                raise

    def _translate(self, r):
        choice = r.choices[0]
        msg = choice.message
        blocks = []
        trace = _reasoning_of(msg)
        if trace:
            blocks.append(Block(type="thinking", thinking=trace))
        if msg.content:
            blocks.append(Block(type="text", text=msg.content))
        for tc in (msg.tool_calls or []):
            try:
                args = json.loads(tc.function.arguments or "{}")
            except json.JSONDecodeError:
                args = {"command": tc.function.arguments}
            blocks.append(Block(type="tool_use", id=tc.id,
                                name=tc.function.name, input=args))
        stop = self.STOP_MAP.get(choice.finish_reason, "end_turn")
        if msg.tool_calls and stop == "end_turn":
            stop = "tool_use"
        u = r.usage
        details = getattr(u, "prompt_tokens_details", None)
        cached = (getattr(details, "cached_tokens", None)
                  or getattr(u, "cached_tokens", None) or 0)
        usage = Usage(input_tokens=getattr(u, "prompt_tokens", 0) or 0,
                      output_tokens=getattr(u, "completion_tokens", 0) or 0,
                      cached_input_tokens=cached)
        return Response(content=blocks, stop_reason=stop, usage=usage)

    @staticmethod
    def serialize_content(content):
        return [b if isinstance(b, dict) else _block_dict(b)
                for b in content]


def make_model(model_string, repo_root):
    if model_string.startswith("mock:"):
        return MockWallFollower(repo_root)
    provider, sep, rest = model_string.partition(":")
    if sep and provider in PROVIDERS:
        model_id, at, effort = rest.partition("@")
        if at and effort not in ("low", "medium", "high", "max"):
            raise ValueError(f"unknown reasoning effort {effort!r} in "
                             f"{model_string!r} (low|medium|high|max)")
        return OpenAICompatModel(provider, model_id, effort or None)
    return AnthropicModel(model_string)
