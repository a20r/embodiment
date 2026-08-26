"""Model layer for the episode loop.

Two implementations behind one tiny interface:

  AnthropicModel      real Messages API calls via the official SDK
  MockWallFollower    deterministic scripted agent (model: mock:wall-follower)

Both return SDK-shaped objects (content blocks with .type/.text/.id/.name/
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


class AnthropicModel:
    def __init__(self, model):
        import anthropic
        self.model = model
        self.client = anthropic.Anthropic()
        self._anthropic = anthropic

    def create(self, system, messages, max_tokens=16000):
        a = self._anthropic
        attempts = 0
        while True:
            attempts += 1
            try:
                with self.client.messages.stream(
                    model=self.model,
                    max_tokens=max_tokens,
                    system=system,
                    thinking={"type": "adaptive"},
                    tools=[BASH_TOOL],
                    cache_control={"type": "ephemeral"},
                    messages=messages,
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


def make_model(model_string, repo_root):
    if model_string.startswith("mock:"):
        return MockWallFollower(repo_root)
    return AnthropicModel(model_string)
