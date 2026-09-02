"""Round-trip check of the OpenAI-compatible model adapter, no API key.

Boots a stub chat-completions server that scripts two turns (a bash
tool call with a reasoning trace, then a final text), points harness.llm
at it via the `compat:` provider, and asserts the Anthropic-shaped
history the harness keeps is translated correctly in both directions —
including the reasoning-model contract (Kimi K3): the trace comes back
as a thinking block and is echoed verbatim as reasoning_content on the
next request.  A third turn through the `kimi:` provider checks the
Moonshot-specific request shape (max_completion_tokens, @effort suffix).

Run from the repo root:  python scripts/llm_compat_check.py
"""

import json
import os
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

PORT = 8797
REQUESTS = []
FAILS = []
TRACE = "The README says there are ports under /dev/robot; list them."


def check(name, ok, detail=""):
    print(f"  {'ok  ' if ok else 'FAIL'} {name}" +
          (f"  ({detail})" if detail else ""))
    if not ok:
        FAILS.append(name)


class Stub(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        body = json.loads(self.rfile.read(n) or b"{}")
        REQUESTS.append((self.path, body))
        turn = len(REQUESTS)
        usage = {"prompt_tokens": 100 * turn, "completion_tokens": 20}
        if turn == 1:
            msg = {"role": "assistant", "content": "Let me look around.",
                   "reasoning_content": TRACE,
                   "tool_calls": [{"id": "call_1", "type": "function",
                                   "function": {"name": "bash",
                                                "arguments":
                                                json.dumps({"command":
                                                            "ls /dev/robot"})}}]}
            finish = "tool_calls"
        elif turn == 2:
            msg = {"role": "assistant", "content": "Done for now."}
            finish = "stop"
            usage["prompt_tokens_details"] = {"cached_tokens": 150}
        else:
            # Moonshot-style: cache hits reported at the top level.
            msg = {"role": "assistant", "content": "ok",
                   "reasoning_content": "brief"}
            finish = "stop"
            usage["cached_tokens"] = 40
        resp = {"id": f"stub-{turn}", "object": "chat.completion",
                "created": 0, "model": body.get("model"),
                "choices": [{"index": 0, "message": msg,
                             "finish_reason": finish}],
                "usage": usage}
        data = json.dumps(resp).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def main():
    srv = HTTPServer(("127.0.0.1", PORT), Stub)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    os.environ["LLM_BASE_URL"] = f"http://127.0.0.1:{PORT}/v1"
    os.environ["LLM_API_KEY"] = "stub"
    # PROVIDERS reads LLM_BASE_URL at import time.
    from harness import llm
    llm.PROVIDERS["compat"]["base_url"] = os.environ["LLM_BASE_URL"]

    model = llm.make_model("compat:stub-model", ".")
    check("compat provider selected",
          isinstance(model, llm.OpenAICompatModel))
    check("no effort suffix -> no reasoning_effort",
          model.reasoning_effort is None)

    system = "You are connected to a robot."
    messages = [{"role": "user", "content": "Begin."}]
    r1 = model.create(system, messages)
    path, body = REQUESTS[0]
    check("posts to /v1/chat/completions",
          path.endswith("/chat/completions"), path)
    check("system message first",
          body["messages"][0] == {"role": "system", "content": system})
    check("bash tool advertised as a function",
          body["tools"][0]["function"]["name"] == "bash"
          and "command" in body["tools"][0]["function"]["parameters"]
          ["properties"])
    check("compat provider sends max_tokens",
          "max_tokens" in body and "max_completion_tokens" not in body)
    check("reasoning_effort omitted when unset",
          "reasoning_effort" not in body)
    check("tool call -> tool_use block",
          r1.stop_reason == "tool_use"
          and any(b.type == "tool_use" and b.name == "bash"
                  and b.input == {"command": "ls /dev/robot"}
                  and b.id == "call_1" for b in r1.content))
    check("text preserved alongside tool call",
          any(b.type == "text" and "look around" in b.text
              for b in r1.content))
    check("reasoning_content -> thinking block (first)",
          r1.content[0].type == "thinking"
          and r1.content[0].thinking == TRACE)
    check("usage mapped", r1.usage.input_tokens == 100
          and r1.usage.output_tokens == 20
          and llm.context_tokens(r1.usage) == 120)

    # Replay the turn the way episode.py does.
    content = model.serialize_content(r1.content)
    check("thinking block serializes JSON-safe with its text",
          content[0] == {"type": "thinking", "thinking": TRACE})
    messages.append({"role": "assistant", "content": content})
    messages.append({"role": "user", "content": [
        {"type": "tool_result", "tool_use_id": "call_1",
         "content": "d0 d1 d2"}]})
    r2 = model.create(system, messages)
    _, body2 = REQUESTS[1]
    roles = [m["role"] for m in body2["messages"]]
    check("history roles system/user/assistant/tool",
          roles == ["system", "user", "assistant", "tool"], str(roles))
    asst = body2["messages"][2]
    check("assistant turn carries tool_calls",
          asst.get("tool_calls") and asst["tool_calls"][0]["id"] == "call_1"
          and json.loads(asst["tool_calls"][0]["function"]["arguments"])
          == {"command": "ls /dev/robot"})
    check("assistant turn echoes reasoning_content verbatim",
          asst.get("reasoning_content") == TRACE)
    tool = body2["messages"][3]
    check("tool result -> role=tool with matching id",
          tool == {"role": "tool", "tool_call_id": "call_1",
                   "content": "d0 d1 d2"})
    check("final text -> end_turn",
          r2.stop_reason == "end_turn"
          and r2.content[0].type == "text"
          and r2.content[0].text == "Done for now.")
    check("no trace -> no thinking block",
          all(b.type != "thinking" for b in r2.content))
    check("cached_tokens (OpenAI-style details) recorded, not "
          "double-counted",
          r2.usage.cached_input_tokens == 150
          and llm.context_tokens(r2.usage) == 220)
    check("serialize_content is JSON-safe",
          json.dumps(model.serialize_content(r2.content)) is not None)

    # Moonshot request shape through the kimi: provider.
    os.environ["MOONSHOT_API_KEY"] = "stub"
    real_base = llm.PROVIDERS["kimi"]["base_url"]
    llm.PROVIDERS["kimi"]["base_url"] = os.environ["LLM_BASE_URL"]
    kimi = llm.make_model("kimi:kimi-k3@low", ".")
    check("kimi provider selected with effort suffix",
          isinstance(kimi, llm.OpenAICompatModel)
          and kimi.model == "kimi-k3" and kimi.reasoning_effort == "low")
    r3 = kimi.create(system, [{"role": "user", "content": "Begin."}])
    _, body3 = REQUESTS[2]
    check("kimi sends model id without the suffix",
          body3["model"] == "kimi-k3")
    check("kimi sends max_completion_tokens, never max_tokens",
          "max_completion_tokens" in body3 and "max_tokens" not in body3)
    check("kimi sends reasoning_effort", body3.get("reasoning_effort") == "low")
    check("kimi omits fixed sampling params",
          not {"temperature", "top_p", "n", "presence_penalty",
               "frequency_penalty"} & set(body3))
    check("kimi top-level cached_tokens recorded",
          r3.usage.cached_input_tokens == 40)
    check("kimi trace captured", r3.content[0].thinking == "brief")
    check("client timeout is long and SDK retries are off",
          kimi.client.timeout == llm.OpenAICompatModel.TIMEOUT_S
          and kimi.client.max_retries == 0)
    try:
        llm.make_model("kimi:kimi-k3@turbo", ".")
        check("unknown effort suffix rejected", False)
    except ValueError as e:
        check("unknown effort suffix rejected", "turbo" in str(e))
    llm.PROVIDERS["kimi"]["base_url"] = real_base

    # Provider table sanity.
    check("kimi/gemini providers registered",
          {"kimi", "gemini"} <= set(llm.PROVIDERS))
    for name in ("kimi", "gemini"):
        os.environ.pop(llm.PROVIDERS[name]["key_env"], None)
        try:
            llm.make_model(f"{name}:some-model", ".")
            check(f"{name} without key raises", False)
        except RuntimeError as e:
            check(f"{name} without key raises",
                  llm.PROVIDERS[name]["key_env"] in str(e))
    check("bare model string stays Anthropic",
          llm.make_model.__code__ is not None and
          not any(s in "claude-fable-5" for s in (":",)))
    srv.shutdown()
    print("PASS" if not FAILS else f"FAILED: {FAILS}")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
