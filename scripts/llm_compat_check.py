"""Round-trip check of the OpenAI-compatible model adapter, no API key.

Boots a stub chat-completions server that scripts two turns (a bash
tool call, then a final text), points harness.llm at it via the
`compat:` provider, and asserts the Anthropic-shaped history the
harness keeps is translated correctly in both directions.

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
        if turn == 1:
            msg = {"role": "assistant", "content": "Let me look around.",
                   "tool_calls": [{"id": "call_1", "type": "function",
                                   "function": {"name": "bash",
                                                "arguments":
                                                json.dumps({"command":
                                                            "ls /dev/robot"})}}]}
            finish = "tool_calls"
        else:
            msg = {"role": "assistant", "content": "Done for now."}
            finish = "stop"
        resp = {"id": f"stub-{turn}", "object": "chat.completion",
                "created": 0, "model": body.get("model"),
                "choices": [{"index": 0, "message": msg,
                             "finish_reason": finish}],
                "usage": {"prompt_tokens": 100 * turn,
                          "completion_tokens": 20}}
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
    check("tool call -> tool_use block",
          r1.stop_reason == "tool_use"
          and any(b.type == "tool_use" and b.name == "bash"
                  and b.input == {"command": "ls /dev/robot"}
                  and b.id == "call_1" for b in r1.content))
    check("text preserved alongside tool call",
          any(b.type == "text" and "look around" in b.text
              for b in r1.content))
    check("usage mapped", r1.usage.input_tokens == 100
          and r1.usage.output_tokens == 20
          and llm.context_tokens(r1.usage) == 120)

    # Replay the turn the way episode.py does.
    content = model.serialize_content(r1.content)
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
    tool = body2["messages"][3]
    check("tool result -> role=tool with matching id",
          tool == {"role": "tool", "tool_call_id": "call_1",
                   "content": "d0 d1 d2"})
    check("final text -> end_turn",
          r2.stop_reason == "end_turn"
          and r2.content[0].type == "text"
          and r2.content[0].text == "Done for now.")
    check("serialize_content is JSON-safe",
          json.dumps(model.serialize_content(r2.content)) is not None)

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
