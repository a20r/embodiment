"""Round-trip check of the OpenAI-compatible model adapter, no API key.

Boots a stub chat-completions server and drives harness.llm against it
two ways: the `compat:` provider (plain JSON responses) and the `kimi:`
provider (streamed SSE, Moonshot request shape).  Asserts the
Anthropic-shaped history the harness keeps is translated correctly in
both directions, including the reasoning-model contract Kimi K3 lives
by: the trace comes back as a thinking block (an empty trace included)
and is echoed verbatim as reasoning_content on the next request; a
tool-call turn is padded with "" when nothing was stored; signed
Anthropic thinking blocks are never replayed.  Also covers the error
mapping (400 content_filter -> refusal, typed 429s, Retry-After), the
truncated-tool-call guard, cached-token accounting, and model_spec.

Run from the repo root:  python scripts/llm_compat_check.py
"""

import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

PORT = 8797
REQUESTS = []
FAILS = []
SEEN = set()
TRACE = "The README says there are ports under /dev/robot; list them."
RAW_ARGS = '{"command":"ls   /dev/robot"}'     # odd spacing: echoed as-is
FRAGMENT = '{"command": "rm -rf'


def check(name, ok, detail=""):
    print(f"  {'ok  ' if ok else 'FAIL'} {name}" +
          (f"  ({detail})" if detail else ""))
    if not ok:
        FAILS.append(name)


def last_text(body):
    m = body["messages"][-1]
    c = m.get("content")
    return c if isinstance(c, str) else json.dumps(c)


def call(cid, args):
    return {"id": cid, "type": "function",
            "function": {"name": "bash", "arguments": args}}


class Stub(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, code, obj, headers=()):
        data = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        for k, v in headers:
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def _sse(self, chunks):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.end_headers()
        for ch in chunks:
            ch.setdefault("id", "stub")
            ch.setdefault("object", "chat.completion.chunk")
            ch.setdefault("created", 0)
            ch.setdefault("model", "stub")
            self.wfile.write(f"data: {json.dumps(ch)}\n\n".encode())
        self.wfile.write(b"data: [DONE]\n\n")

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        body = json.loads(self.rfile.read(n) or b"{}")
        REQUESTS.append((self.path, body))
        key = last_text(body)
        turn = len(REQUESTS)
        usage = {"prompt_tokens": 100 * turn, "completion_tokens": 20}

        if not body.get("stream"):
            if "[sensitive]" in key:      # Z.ai moderation finish_reason
                return self._json(200, {
                    "id": "stub-s", "object": "chat.completion",
                    "created": 0, "model": body.get("model"),
                    "choices": [{"index": 0, "finish_reason": "sensitive",
                                 "message": {"role": "assistant",
                                             "content": ""}}],
                    "usage": usage})
            if key == "Begin.":
                msg = {"role": "assistant", "content": "Let me look around.",
                       "reasoning_content": TRACE,
                       "tool_calls": [call("call_1", json.dumps(
                           {"command": "ls /dev/robot"}))]}
                finish = "tool_calls"
            else:
                msg = {"role": "assistant", "content": "Done for now."}
                finish = "stop"
                usage["prompt_tokens_details"] = {"cached_tokens": 150}
            return self._json(200, {
                "id": f"stub-{turn}", "object": "chat.completion",
                "created": 0, "model": body.get("model"),
                "choices": [{"index": 0, "message": msg,
                             "finish_reason": finish}],
                "usage": usage})

        # Streamed (kimi) scenarios, keyed on the last message.
        err = {"message": "stub", "type": None, "param": None, "code": None}
        if "[cf]" in key:
            err.update(message="The request was rejected because it was "
                       "considered high risk", type="content_filter")
            return self._json(400, {"error": err})
        if "[400]" in key:
            err.update(type="invalid_request_error")
            return self._json(400, {"error": err})
        if "[quota]" in key:
            err.update(type="exceeded_current_quota_error")
            return self._json(429, {"error": err})
        if "[overload-forever]" in key or (
                "[overload]" in key and "[overload]" not in SEEN):
            SEEN.add("[overload]")
            err.update(type="engine_overloaded_error")
            return self._json(429, {"error": err}, [("Retry-After", "1")])

        def delta(d, finish=None, **extra):
            ch = {"choices": [dict(index=0, delta=d, finish_reason=finish,
                                   **extra)]}
            return ch

        if "[k1]" in key:
            chunks = [delta({"role": "assistant", "reasoning_content": "br"}),
                      delta({"reasoning_content": "ief"}),
                      delta({"content": "o"}), delta({"content": "k"}),
                      delta({"tool_calls": [{"index": 0, "id": "call_k1",
                                             "type": "function",
                                             "function": {"name": "bash",
                                                          "arguments":
                                                          RAW_ARGS[:9]}}]}),
                      delta({"tool_calls": [{"index": 0, "function": {
                          "arguments": RAW_ARGS[9:]}}]}),
                      delta({}, "tool_calls"),
                      {"choices": [], "usage": dict(
                          usage, cached_tokens=40,
                          prompt_tokens_details={"cached_tokens": 40})}]
        elif "[k-empty]" in key:
            chunks = [delta({"role": "assistant", "reasoning_content": ""}),
                      delta({"tool_calls": [{"index": 0, "id": "call_k2",
                                             "type": "function",
                                             "function": {
                                                 "name": "bash",
                                                 "arguments":
                                                 '{"command":"pwd"}'}}]}),
                      delta({}, "tool_calls"),
                      {"choices": [], "usage": usage}]
        elif "[len]" in key or "[len-ok]" in key:
            fin = "length" if "[len]" in key else "tool_calls"
            chunks = [delta({"role": "assistant", "reasoning_content": "x"}),
                      delta({"tool_calls": [{"index": 0, "id": "call_k3",
                                             "type": "function",
                                             "function": {"name": "bash",
                                                          "arguments":
                                                          FRAGMENT}}]}),
                      delta({}, fin),
                      {"choices": [], "usage": usage}]
        elif "[usage-in-choice]" in key:
            chunks = [delta({"role": "assistant", "content": "fine"}),
                      delta({}, "stop", usage=dict(usage, cached_tokens=7))]
        else:
            chunks = [delta({"role": "assistant", "reasoning_content": "t"}),
                      delta({"content": "ok"}), delta({}, "stop"),
                      {"choices": [], "usage": usage}]
        return self._sse(chunks)


def main():
    srv = HTTPServer(("127.0.0.1", PORT), Stub)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{PORT}/v1"
    os.environ["LLM_BASE_URL"] = base
    os.environ["LLM_API_KEY"] = "stub"
    # PROVIDERS reads LLM_BASE_URL at import time.
    from harness import llm
    llm.PROVIDERS["compat"]["base_url"] = base

    print("== compat: plain JSON ==")
    model = llm.make_model("compat:stub-model", ".")
    check("compat provider selected",
          isinstance(model, llm.OpenAICompatModel))
    check("no effort suffix, no provider default -> none",
          model.reasoning_effort is None)
    check("compat does not stream", not model.stream)

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
    check("compat sends max_tokens, no stream",
          "max_tokens" in body and "max_completion_tokens" not in body
          and "stream" not in body)
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
    check("assistant text kept alongside tool_calls",
          asst.get("content") == "Let me look around.")
    tool = body2["messages"][3]
    check("tool result -> role=tool with matching id",
          tool == {"role": "tool", "tool_call_id": "call_1",
                   "content": "d0 d1 d2"})
    check("final text -> end_turn",
          r2.stop_reason == "end_turn"
          and r2.content[0].type == "text"
          and r2.content[0].text == "Done for now.")
    check("no trace field -> no thinking block",
          all(b.type != "thinking" for b in r2.content))
    check("cached_tokens (OpenAI-style details) recorded, not "
          "double-counted",
          r2.usage.cached_input_tokens == 150
          and llm.context_tokens(r2.usage) == 220)
    check("serialize_content is JSON-safe",
          json.dumps(model.serialize_content(r2.content)) is not None)

    print("== history translation (no server) ==")
    bare = [{"role": "user", "content": "go"},
            {"role": "assistant", "content": [
                {"type": "tool_use", "id": "t1", "name": "bash",
                 "input": {"command": "ls"}}]}]
    a_plain = llm.to_openai_messages("s", bare)[2]
    a_pad = llm.to_openai_messages("s", bare, pad_reasoning=True)[2]
    check("no stored trace, no padding -> key absent",
          "reasoning_content" not in a_plain)
    check("no stored trace, padding -> reasoning_content ''",
          a_pad.get("reasoning_content") == "")
    check("tool-call turn without text omits content key",
          "content" not in a_pad and "content" not in a_plain)
    signed = [{"role": "user", "content": "go"},
              {"role": "assistant", "content": [
                  {"type": "thinking", "thinking": "claude's trace",
                   "signature": "sig"},
                  {"type": "text", "text": "hi"}]}]
    s_plain = llm.to_openai_messages("s", signed)[2]
    s_pad = llm.to_openai_messages("s", signed, pad_reasoning=True)[2]
    check("signed Anthropic thinking never replayed as a trace",
          "reasoning_content" not in s_plain
          and "reasoning_content" not in s_pad
          and s_plain["content"] == "hi")
    raw_hist = [{"role": "user", "content": "go"},
                {"role": "assistant", "content": [
                    {"type": "tool_use", "id": "t1", "name": "bash",
                     "input": {"command": "ls   /dev/robot"},
                     "raw": RAW_ARGS}]}]
    check("stored raw arguments echoed byte-for-byte",
          llm.to_openai_messages("s", raw_hist)[2]["tool_calls"][0]
          ["function"]["arguments"] == RAW_ARGS)

    print("== kimi: streamed SSE, Moonshot request shape ==")
    os.environ["MOONSHOT_API_KEY"] = "stub"
    real_base = llm.PROVIDERS["kimi"]["base_url"]
    llm.PROVIDERS["kimi"]["base_url"] = base
    kimi = llm.make_model("kimi:kimi-k3@low", ".")
    check("kimi provider selected with effort suffix",
          isinstance(kimi, llm.OpenAICompatModel)
          and kimi.model == "kimi-k3" and kimi.reasoning_effort == "low")
    check("kimi pads reasoning and streams",
          kimi.pad_reasoning and kimi.stream)
    n0 = len(REQUESTS)
    hist = [{"role": "user", "content": "[k1] Begin."}]
    r3 = kimi.create(system, hist, max_tokens=65536)
    _, body3 = REQUESTS[n0]
    check("kimi sends model id without the suffix",
          body3["model"] == "kimi-k3")
    check("kimi sends max_completion_tokens (per-turn cap), never "
          "max_tokens",
          body3.get("max_completion_tokens") == 65536
          and "max_tokens" not in body3)
    check("kimi sends reasoning_effort", body3.get("reasoning_effort") == "low")
    check("kimi streams with usage",
          body3.get("stream") is True
          and body3.get("stream_options") == {"include_usage": True})
    check("kimi omits fixed sampling params and Anthropic thinking",
          not {"temperature", "top_p", "n", "presence_penalty",
               "frequency_penalty", "thinking"} & set(body3))
    check("streamed trace folded (first block)",
          r3.content[0].type == "thinking"
          and r3.content[0].thinking == "brief")
    check("streamed text folded",
          r3.content[1].type == "text" and r3.content[1].text == "ok")
    tu = [b for b in r3.content if b.type == "tool_use"]
    check("streamed tool call folded across chunks",
          len(tu) == 1 and tu[0].id == "call_k1"
          and tu[0].input == {"command": "ls   /dev/robot"}
          and tu[0].raw == RAW_ARGS and r3.stop_reason == "tool_use")
    check("top-level + details cached_tokens counted once",
          r3.usage.cached_input_tokens == 40
          and llm.context_tokens(r3.usage)
          == r3.usage.input_tokens + r3.usage.output_tokens)
    hist.append({"role": "assistant",
                 "content": kimi.serialize_content(r3.content)})
    hist.append({"role": "user", "content": [
        {"type": "tool_result", "tool_use_id": "call_k1",
         "content": "d0 d1"}]})
    kimi.create(system, hist)
    asst3 = REQUESTS[n0 + 1][1]["messages"][2]
    check("kimi echo: reasoning_content verbatim, raw arguments, text",
          asst3.get("reasoning_content") == "brief"
          and asst3["tool_calls"][0]["function"]["arguments"] == RAW_ARGS
          and asst3.get("content") == "ok")

    n0 = len(REQUESTS)
    hist = [{"role": "user", "content": "[k-empty] Begin."}]
    r4 = kimi.create(system, hist)
    check("empty trace -> thinking '' block, no text block",
          r4.content[0].type == "thinking" and r4.content[0].thinking == ""
          and all(b.type != "text" for b in r4.content))
    ser = kimi.serialize_content(r4.content)
    check("empty trace serializes without a signature",
          ser[0] == {"type": "thinking", "thinking": ""})
    hist.append({"role": "assistant", "content": ser})
    hist.append({"role": "user", "content": [
        {"type": "tool_result", "tool_use_id": "call_k2",
         "content": "/bot"}]})
    kimi.create(system, hist)
    asst4 = REQUESTS[n0 + 1][1]["messages"][2]
    check("empty trace echoed as reasoning_content '' with no content key",
          "reasoning_content" in asst4 and asst4["reasoning_content"] == ""
          and "content" not in asst4 and asst4["tool_calls"][0]["id"]
          == "call_k2")

    print("== kimi: effort selection ==")
    dflt = llm.make_model("kimi:kimi-k3", ".")
    check("kimi without suffix -> explicit default effort max",
          dflt.reasoning_effort == "max")
    n0 = len(REQUESTS)
    dflt.create(system, [{"role": "user", "content": "hi"}])
    check("default effort is sent on the wire",
          REQUESTS[n0][1].get("reasoning_effort") == "max")
    try:
        llm.make_model("kimi:kimi-k3@medium", ".")
        check("kimi rejects @medium at make_model time", False)
    except ValueError as e:
        check("kimi rejects @medium at make_model time",
              "low|high|max" in str(e))
    os.environ["OPENAI_API_KEY"] = "stub"
    oa = llm.make_model("openai:gpt-x@medium", ".")
    check("openai accepts @medium",
          oa.reasoning_effort == "medium"
          and oa.tokens_param == "max_completion_tokens")
    os.environ.pop("OPENAI_API_KEY")

    print("== kimi: errors ==")
    import openai
    n0 = len(REQUESTS)
    rcf = kimi.create(system, [{"role": "user", "content": "[cf] x"}])
    check("400 content_filter -> refusal, no retry inside create",
          rcf.stop_reason == "refusal" and rcf.content == []
          and len(REQUESTS) == n0 + 1)
    try:
        kimi.create(system, [{"role": "user", "content": "[400] x"}])
        check("other 400s still raise", False)
    except openai.BadRequestError:
        check("other 400s still raise", True)
    n0 = len(REQUESTS)
    try:
        kimi.create(system, [{"role": "user", "content": "[quota] x"}])
        check("429 exceeded_current_quota_error raises at once", False)
    except openai.RateLimitError:
        check("429 exceeded_current_quota_error raises at once",
              len(REQUESTS) == n0 + 1)
    n0 = len(REQUESTS)
    t0 = time.time()
    r_ov = kimi.create(system, [{"role": "user", "content": "[overload] x"}])
    dt = time.time() - t0
    check("429 engine_overloaded_error retried after Retry-After",
          r_ov.stop_reason == "end_turn" and len(REQUESTS) == n0 + 2
          and 0.9 <= dt < 3.0, f"dt={dt:.2f}s")
    kimi.RETRY_WINDOW_S = 2
    n0 = len(REQUESTS)
    t0 = time.time()
    try:
        kimi.create(system, [{"role": "user",
                              "content": "[overload-forever] x"}])
        check("429 retries bounded by RETRY_WINDOW_S", False)
    except openai.RateLimitError:
        dt = time.time() - t0
        check("429 retries bounded by RETRY_WINDOW_S",
              2.0 <= dt < 6.0 and len(REQUESTS) - n0 >= 2, f"dt={dt:.2f}s")
    kimi.RETRY_WINDOW_S = llm.OpenAICompatModel.RETRY_WINDOW_S

    print("== kimi: truncation and usage placement ==")
    r_len = kimi.create(system, [{"role": "user", "content": "[len] x"}])
    check("truncated tool call (finish=length) is never executed",
          r_len.stop_reason == "max_tokens"
          and all(b.type != "tool_use" for b in r_len.content))
    r_ok = kimi.create(system, [{"role": "user", "content": "[len-ok] x"}])
    tu = [b for b in r_ok.content if b.type == "tool_use"]
    check("unparseable-but-complete arguments kept as the command",
          len(tu) == 1 and tu[0].input == {"command": FRAGMENT})
    r_u = kimi.create(system, [{"role": "user",
                                "content": "[usage-in-choice] x"}])
    check("usage inside choices[0] is picked up",
          r_u.usage.cached_input_tokens == 7 and r_u.usage.output_tokens == 20
          and r_u.content[0].text == "fine")
    check("client timeout is long and SDK retries are off",
          kimi.client.timeout == llm.OpenAICompatModel.TIMEOUT_S
          and kimi.client.max_retries == 0)

    print("== model_spec ==")
    spec = llm.model_spec(kimi)
    check("model_spec describes provider/model/effort",
          spec == dict(provider="kimi", model_id="kimi-k3",
                       reasoning_effort="low",
                       tokens_param="max_completion_tokens", stream=True))
    check("model_spec carries no endpoint or key",
          "http" not in json.dumps(spec) and "stub" not in json.dumps(spec))
    check("model_spec for mock",
          llm.model_spec(llm.make_model("mock:wall-follower", "."))
          == dict(provider="mock", model_id="wall-follower"))
    llm.PROVIDERS["kimi"]["base_url"] = real_base

    print("== zai: GLM request shape ==")
    os.environ["ZAI_API_KEY"] = "stub"
    real_zai = llm.PROVIDERS["zai"]["base_url"]
    llm.PROVIDERS["zai"]["base_url"] = base
    zai = llm.make_model("zai:glm-5.3-flash@low", ".")
    check("zai provider selected: plain JSON, max_tokens, no padding",
          isinstance(zai, llm.OpenAICompatModel) and not zai.stream
          and zai.tokens_param == "max_tokens" and not zai.pad_reasoning
          and zai.reasoning_effort == "low")
    n0 = len(REQUESTS)
    rz = zai.create(system, [{"role": "user", "content": "Begin."}])
    bz = REQUESTS[n0][1]
    check("zai sends thinking enabled with clear_thinking false",
          bz.get("thinking") == {"type": "enabled",
                                 "clear_thinking": False})
    check("zai sends reasoning_effort and max_tokens",
          bz.get("reasoning_effort") == "low" and "max_tokens" in bz
          and "max_completion_tokens" not in bz and "stream" not in bz)
    check("zai trace captured like any other provider",
          rz.content[0].type == "thinking" and rz.content[0].thinking == TRACE)
    check("zai default effort is max",
          llm.make_model("zai:glm-5.3-flash", ".").reasoning_effort == "max")
    check("zai accepts @xhigh",
          llm.make_model("zai:glm-5.3-flash@xhigh", ".").reasoning_effort
          == "xhigh")
    try:
        llm.make_model("zai:glm-5.3-flash@none", ".")
        check("zai rejects @none (thinking cannot be disabled)", False)
    except ValueError:
        check("zai rejects @none (thinking cannot be disabled)", True)
    rs = zai.create(system, [{"role": "user", "content": "[sensitive] x"}])
    check("finish_reason sensitive -> refusal", rs.stop_reason == "refusal")
    check("model_spec records the thinking config",
          llm.model_spec(zai)["extra_body"]["thinking"]["clear_thinking"]
          is False)
    llm.PROVIDERS["zai"]["base_url"] = real_zai
    os.environ.pop("ZAI_API_KEY")

    print("== gemini: effort mapping ==")
    os.environ["GEMINI_API_KEY"] = "stub"
    gm = llm.make_model("gemini:gemini-3.8-flash@medium", ".")
    check("gemini accepts @medium and asks for thought summaries",
          gm.reasoning_effort == "medium" and gm.tokens_param == "max_tokens"
          and gm.extra_body == {"google": {"thinking_config": {
              "include_thoughts": True}}})
    check("gemini default effort is high (3.x default, made explicit)",
          llm.make_model("gemini:gemini-3.8-flash", ".").reasoning_effort
          == "high")
    try:
        llm.make_model("gemini:gemini-3.8-flash@max", ".")
        check("gemini rejects @max (no such thinking level)", False)
    except ValueError as e:
        check("gemini rejects @max (no such thinking level)",
              "minimal|low|medium|high" in str(e))
    os.environ.pop("GEMINI_API_KEY")

    print("== provider table ==")
    check("kimi/zai/deepseek/gemini providers registered",
          {"kimi", "zai", "deepseek", "gemini"} <= set(llm.PROVIDERS))
    check("deepseek pads reasoning (400 without it when tools present)",
          llm.PROVIDERS["deepseek"]["pad_reasoning"] is True
          and llm.PROVIDERS["deepseek"]["default_effort"] == "high")
    for name in ("kimi", "zai", "deepseek", "gemini"):
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
