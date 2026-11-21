"""The laptop driver of Blink: hands the sim's hazard event to the OpenClaw agent `blink` and times every step.

    python3 scripts/claw/drive.py gate --brain kimi --effort max --event demo/events/worker.json --repeat 5
    python3 scripts/claw/drive.py live --brain gpu --effort on --session <sim-runner run name>

Brains (BRAINS): `kimi` (Kimi K3.1 Pro on Cerebras), `qwen` (Qwen 3.8 27B on Cerebras, the stand-in before Kimi
was reachable) and `gpu` (a self-hosted vLLM on gpu-host, a Qwen3.8-27B fine-tune, through an ssh forward of its port
11434). --effort is the brain's reasoning setting, set by the tap on every model request: Cerebras'
`reasoning_effort` (Kimi: low, medium, high, max), vLLM's `chat_template_kwargs.enable_thinking` (on, off).

The agent reaches its model through a tap this script runs on 127.0.0.1:18790 (blinkcerebras and blinkvllm in
OpenClaw's models.providers point at it; scripts/claw/openclaw.sh setup writes them). The tap forwards each request
with the effort set and stream usage on, streams the answer back as it arrives, and records every streamed piece
(reasoning, text, tool call name and arguments) with its wall time, and the call's usage. Authorization passes
through and is never recorded. A request the provider refuses (HTTP 429) the tap sends again itself, after each of
RETRY_DELAYS (8 resends, at most 3 s apart); OpenClaw's own backoff doubles from 1 s to 30 s.

The sim side is one protocol connection (robot_mcp.sim relay through scripts/claw/box_cmd.sh): the event and the
outcome arrive on it. `gate` runs robot_mcp.replay on gpu-host with --event (a recorded sim event) and fires it
--repeat times, each into a fresh OpenClaw session; `live` waits for a live session's own event and hands it to the
session its warm-up turn opened (OpenClaw starts the MCP server once per session). Each event goes to the agent as
one prompt (EVENT_PROMPT) with EVENT_CAMERA's frame, asked for on the relay as the event arrives, attached as an
image: the agent sees the site in the same model call that acts. Before the first event, one warm-up turn (WARMUP,
one model call) starts the MCP server and fills the provider's prompt cache, as a robot's brain is up before its
hazard.

The MCP server's log (robot_mcp/server.py, gpu-host's clock) gives each tool call's request t_wall_ns and the sim's
reply (applied_at_t_sim, late_s); L is the first action's (ACTIONS) request t_wall_ns minus the event's, both read on
gpu-host's clock. Tap times are this laptop's, shifted onto gpu-host's by an offset measured over one ssh connection
(min-RTT sample).

Writes demo/results/<run>.json: compose_split.sh's keys (run, model, reaction_ms, tokens_per_s, outcome), the event,
the outcome, every tool call, every model call with its token stream, the event frame (kept byte for byte as
camera/<event t_wall_ns>-<camera>-event.jpg beside the MCP log) and the agent's final text. A refused model call stays
in the list with its status and error; an episode's retry_s sums the time from each refused request to the next
request, so L splits into retries, queue and decode.
"""

import argparse
import contextlib
import http.client
import json
import os
import queue
import shlex
import socket
import statistics
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import ClassVar
from urllib.parse import urlsplit

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
HOST = os.environ.get("BLINK_SIM_HOST", "gpu-host")
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", HOST]
TAP_PORT = 18790
VLLM_PORT = 18791
REPLAY_PORT = 47811
GATEWAY = os.environ.get("OPENCLAW_GATEWAY_URL", "http://127.0.0.1:18789")
UPSTREAM = {"cerebras": "https://api.cerebras.ai/v1", "vllm": f"http://127.0.0.1:{VLLM_PORT}/v1"}
BRAINS = {
    "kimi": {
        "provider": "blinkcerebras",
        "model": "moonshotai-kimi-k3.1-pro-trial",
        "label": "Kimi K3.1 Pro on Cerebras",
    },
    "qwen": {"provider": "blinkcerebras", "model": "qwen-3.8-27b", "label": "Qwen 3.8 27B on Cerebras"},
    "gpu": {"provider": "blinkvllm", "model": "qwen-27b", "label": "Qwen3.8-27B fine-tune on GPU (vLLM)"},
}
ACTIONS = {"site_estop", "site_alert", "robot_dispatch"}
"""The tool calls that act on the hazard: the first one's request ends L."""
EVENT_PROMPT = "EVENT {}"
EVENT_CAMERA = "chase"
"""The camera whose frame, taken when the event arrives, rides in the event prompt (the CCTV up the aisle)."""
WARMUP = "Warm-up, no hazard: answer only OK."
RETRY_DELAYS = (0.5, 1.0, 2.0, 3.0, 3.0, 3.0, 3.0, 3.0)
"""The tap's waits (s) before each resend of a refused (HTTP 429) model request: 8 resends, at most 3 s apart.
OpenClaw's own backoff (1 s doubling to 30 s) sees a refusal only once these are spent."""


def now():
    """Wall seconds."""
    return time.time()


def brain_effort(req, effort):
    """Set the run's reasoning setting on one model request."""
    if effort in {"on", "off"}:
        req.pop("reasoning_effort", None)
        req["chat_template_kwargs"] = {**req.get("chat_template_kwargs", {}), "enable_thinking": effort == "on"}
    elif effort:
        req["reasoning_effort"] = effort


class Tap(BaseHTTPRequestHandler):
    """POST /<upstream>/... forwarded to UPSTREAM[upstream], streamed back, recorded into Tap.calls."""

    calls: ClassVar[list] = []
    effort: ClassVar[str | None] = None
    lock = threading.Lock()
    protocol_version = "HTTP/1.1"

    def log_message(self, format, *args):  # noqa: A002 - BaseHTTPRequestHandler's signature
        """Quiet."""

    def do_GET(self):
        """Pass through (model listings)."""
        self.forward(b"", None)

    def do_POST(self):
        """Forward one request and record its answer."""
        t_req = now()
        self.forward(self.rfile.read(int(self.headers.get("Content-Length", 0))), t_req)

    def forward(self, body, t_req):
        """Send body upstream with the effort set, stream the answer back, record a chat request.

        A refused chat request (HTTP 429) is sent again after each of RETRY_DELAYS, each refusal recorded as its own
        call; only the last answer reaches OpenClaw."""
        name, _, rest = self.path.lstrip("/").partition("/")
        if name not in UPSTREAM:
            self.send_error(404, f"no upstream {name}")
            return
        req = json.loads(body) if body else None
        seen = sent_effort = None
        if req is not None:
            seen = {k: req.get(k) for k in ("reasoning_effort", "chat_template_kwargs", "reasoning") if k in req}
            brain_effort(req, Tap.effort)
            sent_effort = {k: req.get(k) for k in ("reasoning_effort", "chat_template_kwargs") if k in req}
            if req.get("stream"):
                req.setdefault("stream_options", {})["include_usage"] = True
            body = json.dumps(req).encode()
        up = urlsplit(UPSTREAM[name])
        conn_cls = http.client.HTTPSConnection if up.scheme == "https" else http.client.HTTPConnection
        skip = {"host", "content-length", "accept-encoding", "connection"}
        headers = {k: v for k, v in self.headers.items() if k.lower() not in skip}
        headers["Accept-Encoding"] = "identity"

        def send():
            conn = conn_cls(up.netloc, timeout=300)
            conn.request(self.command, f"{up.path}/{rest}".replace("//", "/"), body=body or None, headers=headers)
            return conn, conn.getresponse()

        conn, resp = send()
        for delay in RETRY_DELAYS if req is not None else ():
            if resp.status != 429:
                break
            rec = record(name, req, len(body), t_req, resp.status)
            scan(rec, resp.read(), now())
            conn.close()
            rec["t_end"] = now()
            with Tap.lock:
                Tap.calls.append(rec)
            time.sleep(delay)
            t_req = now()
            conn, resp = send()
        self.send_response(resp.status)
        for k, v in resp.getheaders():
            if k.lower() not in {"transfer-encoding", "content-length", "connection", "content-encoding"}:
                self.send_header(k, v)
        self.send_header("Connection", "close")
        self.end_headers()
        rec = None
        if req is not None:
            rec = record(name, req, len(body), t_req, resp.status)
            rec["effort_from_openclaw"], rec["effort_sent"] = seen, sent_effort
        while chunk := resp.readline():
            self.wfile.write(chunk)
            self.wfile.flush()
            if rec is not None:
                scan(rec, chunk, now())
        conn.close()
        if rec is not None:
            rec["t_end"] = now()
            with Tap.lock:
                Tap.calls.append(rec)
        self.close_connection = True


def record(upstream, req, size, t_req, status):
    """A model call's record before its answer."""
    msgs = req.get("messages", [])
    return {
        "upstream": upstream,
        "model": req.get("model"),
        "status": status,
        "t_req": t_req,
        "t_first_token": None,
        "t_tool_call": None,
        "request_bytes": size,
        "n_messages": len(msgs),
        "tools": [t.get("function", {}).get("name") for t in req.get("tools", [])],
        "tools_chars": len(json.dumps(req.get("tools", []))),
        "system_chars": sum(
            len(m["content"]) for m in msgs if m.get("role") == "system" and isinstance(m.get("content"), str)
        ),
        "last_user": without_images(
            next((m.get("content") for m in reversed(msgs) if m.get("role") in {"user", "tool"}), None)
        ),
        "stream": [],
        "content": "",
        "reasoning": "",
        "tool_calls": [],
        "usage": None,
        "time_info": None,
        "error": None if status == 200 else "",
    }


def without_images(content):
    """A message's content with each image part's data replaced by its mime type and size."""
    if not isinstance(content, list):
        return content
    out = []
    for part in content:
        url = (part.get("image_url") or {}).get("url", "") if isinstance(part, dict) else ""
        head, _, data = url.partition(",")
        out.append({"type": "image_url", "image": head, "base64_chars": len(data)} if data else part)
    return out


def scan(rec, line, t):
    """Fold one answer line (an SSE data line or a whole JSON body) into rec, each piece with its wall time t."""
    text = line.decode("utf-8", "replace").strip()
    text = text.removeprefix("data:").strip()
    if not text.startswith("{"):
        return
    try:
        msg = json.loads(text)
    except json.JSONDecodeError:
        return
    if rec["error"] is not None:
        rec["error"] += text[:500]
    if msg.get("time_info"):
        rec["time_info"] = msg["time_info"]
    if msg.get("usage"):
        rec["usage"] = msg["usage"]
    for choice in msg.get("choices", []):
        delta = choice.get("delta") or choice.get("message") or {}
        pieces = [
            ("reasoning", delta.get("reasoning") or delta.get("reasoning_content") or ""),
            ("content", delta.get("content") or ""),
        ]
        for tc in delta.get("tool_calls") or []:
            fn = tc.get("function") or {}
            if fn.get("name"):
                rec["tool_calls"].append({"name": fn["name"], "arguments": ""})
                pieces.append(("tool", fn["name"]))
            if fn.get("arguments") and rec["tool_calls"]:
                rec["tool_calls"][-1]["arguments"] += fn["arguments"]
                pieces.append(("args", fn["arguments"]))
        for kind, piece in pieces:
            if not piece:
                continue
            rec["stream"].append([round(t, 4), kind, piece])
            rec["t_first_token"] = rec["t_first_token"] or t
            if kind == "tool":
                rec["t_tool_call"] = rec["t_tool_call"] or t
            elif kind in {"reasoning", "content"}:
                rec[kind] += piece


def port_open(port):
    """True when something listens on 127.0.0.1:port."""
    with socket.socket() as s:
        return s.connect_ex(("127.0.0.1", port)) == 0


def ensure_forward():
    """The ssh forward of gpu-host's vLLM port 11434 to VLLM_PORT; started when not already there."""
    if port_open(VLLM_PORT):
        return
    subprocess.Popen([*SSH, "-o", "ExitOnForwardFailure=yes", "-N", "-L", f"{VLLM_PORT}:127.0.0.1:11434"])
    for _ in range(100):
        if port_open(VLLM_PORT):
            return
        time.sleep(0.1)
    sys.exit(f"drive.py: no ssh forward on {VLLM_PORT}")


def clock_offset():
    """(offset s to add to gpu-host wall times to get this clock's, rtt s), the min-RTT sample of 20 on one ssh."""
    script = "import sys, time\nfor _ in sys.stdin: print(repr(time.time()), flush=True)"
    p = subprocess.Popen(
        [*SSH, f"python3 -uc {shlex.quote(script)}"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True
    )
    assert p.stdin is not None and p.stdout is not None
    best = None
    for _ in range(20):
        t0 = now()
        p.stdin.write("\n")
        p.stdin.flush()
        remote = float(p.stdout.readline())
        t1 = now()
        if best is None or t1 - t0 < best[1]:
            best = ((t0 + t1) / 2 - remote, t1 - t0)
    p.stdin.close()
    p.wait()
    assert best is not None
    return best


def gateway_token():
    """The gateway's bearer token: OPENCLAW_GATEWAY_TOKEN, else ~/.config/blink/gateway.token."""
    token = os.environ.get("OPENCLAW_GATEWAY_TOKEN")
    path = Path.home() / ".config/blink/gateway.token"
    if not token and path.exists():
        token = path.read_text().strip()
    if not token:
        sys.exit("drive.py: no gateway token (scripts/claw/openclaw.sh setup writes ~/.config/blink/gateway.token)")
    return token


def ask(brain, prompt, session=None, jpeg_b64=None):
    """One agent turn over the gateway's chat completions endpoint, streamed: in OpenClaw session `session` (the
    endpoint's `user`; None: a fresh session), with the JPEG attached when given."""
    up = urlsplit(GATEWAY)
    conn = http.client.HTTPConnection(up.netloc, timeout=600)
    content = prompt
    if jpeg_b64 is not None:
        image = {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{jpeg_b64}"}}
        content = [{"type": "text", "text": prompt}, image]
    msg = {"model": "openclaw/blink", "stream": True, "messages": [{"role": "user", "content": content}]}
    if session is not None:
        msg["user"] = session
    body = json.dumps(msg)
    headers = {
        "Authorization": f"Bearer {gateway_token()}",
        "Content-Type": "application/json",
        "x-openclaw-model": f"{BRAINS[brain]['provider']}/{BRAINS[brain]['model']}",
    }
    turn = {"prompt": prompt, "session": session, "t_sent": now(), "t_first_chunk": None, "text": "", "error": None}
    conn.request("POST", "/v1/chat/completions", body=body, headers=headers)
    resp = conn.getresponse()
    if resp.status != 200:
        turn["error"] = f"HTTP {resp.status}: {resp.read().decode()[:500]}"
    while resp.status == 200 and (line := resp.readline()):
        text = line.decode("utf-8", "replace").strip()
        if not text.startswith("data:") or text == "data: [DONE]":
            continue
        turn["t_first_chunk"] = turn["t_first_chunk"] or now()
        msg = json.loads(text[5:])
        if "error" in msg:
            turn["error"] = msg["error"]
        for choice in msg.get("choices", []):
            turn["text"] += (choice.get("delta") or {}).get("content") or ""
    turn["t_done"] = now()
    conn.close()
    return turn


class Relay:
    """robot_mcp.sim relay next to the sim, through box_cmd.sh: every protocol message with its receive time."""

    def __init__(self, target):
        cmd = [str(HERE / "box_cmd.sh"), *target, "--", "python3", "-u", "-m", "robot_mcp.sim", "relay"]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        self.msgs: queue.Queue = queue.Queue()
        self.sent = 0
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        """Queue each message line until the relay ends."""
        assert self.proc.stdout is not None
        for line in self.proc.stdout:
            with contextlib.suppress(json.JSONDecodeError):
                self.msgs.put((now(), json.loads(line)))
        self.msgs.put((now(), None))

    def send(self, req):
        """One request line (the relay stamps t_wall_ns); its protocol id (the relay numbers its requests from 1)."""
        assert self.proc.stdin is not None
        self.proc.stdin.write(json.dumps(req) + "\n")
        self.proc.stdin.flush()
        self.sent += 1
        return self.sent

    def wait(self, timeout, **match):
        """(receive time, message) of the next message whose fields equal match, or (None, None); skips the rest."""
        end = now() + timeout
        while (left := end - now()) > 0:
            try:
                t, msg = self.msgs.get(timeout=left)
            except queue.Empty:
                break
            if msg is None:
                break
            if all(msg.get(k) == v for k, v in match.items()):
                return t, msg
        return None, None

    def close(self):
        """End the relay."""
        with contextlib.suppress(OSError):
            assert self.proc.stdin is not None
            self.proc.stdin.close()
        self.proc.terminate()


def remote_lines(path, since_ns):
    """The JSON lines of a gpu-host file from wall time since_ns on."""
    out = subprocess.run([*SSH, f"cat {path}"], capture_output=True, text=True, check=False).stdout
    rows = [json.loads(line) for line in out.splitlines() if line.strip()]
    return [r for r in rows if r.get("wall_ns", 0) >= since_ns]


def openclaw_target(*target):
    """Point the MCP server robot at target (scripts/claw/openclaw.sh target)."""
    subprocess.run([str(HERE / "openclaw.sh"), "target", *target], check=True, stdout=subprocess.DEVNULL)


def verdict(outcome):
    """SIM_PROTOCOL.md's run verdict from the outcome message: hit or saved."""
    people = outcome["people"].values()
    hit = any(p["hit"] for p in people) if people else outcome["collision"]
    return "hit" if hit else "saved"


def episode(brain, event, t_recv, calls_from, tools, offset):
    """One event's numbers: L from the MCP log, the decision, the first model call's tokens and speed."""
    ev_ns = event["t_wall_ns"]
    mine = [t for t in tools if t["wall_ns"] >= ev_ns]
    acts = [t for t in mine if t["tool"] in ACTIONS]
    first = acts[0] if acts else None
    calls = Tap.calls[calls_from:]
    llm = next((c for c in calls if c["status"] == 200), None)
    refused = [(c, nxt) for c, nxt in zip(calls, [*calls[1:], None], strict=True) if c["status"] != 200]
    usage = (llm or {}).get("usage") or {}
    gen = (llm["t_end"] - llm["t_first_token"]) if llm and llm["t_first_token"] else None
    ev_local = ev_ns / 1e9 + offset
    return {
        "brain": brain,
        "event_t_wall_ns": ev_ns,
        "event_recv_ms": round((t_recv - ev_local) * 1000, 1),
        "l_s": None if first is None else round((first["t_wall_ns"] - ev_ns) / 1e9, 4),
        "decision": None if first is None else {"tool": first["tool"], "args": first["args"]},
        "applied_at_t_sim": None if first is None else first["reply"].get("applied_at_t_sim"),
        "late_s": None if first is None else first["reply"].get("late_s"),
        "tool_calls": [
            {
                **t,
                "t_from_event_s": round((t["t_wall_ns"] - ev_ns) / 1e9, 4) if t.get("t_wall_ns") else None,
                "duration_s": round((t["wall_ns_out"] - t["wall_ns"]) / 1e9, 4),
            }
            for t in mine
        ],
        "steps": [step(c, ev_local) for c in calls],
        "ttft_s": None if not llm or not llm["t_first_token"] else round(llm["t_first_token"] - ev_local, 4),
        "first_tool_token_s": None if not llm or not llm["t_tool_call"] else round(llm["t_tool_call"] - ev_local, 4),
        "prompt_tokens": usage.get("prompt_tokens"),
        "cached_tokens": (usage.get("prompt_tokens_details") or {}).get("cached_tokens"),
        "completion_tokens": usage.get("completion_tokens"),
        "reasoning_tokens": (usage.get("completion_tokens_details") or {}).get("reasoning_tokens"),
        "model_request_s": None if not llm else round(llm["t_req"] - ev_local, 4),
        "queue_time_s": ((llm or {}).get("time_info") or {}).get("queue_time"),
        "prompt_time_s": ((llm or {}).get("time_info") or {}).get("prompt_time"),
        "completion_time_s": ((llm or {}).get("time_info") or {}).get("completion_time"),
        "tokens_per_s": round(usage["completion_tokens"] / gen, 1) if usage.get("completion_tokens") and gen else None,
        "reasoning": (llm or {}).get("reasoning"),
        "said": (llm or {}).get("content"),
        "model_calls": len(calls),
        "refused_calls": len(refused),
        "retry_s": round(sum((nxt or c)["t_req"] - c["t_req"] for c, nxt in refused), 4),
    }


def step(call, ev_local):
    """One model call of the turn: its times from the event (s), Cerebras' time_info, its tokens and decode speed."""
    usage, info = call.get("usage") or {}, call.get("time_info") or {}
    first, tool = call["t_first_token"], call["t_tool_call"]
    decode = call["t_end"] - first if first else None
    tokens = usage.get("completion_tokens")
    return {
        "request_s": round(call["t_req"] - ev_local, 4),
        "first_token_s": None if not first else round(first - ev_local, 4),
        "tool_token_s": None if not tool else round(tool - ev_local, 4),
        "end_s": round(call["t_end"] - ev_local, 4),
        "queue_time_s": info.get("queue_time"),
        "prompt_time_s": info.get("prompt_time"),
        "completion_time_s": info.get("completion_time"),
        "completion_tokens": tokens,
        "reasoning_tokens": (usage.get("completion_tokens_details") or {}).get("reasoning_tokens"),
        "decode_tok_s": round(tokens / decode, 1) if tokens and decode else None,
        "tools": [t["name"] for t in call["tool_calls"]],
        "status": call["status"],
        "error": (call["error"] or "")[:300] or None,
    }


def warm(brain, session):
    """The warm-up turn; its record."""
    turn = ask(brain, WARMUP, session)
    print(
        f"warm-up: {turn['t_done'] - turn['t_sent']:.2f} s, error {turn['error']}, text {turn['text'][:80]!r}",
        flush=True,
    )
    return turn


def event_frame(relay, event, t_recv, offset):
    """EVENT_CAMERA's frame asked for as the event arrives: (its base64 JPEG or None, its record)."""
    rid = relay.send({"op": "site_camera", "name": EVENT_CAMERA})
    t, reply = relay.wait(10.0, id=rid)
    info = {"camera": EVENT_CAMERA, "t_sim": (reply or {}).get("t_sim"), "error": (reply or {}).get("error")}
    info["received_s"] = None if t is None else round(t - (event["t_wall_ns"] / 1e9 + offset), 4)
    info["after_recv_s"] = None if t is None else round(t - t_recv, 4)
    jpeg = (reply or {}).get("jpeg_b64")
    if jpeg is None:
        print(f"drive.py: no {EVENT_CAMERA} frame for the event ({info}); the prompt goes without it", flush=True)
    return jpeg, info


def keep_frame(jpeg, info, event, log_path):
    """Keep the event's frame byte for byte beside the MCP server's (camera/<event t_wall_ns>-<name>-event.jpg next
    to log_path on gpu-host), named in info."""
    path = f"{log_path.rsplit('/', 1)[0]}/camera/{event['t_wall_ns']}-{info['camera']}-event.jpg"
    subprocess.run(
        [*SSH, f"mkdir -p {path.rsplit('/', 1)[0]} && base64 -d > {path}"], input=jpeg, text=True, check=True
    )
    info["jpeg"] = path


def main():
    """Run one mode and write its result file."""
    ap = argparse.ArgumentParser(description=(__doc__ or "").split("\n")[0])
    ap.add_argument("mode", choices=["gate", "live"])
    ap.add_argument("--brain", choices=list(BRAINS), required=True)
    ap.add_argument("--effort", help="reasoning setting (module doc); default: what OpenClaw sends")
    ap.add_argument("--event", help="gate: a recorded sim event (JSON)")
    ap.add_argument("--repeat", type=int, default=5)
    ap.add_argument("--session", help="live: the sim-runner run whose sim the brain drives")
    ap.add_argument("--run", help="result name (live default: the session; gate default gate-<brain>-<effort>-<time>)")
    ap.add_argument("--timeout", type=float, default=600.0, help="live: seconds to wait for the event")
    args = ap.parse_args()
    if args.brain == "gpu":
        ensure_forward()
    Tap.effort = args.effort
    server = ThreadingHTTPServer(("127.0.0.1", TAP_PORT), Tap)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    offset, rtt = clock_offset()
    start_ns = time.time_ns() - int(offset * 1e9) - 1_000_000_000
    replay = None
    if args.mode == "gate":
        files = ["robot_mcp/__init__.py", "robot_mcp/sim.py", "robot_mcp/server.py", "robot_mcp/replay.py"]
        tar = subprocess.Popen(["tar", "-C", str(REPO), "-cf", "-", *files], stdout=subprocess.PIPE)
        subprocess.run([*SSH, "mkdir -p ~/sim2/blink/gate && tar -C ~/sim2/blink -xf -"], stdin=tar.stdout, check=True)
        with Path(args.event).open() as ev_file:
            subprocess.run([*SSH, "cat > ~/sim2/blink/gate/event.json"], stdin=ev_file, check=True)
        # The replay lives as long as this ssh's stdin: closing it at the end kills the replay.
        cmd = f"cd ~/sim2/blink && {{ exec python3 -u -m robot_mcp.replay gate/event.json --port {REPLAY_PORT} & }}; p=$!; cat > /dev/null; kill $p"
        replay = subprocess.Popen([*SSH, f"bash -c {shlex.quote(cmd)}"], stdin=subprocess.PIPE)
        time.sleep(2.0)
        target = ["--replay", str(REPLAY_PORT)]
        openclaw_target(*target)
        relay = Relay(target)
        log_path = "~/sim2/blink/mcp.jsonl"
    else:
        target = [args.session]
        openclaw_target(*target)
        relay = Relay(target)
        log_path = f"~/sim-runs/{args.session}/mcp.jsonl"
    # live: the warm-up and the event share one OpenClaw session, so the event turn finds the MCP server up (OpenClaw
    # starts one per session); gate: every turn in a fresh session, so no event sees an earlier one.
    session = (args.run or args.session) if args.mode == "live" else None
    warmup = warm(args.brain, session)
    episodes, turns, outcome = [], [], None
    n = args.repeat if args.mode == "gate" else 1
    for i in range(n):
        if args.mode == "gate":
            time.sleep(1.0)
            relay.send({"op": "fire"})
        t_recv, event = relay.wait(30.0 if args.mode == "gate" else args.timeout, type="event")
        if event is None:
            sys.exit("drive.py: no event")
        if t_recv < warmup["t_done"]:
            sys.exit("drive.py: the event came before the warm-up turn ended, so L would include the warm-up")
        jpeg, frame = event_frame(relay, event, t_recv, offset)
        calls_from = len(Tap.calls)
        payload = {k: v for k, v in event.items() if k not in {"type", "t_wall_ns"}}
        turn = ask(args.brain, EVENT_PROMPT.format(json.dumps(payload, separators=(",", ":"))), session, jpeg)
        turn["calls_from"], turn["frame"] = calls_from, frame
        if jpeg is not None:
            keep_frame(jpeg, frame, event, log_path)
        turns.append((turn, event, t_recv))
        print(
            f"event {i}: frame {frame}, turn {turn['t_done'] - t_recv:.2f} s, error {turn['error']}, "
            f"text {turn['text'][:160]!r}",
            flush=True,
        )
    if args.mode == "live":
        _, outcome = relay.wait(120.0, type="outcome")
    time.sleep(0.5)
    tools = remote_lines(log_path, start_ns)
    for turn, event, t_recv in turns:
        episodes.append({**episode(args.brain, event, t_recv, turn["calls_from"], tools, offset), "turn": turn})
    relay.close()
    if replay is not None:
        with contextlib.suppress(OSError):
            assert replay.stdin is not None
            replay.stdin.close()
        try:
            replay.wait(timeout=15)
        except subprocess.TimeoutExpired:
            replay.kill()
    server.shutdown()
    ls = [e["l_s"] for e in episodes if e["l_s"] is not None]
    run = args.run or (
        args.session if args.mode == "live" else f"gate-{args.brain}-{args.effort}-{time.strftime('%Y%m%d-%H%M%S')}"
    )
    first = episodes[0] if episodes else {}
    result = {
        "run": run,
        "mode": args.mode,
        "brain": args.brain,
        "model": BRAINS[args.brain]["label"],
        "model_id": BRAINS[args.brain]["model"],
        "effort": args.effort,
        "session": args.session,
        "reaction_ms": round(statistics.median(ls) * 1000) if ls else None,
        "tokens_per_s": first.get("tokens_per_s"),
        "outcome": None if outcome is None else verdict(outcome),
        "outcome_msg": outcome,
        "l_s": ls,
        "l_median_s": round(statistics.median(ls), 4) if ls else None,
        "l_max_s": round(max(ls), 4) if ls else None,
        "clock_offset_s": round(offset, 4),
        "clock_rtt_s": round(rtt, 4),
        "warmup": warmup,
        "episodes": episodes,
        "model_calls": Tap.calls,
    }
    if episodes and args.mode == "live":
        result["t_event"] = episodes[0]["event_t_wall_ns"] / 1e9
        result["event"] = turns[0][1]
    out = REPO / "demo/results" / f"{run}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=1) + "\n")
    for e in episodes:
        print(
            f"L {e['l_s']} s  decision {e['decision']}  ttft {e['ttft_s']}  tokens {e['completion_tokens']} "
            f"(reasoning {e['reasoning_tokens']}, cached prompt {e['cached_tokens']}/{e['prompt_tokens']})  queue {e['queue_time_s']}  "
            f"{e['tokens_per_s']} tok/s  said {(e['said'] or '').strip()[:120]!r}"
        )
        for t in e["tool_calls"]:
            print(
                f"  tool {t['tool']:<16} +{t['t_from_event_s']} s  took {t['duration_s']} s  "
                f"applied_at_t_sim {t['reply'].get('applied_at_t_sim')}  late {t['reply'].get('late_s')}  {t['reply'].get('error') or ''}"
            )
    t0 = Tap.calls[0]["t_req"] if Tap.calls else 0.0
    for c in Tap.calls:
        u = c["usage"] or {}
        print(
            f"  call +{c['t_req'] - t0:7.2f} s  status {c['status']}  first token {((c['t_first_token'] or c['t_end']) - c['t_req']):.2f}  "
            f"tool {((c['t_tool_call'] or c['t_end']) - c['t_req']):.2f}  end {c['t_end'] - c['t_req']:.2f}  {[t['name'] for t in c['tool_calls']]}  "
            f"prompt {u.get('prompt_tokens')} (cached {(u.get('prompt_tokens_details') or {}).get('cached_tokens')}) out {u.get('completion_tokens')}  "
            f"{(c['error'] or '')[:200]}"
        )
    print(f"median L {result['l_median_s']} max {result['l_max_s']}; outcome {result['outcome']}; wrote {out}")
    with contextlib.suppress(Exception):
        server.server_close()


if __name__ == "__main__":
    main()
