"""Scripted Blink client: waits for EVENT, asks get_state at L/2, sends evade at L (wall seconds after the event's
t_wall_ns, real timestamps), takes one snapshot, waits for the outcome. Plain python3, no deps.

Usage: blink_client.py --latency 0.3 [--direction auto|left|right|forward|back] [--port 47800] [--out DIR]
'auto' picks the direction from the first get_state reply (free_m[d] > exit_m[d]); when the reply has not arrived
by L, it uses --fallback.
"""

import argparse
import base64
import json
import os
import socket
import time

ap = argparse.ArgumentParser()
ap.add_argument("--latency", type=float, required=True)
ap.add_argument("--direction", default="auto")
ap.add_argument("--fallback", default="left")
ap.add_argument("--duration_ms", type=int, default=3000)
ap.add_argument("--port", type=int, default=int(os.environ.get("BLINK_PORT", "47800")))
ap.add_argument("--out", default=".")
ap.add_argument("--connect_timeout_s", type=float, default=900)
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
LOG = open(os.path.join(a.out, "client_log.jsonl"), "w")

t0 = time.time()
while True:
    try:
        sock = socket.create_connection(("127.0.0.1", a.port))
        break
    except OSError:
        if time.time() - t0 > a.connect_timeout_s:
            raise
        time.sleep(0.5)
sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
rf = sock.makefile("r")


def send(obj):
    obj["t_wall_ns"] = time.time_ns()
    sock.sendall((json.dumps(obj) + "\n").encode())
    LOG.write(json.dumps({"dir": "out", "wall_ns": obj["t_wall_ns"], "msg": obj}) + "\n")
    LOG.flush()


def recv():
    line = rf.readline()
    if not line:
        return None
    m = json.loads(line)
    short = {**m, "jpeg_b64": f"<{len(m['jpeg_b64'])}>"} if "jpeg_b64" in m else m
    LOG.write(json.dumps({"dir": "in", "wall_ns": time.time_ns(), "msg": short}) + "\n")
    LOG.flush()
    return m


def pick(state):
    ok = [
        d
        for d in ("left", "right", "front", "back")
        if state["hazard"]["exit_m"][d] is not None and state["free_m"][d] > state["hazard"]["exit_m"][d]
    ]
    return ok, ({"front": "forward"}.get(ok[0], ok[0]) if len(ok) == 1 else None)


ev = None
while ev is None:
    m = recv()
    if m is None:
        raise SystemExit("sim closed before the event")
    if m.get("type") == "event":
        ev = m
print("CLIENT event", json.dumps(ev), flush=True)
ev_wall = ev["t_wall_ns"] / 1e9
sock.setblocking(False)
buf, state, sent_state, sent_evade, outcome, replies = "", None, False, False, None, {}
while outcome is None:
    now = time.time()
    if not sent_state and now >= ev_wall + a.latency / 2:
        send({"id": 1, "op": "get_state"})
        sent_state = True
    if not sent_evade and now >= ev_wall + a.latency:
        d = a.direction
        if d == "auto":
            d = pick(state)[1] if state else None
            d = d or a.fallback
        send({"id": 2, "op": "evade", "direction": d, "duration_ms": a.duration_ms})
        sent_evade = True
        send({"id": 3, "op": "snapshot"})
    try:
        chunk = sock.recv(1 << 20).decode()
        if not chunk:
            break
        buf += chunk
    except BlockingIOError:
        time.sleep(0.001)
        continue
    while "\n" in buf:
        line, buf = buf.split("\n", 1)
        m = json.loads(line)
        short = {**m, "jpeg_b64": f"<{len(m['jpeg_b64'])}>"} if "jpeg_b64" in m else m
        LOG.write(json.dumps({"dir": "in", "wall_ns": time.time_ns(), "msg": short}) + "\n")
        LOG.flush()
        if m.get("id") == 1:
            state = m
            print("CLIENT state", json.dumps(m), "escape_dirs", pick(m)[0], flush=True)
        elif m.get("id") == 3 and "jpeg_b64" in m:
            open(os.path.join(a.out, "snapshot.jpg"), "wb").write(base64.b64decode(m["jpeg_b64"]))
            print("CLIENT snapshot t_sim", m["t_sim"], flush=True)
        elif m.get("type") == "outcome":
            outcome = m
        elif "id" in m:
            print("CLIENT reply", json.dumps(m), flush=True)
print("CLIENT outcome", json.dumps(outcome), flush=True)
