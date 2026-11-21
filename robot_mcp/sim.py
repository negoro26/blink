"""A client of the Blink sim protocol (SIM_PROTOCOL.md), stdlib only, and its command line.

    python3 -m robot_mcp.sim events                       # print every event and outcome line, then exit
    python3 -m robot_mcp.sim proof '{"at": 1.0, "op": "site_estop", "vehicle": "forklift"}' ...
    python3 -m robot_mcp.sim proof --file calls.jsonl      # one such request per line
    python3 -m robot_mcp.sim call site_get_state          # one request, its reply on stdout

The sim listens on $BLINK_HOST:$BLINK_PORT (default 127.0.0.1:47800), on the sim's host and inside its box: run
this there (scripts/claw/box_cmd.sh enters a live session's box). Every request carries the client's CLOCK_REALTIME
`t_wall_ns` at send time, so the sim maps it onto its own clock (the catch-up rule).

`proof` is a scripted brain: it waits for the event, sends each request (its op and fields) so that its t_wall_ns
is the event's plus its `at` seconds, without waiting for earlier replies, and prints every request with its reply,
then the outcome, as JSON lines. No request is the no-brain run.
"""

import argparse
import json
import os
import socket
import sys
import threading
import time
from operator import itemgetter


class Sim:
    """One connection to the sim: requests matched to replies by id, unsolicited messages to `on_message`."""

    def __init__(self, host=None, port=None, on_message=None, timeout=30.0):
        host = host or os.environ.get("BLINK_HOST", "127.0.0.1")
        port = int(port or os.environ.get("BLINK_PORT", "47800"))
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(None)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.on_message = on_message
        self.lock = threading.Lock()
        self.next_id = 0
        self.waiting: dict[int, dict | None] = {}
        self.event = None
        self.outcome = None
        self.got = threading.Condition()
        self.closed = False
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        """Dispatch every line from the sim until it closes."""
        with self.sock.makefile("r", encoding="utf-8") as lines:
            for line in lines:
                self.take(json.loads(line))
        with self.got:
            self.closed = True
            self.got.notify_all()

    def take(self, msg):
        """Keep one message from the sim: the event, the outcome or a reply; then hand it to on_message."""
        with self.got:
            if msg.get("type") == "event":
                self.event = msg
            elif msg.get("type") == "outcome":
                self.outcome = msg
            elif msg.get("id") in self.waiting:
                self.waiting[msg["id"]] = msg
            self.got.notify_all()
        if self.on_message:
            self.on_message(msg)

    def send(self, op, **fields):
        """Send one request stamped with t_wall_ns now; its id and the request."""
        with self.lock:
            self.next_id += 1
            rid = self.next_id
            with self.got:
                self.waiting[rid] = None
            req = {"id": rid, "op": op, **fields, "t_wall_ns": time.time_ns()}
            self.sock.sendall((json.dumps(req) + "\n").encode())
        return rid, req

    def wait(self, rid, timeout=120.0):
        """The reply to request rid."""
        with self.got:
            if not self.got.wait_for(lambda: self.waiting[rid] is not None or self.closed, timeout):
                raise TimeoutError(f"no reply to request {rid} in {timeout} s")
            reply = self.waiting.pop(rid)
        if reply is None:
            raise ConnectionError("the sim closed the connection")
        return reply

    def request(self, op, **fields):
        """(request, reply) of one round trip."""
        rid, req = self.send(op, **fields)
        return req, self.wait(rid)

    def wait_for(self, attr, timeout=None):
        """Block until the event or outcome has arrived; it (or None at timeout or close)."""
        with self.got:
            self.got.wait_for(lambda: getattr(self, attr) is not None or self.closed, timeout)
            return getattr(self, attr)


def emit(msg):
    """One JSON line on stdout."""
    print(json.dumps(msg), flush=True)


def main():
    """The command line (module doc)."""
    ap = argparse.ArgumentParser(description=(__doc__ or "").split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("events", help="print the event and outcome lines, exit after the outcome")
    sub.add_parser("relay", help="send each stdin line (a request without t_wall_ns) and print every message")
    proof = sub.add_parser("proof", help="send timed requests after the event, print the replies and the outcome")
    proof.add_argument("requests", nargs="*", help='JSON objects {"at": s after the event, "op": ..., fields...}')
    proof.add_argument("--file", help="more such objects, one per line")
    call = sub.add_parser("call", help="one request: op and key=value fields (numbers parsed as JSON)")
    call.add_argument("op")
    call.add_argument("fields", nargs="*")
    args = ap.parse_args()
    if args.cmd == "events":
        sim = Sim(on_message=emit)
        sim.wait_for("outcome")
        return 0
    if args.cmd == "relay":
        sim = Sim(on_message=emit)
        for line in sys.stdin:
            if line.strip():
                req = json.loads(line)
                _, sent = sim.send(req.pop("op"), **req)
                emit({"sent": sent})
        return 0
    if args.cmd == "call":
        fields = {}
        for f in args.fields:
            k, _, v = f.partition("=")
            try:
                fields[k] = json.loads(v)
            except json.JSONDecodeError:
                fields[k] = v
        req, reply = Sim().request(args.op, **fields)
        emit({"request": req, "reply": reply})
        return 0
    sim = Sim()
    event = sim.wait_for("event")
    if event is None:
        sys.exit("sim.py: the sim closed before its event")
    emit({"event": event, "t_recv_ns": time.time_ns()})
    script = [json.loads(r) for r in args.requests]
    if args.file:
        with open(args.file, encoding="utf-8") as f:
            script += [json.loads(line) for line in f if line.strip()]
    sent = []
    for item in sorted(script, key=itemgetter("at")):
        fields = {k: v for k, v in item.items() if k not in {"at", "op"}}
        at_ns = event["t_wall_ns"] + int(item["at"] * 1e9)
        while (left := at_ns - time.time_ns()) > 0:
            time.sleep(min(left / 1e9, 0.05) if left > 2_000_000 else 0)
        sent.append(sim.send(item["op"], **fields))
    for rid, req in sent:
        emit({"request": req, "reply": sim.wait(rid), "l_s": (req["t_wall_ns"] - event["t_wall_ns"]) / 1e9})
    outcome = sim.wait_for("outcome")
    emit({"outcome": outcome})
    return 0


if __name__ == "__main__":
    sys.exit(main())
