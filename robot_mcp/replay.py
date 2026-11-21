"""A recorded sim for the latency gate: serves the Blink protocol (SIM_PROTOCOL.md) from one recorded event, no physics.

    python3 -m robot_mcp.replay EVENT.json [--port 47811]

EVENT.json is a sim's `event` message (a live run's blink_log.jsonl broadcast line), its `state` the get_state reply
at the event. A client's `{"op": "fire"}` sends that event again to every client, stamped with t_wall_ns now and the
recorded t_sim, and restarts the time mapping. Then, as the sim would at each request's mapped time t_target:
get_state answers the recorded state with t_sim = t_target and the hazard's time_to_impact_s counted down;
evade and set_goal answer applied_at_t_sim = t_target (late_s 0: nothing moves); site_estop answers as the scene
does (an error when hazard.site_estop is false); snapshot answers front.jpg beside EVENT.json, if any. Every message
in and out is a line of ./blink_log.jsonl.
"""

import argparse
import base64
import copy
import json
import socket
import threading
import time
from pathlib import Path


class Replay:
    """The recorded event and the clients it goes to."""

    def __init__(self, event, front, log):
        self.event, self.front, self.log = event, front, log
        self.fired: dict | None = None
        self.lock = threading.Lock()
        self.clients = []

    def write(self, **fields):
        """One log line."""
        with self.lock:
            self.log.write(json.dumps({"wall_ns": time.time_ns(), **fields}) + "\n")

    def send(self, conn, msg):
        """One line to one client."""
        try:
            conn.sendall((json.dumps(msg) + "\n").encode())
        except OSError:
            return
        self.write(dir="out", msg={k: v for k, v in msg.items() if k != "jpeg_b64"})

    def fire(self):
        """The event again, now."""
        ev: dict = {**self.event, "t_wall_ns": time.time_ns()}
        with self.lock:
            self.fired = ev
            clients = list(self.clients)
        for c in clients:
            self.send(c, ev)
        return ev

    def answer(self, req):
        """The reply body of one request at its mapped sim time."""
        op, ev = req.get("op"), self.fired
        if op == "fire":
            return {"fired_t_wall_ns": self.fire()["t_wall_ns"]}
        if ev is None:
            return {"error": "replay: no event fired yet"}
        t = ev["t_sim"] + (req.get("t_wall_ns", time.time_ns()) - ev["t_wall_ns"]) / 1e9
        if op == "get_state":
            state = copy.deepcopy(ev["state"])
            state["t_sim"] = round(t, 4)
            tti = state["hazard"].get("time_to_impact_s")
            if tti is not None:
                state["hazard"]["time_to_impact_s"] = round(max(0.0, tti - (t - ev["t_sim"])), 3)
            return {**state, "late_s": 0.0}
        if op in {"evade", "set_goal"}:
            return {"applied_at_t_sim": round(t, 4), "late_s": 0.0}
        if op == "site_estop":
            if not ev["state"]["hazard"].get("site_estop"):
                return {"error": "no site e-stop on this vehicle"}
            return {"error": "replay: site_estop is not modelled"}
        if op == "snapshot":
            if self.front is None:
                return {"error": "no camera"}
            return {"t_sim": round(t, 4), "jpeg_b64": base64.b64encode(self.front).decode(), "late_s": 0.0}
        return {"error": f"unknown op {op!r}"}

    def serve(self, conn):
        """Answer one client's requests until it closes."""
        with self.lock:
            self.clients.append(conn)
        with conn, conn.makefile("r", encoding="utf-8") as lines:
            for line in lines:
                req = json.loads(line)
                self.write(dir="in", msg=req)
                self.send(conn, {"id": req.get("id"), **self.answer(req)})
        with self.lock:
            self.clients.remove(conn)


def main():
    """Serve until killed."""
    ap = argparse.ArgumentParser(description=(__doc__ or "").split("\n")[0])
    ap.add_argument("event", help="a sim's event message (JSON)")
    ap.add_argument("--port", type=int, default=47811)
    args = ap.parse_args()
    path = Path(args.event)
    front = path.with_name("front.jpg")
    with Path("blink_log.jsonl").open("a", buffering=1, encoding="utf-8") as log:
        replay = Replay(json.loads(path.read_text()), front.read_bytes() if front.exists() else None, log)
        sock = socket.create_server(("127.0.0.1", args.port))
        print(f"replay: {path} on 127.0.0.1:{args.port}", flush=True)
        while True:
            conn, _ = sock.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(target=replay.serve, args=(conn,), daemon=True).start()


if __name__ == "__main__":
    main()
