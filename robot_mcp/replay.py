"""A recorded sim: serves the Blink protocol (SIM_PROTOCOL.md) from one recorded event, no physics.

    python3 -m robot_mcp.replay EVENT.json [--port 47811]

EVENT.json is a sim's `event` message (a live run's blink_log.jsonl broadcast line), its `state` the site_get_state
reply at the event. A client's `{"op": "fire"}` sends that event again to every client, stamped with t_wall_ns now and
the recorded t_sim, and restarts the time mapping. Then, as the sim would at each request's mapped time t_target:
site_get_state answers the recorded state with t_sim = t_target and every time_to_contact_s counted down (nothing
moves: no action changes it); site_estop, site_alert, robot_dispatch and fleet_dispatch answer applied_at_t_sim = t_target;
incident_report answers logged_at_t_sim; site_camera answers <name>.jpg beside EVENT.json, if any. Every message in
and out is a line of ./blink_log.jsonl.
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

    def __init__(self, event, cameras, log):
        self.event, self.cameras, self.log = event, cameras, log
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
        now = round(t, 4)
        if op == "site_get_state":
            state = copy.deepcopy(ev["state"])
            state["t_sim"] = now
            for who in [*state["people"], *state["robots"]]:
                if who.get("time_to_contact_s") is not None:
                    who["time_to_contact_s"] = round(max(0.0, who["time_to_contact_s"] - (t - ev["t_sim"])), 3)
            return {**state, "late_s": 0.0}
        if op == "site_estop":
            return {"vehicle": req.get("vehicle"), "applied_at_t_sim": now, "late_s": 0.0}
        if op == "site_alert":
            return {"person": req.get("person"), "applied_at_t_sim": now, "late_s": 0.0}
        if op == "robot_dispatch":
            return {"robot": req.get("robot"), "applied_at_t_sim": now, "late_s": 0.0}
        if op == "fleet_dispatch":
            robots = sorted(r.get("robot") for r in req.get("routes", ()))
            return {"applied_at_t_sim": now, "dispatched": robots, "late_s": 0.0}
        if op == "incident_report":
            return {"logged_at_t_sim": now, "late_s": 0.0}
        if op == "site_camera":
            jpeg = self.cameras.get(req.get("name"))
            if jpeg is None:
                return {"error": f"unknown camera {req.get('name')!r}"}
            return {"t_sim": now, "camera": req["name"], "jpeg_b64": base64.b64encode(jpeg).decode(), "late_s": 0.0}
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
    cameras = {name: path.with_name(f"{name}.jpg") for name in ("chase", "front")}
    with Path("blink_log.jsonl").open("a", buffering=1, encoding="utf-8") as log:
        replay = Replay(
            json.loads(path.read_text()), {k: p.read_bytes() for k, p in cameras.items() if p.exists()}, log
        )
        sock = socket.create_server(("127.0.0.1", args.port))
        print(f"replay: {path} on 127.0.0.1:{args.port}", flush=True)
        while True:
            conn, _ = sock.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            threading.Thread(target=replay.serve, args=(conn,), daemon=True).start()


if __name__ == "__main__":
    main()
