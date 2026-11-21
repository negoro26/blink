"""Blink's MCP server: six tools over stdio, each a thin client of the sim protocol (SIM_PROTOCOL.md, robot_mcp/sim.py).

    python -m robot_mcp.server      # on the sim's host, in its network (scripts/claw/box_cmd.sh enters a session's box)

Each tool is the protocol op of its name: site_get_state (vehicles, people, robot, each with its predicted contact),
site_camera (one named camera's frame, rendered on request), site_estop (the site's safety network brakes a vehicle),
site_alert (a person's wearable alert), robot_dispatch (a robot walks to a world point), incident_report (the
incident, into the sim's log).

The same server drives any sim that serves the protocol (the Kit-less runner, the Kit scene) on $BLINK_HOST:$BLINK_PORT.
Every call is one JSON line in $ROBOT_MCP_LOG (default $BOX_RUN/mcp.jsonl, else ./mcp.jsonl): the tool, its
arguments, the wall clock (CLOCK_REALTIME ns) on entry, the request's t_wall_ns, the reply (applied_at_t_sim, late_s)
and the wall clock on exit. Each frame site_camera returns is also kept, byte for byte, as camera/<wall ns on exit>-<name>.jpg
beside the log; the log line names it (`jpeg`).
"""

import asyncio
import base64
import json
import os
import threading
import time
from pathlib import Path
from typing import Literal

from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.utilities.types import Image

from .sim import Sim


class Link:
    """The connection to the sim, made on first use and remade once after it drops; the call log."""

    def __init__(self, log_path):
        self.sim = None
        self.lock = threading.Lock()
        self.log = Path(log_path).open("a", encoding="utf-8", buffering=1)  # noqa: SIM115 - for the process's life
        self.frames = Path(log_path).parent / "camera"

    def request(self, op, **fields):
        """(request, reply) of one protocol round trip."""
        for attempt in (0, 1):
            with self.lock:
                if self.sim is None or self.sim.closed:
                    self.sim = Sim()
                sim = self.sim
            try:
                return sim.request(op, **fields)
            except (ConnectionError, OSError):
                with self.lock:
                    self.sim = None
                if attempt:
                    raise
        raise AssertionError

    def call(self, op, **args):
        """One tool call (the protocol op of its name): the request, logged; the reply without its id."""
        wall_in = time.time_ns()
        try:
            req, reply = self.request(op, **args)
        except (ConnectionError, OSError, TimeoutError) as e:
            req, reply = None, {"error": f"sim unreachable: {e}"}
        reply = {k: v for k, v in reply.items() if k != "id"}
        logged = {k: v for k, v in reply.items() if k != "jpeg_b64"}
        record = {"tool": op, "args": args, "wall_ns": wall_in, "t_wall_ns": req and req["t_wall_ns"]}
        wall_out = time.time_ns()
        if "jpeg_b64" in reply:
            self.frames.mkdir(exist_ok=True)
            jpeg = self.frames / f"{wall_out}-{args.get('name', 'camera')}.jpg"
            jpeg.write_bytes(base64.b64decode(reply["jpeg_b64"]))
            record["jpeg"] = str(jpeg)
        self.log.write(json.dumps({**record, "reply": logged, "wall_ns_out": wall_out}) + "\n")
        return reply


def serve(link):
    """Register the six tools and serve stdio until stdin closes."""
    mcp = MCPServer("robot")

    async def call(op, **args):
        return await asyncio.to_thread(link.call, op, **args)

    @mcp.tool(structured_output=False)
    async def site_get_state() -> str:
        """Site now (world frame, m, s): vehicles, people, robot, each with time_to_contact_s (null: none predicted)."""
        return json.dumps(await call("site_get_state"))

    @mcp.tool(structured_output=False)
    async def site_camera(name: Literal["chase", "front"]) -> Image | str:
        """A camera's view now: chase (CCTV up the aisle) or front (the robot's)."""
        reply = await call("site_camera", name=name)
        if "jpeg_b64" not in reply:
            return json.dumps(reply)
        return Image(data=base64.b64decode(reply["jpeg_b64"]), format="jpeg")

    @mcp.tool(structured_output=False)
    async def site_estop(vehicle: str) -> str:
        """Brake a vehicle now (site safety network)."""
        return json.dumps(await call("site_estop", vehicle=vehicle))

    @mcp.tool(structured_output=False)
    async def site_alert(person: str, message: str) -> str:
        """Alert a person now (wearable buzzer and voice); they step clear after their reaction time."""
        return json.dumps(await call("site_alert", person=person, message=message))

    @mcp.tool(structured_output=False)
    async def robot_dispatch(robot: str, x: float, y: float) -> str:
        """Send a robot to walk to world (x, y)."""
        return json.dumps(await call("robot_dispatch", robot=robot, x=x, y=y))

    @mcp.tool(structured_output=False)
    async def incident_report(text: str) -> str:
        """File the incident report (what happened, actions, times, outcome)."""
        return json.dumps(await call("incident_report", text=text))

    mcp.run("stdio")


def main():
    """Serve until stdin closes."""
    log = os.environ.get("ROBOT_MCP_LOG") or str(Path(os.environ.get("BOX_RUN", ".")) / "mcp.jsonl")
    serve(Link(log))


if __name__ == "__main__":
    main()
