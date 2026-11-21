"""Blink's MCP server: five tools over stdio, each a thin client of the sim protocol (SIM_PROTOCOL.md, robot_mcp/sim.py).

    python -m robot_mcp.server      # on the sim's host, in its network (scripts/claw/box_cmd.sh enters a session's box)

robot_get_state  the sim's get_state reply: pose (world), velocity, the hazard and free space (robot frame)
robot_set_goal   set_goal: walk to a world pose
robot_evade      evade: 1.0 m/s along a direction of the robot frame for duration_ms (default 1500), then hold
robot_snapshot   snapshot: the front camera, 640x360 JPEG
site_estop       site_estop: the site's safety network brakes the hazard vehicle

The same server drives any sim that serves the protocol (the Kit-less runner, the Kit scene) on $BLINK_HOST:$BLINK_PORT.
Every call is one JSON line in $ROBOT_MCP_LOG (default $BOX_RUN/mcp.jsonl, else ./mcp.jsonl): the tool, its
arguments, the wall clock (CLOCK_REALTIME ns) on entry, the request's t_wall_ns, the reply (applied_at_t_sim, late_s)
and the wall clock on exit.
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

    def call(self, tool, args, op, **fields):
        """One tool call: the protocol request, logged; the reply without its id."""
        wall_in = time.time_ns()
        try:
            req, reply = self.request(op, **fields)
        except (ConnectionError, OSError, TimeoutError) as e:
            req, reply = None, {"error": f"sim unreachable: {e}"}
        reply = {k: v for k, v in reply.items() if k != "id"}
        logged = {k: v for k, v in reply.items() if k != "jpeg_b64"}
        record = {"tool": tool, "args": args, "wall_ns": wall_in, "t_wall_ns": req and req["t_wall_ns"]}
        self.log.write(json.dumps({**record, "reply": logged, "wall_ns_out": time.time_ns()}) + "\n")
        return reply


def serve(link):
    """Register the five tools and serve stdio until stdin closes."""
    mcp = MCPServer("robot")

    async def call(tool, args, op, **fields):
        return await asyncio.to_thread(link.call, tool, args, op, **fields)

    @mcp.tool(structured_output=False)
    async def robot_get_state() -> str:
        """The robot's state now. pose: world frame (m, rad). Robot frame (x forward, y left, m) for the rest: vel; hazard {kind, x, y, vx, vy, time_to_impact_s, exit_m: per direction the distance to move to leave its impact zone (0: outside, null: never), site_estop: whether site_estop works}; free_m: per direction the distance to the nearest static obstacle (null: none)."""
        return json.dumps(await call("robot_get_state", {}, "get_state"))

    @mcp.tool(structured_output=False)
    async def robot_evade(direction: Literal["left", "right", "front", "back", "stop"], duration_ms: int = 1500) -> str:
        """Move at once at 1.0 m/s along a direction of the robot frame for duration_ms, then hold; stop holds still. Returns when applied."""
        args = {"direction": direction, "duration_ms": duration_ms}
        return json.dumps(await call("robot_evade", args, "evade", **args))

    @mcp.tool(structured_output=False)
    async def robot_set_goal(x: float, y: float, yaw: float) -> str:
        """Walk to a world-frame pose (m, rad), then hold it."""
        args = {"x": x, "y": y, "yaw": yaw}
        return json.dumps(await call("robot_set_goal", args, "set_goal", **args))

    @mcp.tool(structured_output=False)
    async def robot_snapshot() -> Image | str:
        """The front camera's view now (640x360 JPEG)."""
        reply = await call("robot_snapshot", {}, "snapshot")
        if "jpeg_b64" not in reply:
            return json.dumps(reply)
        return Image(data=base64.b64decode(reply["jpeg_b64"]), format="jpeg")

    @mcp.tool(structured_output=False)
    async def site_estop() -> str:
        """The site's safety network brakes the hazard vehicle now. Returns when it will stop and its gap to the robot then."""
        return json.dumps(await call("site_estop", {}, "site_estop"))

    mcp.run("stdio")


def main():
    """Serve until stdin closes."""
    log = os.environ.get("ROBOT_MCP_LOG") or str(Path(os.environ.get("BOX_RUN", ".")) / "mcp.jsonl")
    serve(Link(log))


if __name__ == "__main__":
    main()
