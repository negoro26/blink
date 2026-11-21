#!/usr/bin/env bash
set -euo pipefail

usage="usage: robot_mcp/env.sh
  Builds, on gpu-host (BLINK_SIM_HOST), the venv robot_mcp/server.py runs in: ~/sim2/envs/robot-mcp, a uv venv on
  Python 3.12 holding only the mcp SDK (pinned below); robot_mcp/sim.py itself needs only the standard library.
  Idempotent: an existing venv with this mcp version is left as it is. Ends with an import check."
[[ "${1:-}" != -h && "${1:-}" != --help ]] || { echo "$usage"; exit 0; }
MCP_VERSION=2.3.0

ssh -o BatchMode=yes -o ConnectTimeout=10 "${BLINK_SIM_HOST:-gpu-host}" bash -s "$MCP_VERSION" <<'EOF'
set -euo pipefail
E=~/sim2/envs/robot-mcp
if [[ "$("$E/bin/python" -c 'import importlib.metadata as m; print(m.version("mcp"))' 2>/dev/null)" != "$1" ]]; then
    uv venv -q --allow-existing --python 3.12 "$E"
    uv pip install -q --python "$E/bin/python" "mcp==$1"
fi
"$E/bin/python" -c 'import importlib.metadata as m
from mcp.server.mcpserver import MCPServer
print("robot-mcp env: mcp", m.version("mcp"), "python", __import__("sys").version.split()[0])'
EOF
