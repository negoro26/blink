#!/usr/bin/env bash
set -euo pipefail

usage="usage: openclaw.sh setup | gateway | target <session-name> | target --replay <port>
  The OpenClaw side of Blink, on the laptop (OpenClaw under mise, run from ~/Projects/hackathon).
  setup    writes Blink's part of ~/.openclaw/openclaw.json in one validated config patch, and removes the
           earlier go2 entries: the agent blink (workspace ~/.openclaw/workspace-blink holding only
           robot_mcp/openclaw/AGENTS.md, no skills, thinking high in OpenClaw so the Control UI shows the reasoning
           stream (drive.py's tap still sets each run's reasoning setting on the wire); tools: the minimal profile
           plus the MCP server robot's six site tools), the MCP server
           robot (scripts/claw/box_cmd.sh with no arguments: its target file says which sim), the providers
           blinkcerebras (Cerebras, key from CEREBRAS_API_KEY: Kimi K3.1 Pro and Qwen 3.8 27B) and blinkvllm
           (a self-hosted vLLM on gpu-host), both through scripts/claw/drive.py's timing tap on 127.0.0.1:18790,
           and the gateway's /v1/chat/completions endpoint with a token kept in ~/.config/blink/gateway.token
           (made once, mode 600); and the agent's session settings (its agentDir's settings.json):
           retry.provider.maxRetries 8, maxRetryDelayMs 3000 (a 429 asking for a longer wait ends the turn
           instead of stalling it; drive.py's tap resends refused requests itself first). Idempotent.
  gateway  runs the gateway in the foreground with ~/.config/cerebras.env and that token in its environment.
  target   points the MCP server robot at a live session's box or at robot_mcp.replay's port: writes
           ~/.config/blink/target and disposes the gateway's MCP runtimes, so the next turn starts the server
           against the new target."
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
REPO="$PWD"
CONF="$HOME/.config/blink"
TOKEN_FILE="$CONF/gateway.token"
WORKSPACE="$HOME/.openclaw/workspace-blink"
oc() {  # a running gateway needs its token for config and MCP commands
    (cd "$HOME/Projects/hackathon" && OPENCLAW_GATEWAY_TOKEN="$(cat "$TOKEN_FILE" 2>/dev/null)" mise exec -- openclaw "$@")
}

case "${1:-}" in
    setup)
        mkdir -p "$CONF" "$WORKSPACE"
        [[ -s "$TOKEN_FILE" ]] || (umask 077 && head -c 32 /dev/urandom | od -An -tx1 | tr -d ' \n' > "$TOKEN_FILE")
        cp "$REPO/robot_mcp/openclaw/AGENTS.md" "$WORKSPACE/AGENTS.md"
        mkdir -p "$HOME/.openclaw/agents/blink/agent"
        echo '{"retry": {"provider": {"maxRetries": 8, "maxRetryDelayMs": 3000}}}' > "$HOME/.openclaw/agents/blink/agent/settings.json"
        kimi='{"id": "moonshotai-kimi-k3.1-pro-trial", "name": "Kimi K3.1 Pro (Cerebras)", "input": ["text", "image"], "reasoning": true, "contextWindow": 131072, "maxTokens": 8192}'
        qwen='{"id": "qwen-3.8-27b", "name": "Qwen 3.8 27B (Cerebras)", "input": ["text", "image"], "reasoning": true, "contextWindow": 65536, "maxTokens": 8192}'
        vllm='{"id": "qwen-27b", "name": "Qwen3.8-27B fine-tune (vLLM, GPU)", "input": ["text", "image"], "reasoning": true, "contextWindow": 65536, "maxTokens": 8192,
            "compat": {"thinkingFormat": "qwen-chat-template"}}'
        oc config patch --stdin <<EOF
{
  "gateway": {
    "auth": {"mode": "token", "token": {"source": "env", "provider": "default", "id": "OPENCLAW_GATEWAY_TOKEN"}},
    "http": {"endpoints": {"chatCompletions": {"enabled": true}}}
  },
  "tools": {"toolSearch": {"mode": "tools", "exclude": [{"source": "mcp", "name": "robot__*"}]}},
  "mcp": {"servers": {
    "go2": null,
    "robot": {"command": "$REPO/scripts/claw/box_cmd.sh", "args": [], "connectionTimeoutMs": 60000, "requestTimeoutMs": 120000}
  }},
  "models": {"providers": {
    "go2cerebras": null,
    "go2vllm": null,
    "blinkcerebras": {"baseUrl": "http://127.0.0.1:18790/cerebras", "api": "openai-completions",
      "apiKey": {"source": "env", "provider": "default", "id": "CEREBRAS_API_KEY"}, "models": [$kimi, $qwen]},
    "blinkvllm": {"baseUrl": "http://127.0.0.1:18790/vllm", "api": "openai-completions", "apiKey": "none", "models": [$vllm]}
  }},
  "agents": {"entries": {
    "go2": null,
    "blink": {
      "name": "blink",
      "workspace": "$WORKSPACE",
      "agentDir": "$HOME/.openclaw/agents/blink/agent",
      "model": {"primary": "blinkcerebras/moonshotai-kimi-k3.1-pro-trial"},
      "skills": [],
      "thinkingDefault": "high",
      "tools": {"profile": "minimal", "alsoAllow": ["robot__*"], "deny": ["session_status", "gateway"]}
    }
  }}
}
EOF
        echo "openclaw.sh: config patched; gateway token in $TOKEN_FILE"
        ;;
    gateway)
        set -a
        # shellcheck disable=SC1091
        . "$HOME/.config/cerebras.env"
        set +a
        OPENCLAW_GATEWAY_TOKEN="$(cat "$TOKEN_FILE")"
        export OPENCLAW_GATEWAY_TOKEN
        cd "$HOME/Projects/hackathon"
        exec mise exec -- openclaw gateway run
        ;;
    target)
        shift
        [[ $# -ge 1 ]] || { echo "$usage" >&2; exit 2; }
        mkdir -p "$CONF"
        printf '%s\n' "$@" > "$CONF/target"
        oc mcp reload
        ;;
    *)
        echo "$usage"
        [[ "${1:-}" == -h || "${1:-}" == --help ]]
        ;;
esac
