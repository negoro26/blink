"""Re-release a live run's logged tool calls as if its model had decoded at another speed: a robot_mcp.sim proof script.

    python3 scripts/claw/rerelease.py demo/results/<live run>.json --tok-s 31.0 > calls.jsonl
    python3 scripts/claw/rerelease.py demo/results/<live run>.json --delay 5.0 > calls.jsonl

The live run's result (drive.py) holds, from the event on, every model call (request, first token, last token, its
completion tokens) and every tool call the MCP server made (its op, arguments and request time). Each tool call is
re-timed by the decode time the model calls before it would have taken at --tok-s instead of as measured: a model
call that ended before the tool call adds tokens / tok_s - (last token - first token) to it. Everything else (the
event's hop to the laptop, OpenClaw, the provider's queue and prefill, the network, the MCP hop, the tool's own
time) stays as measured. --delay instead shifts every tool call by a fixed number of seconds (the same decision,
released late). The script's lines are `{"at": s after the event, "op": tool, ...arguments}`, sorted; stderr gets
the schedule.
"""

import argparse
import json
import sys
from operator import itemgetter


def schedule(episode, tok_s=None, delay=0.0):
    """[(at, tool call)] of one episode's tool calls, re-timed."""
    steps = [s for s in episode["steps"] if s["first_token_s"] is not None and s["completion_tokens"]]
    out = []
    for call in episode["tool_calls"]:
        t = call["t_from_event_s"]
        shift = delay
        if tok_s:
            shift += sum(
                s["completion_tokens"] / tok_s - (s["end_s"] - s["first_token_s"]) for s in steps if s["end_s"] <= t
            )
        out.append((round(t + shift, 4), call))
    return sorted(out, key=itemgetter(0))


def main():
    """Print the proof script."""
    ap = argparse.ArgumentParser(description=(__doc__ or "").split("\n")[0])
    ap.add_argument("result", help="a live run's demo/results/<run>.json")
    speed = ap.add_mutually_exclusive_group(required=True)
    speed.add_argument("--tok-s", type=float, help="decode speed [tokens/s] to re-release at")
    speed.add_argument("--delay", type=float, help="seconds to add to every tool call")
    args = ap.parse_args()
    with open(args.result, encoding="utf-8") as f:
        episode = json.load(f)["episodes"][0]
    for at, call in schedule(episode, args.tok_s, args.delay or 0.0):
        print(json.dumps({"at": at, "op": call["tool"], **call["args"]}))
        print(f"{call['tool']:<16} live +{call['t_from_event_s']:.3f} s -> +{at:.3f} s", file=sys.stderr)


if __name__ == "__main__":
    main()
