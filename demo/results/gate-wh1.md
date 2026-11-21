# Latency gate, warehouse1 (written 2026-11-21 20:58 EET, before any gate number was read)

- Event: demo/events/wh1.json, the sim's own event of run 20261121-202422-wh-proof-l03 (t_sim 40.50,
  time_to_impact_s 2.376 from the forklift's schedule).
- Clear time: 1.36 s (left, 1.03 m at 1.0 m/s, run 20261121-165609-wh-clear2).
- Margin = window - clear time = 2.376 - 1.36 = 1.016 s.
- L = the first robot_evade (or site_estop) request's t_wall_ns minus the event's t_wall_ns, both on gpu-host's clock,
  through the whole path: relay to the laptop, OpenClaw gateway, model, MCP tool call back over ssh.
- GO for a brain at one setting if, over 5 replays, median L <= margin - 0.25 = 0.766 s and max L <= 1.016 s, and
  every decision is evade left.
- Kimi K3.1 Pro (moonshotai-kimi-k3.1-pro-trial) runs reasoning_effort max, high, medium, low in that order; the
  robot brain uses the highest effort that passes. Lower efforts are measured only to find that one.
- The GPU baseline (vLLM qwen-27b) runs with thinking on and off, and keeps the setting that decides correctly
  fastest.

## Measured 2026-11-21 21:00-21:20 EET (OpenClaw with Tool Search on: the model reached robot_evade through dispatch_action)

| effort | L s (5 replays) | median | max | queue_time s | event to model request s | reasoning tokens | decode tok/s | decision | verdict |
|---|---|---|---|---|---|---|---|---|---|
| max | 11.88, 7.37, 6.11, 10.22, 11.11 | 10.22 | 11.88 | 2.57, 1.60, 1.20, 6.04, 6.94 | 2.74, 2.63, 2.34, 1.80, 2.02 | 1060, 618, 525, 511, 520 | 299, 293, 344, 328, 394 | evade left x5 | NO-GO |
| high | 5.12, 6.26, 10.03, 11.43, 7.77 | 7.77 | 11.43 | 1.09, 0.35, 6.08, 6.78, 3.13 | 1.73, 2.63, 2.11, 2.36, 2.36 | 405, 415, 415, 457, 457 | 344, 332, 385, 310, 306 | evade left x5 | NO-GO |
| medium | 5.24, 7.62, 9.20, 7.99, 9.29 | 7.99 | 9.29 | 0.32, 0.46, 4.60, 3.40, 4.64 | 2.64, 1.77, 2.23, 2.65, 2.16 | 461, 524, 453, 417, 514 | 342, 302, 351, 419, 383 | evade left x5 | NO-GO |
| low | 8.75, 3.96, 7.61, 5.94, 14.65 | 7.61 | 14.65 | 0.99, 0.01, 1.16, 0.59, 10.08 | 2.45, 2.27, 2.14, 2.23, 2.61 | 61, 223, 415, 381, 337 | 265, 262, 355, 295, 302 | evade left x5 | NO-GO |

Verdict: NO-GO at every effort. No effort passes, so the rule picks none. What limits L (measured): Cerebras queue_time
0.01 to 10.1 s per call under hackathon load, OpenClaw's own time from the event to the model request 1.7 to 2.7 s,
and 61 to 1,060 reasoning tokens per decision (low 61 to 415, max 511 to 1,060) at 260 to 420 tok/s. Every decision was evade left, the right one.

## Where L goes (measured 2026-11-21 21:25-21:50 EET)

- Inside OpenClaw: the gateway receives the turn 6 to 23 ms after the sim's event reaches the laptop (216 ms once), and
  sends the model request 1.72 to 2.73 s later (20 turns above). The request is 26.8 kB: a 20,930-character system
  prompt and the tool directory, about 6,040 prompt tokens of which 5,120 were cached. The gateway log shows only
  admission (~35 ms) for that span; the rest (prompt assembly, session and MCP runtime build) is not broken down there.
- Lean floor: the same AGENTS.md as the system prompt, the same EVENT and the five robot tools sent straight to Cerebras
  (744 prompt tokens, none cached), streamed:

| effort | request to first token s | request to tool call s | queue_time s | tool call minus queue s | reasoning tokens | decision |
|---|---|---|---|---|---|---|
| low | 21.55 | 21.93 | 21.17 | 0.76 | 98 | robot_evade left |
| low | 22.36 | 23.01 | 22.01 | 1.00 | 192 | robot_evade left |
| low | 11.68 | 12.24 | 11.19 | 1.06 | 153 | robot_evade left |
| low | 13.48 | 13.91 | 13.04 | 0.86 | 94 | robot_evade left |
| low | 7.96 | 8.46 | 7.52 | 0.95 | 45 | robot_evade left |
| high | 11.61 | 12.36 | 11.19 | 1.17 | 221 | robot_evade left |
| high | 5.72 | 6.85 | 5.26 | 1.59 | 209 | robot_evade left |
| high | 5.18 | 6.36 | 4.61 | 1.75 | 240 | robot_evade left |
| high | 16.20 | 17.03 | 15.82 | 1.21 | 262 | robot_evade left |
| high | 11.27 | 12.12 | 10.81 | 1.31 | 217 | robot_evade left |

At this hour queue_time (4.6 to 22.0 s) is nearly all of L. Without it the floor from request to tool call is 0.76 to
1.06 s at effort low and 1.17 to 1.75 s at high, before OpenClaw's ~2.2 s and the ssh hops.

## Scripted proof of the catch-up rule (no model)

| run | evade left at L | applied at t_sim | outcome |
|---|---|---|---|
| 20261121-202422-wh-proof-l03 (ovrtx, 25 Hz cameras) | 0.300 s | 40.82 | collision false, min gap 0.491 m |
| 20261121-204152-wh-proof-l30 (newton_warp, no cameras) | 3.000 s | 43.52 | collision true, contact 42.88, 61.9 kN |

Sim speed on gpu-host GPU 3 (shared, load 110 to 250): 0.29 times real time with ovrtx rendering both cameras at 25 Hz;
2.14 to 2.24 times real time (427 to 448 physics steps/s) with no cameras, held to 1.0 by the wall mapping after the event.
