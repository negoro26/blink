# blink

blink is a site-safety supervisor. an OpenClaw agent running Kimi K3.1 Pro on Cerebras watches an industrial site and
runs the response when a vehicle is about to hit a person: brake the vehicle on the site's safety network, alert the
person's wearable, dispatch a robot, file the incident report. the robots keep their own reflexes; the brain owns the
multi-tool response.

in simulation a forklift reverses down a warehouse aisle toward a worker picking at a rack. the sim raises the event
from its ground truth about 12 s before contact and the brain has until then to act. a second scenario sends a fleet
of 20 ANYmal-D robots to search the same warehouse for a worker who collapsed somewhere in the aisles.

## what's here

- `SIM_PROTOCOL.md`: the one boundary between brain and sim, newline-delimited JSON over TCP. any sim (or a real
  site gateway) that speaks it can be driven by the brain.
- `robot_mcp/`: the brain side. `server.py` is the MCP server OpenClaw runs (six tools: `site_get_state`,
  `site_camera`, `site_estop`, `site_alert`, `fleet_dispatch`, `incident_report`), `sim.py` the stdlib protocol client
  and a scripted brain (`proof`), `replay.py` a recorded sim that serves one event with no physics,
  `openclaw/AGENTS.md` the agent's whole instruction.
- `scripts/claw/`: `openclaw.sh` writes the OpenClaw config and runs the gateway, `drive.py` hands the agent the sim's
  event and logs every model call and tool call with its timing, `rerelease.py` re-times a recorded decision at
  another decode speed.
- `demo/`: the warehouse scenarios (`scenario_warehouse.yaml`, `scenario_worker.yaml`, `scenario_worker_chase.yaml`),
  the forklift and worker asset builders (NVIDIA Isaac Sim assets, CC BY 4.0), recorded events, the recording
  compositor and contact sheets, and every run's result in `demo/results/`. `demo/README.md` has the scene in detail.
- `kit/`: a standalone Isaac Sim 6.2 scene (a forklift knocks boxes off a rack above an ANYmal) with its own reflex
  and post-processing.

## what's not here

the simulator side is a private isaac sim stack; it speaks `SIM_PROTOCOL.md`. so is the launch tooling around the
sim. docstrings that name `scripts/claw/box_cmd.sh` or `scripts/blink/` point at that private side.

## run the brain against a recording

needs python 3.12 and `mcp==2.3.0` for the server; the protocol client and the replay are stdlib only.

```
python3 -m robot_mcp.replay demo/events/worker.json --port 47811      # the recorded worker event, no physics
BLINK_PORT=47811 python3 -m robot_mcp.sim call fire                 # send the event again to every client
BLINK_PORT=47811 python3 -m robot_mcp.sim call site_estop vehicle=forklift
```

the MCP server is the same against the replay or a live sim:

```
BLINK_PORT=47811 uv run --python 3.12 --with mcp==2.3.0 python -m robot_mcp.server
```

register that command as an MCP server named `robot` in OpenClaw (or adapt `scripts/claw/openclaw.sh setup`), put
`robot_mcp/openclaw/AGENTS.md` in the agent's workspace, fire the event and hand the agent the event line. every call
lands in `./mcp.jsonl` and every protocol message in `./blink_log.jsonl`.

## results

L is the time from the sim's event to the brain's first action request, on the sim host's clock. warehouse worker
scenario: the alert saves the worker up to 9.975 s after the event, the e-stop up to 11.845 s.

| run | brain | L | decode | outcome |
|---|---|---|---|---|
| `site-proof-alert2` | scripted `site_alert` | 2.00 s | | saved |
| `site-live-k2` | Kimi, effort high | | 323 tok/s | hit (13 refused model calls, lost MCP server) |
| `site-live-k3` | Kimi, effort high | 11.35 s | 321 tok/s | saved |
| `site-live-k4` | Kimi, effort high | 11.10 s | 333 tok/s | saved |
| `k4-rr200.8` | k4's decision re-released at 200.8 tok/s | 11.98 s | | hit |
| `k4-rr110` | k4's decision at 110 tok/s | 13.80 s | | hit |
| `k4-rr31` | k4's decision at 31 tok/s | 24.09 s | | hit |
| `k4-delay2.704` | k4's decision 2.704 s late | 13.80 s | | hit |

the earlier robot-evade gate (`demo/results/gate-wh1.md`): the forklift left 2.38 s and the robot needed 1.36 s to
step clear, so the gate asked for a median L under 0.77 s. Kimi's median L was 7.6 to 10.2 s at every reasoning
effort, so the robot keeps its own reflex and the brain moved up to supervising the site.

fleet search in the same warehouse, 20 robots, one man down in an aisle the cameras only partly see (search window
150 s after the alarm; scenario `demo/scenario_fleet_warehouse.yaml`):

| run | brain | dispatch after alarm | decode | outcome |
|---|---|---|---|---|
| `fleetwh-smoke` | scripted `fleet_dispatch` | | | found by anymal01 13.4 s after the alarm; 3 robots ended down (2 where scripted routes crossed on the dock, anymal06 in an aisle) |
| `fleetwh-live1` | Kimi, effort medium, 14,086 tokens | 57.28 s | 281.9 tok/s | 20/20 routes applied, found by anymal02 71.28 s after the alarm, 0 falls; Kimi guessed aisle A4, he lay in A2, the plan covered every aisle |
| `fleetwh-rr200` | live1's plan re-released at 200.8 tok/s | 77.46 s | | found 91.46 s after the alarm, 0 falls |
| `fleetwh-rr110` | live1's plan at 110 tok/s | 135.37 s | | found 149.36 s after the alarm, 0.64 s inside the window |
| at 31 tok/s | schedule only | 461.70 s | | past the window: that fleet never starts |

## credits

- warehouse hall and forklift: NVIDIA Isaac Sim assets (`Isaac/Environments/Simple_Warehouse`, `Isaac/Props/Forklift`),
  CC BY 4.0.
- ANYmal-D and its flat-terrain locomotion policy: NVIDIA Isaac Lab 3.0.0 (BSD-3-Clause) and its published
  `Isaac-Velocity-Flat-AnymalD` RSL-RL checkpoint.
