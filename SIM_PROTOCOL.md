# Blink sim protocol (one boundary, two sims)

The brain (MCP server `robot_mcp`, run by OpenClaw) talks to the running sim over one TCP socket with newline-delimited JSON. Both sims implement the same server: the Kit-less runner (`run_legged.py`) and the Isaac Sim Kit scene (`kit/scene.py`). No ROS between the brain and the sim.

## Transport
- The sim listens on `127.0.0.1:$BLINK_PORT` (default 47800) on the sim host. The MCP server runs on the same host (OpenClaw starts it over ssh stdio) and connects to it.
- One JSON object per line, UTF-8. Requests carry `id`; replies echo it.

## Time: the catch-up rule (the race is in sim time, faithful to the brain's wall-clock latency)
- When the hazard becomes detectable, the sim sends `event` with `t_sim` and `t_wall_ns` taken at the same instant.
- Every request carries the client's `t_wall_ns` at send time (CLOCK_REALTIME; the MCP server and the sim share a host, so one clock). The sim maps it to `t_target = event.t_sim + (req.t_wall_ns - event.t_wall_ns) / 1e9`.
- Before answering or applying a request, the sim steps (with the robot's current command) until its time reaches `t_target`. If its time is already past `t_target` (it ran ahead), it answers at its current time and reports `late_s = t_sim - t_target`.
- Between requests the sim may keep stepping toward the wall-mapped time; it never pauses for the brain and never runs ahead of the wall mapping by more than one control step.
- So the robot experiences exactly the brain's real latency, whatever the sim's real-time factor.

## Messages
Sim to client (unsolicited):
- `{"type":"event","t_sim":40.50,"t_wall_ns":...,"hazard":{"kind":"forklift"|"falling_boxes",...},"time_to_impact_s":2.38}`: `time_to_impact_s` is the true predicted contact time from the scenario's known schedule, not distance over current speed.
- `{"type":"outcome","t_sim":...,"collision":true|false,"t_contact":...|null,"min_gap_m":...,"max_force_n":...}`

Client to sim (requests) and replies:
- `{"id":1,"op":"get_state","t_wall_ns":...}` -> `{"id":1,"t_sim":...,"late_s":0.0,"pose":{"x":..,"y":..,"yaw":..},"vel":{"vx":..,"vy":..,"wz":..},"hazard":{"kind":"forklift"|"falling_boxes","x":..,"y":..,"vx":..,"vy":..,"time_to_impact_s":..,"exit_m":{"left":..,"right":..,"front":..,"back":..}},"free_m":{"left":..,"right":..,"front":..,"back":..}}`. Frames: `pose` is world; `vel`, `hazard.x/y/vx/vy`, `exit_m` and `free_m` are robot frame (x forward, y left). `free_m[d]` = distance along d to the first static collider (racks, walls), ground truth. `hazard.exit_m[d]` = distance along d the robot must travel to leave the predicted impact zone (the forklift's swept band, or the boxes' landing zone), `null` if moving along d never leaves it. A direction escapes iff `free_m[d] > exit_m[d]`; the brain works that out, the sim does not say which.
- `{"id":2,"op":"evade","direction":"left"|"right"|"forward"|"back"|"stop","duration_ms":1500,"t_wall_ns":...}` -> `{"id":2,"applied_at_t_sim":...,"late_s":...}`; 1.0 m/s in the robot frame for `duration_ms`, then hold.
- `{"id":3,"op":"set_goal","x":..,"y":..,"yaw":..,"t_wall_ns":...}` -> `{"id":3,"applied_at_t_sim":...}`; the runner's own follower drives there.
- `{"id":5,"op":"site_estop","t_wall_ns":...}` -> `{"id":5,"applied_at_t_sim":...,"stops_at_t_sim":...,"stop_gap_m":...}`; the site's safety network stops the hazard vehicle: it brakes at the scenario's fixed deceleration (default 4.0 m/s^2) from the mapped sim time. A scene without a vehicle answers `{"id":5,"error":"no vehicle"}`. The outcome then also reports `person_hit` (true/false) and `person_min_gap_m` when the scene has a person at risk.
- `{"id":4,"op":"snapshot","t_wall_ns":...}` -> `{"id":4,"t_sim":...,"jpeg_b64":"..."}` (front/mast camera, 640x360, quality 80).
- Errors: `{"id":n,"error":"text"}`.

## Logged by the sim (for the film and the audit)
`blink_log.jsonl` in the run dir: every message in and out with `t_sim` and wall time, the mapped `t_target`, `late_s`, and the command applied.
