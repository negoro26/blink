# Blink sim protocol (one boundary, any sim)

The brain (the MCP server `robot_mcp`, run by OpenClaw) talks to a running sim over one TCP socket of newline-delimited
JSON. A sim implements the server side: here the Kit-less runner (`run_legged.py` with
`the sim's blink module`). No ROS between the brain and the sim. To put the brain on another robot or sim, serve these
messages and point `robot_mcp` at the socket (`BLINK_HOST`, `BLINK_PORT`).

## Transport
- The sim listens on `127.0.0.1:$BLINK_PORT` (default 47800) on the sim host. The MCP server runs on the same host
  (OpenClaw starts it over ssh stdio) and connects to it.
- One JSON object per line, UTF-8. Requests carry `id`; replies echo it. Errors: `{"id": n, "error": "text"}`.

## Time: the catch-up rule (the race is in sim time, faithful to the brain's wall-clock latency)
- When the hazard becomes detectable, the sim sends `event` with `t_sim` and `t_wall_ns` taken at the same instant.
- Every request carries the client's `t_wall_ns` at send time (CLOCK_REALTIME; the MCP server and the sim share a
  host, so one clock). The sim maps it to `t_target = event.t_sim + (req.t_wall_ns - event.t_wall_ns) / 1e9`.
- Before answering or applying a request, the sim steps until its time reaches `t_target`. If it is already past
  `t_target`, it answers at its current time and reports `late_s = t_sim - t_target`.
- The sim never pauses for the brain and never runs ahead of the wall mapping by more than one control step, so the
  site experiences exactly the brain's real latency, whatever the sim's real-time factor.
- A request that arrives before the event, or without an integer `t_wall_ns`, has no `t_target`: the sim applies and
  answers it at its next control step, at its current time, with `late_s` 0. The mapping is fixed on arrival, so it
  stays unmapped even if the event fires before that step. Before the event nothing ties the sim's pace to the wall
  clock.

## The site
One vehicle (`vehicles[0]`, e.g. `forklift`) drives a fixed sim-time schedule. People stand where the scenario puts
them; an alerted person starts, `reaction_s` after the alert, toward a safe point at walking speed. A robot stands
by and walks to a point when dispatched. A person is hit when the vehicle's collider footprint reaches his disc: the
verdict is geometric, from the two schedules, judged every physics step. Poses are world frame (x, y m; yaw rad).

## Messages
Sim to client (unsolicited):
- `{"type":"event","t_sim":121.0,"t_wall_ns":...,"time_to_contact_s":11.975,"person":"worker1","alert_deadline_t_sim":130.975,"estop_deadline_t_sim":132.845,"deadline_t_sim":132.845,"deadline_s":11.845,"state":{...}}`:
  the first policy step at which the vehicle moves at 1 m/s on a schedule that meets the robot or a person.
  `time_to_contact_s` is the earliest predicted contact from the schedules. For the person it meets first, the
  deadlines are the last sim time at which an alert alone, or an e-stop alone, still keeps the vehicle off him (from
  the same schedules and geometry, on the 5 ms physics grid), and `deadline_t_sim` the later of the two (null: that
  action cannot save him). `state` is the `site_get_state` reply at that instant.
- `{"type":"outcome","t_sim":...,"collision":bool,"t_contact":...,"min_gap_m":...,"max_force_n":...,"people":{"worker1":{"hit":bool,"t_contact":...|null,"min_gap_m":...}}}`
  once the vehicle has stopped: the robot's physical contact verdict, and each person's geometric one. The run's
  verdict is `hit` when any person is hit, else `saved`; in a scenario with no people, `hit` when the robot collided.

Client to sim (requests, each with `id`, `op`, `t_wall_ns`) and replies (each with `late_s`):
- `site_get_state` -> `{"t_sim", "vehicles": [{"id","x","y","yaw","speed_mps","reversing","estop","estop_available","stops_at_t_sim"}], "people": [{"id","x","y","yaw","speed_mps","time_to_contact_s","alert_deadline_s","estop_deadline_s","reaction_s","alerted","safe"}], "robot": {"id","x","y","yaw","vx","vy","time_to_contact_s"}}`.
  Contact times and deadlines (seconds from now) are recomputed from the schedules as they are now (after any e-stop
  or alert); null: none predicted, or that action can no longer save the person, or it has been used.
- `site_camera`, `name`: `chase` (a fixed CCTV view) or `front` (the robot's) -> `{"t_sim","camera","jpeg_b64"}`, 640x360
  JPEG, rendered at that step on request (no periodic rendering).
- `site_estop`, `vehicle` -> `{"vehicle","applied_at_t_sim","stops_at_t_sim"}`: the site's safety network brakes the
  vehicle at the scenario's e-stop deceleration from the mapped sim time. Unknown vehicle, or one without a networked
  e-stop: error.
- `site_alert`, `person`, `message` -> `{"person","applied_at_t_sim","reaction_s","moves_at_t_sim"}`: the person's
  wearable alert. The first alert counts; a repeat returns the first one's times. The message is logged.
- `robot_dispatch`, `robot`, `x`, `y` -> `{"robot","applied_at_t_sim","target"}`: the robot's own follower walks it
  there, then it holds.
- `incident_report`, `text` -> `{"logged_at_t_sim"}`: a `{"type":"incident_report","t_sim","text"}` line in the
  sim's log. Nothing leaves the sim.

## Logged by the sim (for the film and the audit)
`blink_log.jsonl` in the run dir: every message in and out with `t_sim` and wall time, the mapped `t_target`,
`late_s`, the command applied, and the incident reports. `scenario.jsonl`: the vehicle, robot and people poses at
`odom_hz` and one line per site event (`car_go`, `site_estop`, `site_alert`, `person_contact`, `car_stopped`).
