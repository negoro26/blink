# Blink demo

## Scene: warehouse1, a forklift and ANYmal-D (the video's)

ANYmal-D stands in a rack aisle of NVIDIA's Simple_Warehouse, scanning a pallet bay on its right and facing up the
aisle. A forklift at the aisle's north end reverses down the aisle at a fixed sim time, counterweight first, at
12.6 km/h. Its swept band covers the robot. Standing still, the classical safety reflex, gets the robot hit. Backing
away or walking forward stays in the forklift's path, and the forklift is faster. Stepping right walks into the rack.
Only a sidestep left, 1.03 m into the aisle's west half, clears it. `demo/scenario_warehouse.yaml` holds every number:
the poses, the forklift's schedule, the racks, the blocked and free sides, the clear time and the decision window.

Credit and licences:

- The hall is NVIDIA's Isaac Sim asset `Isaac/Environments/Simple_Warehouse` (`full_warehouse.usd`), CC-BY-4.0
  (its `Stage/LICENSE`). The video credits it: "Warehouse scene: NVIDIA Isaac Sim assets, CC BY 4.0".
- The forklift is NVIDIA's Isaac Sim prop `Isaac/Props/Forklift` (`forklift.usd`, the same model the warehouse places
  as set dressing). Its folder ships its own `LICENSE`, CC-BY-4.0 (Attribution 4.0 International), and so does its
  `Materials/LICENSE`; the same credit covers it: "Forklift: NVIDIA Isaac Sim assets, CC BY 4.0".
- The stage is NVIDIA's `Isaac/Samples/Replicator/Incidents/full_warehouse_with_incident_tags.usd` (Isaac Sim 6.2), a
  17 KB layer over `full_warehouse.usd` that tags its loose, flammable and leakable items; its folder ships no LICENSE
  file (`.../Incidents/LICENSE` and `.../Replicator/LICENSE` return 404), so it is credited with the warehouse. The
  measured runs below rendered `full_warehouse.usd` directly, before the switch; the geometry of the aisle is the same.
- All are copied unchanged into the sim's world directory by the sim's warehouse1 build script (private, with the
  sim); the forklift's wrapper `forklift.usda` references NVIDIA's `Props/Forklift/forklift.usd`.

What is real and what is given:

- The robot is ANYmal-D on Isaac Lab's published flat-terrain policy (`run_legged`), 5 ms physics steps,
  50 Hz control. With no command it holds its pose (station keeping on the true pose; the policy alone turns about
  2 deg/s on a zero command). The brain moves it only through the Blink sim protocol (`SIM_PROTOCOL.md`,
  `the sim's blink module`): `robot_dispatch` walks it to a point with the runner's follower, then it holds.
- The forklift is the prop wrapped by `demo/build_forklift.py` into a rigid body: the prop's own mesh collision is
  removed and one box over the body (counterweight to mast, 2.31 x 1.21 m, 8 cm above the floor) is its collider.
  Its pose follows the scenario's sim-time schedule, written before every physics step, so two runs drive it
  identically; the collider is real contact in Newton and a hit moves the robot through the solver.
- The two rack rows of the aisle are static box colliders (`obstacles:`), so the robot cannot step into a rack; the
  rest of the hall is only seen. The floor is the contact plane at z 0.
- Perception is ground truth and the sim announces the hazard itself: the protocol's `event` goes out at the first
  policy step at which the forklift moves at 1 m/s on a schedule that meets the robot (t_sim 40.50), with
  `time_to_impact_s` from that schedule (2.376 s; the no-brain runs measured 2.375 and 2.380 s), and the `outcome`
  once the forklift has stopped. Every request is applied at the sim time its wall clock maps to (the catch-up rule),
  so the robot gets exactly the brain's real latency whatever the sim's speed.
- The front camera sits on a mast, 0.95 m above the floor, looking along the robot's heading (`front_camera:`); the
  chase camera is world-fixed behind the robot's left shoulder, looking up the aisle (`chase:`). The hall is lit by
  its own 37 ceiling lights and two distant lights at the runner's ovrtx camera exposure, as in the contact sheet.

Run it: the sim side is a private Isaac Sim stack, so this is the shape of a run, not a command you can paste. Live
and proof runs take the cheap camera path (`--renderer newton_warp`; with a scenario the cameras render only when the
brain calls `site_camera`); the track holds every policy step's base pose, joint angles and forklift pose. The sim
serves `SIM_PROTOCOL.md` on 47800 with `--scenario demo/scenario_worker.yaml`, then:

```
python3 scripts/claw/drive.py live --brain kimi --effort high --session <run name>
```

`python3 -m robot_mcp.sim proof '{"at": L, "op": ..., ...}'` in place of the brain sends scripted requests L s after
the event (the catch-up rule's check). The film renders a live run again with ovrtx from its track, nothing re-simulated:
`SIM_ARGS="--renderer ovrtx --camera-hz 30 --scenario ... --replay <run>/track.csv --record {out}/record --record-from 36"`
writes `record/{chase.mp4,front.mp4,frames.csv}` and copies the run's `scenario.jsonl` beside them;
`demo/frame_sheet.sh` reads that dir, and `demo/compose_split.sh left.json right.json out.mp4` stitches two runs side
by side with a HUD from `demo/results/<run>.json`.

### Measured (gpu-host GPU 3, shared, load 36 to 70; sim seconds unless marked)

| run | what | result |
|---|---|---|
| `20261121-162124-wh-walk` | Go2 (run_isaac.py, ovrtx) walks its 6 x 4 m loop on warehouse1's floor | rc 0, 0 fall samples from 5 s, path 14.15 m, base 0.27 to 0.35 m above the floor, max roll 3.9 deg, pitch 2.9 deg; `front.png` shows the hall |
| `20261121-165609-wh-clear2` | ANYmal-D at `robot_start`, `/cmd_vel` steps of 1.0 m/s from standing, forklift leaving at 44 s | left (the free side) from the start: the 1.03 m clear distance in 1.36 s and 1.34 s; right: the rack's collider stops it after 0.56 m, still in the band; back and forward: 1.03 m in 1.30 s and 1.26 s, along the forklift's line, so never out of its band; a 2.4 s sidestep left made the forklift miss: `collision false`, min gap 1.024 m |
| `20261121-171240-wh-hit1` | no brain, the robot holds its pose | EVENT at t_sim 40.50 (closing 1.0 m/s, tti 6.90 s, distance 7.23 m, bearing -0.6 deg), contact 42.875, window 2.375 s, `collision true`, peak contact force 82.7 kN, the robot knocked over (fallen from 43.22) |
| `20261121-172812-wh-hit2` | the same, again | EVENT 40.50, contact 42.880, window 2.380 s, `collision true`, peak 52.2 kN, fallen from 43.20 |

Clear time 1.36 s (the slower left step), so the window is set to 1.36 + 1.0 = 2.36 s: the forklift's counterweight
starts 7.5 m up the aisle from the robot's base and the forklift goes at 2 m/s² up to 3.5 m/s, so EVENT, which fires as
it passes 1 m/s 0.5 s after it starts, comes 2.36 s before contact by the schedule; the runs measured 2.375 and
2.380 s. The contact times are 5 ms apart, the robot's stance under station keeping. The clear runs stepped at 0.44 to
0.76 times real time with ovrtx.
