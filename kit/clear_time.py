"""Clear time from a run's track: seconds from the evade command until the footprint leaves the predicted landing zone.

Usage: python3 clear_time.py RUN_DIR T_COMMAND   (free direction 'left' = world +x for the scene's robot, yaw -90)
"""

import json
import sys

run, t0 = sys.argv[1], float(sys.argv[2])
shot = json.load(open(f"{run}/shot_used.json"))
zone_east, half_wid = shot["zone"][2], shot["robot"]["half_wid"]
rows = [json.loads(line) for line in open(f"{run}/track.jsonl")]
start = next(r for r in rows if r["t"] >= t0)
out = next((r for r in rows if r["t"] >= t0 and r["x"] - half_wid > zone_east), None)
cruise = [r for r in rows if t0 + 1.0 <= r["t"] <= t0 + 2.0]
v = (cruise[-1]["x"] - cruise[0]["x"]) / (cruise[-1]["t"] - cruise[0]["t"])
print(
    json.dumps(
        {
            "run": run,
            "t_command": t0,
            "start_x": start["x"],
            "exit_m_left_at_command": round(zone_east - (start["x"] - half_wid), 3),
            "t_out_of_zone": out["t"] if out else None,
            "clear_time_s": round(out["t"] - t0, 3) if out else None,
            "cruise_speed_mps": round(v, 3),
        }
    )
)
