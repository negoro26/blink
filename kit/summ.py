"""Print the outcome line of each run: python3 summ.py TAG [TAG...] (reads out/TAG/outcome.json + scene.log)."""

import json
import sys

for tag in sys.argv[1:]:
    try:
        o = json.load(open(f"out/{tag}/outcome.json"))
    except OSError as e:
        print(tag, "missing", e)
        continue
    done = [line for line in open(f"out/{tag}/scene.log") if "done t_sim" in line]
    print(
        f"{tag:10s} coll={o['collision']!s:5s} t_contact={o['t_contact']} gap={o['min_gap_m']} F={o['max_force_n']} "
        f"landed={o['landed_centres_xy']} final={o['robot_final']} minz={o['robot_min_z']} tilt={o['robot_max_tilt_deg']} "
        f"| {done[-1].split('done ')[1].strip() if done else ''}"
    )
