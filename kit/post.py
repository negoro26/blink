"""Blink run post-processing: chase.mp4, mast.mp4, review.mp4 (chase + mast inset), sheet.jpg (labelled stills).

Usage: python3 post.py RUN_DIR [title]
"""

import json
import os
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

run = sys.argv[1]
title = sys.argv[2] if len(sys.argv) > 2 else os.path.basename(run.rstrip("/"))
frames = [json.loads(line) for line in open(os.path.join(run, "frames.jsonl"))]
out = json.load(open(os.path.join(run, "outcome.json")))
t_ev = out["t_event"]
ff = ["ffmpeg", "-y", "-loglevel", "error", "-framerate", "30"]
grade = {"chase": "noise=alls=5:allf=t+u:all_seed=7,eq=contrast=0.94:brightness=0.02:saturation=0.90"}
for cam in ("chase", "mast"):
    subprocess.run(
        [
            *ff,
            "-i",
            f"{run}/{cam}/%05d.png",
            "-c:v",
            "libx264",
            *(["-vf", grade[cam]] if cam in grade else []),
            "-pix_fmt",
            "yuv420p",
            "-crf",
            "18",
            f"{run}/{cam}.mp4",
        ],
        check=True,
    )
subprocess.run(
    [
        *ff,
        "-i",
        f"{run}/chase/%05d.png",
        "-framerate",
        "30",
        "-i",
        f"{run}/mast/%05d.png",
        "-filter_complex",
        "[0:v]" + grade["chase"] + "[base];[1:v]scale=640:360,pad=648:368:4:4:white[m];[base][m]overlay=W-w-24:24",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-crf",
        "20",
        f"{run}/review.mp4",
    ],
    check=True,
)
try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 26)
except OSError:
    font = ImageFont.load_default(size=26)
t_end = out["t_sim"] + 0.5
picks = [t_ev - 0.5 + k * (t_end - t_ev + 0.5) / 7 for k in range(8)]
W, H = 960, 540
sheet = Image.new("RGB", (W * 2, H * 4 + 60), (20, 20, 20))
d0 = ImageDraw.Draw(sheet)
d0.text(
    (12, 14),
    f"{title}: collision={out['collision']} t_contact={out['t_contact']} min_gap={out['min_gap_m']} "
    f"max_force={out['max_force_n']} N  (EVENT t={t_ev:.2f})",
    fill=(255, 255, 255),
    font=font,
)
for n, t in enumerate(picks):
    f = min(frames, key=lambda r: abs(r["t_sim"] - t))
    im = Image.open(f"{run}/chase/{f['i']:05d}.png").convert("RGB").resize((W, H))
    m = Image.open(f"{run}/mast/{f['i']:05d}.png").convert("RGB").resize((W // 4, H // 4))
    im.paste(m, (W - W // 4 - 8, 52))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, W, 44], fill=(0, 0, 0))
    d.text((12, 8), f"t_sim={f['t_sim']:.2f}  (EVENT{f['t_sim'] - t_ev:+.2f} s)", fill=(255, 255, 255), font=font)
    sheet.paste(im, ((n % 2) * W, 60 + (n // 2) * H))
sheet.save(f"{run}/sheet.jpg", quality=88)
print("POST", run, len(frames), "frames", f"{run}/sheet.jpg")
