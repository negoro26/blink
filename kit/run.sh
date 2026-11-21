#!/usr/bin/env bash
# Blink Kit scene: every acceptance run end to end, in the gpu-host Jupyter container (one CUDA process at a time).
# Usage: GPU=5 ~/work/blink-kit/run.sh      (GPU: never 1; 2/4 are vLLM; 3/6 other lanes)
# Needs the Isaac Sim 6.2 standalone install at ~/work/kit-proto/isaacsim (KitProto's install.sh) and ffmpeg.
set -euo pipefail
cd ~/work/blink-kit
./kitrun.sh nb1 scene.py --out out/nb1               # no brain, live stepping (no render)
./kitrun.sh nb2 scene.py --out out/nb2 --record      # no brain, recorded: same contact time as nb1
./kitrun.sh rf1 scene.py --out out/rf1 --reflex      # the robot's local reflex, live stepping
./kitrun.sh rf2 scene.py --out out/rf2 --reflex --record
./proto_run.sh p03 0.3 auto --record                 # scripted protocol client: evade at EVENT + 0.3 s -> escapes
./proto_run.sh p30 3.0 auto --record                 # scripted protocol client: evade at EVENT + 3.0 s -> hit
for r in nb2 rf2 p03 p30; do python3 post.py out/$r; done
python3 summ.py nb1 nb2 rf1 rf2 p03 p30
