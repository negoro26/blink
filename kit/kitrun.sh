#!/usr/bin/env bash
# Run one Isaac Sim python script pinned to a single GPU (never 1; 2/4 vLLM; 3/6 other lanes), with a GPU monitor.
# Usage: GPU=5 ./kitrun.sh <tag> <script.py> [args...]   (logs/<tag>.log, logs/<tag>.gpu)
# Pinning per Isaac Sim 6.2 install FAQ (Multi-GPU): CUDA_VISIBLE_DEVICES + active_cuda_gpus=[0] + multi_gpu=False.
set -u
B=~/work/blink-kit
ISAAC=~/work/kit-proto/isaacsim
TAG=$1; shift
GPU=${GPU:-5}
case $GPU in 1|2|3|4|6) echo "refusing GPU $GPU"; exit 2;; esac
if pgrep -f "kit-proto/isaacsim/kit/python/bin/python3" > /dev/null; then echo "another Kit process is running"; exit 3; fi
mkdir -p $B/logs
export CUDA_VISIBLE_DEVICES=$GPU OMP_NUM_THREADS=${OMP:-32} ACCEPT_EULA=Y OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y
( while true; do echo "$(date -u +%T) $(timeout 20 nvidia-smi --query-gpu=index,memory.used,utilization.gpu --format=csv,noheader -i $GPU)"; sleep 5; done ) > $B/logs/$TAG.gpu 2>&1 &
MON=$!
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_EXE CONDA_PYTHON_EXE PYTHONPATH PYTHONHOME
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | paste -sd:)
t0=$(date +%s)
cd $B
nice -n 10 $ISAAC/python.sh "$@" > $B/logs/$TAG.log 2>&1
rc=$?
kill $MON
echo "KITRUN tag=$TAG rc=$rc wall_s=$(( $(date +%s) - t0 ))" | tee -a $B/logs/$TAG.log
