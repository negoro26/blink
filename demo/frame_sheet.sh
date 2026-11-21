#!/usr/bin/env bash
set -euo pipefail

usage="usage: demo/frame_sheet.sh <recording dir> <out.png> <sim s>...
  One PNG: for each sim second given, the chase camera frame over the front camera frame at that
  time, side by side, each 640 px wide and labelled with its sim time. The recording dir is what
  run_isaac.py --replay --record writes (chase.mp4, front.mp4, frames.csv)."
[[ "${1:-}" != -h && "${1:-}" != --help && $# -ge 3 ]] || { echo "$usage"; [[ $# -ge 3 ]]; exit; }
DIR="$1" OUT="$2"
shift 2
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
T0="$(sed -n 2p "$DIR/frames.csv" | cut -d, -f2)"
FONT="${CLAW_FONT:-DejaVu Sans}"
cols=()
for t in "$@"; do
    off="$(awk -v t="$t" -v a="$T0" 'BEGIN { printf "%.4f", t - a }')"
    for v in chase front; do
        ffmpeg -loglevel error -y -ss "$off" -i "$DIR/$v.mp4" -frames:v 1 \
            -vf "scale=640:-2,drawtext=font='$FONT':text='t=$t s $v':x=10:y=10:fontsize=24:fontcolor=white:borderw=2:bordercolor=black" \
            "$TMP/$v-$t.png"
    done
    ffmpeg -loglevel error -y -i "$TMP/chase-$t.png" -i "$TMP/front-$t.png" -filter_complex "[0][1]vstack" "$TMP/col-$t.png"
    cols+=(-i "$TMP/col-$t.png")
done
n=$(( ${#cols[@]} / 2 ))
if (( n > 1 )); then
    ffmpeg -loglevel error -y "${cols[@]}" -filter_complex "hstack=inputs=$n" "$OUT"
else
    cp "$TMP/col-$1.png" "$OUT"
fi
echo "frame_sheet: $OUT ($n times)"
