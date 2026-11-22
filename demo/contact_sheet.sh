#!/usr/bin/env bash
set -euo pipefail

usage="usage: demo/contact_sheet.sh <video.mp4> <frames.csv> <out.png> [n=6]
  n frames of one replay video spread evenly over it, 3 per row at 640 px, each labelled with its sim time from the
  replay's frames.csv (frame,t): the still to judge a shot by at phone size."
[[ "${1:-}" != -h && "${1:-}" != --help && $# -ge 3 ]] || { echo "$usage"; [[ $# -ge 3 ]]; exit; }
video="$1" csv="$2" out="$3" n="${4:-6}"
FONT="${CLAW_FONT:-DejaVu Sans}"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
frames=$(($(wc -l < "$csv") - 1))
((n >= 2)) || { echo "$usage"; exit 2; }
inputs=() layout=""
for ((i = 0; i < n; i++)); do
    f=$(((frames - 1) * i / (n - 1)))
    t="$(awk -F, -v f="$f" '$1 == f { print $2 }' "$csv")"
    ffmpeg -loglevel error -y -i "$video" -vf "select=eq(n\,$f),scale=640:-2,drawtext=font='$FONT':text='t=$t s':x=10:y=10:fontsize=22:fontcolor=white:borderw=2:bordercolor=black" \
        -frames:v 1 "$TMP/$i.png"
    inputs+=(-i "$TMP/$i.png")
    layout+="${layout:+|}$((i % 3 * 640))_$((i / 3 * 360))"
done
ffmpeg -loglevel error -y "${inputs[@]}" -filter_complex "xstack=inputs=$n:layout=$layout:fill=black" "$out"
echo "contact_sheet: $out ($n frames of $frames)"
