#!/usr/bin/env bash
set -euo pipefail

usage="usage: demo/compose_split.sh <left.json> <right.json> <out.mp4> [--from S] [--to S]
  Stitches two recorded runs side by side, 1920x1080 at 60 fps, at most 60 s: each half is the run's
  chase camera (960x540) over its front camera (960x540), with a HUD read from the run's
  demo/results/<run>.json: \"model\" (the label), \"reaction_ms\" (else t_tool_call - t_event, in ms),
  \"tokens_per_s\" and \"outcome\". The outcome appears in large type from the moment the sim decided it
  (the car's contact, or the car passing), read from the recording's scenario.jsonl.
  Each result's \"run\" names its recording, \${CLAW_REC:-out/recordings}/<run>/ with chase.mp4,
  front.mp4, frames.csv (the replay's frame times) and scenario.jsonl (the live run's), as
  run_isaac.py --replay --record writes them. --from/--to are sim seconds (default: the whole
  recording, cut to 60 s). The HUD shows nothing but those fields: no paths, hosts or keys."
[[ "${1:-}" != -h && "${1:-}" != --help ]] || { echo "$usage"; exit 0; }
LEFT="${1:?$usage}" RIGHT="${2:?$usage}" OUT="${3:?$usage}"
shift 3
FROM="" TO=""
while (($#)); do
    case "$1" in
        --from) FROM="$2"; shift 2 ;;
        --to) TO="$2"; shift 2 ;;
        *) echo "$usage" >&2; exit 2 ;;
    esac
done
REC="${CLAW_REC:-out/recordings}"
FONT="${CLAW_FONT:-DejaVu Sans}"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# side <result.json> <tag>: writes $TMP/<tag>.{model,stats,outcome} and prints "<dir> <t_first> <t_outcome>"
side() {
    local json="$1" tag="$2" run dir
    run="$(jq -r .run "$json")"
    dir="$REC/$run"
    [[ -f "$dir/chase.mp4" && -f "$dir/front.mp4" && -f "$dir/frames.csv" ]] || { echo "compose: no recording in $dir" >&2; exit 2; }
    jq -r .model "$json" > "$TMP/$tag.model"
    jq -r '"reaction \((.reaction_ms // (((.t_tool_call // 0) - (.t_event // 0)) * 1000)) | round) ms   \((.tokens_per_s // 0) | round) tok/s"' \
        "$json" > "$TMP/$tag.stats"
    jq -r '.outcome | ascii_upcase' "$json" > "$TMP/$tag.outcome"
    local t_first t_out
    t_first="$(sed -n 2p "$dir/frames.csv" | cut -d, -f2)"
    t_out="$(jq -r 'select(.event == "contact" or .event == "car_stopped") | .t' "$dir/scenario.jsonl" | head -1)"
    echo "$dir $t_first ${t_out:-1e9}"
}
read -r LDIR LFIRST LOUT <<< "$(side "$LEFT" l)"
read -r RDIR RFIRST ROUT <<< "$(side "$RIGHT" r)"
FROM="${FROM:-$(awk -v a="$LFIRST" -v b="$RFIRST" 'BEGIN { print (a > b ? a : b) }')}"
LAST="$(tail -1 "$LDIR/frames.csv" | cut -d, -f2)"
TO="${TO:-$(awk -v f="$FROM" -v l="$LAST" 'BEGIN { print (l - f > 60 ? f + 60 : l) }')}"
DUR="$(awk -v f="$FROM" -v t="$TO" 'BEGIN { d = t - f; print (d > 60 ? 60 : d) }')"
ss() { awk -v f="$FROM" -v a="$1" 'BEGIN { printf "%.4f", f - a }'; }
show() { awk -v f="$FROM" -v o="$1" 'BEGIN { printf "%.3f", o - f }'; }

hud() {  # hud <tag> <t_outcome rel>: the drawtext chain for one half
    local t="$1" at="$2"
    printf "drawbox=x=0:y=0:w=960:h=92:color=black@0.55:t=fill,"
    printf "drawtext=font='%s':textfile=%s:x=24:y=14:fontsize=34:fontcolor=white," "$FONT" "$TMP/$t.model"
    printf "drawtext=font='%s':textfile=%s:x=24:y=56:fontsize=24:fontcolor=0xD0D0D0," "$FONT" "$TMP/$t.stats"
    printf "drawtext=font='%s':textfile=%s:x=(w-text_w)/2:y=200:fontsize=96:fontcolor=white:borderw=6:bordercolor=black:enable='gte(t,%s)'" \
        "$FONT" "$TMP/$t.outcome" "$at"
}

ffmpeg -loglevel error -y \
    -ss "$(ss "$LFIRST")" -t "$DUR" -i "$LDIR/chase.mp4" \
    -ss "$(ss "$LFIRST")" -t "$DUR" -i "$LDIR/front.mp4" \
    -ss "$(ss "$RFIRST")" -t "$DUR" -i "$RDIR/chase.mp4" \
    -ss "$(ss "$RFIRST")" -t "$DUR" -i "$RDIR/front.mp4" \
    -filter_complex "
      [0:v]scale=960:540,setsar=1[lc]; [1:v]scale=960:540,setsar=1[lf];
      [2:v]scale=960:540,setsar=1[rc]; [3:v]scale=960:540,setsar=1[rf];
      [lc][lf]vstack=2,$(hud l "$(show "$LOUT")")[L];
      [rc][rf]vstack=2,$(hud r "$(show "$ROUT")")[R];
      [L][R]hstack=2,drawbox=x=958:y=0:w=4:h=1080:color=white@0.8:t=fill,fps=60,format=yuv420p[v]" \
    -map "[v]" -c:v libx264 -preset slow -crf 18 -r 60 -t "$DUR" "$OUT"
echo "compose: $OUT, $DUR s from sim t=$FROM, left $(cat "$TMP/l.model"), right $(cat "$TMP/r.model")"
