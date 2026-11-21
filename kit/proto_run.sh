#!/usr/bin/env bash
# One protocol test: the Kit scene serves (and records), the scripted client evades at L seconds after EVENT.
# Usage: GPU=5 ./proto_run.sh <tag> <latency_s> [direction=auto] [scene args...]
set -u
cd ~/work/blink-kit
TAG=$1; L=$2; DIR=${3:-auto}; shift 3 || shift $#
PORT=${BLINK_PORT:-47811}
rm -rf out/$TAG
BLINK_PORT=$PORT ./kitrun.sh $TAG scene.py --out out/$TAG --serve --port $PORT "$@" &
KP=$!
python3 blink_client.py --latency $L --direction $DIR --port $PORT --out out/$TAG/client > logs/$TAG.client 2>&1
CRC=$?
wait $KP
echo "PROTO tag=$TAG L=$L client_rc=$CRC"
grep "^CLIENT" logs/$TAG.client | cut -c1-600
