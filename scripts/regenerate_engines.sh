#!/usr/bin/env bash
#
# Rebuild every TensorRT engine the launch graph uses from its ONNX source.
#
# TensorRT engines are not portable across GPU SM, TensorRT version or CUDA
# driver, so run this on the target Jetson after every flash or JetPack
# change (JetPack 6.2 = TensorRT 10.3, JetPack 7.2 = TensorRT 10.16), then
# rebuild the workspace so the new engines are installed into share/.
#
# What it builds (names match config/config.yaml):
#   object-detection/yolo26_cw.engine           YOLO26s end2end, FP16
#   sign-classifier/classify.engine             YOLOv8-cls, FP16, dyn batch 1-20
#   lane-detection/lane_detect.engine           ERFNet lane, FP16
#   face-detection/yolov11n-facefp16.engine     YOLOv11n-face, FP16
#   gaze-estimation/resnet18_gaze_fp16.engine   ResNet18 gaze, FP16, 448x448
#   stereo-depth/fast_foundationstereo.engine   Fast-FoundationStereo 320x320,
#                                               FP16 (ONNX kept under archive/)
#
# Existing engines are renamed to <name>.prev.<TIMESTAMP> before the new one
# is written, so a failed build never leaves you without an engine.
#
# Usage: bash scripts/regenerate_engines.sh [name ...]
#   e.g. bash scripts/regenerate_engines.sh classify lane
#   names: detect classify lane face gaze stereo (default: all)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GRAPHS="$REPO_ROOT/src/graphs"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
STAMP="$(date +%Y%m%d-%H%M%S)"
LOG_DIR="$REPO_ROOT/log/engines-$STAMP"

if [[ ! -x "$TRTEXEC" ]]; then
    echo "ERROR: trtexec not found at $TRTEXEC. Set TRTEXEC or install TensorRT (nvidia-jetpack)." >&2
    exit 1
fi
mkdir -p "$LOG_DIR"

build() {
    local name="$1" onnx="$2" engine="$3"
    shift 3
    echo "==> $(basename "$engine") <- $(basename "$onnx")"
    if [[ -f "$engine" ]]; then
        mv "$engine" "${engine}.prev.${STAMP}"
    fi
    if "$TRTEXEC" --onnx="$onnx" --saveEngine="$engine" --fp16 "$@" \
            > "$LOG_DIR/$name.log" 2>&1; then
        echo "    ok ($(du -h "$engine" | cut -f1), log: $LOG_DIR/$name.log)"
    else
        echo "    FAILED, see $LOG_DIR/$name.log" >&2
        tail -5 "$LOG_DIR/$name.log" >&2
        if [[ -f "${engine}.prev.${STAMP}" ]]; then
            mv "${engine}.prev.${STAMP}" "$engine"
        fi
        return 1
    fi
}

want() {
    [[ ${#TARGETS[@]} -eq 0 ]] && return 0
    local t
    for t in "${TARGETS[@]}"; do
        [[ "$t" == "$1" ]] && return 0
    done
    return 1
}

TARGETS=("$@")
FAILED=()

want detect && { build detect \
    "$GRAPHS/object-detection/yolo26_cw.onnx" \
    "$GRAPHS/object-detection/yolo26_cw.engine" || FAILED+=(detect); }

# node_classify allocates for maxBatchSize=20 with optBatchSize=5.
want classify && { build classify \
    "$GRAPHS/sign-classifier/classify_dynamic.onnx" \
    "$GRAPHS/sign-classifier/classify.engine" \
    --minShapes=images:1x3x320x320 \
    --optShapes=images:5x3x320x320 \
    --maxShapes=images:20x3x320x320 || FAILED+=(classify); }

want lane && { build lane \
    "$GRAPHS/lane-detection/ERF_lane_post.onnx" \
    "$GRAPHS/lane-detection/lane_detect.engine" || FAILED+=(lane); }

want face && { build face \
    "$GRAPHS/face-detection/yolov11n-face.onnx" \
    "$GRAPHS/face-detection/yolov11n-facefp16.engine" || FAILED+=(face); }

want gaze && { build gaze \
    "$GRAPHS/gaze-estimation/resnet18_gaze.onnx" \
    "$GRAPHS/gaze-estimation/resnet18_gaze_fp16.engine" \
    --minShapes=input:1x3x448x448 \
    --optShapes=input:1x3x448x448 \
    --maxShapes=input:1x3x448x448 || FAILED+=(gaze); }

# The production FFS ONNX needs no custom plugin (the plugin-based variant
# and its libgwc_plugin.so are the legacy path under archive/).
want stereo && { build stereo \
    "$REPO_ROOT/archive/src/graphs/stereo-depth/fast_foundationstereo.onnx" \
    "$GRAPHS/stereo-depth/fast_foundationstereo.engine" \
    --memPoolSize=workspace:4096 || FAILED+=(stereo); }

echo
if [[ ${#FAILED[@]} -gt 0 ]]; then
    echo "Failed: ${FAILED[*]}" >&2
    exit 1
fi
echo "Engines rebuilt. Previous engines were kept as <name>.prev.${STAMP};"
echo "delete them once the new ones are verified, then rebuild the workspace:"
echo "  colcon build --packages-select visionconnect"
