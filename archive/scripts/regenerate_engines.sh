#!/usr/bin/env bash
#
# Regenerate every TensorRT engine in this repo from its ONNX source.
#
# Why this is needed: TensorRT engines are not portable across GPU SM,
# TensorRT version, or CUDA driver. Whenever any of those change (Jetson
# reflash, JetPack update, swapping device), every .engine in the tree must
# be rebuilt on the target machine before the project will run.
#
# What it builds:
#   - object-detection/detect.engine               (YOLOv8, FP16, 1x3x640x640)
#   - sign-classifier/classify.engine              (YOLOv8 cls, FP16, dyn batch 1-20)
#   - lane-detection/lane_detect.engine            (ERFNet lane, FP16, 1x3x208x976)
#   - face-detection/yolov11n-facefp16.engine      (YOLOv11n-face, FP16, 1x3x640x640)
#   - gaze-estimation/resnet18_gaze_fp16.engine    (ResNet18 gaze, FP16, 1x3x448x448)
#   - stereo-depth/libgwc_plugin.so                (GWC custom TRT plugin, sm_87)
#   - stereo-depth/stereo_plugin_320_iters4.engine (Fast-FoundationStereo, FP16,
#                                                   320x320 input, 4 GRU iters,
#                                                   builderOptimizationLevel 5)
#
# Existing engine files are renamed to <name>.prev.<TIMESTAMP> before the new
# one is written, so a failed build never leaves you without a working engine.
#
# Total build time on Orin NX 16GB MAXN_SUPER + jetson_clocks: ~20 min.
# (~10 min of that is the FFS engine with --builderOptimizationLevel=5.)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GRAPHS="$REPO_ROOT/src/graphs"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
STAMP="$(date +%Y%m%d-%H%M%S)"

if [[ ! -x "$TRTEXEC" ]]; then
    echo "ERROR: trtexec not found at $TRTEXEC. Set TRTEXEC env var or install TensorRT." >&2
    exit 1
fi

backup_engine() {
    local f="$1"
    [[ -f "$f" ]] && mv -v "$f" "${f}.prev.${STAMP}"
}

build_static_fp16() {
    local onnx="$1" engine="$2"
    echo "==> $(basename "$engine") <- $(basename "$onnx") (static FP16)"
    backup_engine "$engine"
    "$TRTEXEC" --onnx="$onnx" --saveEngine="$engine" --fp16
}

#------------------------------------------------------------------------------
# 1. Object detection (YOLOv8) - static 1x3x640x640
#------------------------------------------------------------------------------
build_static_fp16 \
    "$GRAPHS/object-detection/yolov8-detect.onnx" \
    "$GRAPHS/object-detection/detect.engine"

#------------------------------------------------------------------------------
# 2. Sign classifier - dynamic batch (1, 5, 20)
#------------------------------------------------------------------------------
echo "==> classify.engine <- classify_dynamic.onnx (dynamic batch 1-20)"
backup_engine "$GRAPHS/sign-classifier/classify.engine"
"$TRTEXEC" \
    --onnx="$GRAPHS/sign-classifier/classify_dynamic.onnx" \
    --saveEngine="$GRAPHS/sign-classifier/classify.engine" \
    --fp16 \
    --minShapes=images:1x3x320x320 \
    --optShapes=images:5x3x320x320 \
    --maxShapes=images:20x3x320x320

#------------------------------------------------------------------------------
# 3. Lane detection (ERF lane post-processed ONNX) - static 1x3x208x976
#------------------------------------------------------------------------------
build_static_fp16 \
    "$GRAPHS/lane-detection/ERF_lane_post.onnx" \
    "$GRAPHS/lane-detection/lane_detect.engine"

#------------------------------------------------------------------------------
# 4. Face detection (YOLOv11n) - static 1x3x640x640
#------------------------------------------------------------------------------
build_static_fp16 \
    "$GRAPHS/face-detection/yolov11n-face.onnx" \
    "$GRAPHS/face-detection/yolov11n-facefp16.engine"

#------------------------------------------------------------------------------
# 5. Gaze estimation (ResNet18) - dynamic input shape, all batch=1
#------------------------------------------------------------------------------
echo "==> resnet18_gaze_fp16.engine <- resnet18_gaze.onnx (dynamic, batch=1)"
backup_engine "$GRAPHS/gaze-estimation/resnet18_gaze_fp16.engine"
"$TRTEXEC" \
    --onnx="$GRAPHS/gaze-estimation/resnet18_gaze.onnx" \
    --saveEngine="$GRAPHS/gaze-estimation/resnet18_gaze_fp16.engine" \
    --fp16 \
    --minShapes=input:1x3x448x448 \
    --optShapes=input:1x3x448x448 \
    --maxShapes=input:1x3x448x448

#------------------------------------------------------------------------------
# 6. Fast-FoundationStereo - GWC custom plugin + opt level 5
#------------------------------------------------------------------------------
echo "==> libgwc_plugin.so + stereo_plugin_320_iters4.engine"
cd "$GRAPHS/stereo-depth"

# Build the custom GWC plugin for sm_87 (Orin). Edit ARCH in the Makefile if
# you are targeting a different Jetson.
make libgwc_plugin.so

backup_engine "$GRAPHS/stereo-depth/stereo_plugin_320_iters4.engine"
"$TRTEXEC" \
    --onnx=stereo_combined_320_iters4.onnx \
    --saveEngine=stereo_plugin_320_iters4.engine \
    --staticPlugins=./libgwc_plugin.so \
    --fp16 \
    --builderOptimizationLevel=5 \
    --memPoolSize=workspace:8192

cd "$REPO_ROOT"

echo
echo "All engines rebuilt successfully."
echo "Previous engines saved as <name>.prev.${STAMP}; remove them once you have"
echo "verified the new ones work."
