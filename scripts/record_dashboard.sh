#!/usr/bin/env bash
# record_dashboard.sh — low-overhead screen recording for the VisionSense dashboard.
#
# Uses the Orin's dedicated NVENC hardware encoder (nvv4l2h264enc), which is a
# separate silicon block from the CUDA/GPU cores VisionSense's TensorRT inference
# uses. That's why this won't drop frames the way GNOME's built-in software
# screencast (Ctrl+Alt+Shift+R) does under VisionSense load.
#
# Usage:
#   ./record_dashboard.sh                 # records full screen :0 to ~/recordings/<timestamp>.mp4
#   ./record_dashboard.sh my_clip.mp4     # custom output path
#   BITRATE=20000000 ./record_dashboard.sh   # bump bitrate (bits/sec) for crisper text
#   FPS=60 ./record_dashboard.sh          # 60 fps capture
#
# Stop with Ctrl+C — the script forwards a clean EOS so the .mp4 is finalized
# (a hard kill leaves an unplayable file with no moov atom).
#
# Region capture: uncomment the startx/starty/endx/endy line below to grab a
# sub-rectangle instead of the whole screen.
set -euo pipefail

OUT="${1:-$HOME/recordings/dashboard_$(date +%Y%m%d_%H%M%S).mp4}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-12000000}"   # 12 Mbps — good for dashboard text legibility
DISPLAY_NAME="${DISPLAY:-:0}"

mkdir -p "$(dirname "$OUT")"

echo "Recording $DISPLAY_NAME @ ${FPS}fps, ${BITRATE} bps -> $OUT"
echo "Press Ctrl+C to stop."

# exec so Ctrl+C (SIGINT) hits gst-launch directly; -e turns it into a clean EOS.
exec gst-launch-1.0 -e \
  ximagesrc display-name="$DISPLAY_NAME" use-damage=0 \
    `# startx=0 starty=0 endx=1919 endy=1079  # <- uncomment for region capture` \
  ! "video/x-raw,framerate=${FPS}/1" \
  ! nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' \
  ! nvv4l2h264enc bitrate="$BITRATE" maxperf-enable=1 insert-sps-pps=1 \
  ! h264parse ! qtmux ! filesink location="$OUT"
