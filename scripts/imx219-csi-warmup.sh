#!/bin/bash
# imx219-csi-warmup.sh — pre-flight for visionsense-imx219.service.
#
# At fresh boot, opening IMX219 through nvargus fails (~5s in) with
#   SCF: Error ResourceError: Error 15 Received for sensor 1
#   PD_CRC_ERR / Argus Correctable Error Status
# even though only sensor 0 is being requested. Argus probes ALL CSI sensors
# during a session, and the Arducam (sensor 1) misbehaves on probe until its
# V4L2 driver has done at least one stream-on/stream-off cycle in the current
# boot. Once that's happened, Argus's sensor 1 probe stays clean — IMX219
# captures run indefinitely without the cascading CRC errors.
#
# This script does that one-time V4L2 warmup so the persistent IMX219 service
# can claim the Argus stack cleanly at boot, without depending on the user
# launching VisionSense first.
#
# Safe to run repeatedly. Exits 0 even on failure — failing to warm up the
# Arducam shouldn't block the IMX219 service from trying anyway.

set +e

DEV=/dev/video1
W=3840
H=1200
FMT=UYVY
FRAMES=5

# Wait briefly for /dev/video1 to appear (boot timing slop).
for _ in 1 2 3 4 5; do
    [ -c "$DEV" ] && break
    sleep 1
done

if [ ! -c "$DEV" ]; then
    echo "imx219-csi-warmup: $DEV missing — skipping warmup"
    exit 0
fi

# If something else already owns the device, skip — VisionSense's
# camera_stereo node is already streaming and the CSI is already warm.
if fuser -s "$DEV" 2>/dev/null; then
    echo "imx219-csi-warmup: $DEV in use — skipping warmup"
    exit 0
fi

# Brief stream-on/stream-off cycle on the Arducam. v4l2-ctl exits cleanly
# after --stream-count frames so the kernel sees a proper STREAMOFF (no
# wedge — see project memory feedback_camera_shutdown).
v4l2-ctl -d "$DEV" \
    --set-fmt-video=width=$W,height=$H,pixelformat=$FMT \
    --stream-mmap=4 --stream-count=$FRAMES \
    >/dev/null 2>&1
rc=$?
if [ $rc -eq 0 ]; then
    echo "imx219-csi-warmup: $DEV warmed ($FRAMES frames @ ${W}x${H} $FMT)"
else
    echo "imx219-csi-warmup: $DEV warmup returned $rc (continuing anyway)"
fi

# Tiny settle so the CSI port is fully idle before Argus opens it.
sleep 1
exit 0
