#!/bin/bash
# VisionSense Launcher Script
# Kept for desktop icons that point here; the real launcher is the one at the
# repo root (DDS profile, stale-SHM sweep, driver-cam re-kick, ROS distro).
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/launch_visionsense.sh" "$@"
