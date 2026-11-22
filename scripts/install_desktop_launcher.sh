#!/bin/bash
# Install the VisionSense desktop launcher for the current user.
#
# Double-clicking it runs launch_visionsense.sh in a terminal: the full
# pipeline, plus the dashboard in a Chromium kiosk when ui.dashboard is true
# in config/config.yaml. Closing the terminal (or Ctrl+C) stops everything.
#
# Usage: bash scripts/install_desktop_launcher.sh

set -e

VS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ICON="$VS_DIR/launch/dashboard/assets/Logo_Symbol_Dark.png"
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
APPS_DIR="$HOME/.local/share/applications"

mkdir -p "$DESKTOP_DIR" "$APPS_DIR"

for dst in "$DESKTOP_DIR/VisionSense.desktop" "$APPS_DIR/VisionSense.desktop"; do
    cat > "$dst" << DESKTOP
[Desktop Entry]
Type=Application
Version=1.0
Name=VisionSense
Comment=Run the VisionSense pipeline and dashboard
Exec=$VS_DIR/launch_visionsense.sh
Path=$VS_DIR
Icon=$ICON
Terminal=true
Categories=Science;Robotics;
StartupNotify=false
DESKTOP
    chmod +x "$dst"
done

# GNOME only runs a desktop file it trusts ("Allow Launching").
gio set "$DESKTOP_DIR/VisionSense.desktop" metadata::trusted true 2>/dev/null || true

echo "✓ VisionSense launcher installed: $DESKTOP_DIR/VisionSense.desktop"
