# Camera Setup - VisionSense

## Camera Configuration

VisionSense uses two cameras with guaranteed persistent device assignments:

1. **Mono Camera (Driver Monitoring)**: IMX219 camera at I2C address 10-0010
   - Device: `/dev/video0` (V4L2) / `csi://0` (Argus/jetson-utils)
   - Symlink: `/dev/video-mono` → `/dev/video0`
   - Resolution: 1920x1080 (also supports 1640x1232, 3280x2464)
   - Purpose: Driver monitoring and interior cabin view

2. **Stereo Camera (Depth Perception)**: Arducam stereo camera at I2C address 9-000c
   - Device: `/dev/video1` (V4L2 only, not Argus-compatible)
   - Symlink: `/dev/video-stereo` → `/dev/video1`
   - Resolution: 3840x1200 (side-by-side stereo: 1920x1200 per eye)
   - Purpose: Depth estimation and 3D scene understanding

## Persistent Device Names

To ensure camera assignments remain consistent across reboots, udev rules have been configured:

**Rule File**: `/etc/udev/rules.d/99-camera-persistent.rules`

```
# IMX219 mono camera (driver monitoring)
SUBSYSTEM=="video4linux", ATTR{name}=="vi-output, imx219 10-0010", SYMLINK+="video-mono", GROUP="video", MODE="0666"

# Arducam stereo camera (depth perception)
SUBSYSTEM=="video4linux", ATTR{name}=="vi-output, arducam-csi2 9-000c", SYMLINK+="video-stereo", GROUP="video", MODE="0666"
```

### How It Works

The udev rules identify cameras by their hardware names (which include I2C addresses):
- IMX219 at I2C 0x10 → identified as "imx219 10-0010" → `/dev/video0` (V4L2) / `csi://0` (Argus)
- Arducam at I2C 0x0c → identified as "arducam-csi2 9-000c" → `/dev/video1` (V4L2 only)

The rules also create convenient symlinks (`/dev/video-mono` and `/dev/video-stereo`) for reference, though the actual configuration uses the numbered devices for compatibility with jetson-utils.

## Configuration Files

### Main Config: `config/config.yaml`

```yaml
camera:
    ros__parameters:
        resource: "csi://0"  # IMX219 mono camera - uses Argus/nvarguscamerasrc
        width: 1920
        height: 1080

camera_stereo:
    ros__parameters:
        resource: "/dev/video1"  # Arducam stereo camera - V4L2 only
        width: 3840
        height: 1200
```

**Note**: The IMX219 uses `csi://0` (Argus sensor index). The Arducam stereo camera is V4L2-only and uses `/dev/video1`.

### Launch Files

All launch files have been updated to use correct device indices:
- `launch/camera_stereo_test.launch.py` - Uses config.yaml (reads `/dev/video1`)
- `launch/camera_stereo_ess.launch.py` - Default: `/dev/video1` (can override with camera_device argument)
- `launch/stereo_camera.launch.py` - Default: `/dev/video1` (can override with device argument)

## Verification

To verify the camera setup:

```bash
# List all video devices
v4l2-ctl --list-devices

# Check mono camera
v4l2-ctl --device=/dev/video-mono --info

# Check stereo camera
v4l2-ctl --device=/dev/video-stereo --info

# Verify symlinks
ls -la /dev/video*
```

## Troubleshooting

If camera assignments change after reboot:

1. Check if udev rules are present:
   ```bash
   ls -la /etc/udev/rules.d/99-camera-persistent.rules
   ```

2. Reload udev rules:
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

3. Verify symlinks were created:
   ```bash
   ls -la /dev/video-mono /dev/video-stereo
   ```

## Hardware Connections

- **Mono Camera (IMX219)**: Connected to CSI port with I2C address 0x10
- **Stereo Camera (Arducam)**: Connected to CSI port with I2C address 0x0c

**Important**: The Argus sensor index (`csi://0`) corresponds to the only Argus-compatible camera (IMX219). The Arducam stereo is V4L2-only and not visible to Argus.
