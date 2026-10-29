# Changelog

## Unreleased

### Tested

- `VisionSense_NX_2F1R` on a Jetson Orin NX 8GB Super (p3767-0001) with
  JetPack 7.2.1 (L4T R39.2.1) in MAXN_SUPER, an IMX477 on CAM0 and an
  Arducam B0577 AR0234 sync stereo on CAM1 over extension cables. The camera
  overlay, the dashboard's NVENC encoder (`nvv4l2h264enc`) and all TensorRT
  engines (rebuilt on the NX) work without code changes. The stereo pair
  streamed 3840x1200 at 30 fps for 5 minutes with no errors after the two
  frames the VI discards at stream start.
- An Orin Nano SSD moved to an Orin NX needs its boot entry pointed at the
  NX device tree (`kernel_tegra234-p3768-0000+p3767-0001-nv-super.dtb`) and
  `/etc/nvpmodel.conf` linked to `nvpmodel_p3767_0001_super.conf`; reflashing
  QSPI alone leaves both set for the Nano.

### Fixed

- The stereo camera is opened by its udev name `/dev/video-stereo`, which the
  launch file resolves to the real `/dev/videoN`, so a boot that numbers the
  video devices differently no longer opens the wrong camera.
- `camera_stereo` restarts 3 s after it exits (for example when the stream
  stops), which re-sends stream-on to the Arducam adapter.

## v2.0.0 (2026-10-27)

VisionSense on JetPack 7.2: Ubuntu 24.04, ROS 2 Jazzy, CUDA 13.2,
TensorRT 10.16, VPI 4. JetPack 6.2 (Ubuntu 22.04, ROS 2 Humble) stays
supported by the same code.

Tested on a Jetson Orin Nano Super 8 GB with JetPack 7.2.1 (L4T R39.2.1),
an IMX477 on CAM0 and an Arducam B0577 AR0234 sync stereo on CAM1.

### Hardware variants

Release branches per rig, `VisionSense_<board>_<layout>`:

- `VisionSense_Nano_2F1R`: Orin Nano, stereo forward plus a driver camera.
- `VisionSense_Nano_3F0R`: Orin Nano, stereo plus mono camera all forward.
- `VisionSense_NX_2F1R`, `VisionSense_NX_3F0R`: Orin NX placeholders. They
  carry the Nano code and have not been adapted or tested on an Orin NX.

The new `hardware.layout` setting (`2F1R` or `3F0R`) in `config/config.yaml`
picks the layout. In 3F0R, detection, signs, lanes, ADAS, BEV and the fused
view run on the mono camera, the stereo pair produces depth only, and the
driver monitor is off.

### Added

- IMX477 + AR0234 combined device-tree overlay
  (`overlays/tegra234-p3767-camera-p3768-arducam-imx477-combined.dts`),
  verified on R36 and R39. `install_all_deps.sh` takes
  `VS_DRIVER_CAM=imx219|imx477`.
- Persistent camera names `/dev/video-mono` and `/dev/video-stereo`
  (`scripts/99-camera-persistent.rules`).
- `scripts/regenerate_engines.sh`: rebuilds every TensorRT engine the launch
  graph uses (detection, sign classifier, lanes, face, gaze,
  Fast-FoundationStereo) for the installed TensorRT, keeping the previous
  engines as backups.
- `scripts/patch_jetson_inference_cuda13.sh`: makes jetson-inference build
  with CUDA 13 and GCC 13.

### Changed

- `install_all_deps.sh` picks ROS 2 Humble or Jazzy from the Ubuntu release,
  installs the services for the user running it instead of `jetson`, and on
  JetPack 7.2 installs Arducam's T234 kernel package (its installer has no
  Orin Nano entry for R39). The boot entry sets `FDT` explicitly, because the
  UEFI launcher ignores `OVERLAYS` without it.
- Build: no VPI 3 requirement, CUDA code built for sm_87, ROS include paths
  follow `ROS_DISTRO`, and the `common` library declares its ROS
  dependencies (needed on Jazzy).
- The launch scripts and the camera service find the checkout and ROS distro
  themselves instead of assuming `/home/jetson` and Humble.
- The driver-monitor engine paths in `config/config.yaml` are relative to the
  installed `graphs` directory.

### Fixed

- Dashboard on boards without NVENC: WebRTC falls back to software `x264enc`
  when `nvv4l2h264enc` is missing (the Orin Nano has no NVENC, and JetPack 7.2
  does not ship the element). Before, the dashboard exited at start-up.
- Dashboard crash on Ubuntu 24.04 when a browser opened a stream (libsoup2
  from jetson-utils and libsoup3 from webrtcbin in one process). The dashboard
  no longer links jetson-utils.

### Upgrading from JetPack 6.2

The checked-in `.engine` files are TensorRT 10.3 builds. On JetPack 7.2, run
`bash scripts/regenerate_engines.sh`, then rebuild the workspace.

### Known issues

- Software H.264 encoding uses about 75% of all six CPU cores on an Orin Nano
  while a browser views the dashboard, which slows inference.
- On shutdown most nodes crash during Fast DDS shared-memory cleanup instead
  of exiting cleanly. The cameras are not affected.
- 3F0R: lane detection crop, ADAS lane centre and BEV intrinsics still need
  tuning for a forward-mounted IMX477.
