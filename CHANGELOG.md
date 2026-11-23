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

### Added

- OzzMaker SARA-R5 LTE-M GPS + 10DOF support (`VS_GPS_MODULE=sara-r5`,
  `scripts/sara-r5/`). The modem and its GNSS share the 40-pin UART through
  CMUX: `sara-r5-mux` builds on the `n_gsm` line discipline, which JetPack 7.2
  does not ship, so `build_n_gsm.sh` builds it from kernel.org for the running
  kernel. PPP gives an LTE-M link, owned by the NetworkManager connection
  "LTE" so it shows in the network menu and takes over when Wi-Fi and
  Ethernet are down; gpsd reads the GNSS channel. Tested with Wi-Fi off:
  NetworkManager reports full connectivity and DNS, ping and HTTPS work
  over LTE-M while GNSS streams. With a Hologram SIM (APN `hologram`,
  roaming on Verizon LTE-M) a 520 KB web page loads in about 50 s. A Tello
  (T-Mobile) SIM also works, but the network caps it at 198 kbit/s and the
  same page took over 4 minutes.
- `imu_gps` reads the MMC5983MA magnetometer of the SARA-R5 board as well as
  the LIS3MDL of the BerryGPS-IMU.
- Road damage node (`road_damage`) for the 3F0R layout, which is now the
  fleet data collection variant: it segments potholes in the forward mono
  camera image with a YOLOv8s-seg model
  ([saadfrh/Pothole-Detection](https://github.com/saadfrh/Pothole-Detection),
  AGPL-3.0 weights) and takes the driver monitor's place in the launch and
  on the dashboard (**Road** view and state chip). Events are logged to
  `~/VisionSense_data/road_damage` as a CSV row with the GPS position plus the
  annotated frame. Build its engine with
  `bash scripts/regenerate_engines.sh road`. Tested on the Orin NX 8GB with
  sample pothole footage and with the live IMX477 (no false detections on a
  bench scene); not yet tested on a road.
- `camera.flip` in `config/config.yaml` for a mono camera mounted upside down
  (`"rotate-180"`); `VisionSense_NX_3F0R` sets it.
- `scripts/check_stereo_calib.py`: rectifies a live stereo pair with
  `config/stereo_calib.yaml` and reports the vertical offset of matched
  features, to tell whether the calibration still fits the camera.
- Desktop launcher (`scripts/install_desktop_launcher.sh`, run by
  `install_all_deps.sh`): a **VisionSense** icon that runs
  `launch_visionsense.sh` in a terminal and opens the dashboard.

### Changed

- With an IMX477 on CAM0, `visionsense.launch.py` starts the mono camera
  itself and `install_all_deps.sh` no longer installs (and removes)
  `visionsense-imx219.service`. The boot service was a workaround for the
  IMX219, which cannot be reopened through Argus; the IMX477 can. IMX219
  installs are unchanged.

- In 3F0R, detection, signs, lanes, ADAS, BEV and the fused view run on the
  stereo main eye again, as in 2F1R (v2.0.0 moved them to the mono camera).
  The mono camera is only for road damage, and
  `hardware.mono_calibration_file` is gone.
- Stereo baseline defaults are 95 mm, the lens spacing of the current pair:
  `camera_stereo.baseline_mm` (a fallback; depth takes the baseline from the
  calibration file) and the sanity check of `compute_stereo_calib.py`
  (`--baseline-mm`). `capture_stereo_calib.py` opens `/dev/video-stereo`.
- The dashboard's driver view is sent at 640x360 (was 640x480), the mono
  camera's 16:9 shape.

### Fixed

- Stereo depth was unusable with the current camera pair. The calibration in
  the repo belonged to an older assembly (about 107 px of vertical offset
  after rectification), and the two eyes were exchanged:
  `/camera_stereo/left` carried the right camera. `camera_stereo` has a new
  `swap_eyes` parameter (on in `config.yaml`), and `config/stereo_calib.yaml`
  is a new calibration of the current pair: 95 usable chessboard pairs,
  stereo RMS 0.53 px, baseline 94.97 mm. Checked on a live pair: about 1 px
  of vertical offset and positive disparity. `adas.ref_center_x_normalized`
  follows the new left eye's optical center (0.34, was 0.65) and still has
  to be confirmed on a road.
- Purple tint on the IMX477 mono camera with JetPack 7.2, whose ISP tuning
  subtracts too little black level and whose auto white balance has no
  effect. The `camera` node corrects it in software (`camera.white_balance`,
  on by default for an IMX477). Colors are neutral again but less saturated
  than from a correctly tuned ISP.
- Red and blue were swapped in the dashboard's stereo camera view: the
  dashboard encoded the cameras' RGB frames as BGR.
- The mono camera published BGR frames labeled `rgb8`, so the driver
  monitor, road damage and the dashboard all saw its red and blue exchanged.
  It now publishes real RGB.
- When Argus stopped delivering frames, the mono camera node logged a
  timeout every 10 s forever. It now exits after three timeouts and the
  launch file restarts it, which recovers the stream.
- The dashboard did not open on Ubuntu 24.04: the snap Chromium cannot write
  its kiosk profile to `~/.cache` and exited at once. The profile now goes in
  `~/snap/chromium/common` when the snap is installed.
- The dashboard opened in a normal window: `--kiosk` has no effect with that
  Chromium, so `scripts/x11_fullscreen.py` asks the window manager for
  fullscreen.
- `imu_gps` reported twice the real acceleration: the LSM6DSL was set to ±2 g
  but scaled as ±4 g. It is now set to ±4 g.
- Stereo frames green below the top, with every frame failing its payload
  CRC: the combined overlays set the AR0234 `pix_clk_hz` to 154 MHz, from
  which the driver clocked NVCSI at 39 MHz, too slow for 3840x1200 UYVY, so
  its RX FIFOs overflowed. It is now 320 MHz (Arducam's value), giving an
  80 MHz NVCSI clock. Reinstall the overlay and reboot to apply.
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
