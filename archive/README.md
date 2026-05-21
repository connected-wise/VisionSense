# archive/

Code paths the VisionSense project used to ship but no longer runs at runtime.
Nothing in here is referenced by `CMakeLists.txt`, the active launch files
(`launch/visionsense.launch.py`, `launch/test_stereo_depth_lightstereo.launch.py`),
or `config/config.yaml`. Moved here so the previous engineering work isn't lost
if a future switch back is needed; safe to delete the whole folder if you don't
want it.

The active stereo-depth backend is **LightStereo-S** (`scripts/stereo_depth_lightstereo.py`
+ `src/graphs/stereo-depth/lightstereo_s_320x512.engine`).

## What's here and why it was archived

### `scripts/`

| File | Why archived |
|---|---|
| `stereo_depth_monster.py` | RT-MonSter++ Python ROS node. 162 ms inference @ 960×576 — too slow vs LightStereo-S's 16 ms @ 320×512. |
| `depth_colorize.py` | Only consumer was `stereo_depth_ess.launch.py` (also archived). |
| `regenerate_engines.sh` | Rebuilds FFS / GWC plugin engines. None of those engines are used anymore. |

### `launch/`

| File | Why archived |
|---|---|
| `stereo_depth_ess.launch.py` | Isaac ROS NITROS ESS pipeline (RectifyNode → ResizeNode → ESS → DisparityToDepth). Abandoned because ESS produces geometrically wrong depth on the Arducam — the stereo pair isn't factory-rectified. |
| `test_stereo_depth_ess.launch.py` | A/B test wrapper for the above. |
| `test_stereo_depth.launch.py` | Test wrapper for the FFS C++ `stereo_depth` executable (also archived). |
| `test_stereo_depth_monster.launch.py` | Test wrapper for the MonSter Python node. |

### `src/`

| File | Why archived |
|---|---|
| `node_stereo_depth.cpp` | The Fast-FoundationStereo (FFS) C++ ROS node. Replaced by the LightStereo-S Python node. |
| `cuda/stereo_preproc.cu` | CUDA preprocessing kernel that was only linked against `node_stereo_depth.cpp`. |

### `src/graphs/stereo-depth/` — Fast-FoundationStereo artifacts

| File | Size | Why archived |
|---|---|---|
| `fast_foundationstereo.engine` | 32 MB | FFS single-ONNX TRT engine. |
| `fast_foundationstereo.onnx` | 54 MB | FFS exported ONNX. |
| `fast_foundationstereo.yaml` | <1 KB | FFS metadata. |
| `stereo_combined.onnx` | 60 MB | Older FFS export (max_disp 192). |
| `stereo_combined.yaml` | <1 KB | Older FFS metadata. |
| `stereo_combined_320_iters4.onnx` | 51 MB | Newer FFS export with iters=4. |
| `stereo_combined_320_iters4.yaml` | <1 KB | Newer FFS metadata. |
| `stereo_plugin.engine` | 39 MB | FFS engine built against the GWC plugin. |
| `stereo_plugin_320_iters4.engine` | 33 MB | Newer FFS plugin engine. |
| `gwc_plugin.cu` | 7 KB | Group-wise-correlation custom TRT plugin source. |
| `libgwc_plugin.so` | 54 KB | Compiled GWC plugin (sm_87). |
| `Makefile` | <1 KB | Builds `libgwc_plugin.so`. |
| `export_combined.py` | 9 KB | Script that produced the combined ONNX files. |
| `README.md` | 5 KB | Original FFS engine-dir README. |

## How to revive any of this

Files were moved with `git mv` where possible, so git tracks the rename and
`git log --follow archive/<path>` still shows the full history.

Restore a single file:

```bash
git mv archive/src/node_stereo_depth.cpp src/node_stereo_depth.cpp
# then add it back to CMakeLists.txt + the relevant launch file
```

Restore the whole archive:

```bash
git mv archive/* .
```

Permanently delete the archive:

```bash
git rm -r archive/
```

## What stayed (and why)

A few things look related but are **not** archived because they're load-bearing
for the active LightStereo path:

- `fastdds_profile.shm.xml` / `fastdds_profile.udp.xml` / `fastdds_profile.xml` —
  same-host SHM tuning for /camera_stereo/{left,right}/image_raw at 30 Hz.
- `src/cuda/stereo_rotate.cu` — camera_stereo's crop + split + 180° flip
  kernel (camera is mounted upside-down).
- `scripts/capture_stereo_calib.py`, `scripts/compute_stereo_calib.py`,
  `scripts/translate_calib.py`, `scripts/show_stereo_pair.py`,
  `scripts/reset_camera.sh` — stereo calibration tooling, still needed
  whenever the camera is re-mounted.
- `config/stereo_calib.yaml` — the calibration the LightStereo node uses for
  rectification + metric depth.
