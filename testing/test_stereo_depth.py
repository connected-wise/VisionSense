#!/usr/bin/env python3
"""
Standalone test script for stereo depth (disparity) model.
Tests the TensorRT engine without requiring ROS2.

Usage:
    python3 test_stereo_depth.py [--left LEFT_IMG] [--right RIGHT_IMG] [--engine ENGINE_PATH]

Example:
    python3 test_stereo_depth.py --left 000003-left.png --right 000003-right.png
"""

import argparse
import time
import os
import sys
import ctypes
import numpy as np
import cv2

try:
    import tensorrt as trt
except ImportError as e:
    print(f"Error importing TensorRT: {e}")
    print("Make sure TensorRT Python bindings are installed.")
    sys.exit(1)


# Load CUDA runtime library
def load_cuda_library():
    """Load CUDA runtime library for memory operations."""
    cuda_paths = [
        "/usr/local/cuda/lib64/libcudart.so",
        "/usr/lib/aarch64-linux-gnu/libcudart.so",
        "/usr/lib/x86_64-linux-gnu/libcudart.so",
        "libcudart.so",
    ]
    for path in cuda_paths:
        try:
            return ctypes.CDLL(path)
        except OSError:
            continue
    raise RuntimeError("Could not load libcudart.so")


cudart = load_cuda_library()

# CUDA memory operations
cudart.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
cudart.cudaMalloc.restype = ctypes.c_int
cudart.cudaFree.argtypes = [ctypes.c_void_p]
cudart.cudaFree.restype = ctypes.c_int
cudart.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
cudart.cudaMemcpy.restype = ctypes.c_int
cudart.cudaDeviceSynchronize.argtypes = []
cudart.cudaDeviceSynchronize.restype = ctypes.c_int

CUDA_MEMCPY_HOST_TO_DEVICE = 1
CUDA_MEMCPY_DEVICE_TO_HOST = 2


# ImageNet normalization parameters (must match C++ preprocessor)
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class TRTLogger(trt.ILogger):
    def __init__(self):
        trt.ILogger.__init__(self)

    def log(self, severity, msg):
        if severity <= trt.ILogger.WARNING:
            print(f"[TRT] {msg}")


def cuda_malloc(size):
    """Allocate CUDA device memory."""
    ptr = ctypes.c_void_p()
    err = cudart.cudaMalloc(ctypes.byref(ptr), size)
    if err != 0:
        raise RuntimeError(f"cudaMalloc failed with error {err}")
    return ptr


def cuda_free(ptr):
    """Free CUDA device memory."""
    cudart.cudaFree(ptr)


def cuda_memcpy_htod(dst, src_array):
    """Copy from host numpy array to device."""
    src_ptr = src_array.ctypes.data_as(ctypes.c_void_p)
    size = src_array.nbytes
    err = cudart.cudaMemcpy(dst, src_ptr, size, CUDA_MEMCPY_HOST_TO_DEVICE)
    if err != 0:
        raise RuntimeError(f"cudaMemcpy H2D failed with error {err}")


def cuda_memcpy_dtoh(dst_array, src):
    """Copy from device to host numpy array."""
    dst_ptr = dst_array.ctypes.data_as(ctypes.c_void_p)
    size = dst_array.nbytes
    err = cudart.cudaMemcpy(dst_ptr, src, size, CUDA_MEMCPY_DEVICE_TO_HOST)
    if err != 0:
        raise RuntimeError(f"cudaMemcpy D2H failed with error {err}")


def cuda_synchronize():
    """Synchronize CUDA device."""
    cudart.cudaDeviceSynchronize()


def preprocess_image(img_bgr, target_h, target_w):
    """
    Preprocess image to match LightStereo model requirements.
    Matches the C++ StereoPreprocessor exactly.

    Steps:
    1. Convert BGR -> RGB
    2. Resize maintaining aspect ratio to fit target
    3. RightTopPad with edge replication (pad top and right)
    4. Transpose HWC -> CHW and normalize with ImageNet mean/std
    """
    # Step 1: BGR to RGB
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

    in_h, in_w = img_rgb.shape[:2]

    # Step 2: Resize to fit within target while maintaining aspect ratio
    scale = min(target_h / in_h, target_w / in_w)
    new_h = int(in_h * scale)
    new_w = int(in_w * scale)

    resized = cv2.resize(img_rgb, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    # Step 3: RightTopPad to target size (pad top and right with edge replication)
    pad_top = target_h - new_h
    pad_right = target_w - new_w

    padded = cv2.copyMakeBorder(
        resized,
        pad_top, 0,      # top, bottom
        0, pad_right,    # left, right
        cv2.BORDER_REPLICATE
    )

    # Step 4: Convert to float32, normalize, transpose to CHW
    img_float = padded.astype(np.float32) / 255.0

    # Normalize with ImageNet mean/std
    img_normalized = (img_float - MEAN) / STD

    # Transpose HWC -> CHW
    img_chw = np.transpose(img_normalized, (2, 0, 1))

    # Add batch dimension and ensure contiguous
    return np.ascontiguousarray(img_chw.reshape(1, 3, target_h, target_w), dtype=np.float32)


class StereoTRTEngine:
    """TensorRT engine wrapper for stereo depth inference."""

    def __init__(self, engine_path):
        self.logger = TRTLogger()

        print(f"Loading TensorRT engine: {engine_path}")

        # Load and deserialize engine
        with open(engine_path, 'rb') as f:
            engine_data = f.read()

        runtime = trt.Runtime(self.logger)
        self.engine = runtime.deserialize_cuda_engine(engine_data)

        if self.engine is None:
            raise RuntimeError("Failed to deserialize TensorRT engine")

        self.context = self.engine.create_execution_context()

        # Get tensor info
        self.left_name = "left_img"
        self.right_name = "right_img"
        self.output_name = "disp_pred"

        # Get tensor shapes
        left_shape = self.engine.get_tensor_shape(self.left_name)
        right_shape = self.engine.get_tensor_shape(self.right_name)
        output_shape = self.engine.get_tensor_shape(self.output_name)

        self.input_h = left_shape[2]
        self.input_w = left_shape[3]
        self.output_h = output_shape[-2]
        self.output_w = output_shape[-1]

        print(f"  Input shape: {list(left_shape)} (H={self.input_h}, W={self.input_w})")
        print(f"  Output shape: {list(output_shape)} (H={self.output_h}, W={self.output_w})")

        # Calculate sizes
        self.input_size = int(np.prod(left_shape))
        self.output_size = int(np.prod(output_shape))
        self.output_shape = tuple(output_shape)

        # Allocate device memory
        input_bytes = self.input_size * 4  # float32
        output_bytes = self.output_size * 4

        self.d_left = cuda_malloc(input_bytes)
        self.d_right = cuda_malloc(input_bytes)
        self.d_output = cuda_malloc(output_bytes)

        # Set tensor addresses
        self.context.set_tensor_address(self.left_name, self.d_left.value)
        self.context.set_tensor_address(self.right_name, self.d_right.value)
        self.context.set_tensor_address(self.output_name, self.d_output.value)

        # Output buffer
        self.h_output = np.empty(self.output_shape, dtype=np.float32)

        print("Engine loaded successfully!")

    def infer(self, left_data, right_data):
        """Run inference on preprocessed left/right image data."""
        # Copy inputs to device
        cuda_memcpy_htod(self.d_left, left_data)
        cuda_memcpy_htod(self.d_right, right_data)

        # Run inference (execute_v2 is synchronous)
        success = self.context.execute_v2([self.d_left.value, self.d_right.value, self.d_output.value])
        if not success:
            raise RuntimeError("TensorRT execution failed")

        # Copy output back
        cuda_memcpy_dtoh(self.h_output, self.d_output)
        cuda_synchronize()

        return self.h_output.copy()

    def warmup(self, left_data, right_data, iterations=5):
        """Warm up the engine with dummy inference."""
        print(f"Warming up engine ({iterations} iterations)...")
        for i in range(iterations):
            self.infer(left_data, right_data)
        print("Warmup complete!")

    def __del__(self):
        """Cleanup CUDA resources."""
        if hasattr(self, 'd_left'):
            cuda_free(self.d_left)
        if hasattr(self, 'd_right'):
            cuda_free(self.d_right)
        if hasattr(self, 'd_output'):
            cuda_free(self.d_output)


def postprocess_disparity(disparity, max_disparity=192.0):
    """Convert raw disparity to 8-bit visualization."""
    # Squeeze any extra dimensions
    if disparity.ndim > 2:
        disparity = disparity.squeeze()

    # Normalize to 0-255 range
    disp_normalized = np.clip(disparity / max_disparity, 0, 1)
    disp_8u = (disp_normalized * 255).astype(np.uint8)

    return disp_8u


def apply_colormap(disparity_8u):
    """Apply Jet colormap for visualization."""
    return cv2.applyColorMap(disparity_8u, cv2.COLORMAP_JET)


def main():
    parser = argparse.ArgumentParser(description="Test stereo depth model")
    parser.add_argument("--left", type=str, default="000003-left.png",
                        help="Left image path (relative to testing/ or absolute)")
    parser.add_argument("--right", type=str, default="000003-right.png",
                        help="Right image path (relative to testing/ or absolute)")
    parser.add_argument("--engine", type=str,
                        default="../src/graphs/stereo-depth/LightStereo-S-KITTI.engine",
                        help="TensorRT engine path (relative to testing/ or absolute)")
    parser.add_argument("--max-disparity", type=float, default=192.0,
                        help="Maximum disparity value for normalization")
    parser.add_argument("--warmup", type=int, default=5,
                        help="Number of warmup iterations")
    parser.add_argument("--output", type=str, default="disparity_output.png",
                        help="Output disparity image path")
    parser.add_argument("--no-display", action="store_true",
                        help="Skip display window (headless mode)")

    args = parser.parse_args()

    # Get script directory for relative paths
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Resolve paths
    def resolve_path(p):
        if os.path.isabs(p):
            return p
        return os.path.join(script_dir, p)

    left_path = resolve_path(args.left)
    right_path = resolve_path(args.right)
    engine_path = resolve_path(args.engine)
    output_path = resolve_path(args.output)

    # Validate files exist
    print("=" * 60)
    print("Stereo Depth Model Test")
    print("=" * 60)

    for path, name in [(left_path, "Left image"), (right_path, "Right image"),
                       (engine_path, "Engine")]:
        if not os.path.exists(path):
            print(f"ERROR: {name} not found: {path}")
            sys.exit(1)
        print(f"{name}: {path}")

    print(f"Output: {output_path}")
    print(f"Max disparity: {args.max_disparity}")
    print("=" * 60)

    # Load images
    print("\n[1/5] Loading images...")
    left_img = cv2.imread(left_path)
    right_img = cv2.imread(right_path)

    if left_img is None:
        print(f"ERROR: Failed to load left image: {left_path}")
        sys.exit(1)
    if right_img is None:
        print(f"ERROR: Failed to load right image: {right_path}")
        sys.exit(1)

    print(f"  Left image:  {left_img.shape}")
    print(f"  Right image: {right_img.shape}")

    # Load TensorRT engine
    print("\n[2/5] Loading TensorRT engine...")
    engine = StereoTRTEngine(engine_path)

    # Preprocess images
    print("\n[3/5] Preprocessing images...")
    t0 = time.perf_counter()
    left_data = preprocess_image(left_img, engine.input_h, engine.input_w)
    right_data = preprocess_image(right_img, engine.input_h, engine.input_w)
    preprocess_time = (time.perf_counter() - t0) * 1000
    print(f"  Preprocessed to: {left_data.shape}")
    print(f"  Preprocess time: {preprocess_time:.2f} ms")

    # Warmup
    print(f"\n[4/5] Warmup ({args.warmup} iterations)...")
    engine.warmup(left_data, right_data, args.warmup)

    # Run inference
    print("\n[5/5] Running inference...")
    t0 = time.perf_counter()
    disparity = engine.infer(left_data, right_data)
    inference_time = (time.perf_counter() - t0) * 1000

    print(f"  Raw disparity shape: {disparity.shape}")
    print(f"  Disparity range: [{disparity.min():.2f}, {disparity.max():.2f}]")
    print(f"  Inference time: {inference_time:.2f} ms")
    print(f"  FPS: {1000 / inference_time:.1f}")

    # Postprocess
    disp_8u = postprocess_disparity(disparity, args.max_disparity)
    disp_color = apply_colormap(disp_8u)

    # Save output
    cv2.imwrite(output_path, disp_color)
    print(f"\nDisparity saved to: {output_path}")

    # Also save grayscale version
    gray_output = output_path.replace('.png', '_gray.png')
    cv2.imwrite(gray_output, disp_8u)
    print(f"Grayscale disparity saved to: {gray_output}")

    # Display results
    if not args.no_display:
        print("\nDisplaying results (press any key to close)...")

        # Create side-by-side visualization
        h, w = disp_color.shape[:2]
        left_resized = cv2.resize(left_img, (w, h))
        right_resized = cv2.resize(right_img, (w, h))

        # Stack horizontally: left | right | disparity
        combined = np.hstack([left_resized, right_resized, disp_color])

        # Add labels
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(combined, "Left", (10, 30), font, 1, (0, 255, 0), 2)
        cv2.putText(combined, "Right", (w + 10, 30), font, 1, (0, 255, 0), 2)
        cv2.putText(combined, f"Disparity ({inference_time:.1f}ms)", (2*w + 10, 30), font, 1, (0, 255, 0), 2)

        # Resize for display if too large
        max_display_w = 1920
        if combined.shape[1] > max_display_w:
            scale = max_display_w / combined.shape[1]
            combined = cv2.resize(combined, None, fx=scale, fy=scale)

        cv2.imshow("Stereo Depth Test", combined)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    print("\n" + "=" * 60)
    print("TEST COMPLETE")
    print("=" * 60)
    print(f"  Preprocess time: {preprocess_time:.2f} ms")
    print(f"  Inference time:  {inference_time:.2f} ms")
    print(f"  Total time:      {preprocess_time + inference_time:.2f} ms")
    print(f"  Output saved to: {output_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
