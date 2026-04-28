# Build the standalone Fast-FoundationStereo C++ TRT demo on Jetson Orin.
# Requires: CUDA, TensorRT (10.x), OpenCV 4 with GStreamer support.

NVCC      ?= /usr/local/cuda/bin/nvcc
ARCH      ?= -gencode arch=compute_87,code=sm_87

CXXFLAGS  := -O3 -std=c++17
NVCCFLAGS := $(CXXFLAGS) $(ARCH) -Xcompiler -fPIC

INCLUDES  := -I/usr/local/cuda/include \
             -I/usr/include/aarch64-linux-gnu \
             $(shell pkg-config --cflags opencv4)

TRT_LIBS  := -L/usr/lib/aarch64-linux-gnu -lnvinfer
CUDA_LIBS := -L/usr/local/cuda/lib64 -lcudart
CV_LIBS   := $(shell pkg-config --libs opencv4)

all: stereo_trt libgwc_plugin.so

stereo_trt: stereo_trt.cu
	$(NVCC) $(NVCCFLAGS) $(INCLUDES) $< -o $@ $(CUDA_LIBS) $(TRT_LIBS) $(CV_LIBS) -ldl

libgwc_plugin.so: gwc_plugin.cu
	$(NVCC) $(NVCCFLAGS) --shared $(INCLUDES) $< -o $@ $(CUDA_LIBS) $(TRT_LIBS)

clean:
	rm -f stereo_trt libgwc_plugin.so

.PHONY: all clean
