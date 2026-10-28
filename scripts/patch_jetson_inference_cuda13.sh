#!/bin/bash
# patch_jetson_inference_cuda13.sh <jetson-inference checkout>
#
# Makes dusty-nv/jetson-inference (and its jetson-utils submodule) build on
# JetPack 7.2: Ubuntu 24.04 / GCC 13 / CUDA 13. Called by install_all_deps.sh
# on Ubuntu noble before cmake. Safe to run more than once.
#
#  1. jetson-utils: move the submodule to upstream master. The pinned commit's
#     Python-bindings CMake has no noble case: it probes 2.7/3.6/3.7, each
#     resolves to Python 3.12 and configure fails on a duplicate
#     jetson-utils-python-312 target.
#  2. CUDA 13 dropped sm_53/sm_62/sm_72, which both CMakeLists add on aarch64
#     ("nvcc fatal: Unsupported gpu architecture 'compute_53'"). Orin's sm_87
#     stays.
#  3. GCC 13 no longer pulls <cstdint> in transitively; add it to every header
#     that uses uint32_t etc. without including it. (A global -include in
#     CXXFLAGS also reaches nvcc's host pass and breaks the .cu builds.)
#  4. CUDA 13's NPP removed nppGetStreamContext(); fill the NppStreamContext
#     from the device attributes instead (NVIDIA's documented replacement).

set -e

JI_DIR="${1:?usage: $0 <jetson-inference dir>}"
cd "$JI_DIR"

# 1. jetson-utils with the noble Python case
if ! grep -q 'MATCHES "noble"' utils/python/CMakeLists.txt; then
    git -C utils fetch --depth=20 origin master
    git -C utils checkout FETCH_HEAD
fi

# 2. GPU architectures CUDA 13 no longer supports
sed -i -e 's/ -gencode arch=compute_53,code=sm_53 -gencode arch=compute_62,code=sm_62//' \
       -e 's/ -gencode arch=compute_72,code=sm_72//' \
       CMakeLists.txt utils/CMakeLists.txt

# 3. <cstdint>
for h in $(grep -rlE '\bu?int(8|16|32|64)_t\b' --include=*.h utils c plugins tools 2>/dev/null); do
    if ! grep -qE '<cstdint>|<stdint.h>' "$h" && grep -q '^#include' "$h"; then
        sed -i '0,/^#include/s//#include <cstdint>\n#include/' "$h"
    fi
done

# 4. nppGetStreamContext()
python3 - utils/cuda/cudaBayer.cpp <<'EOF'
import sys
path = sys.argv[1]
src = open(path).read()
old = """	NppStreamContext nppStreamContext;
	nppGetStreamContext(&nppStreamContext);
	nppStreamContext.hStream = stream;
"""
new = """	// nppGetStreamContext() was removed in CUDA 13; fill the context from
	// the device attributes instead.
	NppStreamContext nppStreamContext = {};
	nppStreamContext.hStream = stream;
	cudaGetDevice(&nppStreamContext.nCudaDeviceId);
	cudaDeviceGetAttribute(&nppStreamContext.nMultiProcessorCount, cudaDevAttrMultiProcessorCount, nppStreamContext.nCudaDeviceId);
	cudaDeviceGetAttribute(&nppStreamContext.nMaxThreadsPerMultiProcessor, cudaDevAttrMaxThreadsPerMultiProcessor, nppStreamContext.nCudaDeviceId);
	cudaDeviceGetAttribute(&nppStreamContext.nMaxThreadsPerBlock, cudaDevAttrMaxThreadsPerBlock, nppStreamContext.nCudaDeviceId);
	int sharedMemPerBlock = 0;
	cudaDeviceGetAttribute(&sharedMemPerBlock, cudaDevAttrMaxSharedMemoryPerBlock, nppStreamContext.nCudaDeviceId);
	nppStreamContext.nSharedMemPerBlock = sharedMemPerBlock;
	cudaDeviceGetAttribute(&nppStreamContext.nCudaDevAttrComputeCapabilityMajor, cudaDevAttrComputeCapabilityMajor, nppStreamContext.nCudaDeviceId);
	cudaDeviceGetAttribute(&nppStreamContext.nCudaDevAttrComputeCapabilityMinor, cudaDevAttrComputeCapabilityMinor, nppStreamContext.nCudaDeviceId);
	cudaStreamGetFlags(stream, &nppStreamContext.nStreamFlags);
"""
if "nppGetStreamContext(&nppStreamContext)" in src:
    open(path, "w").write(src.replace(old, new))
EOF

echo "jetson-inference patched for CUDA 13 / GCC 13"
