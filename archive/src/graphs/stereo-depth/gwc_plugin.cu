// TensorRT plugin for the Group-Wise Correlation (GWC) volume.
//
// Wraps the same CUDA kernel used in the two-engine demo into an
// IPluginV2DynamicExt so TRT can inline it into a single engine.
//
// Build:  nvcc -shared -o libgwc_plugin.so gwc_plugin.cu ...
// Use:    trtexec --onnx=stereo.onnx --plugins=./libgwc_plugin.so --saveEngine=stereo.engine --fp16

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_fp16.h>
#include <cuda_runtime.h>

#include <cassert>
#include <cstring>
#include <string>
#include <vector>

using namespace nvinfer1;

// ============================================================================
// CUDA kernel
// ============================================================================

template <typename T>
__global__ void gwc_volume_kernel(const T* __restrict__ left,
                                  const T* __restrict__ right,
                                  T* __restrict__ out,
                                  int C, int H, int W, int D, int G,
                                  int normalize) {
  int w  = blockIdx.x * blockDim.x + threadIdx.x;
  int d  = blockIdx.y;
  int hg = blockIdx.z;
  int g  = hg % G;
  int h  = hg / G;
  if (w >= W || h >= H || d >= D) return;

  int K = C / G;
  int wr = w - d;
  bool valid = (wr >= 0);

  float corr = 0.f, lnorm = 0.f, rnorm = 0.f;
  int c0 = g * K;
  for (int k = 0; k < K; ++k) {
    int c = c0 + k;
    int li = (c * H + h) * W + w;
    float lv = static_cast<float>(left[li]);
    lnorm += lv * lv;
    if (valid) {
      int ri = (c * H + h) * W + wr;
      float rv = static_cast<float>(right[ri]);
      rnorm += rv * rv;
      corr  += lv * rv;
    }
  }

  float result = 0.f;
  if (valid) {
    if (normalize) {
      result = corr / (sqrtf(lnorm) * sqrtf(rnorm) + 1e-5f);
    } else {
      result = corr / static_cast<float>(K);
    }
  }
  out[((g * D + d) * H + h) * W + w] = static_cast<T>(result);
}

// ============================================================================
// Plugin
// ============================================================================

static const char* PLUGIN_NAME    = "GWCVolume";
static const char* PLUGIN_VERSION = "1";

class GWCVolumePlugin : public IPluginV2DynamicExt {
 public:
  GWCVolumePlugin(int g, int d, int n) : mG(g), mD(d), mN(n) {}
  GWCVolumePlugin(const void* buf, size_t) {
    auto* p = static_cast<const int*>(buf);
    mG = p[0]; mD = p[1]; mN = p[2];
  }

  const char* getPluginType()    const noexcept override { return PLUGIN_NAME; }
  const char* getPluginVersion() const noexcept override { return PLUGIN_VERSION; }
  int  getNbOutputs()            const noexcept override { return 1; }
  int  initialize()              noexcept override { return 0; }
  void terminate()               noexcept override {}
  void destroy()                 noexcept override { delete this; }

  size_t getSerializationSize() const noexcept override { return 3 * sizeof(int); }
  void serialize(void* buf) const noexcept override {
    auto* p = static_cast<int*>(buf); p[0]=mG; p[1]=mD; p[2]=mN;
  }

  void setPluginNamespace(const char* ns) noexcept override { mNs = ns; }
  const char* getPluginNamespace() const noexcept override { return mNs.c_str(); }

  DataType getOutputDataType(int, const DataType* in, int) const noexcept override {
    return in[0];
  }

  IPluginV2DynamicExt* clone() const noexcept override {
    return new GWCVolumePlugin(mG, mD, mN);
  }

  DimsExprs getOutputDimensions(int, const DimsExprs* in, int,
                                IExprBuilder& b) noexcept override {
    // in[0]=[B,C,H,W]  out=[B,G,D,H,W]
    DimsExprs o; o.nbDims = 5;
    o.d[0]=in[0].d[0]; o.d[1]=b.constant(mG); o.d[2]=b.constant(mD);
    o.d[3]=in[0].d[2]; o.d[4]=in[0].d[3];
    return o;
  }

  bool supportsFormatCombination(int pos, const PluginTensorDesc* io,
                                 int, int) noexcept override {
    if (io[pos].format != TensorFormat::kLINEAR) return false;
    auto dt = io[pos].type;
    if (dt != DataType::kFLOAT && dt != DataType::kHALF) return false;
    return pos == 0 || dt == io[0].type;
  }

  void configurePlugin(const DynamicPluginTensorDesc*, int,
                       const DynamicPluginTensorDesc*, int) noexcept override {}

  size_t getWorkspaceSize(const PluginTensorDesc*, int,
                          const PluginTensorDesc*, int) const noexcept override { return 0; }

  int enqueue(const PluginTensorDesc* inDesc, const PluginTensorDesc*,
              const void* const* inputs, void* const* outputs,
              void*, cudaStream_t stream) noexcept override {
    int C=inDesc[0].dims.d[1], H=inDesc[0].dims.d[2], W=inDesc[0].dims.d[3];
    dim3 blk(128); dim3 grd((W+127)/128, mD, H*mG);
    if (inDesc[0].type == DataType::kFLOAT)
      gwc_volume_kernel<float><<<grd,blk,0,stream>>>(
          (const float*)inputs[0],(const float*)inputs[1],(float*)outputs[0],C,H,W,mD,mG,mN);
    else
      gwc_volume_kernel<__half><<<grd,blk,0,stream>>>(
          (const __half*)inputs[0],(const __half*)inputs[1],(__half*)outputs[0],C,H,W,mD,mG,mN);
    return 0;
  }

 private:
  int mG=8, mD=48, mN=1;
  std::string mNs;
};

// ============================================================================
// Creator
// ============================================================================

class GWCVolumePluginCreator : public IPluginCreator {
 public:
  GWCVolumePluginCreator() {
    mF.emplace_back(PluginField{"num_groups",     nullptr, PluginFieldType::kINT32, 1});
    mF.emplace_back(PluginField{"max_disp_quart", nullptr, PluginFieldType::kINT32, 1});
    mF.emplace_back(PluginField{"normalize",      nullptr, PluginFieldType::kINT32, 1});
    mFC.nbFields = mF.size(); mFC.fields = mF.data();
  }
  const char* getPluginName()    const noexcept override { return PLUGIN_NAME; }
  const char* getPluginVersion() const noexcept override { return PLUGIN_VERSION; }
  PluginFieldCollection const* getFieldNames() noexcept override { return &mFC; }

  IPluginV2* createPlugin(const char*, const PluginFieldCollection* fc) noexcept override {
    int g=8,d=48,n=1;
    for (int i=0;i<fc->nbFields;++i) {
      auto& f=fc->fields[i];
      if (!strcmp(f.name,"num_groups"))     g=*(const int*)f.data;
      if (!strcmp(f.name,"max_disp_quart")) d=*(const int*)f.data;
      if (!strcmp(f.name,"normalize"))      n=*(const int*)f.data;
    }
    return new GWCVolumePlugin(g,d,n);
  }

  IPluginV2* deserializePlugin(const char*, const void* data, size_t len) noexcept override {
    return new GWCVolumePlugin(data, len);
  }

  void setPluginNamespace(const char* ns) noexcept override { mNs = ns; }
  const char* getPluginNamespace() const noexcept override { return mNs.c_str(); }

 private:
  PluginFieldCollection mFC;
  std::vector<PluginField> mF;
  std::string mNs;
};

REGISTER_TENSORRT_PLUGIN(GWCVolumePluginCreator);
