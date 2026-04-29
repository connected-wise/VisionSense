#include <iostream>
#include <fstream>

#include "trtutil.h"
#include "preprocess.h"
#include "NvOnnxParser.h"

using namespace nvinfer1;

void TRTLogger::log(Severity severity, const char *msg) noexcept {
    // Would advise using a proper logging utility such as https://github.com/gabime/spdlog
    // For the sake of this tutorial, will just log to the console.

    // Only log Warnings or more important.
    if (severity <= Severity::kWARNING) {
        std::cout << msg << std::endl;
    }
}

bool Engine::doesFileExist(const std::string &filepath) {
    std::ifstream f(filepath.c_str());
    return f.good();
}

Engine::Engine(const Options &options)
    : m_options(options) {}

bool Engine::build(std::string onnxModelPath) {
    // Only regenerate the engine file if it has not already been generated for the specified options
    m_engineName = serializeEngineOptions(m_options, onnxModelPath);
    std::cout << "Searching for engine file with name: " << m_engineName << std::endl;

    if (doesFileExist(m_engineName)) {
        std::cout << "Engine found, not regenerating..." << std::endl;
        return true;
    }

    if (!doesFileExist(onnxModelPath)) {
        throw std::runtime_error("Could not find model at path: " + onnxModelPath);
    }

    // Was not able to find the engine file, generate...
    std::cout << "Engine not found, generating. This could take a while..." << std::endl;

    // Create our engine builder.
    auto builder = std::unique_ptr<nvinfer1::IBuilder>(nvinfer1::createInferBuilder(m_logger));
    if (!builder) {
        return false;
    }

    // Define an explicit batch size and then create the network (implicit batch size is deprecated).
    // More info here: https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html#explicit-implicit-batch
    auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(explicitBatch));
    if (!network) {
        return false;
    }

    // Create a parser for reading the onnx file.
    auto parser = std::unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, m_logger));
    if (!parser) {
        return false;
    }

    // We are going to first read the onnx file into memory, then pass that buffer to the parser.
    // Had our onnx model file been encrypted, this approach would allow us to first decrypt the buffer.
    std::ifstream file(onnxModelPath, std::ios::binary | std::ios::ate);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> buffer(size);
    if (!file.read(buffer.data(), size)) {
        throw std::runtime_error("Unable to read engine file");
    }

    // Parse the buffer we read into memory.
    auto parsed = parser->parse(buffer.data(), buffer.size());
    if (!parsed) {
        return false;
    }

    // Ensure that all the inputs have the same batch size
    const auto numInputs = network->getNbInputs();
    if (numInputs < 1) {
        throw std::runtime_error("Error, model needs at least 1 input!");
    }
    const auto input0Batch = network->getInput(0)->getDimensions().d[0];
    for (int32_t i = 1; i < numInputs; ++i) {
        if (network->getInput(i)->getDimensions().d[0] != input0Batch) {
            throw std::runtime_error("Error, the model has multiple inputs, each with differing batch sizes!");
        }
    }

    // Check to see if the model supports dynamic batch size or not
    if (input0Batch == -1) {
        std::cout << "Model supports dynamic batch size" << std::endl;
    } else if (input0Batch == 1) {
        std::cout << "Model only supports fixed batch size of 1" << std::endl;
        // If the model supports a fixed batch size, ensure that the maxBatchSize and optBatchSize were set correctly.
        if (m_options.optBatchSize != input0Batch || m_options.maxBatchSize != input0Batch) {
            throw std::runtime_error("Error, model only supports a fixed batch size of 1. Must set Options.optBatchSize and Options.maxBatchSize to 1");
        }
    } else {
        throw std::runtime_error("Implementation currently only supports dynamic batch sizes or a fixed batch size of 1 (your batch size is fixed to "
        + std::to_string(input0Batch) + ")");
    }

    auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
    if (!config) {
        return false;
    }

    // Register a single optimization profile
    IOptimizationProfile *optProfile = builder->createOptimizationProfile();
    for (int32_t i = 0; i < numInputs; ++i) {
        // Must specify dimensions for all the inputs the model expects.
        const auto input = network->getInput(i);
        const auto inputName = input->getName();
        const auto inputDims = input->getDimensions();
        int32_t inputC = inputDims.d[1];
        int32_t inputH = inputDims.d[2];
        int32_t inputW = inputDims.d[3];

        // Specify the optimization profile`
        optProfile->setDimensions(inputName, OptProfileSelector::kMIN, Dims4(1, inputC, inputH, inputW));
        optProfile->setDimensions(inputName, OptProfileSelector::kOPT, Dims4(m_options.optBatchSize, inputC, inputH, inputW));
        optProfile->setDimensions(inputName, OptProfileSelector::kMAX, Dims4(m_options.maxBatchSize, inputC, inputH, inputW));
    }
    config->addOptimizationProfile(optProfile);

    // If the user specified FP16 precision, set that here.
    if (m_options.precision == Precision::FP16) {
        // Ensure the GPU supports FP16 inference
        if (!builder->platformHasFastFp16()) {
            throw std::runtime_error("Error: GPU does not support FP16 precision");
        }
        config->setFlag(BuilderFlag::kFP16);
    }

    // CUDA stream used for profiling by the builder.
    cudaStream_t profileStream;
    checkCudaErrorCode(cudaStreamCreate(&profileStream));
    config->setProfileStream(profileStream);

    // Build the engine
    // If this call fails, it is suggested to increase the logger verbosity to kVERBOSE and try rebuilding the engine.
    // Doing so will provide you with more information on why exactly it is failing.
    std::unique_ptr<IHostMemory> plan{builder->buildSerializedNetwork(*network, *config)};
    if (!plan) {
        return false;
    }

    // Write the engine to disk
    std::ofstream outfile(m_engineName, std::ofstream::binary);
    outfile.write(reinterpret_cast<const char*>(plan->data()), plan->size());

    std::cout << "Success, saved engine to " << m_engineName << std::endl;

    checkCudaErrorCode(cudaStreamDestroy(profileStream));
    return true;
}

Engine::~Engine() {
    // Free the GPU memory
    for (auto & buffer : m_buffers) {
        checkCudaErrorCode(cudaFree(buffer));
    }

    m_buffers.clear();

    if (m_uploadBuffer) {
        cudaFree(m_uploadBuffer);
        m_uploadBuffer = nullptr;
        m_uploadBufferBytes = 0;
    }
}

bool Engine::loadNetwork(std::string trtModelPath) {
    // Read the serialized model from disk
    std::ifstream file(trtModelPath, std::ios::binary | std::ios::ate);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> buffer(size);
    if (!file.read(buffer.data(), size)) {
        throw std::runtime_error("Unable to read engine file");
    }

    // Create a runtime to deserialize the engine file.
    m_runtime = std::unique_ptr<IRuntime> {createInferRuntime(m_logger)};
    if (!m_runtime) {
        return false;
    }

    // Set the device index
    auto ret = cudaSetDevice(m_options.deviceIndex);
    if (ret != 0) {
        int numGPUs;
        cudaGetDeviceCount(&numGPUs);
        auto errMsg = "Unable to set GPU device index to: " + std::to_string(m_options.deviceIndex) +
                ". Note, your device has " + std::to_string(numGPUs) + " CUDA-capable GPU(s).";
        throw std::runtime_error(errMsg);
    }

    // Create an engine, a representation of the optimized model.
    m_engine = std::unique_ptr<nvinfer1::ICudaEngine>(m_runtime->deserializeCudaEngine(buffer.data(), buffer.size()));
    if (!m_engine) {
        return false;
    }

    // The execution context contains all of the state associated with a particular invocation
    m_context = std::unique_ptr<nvinfer1::IExecutionContext>(m_engine->createExecutionContext());
    if (!m_context) {
        return false;
    }

    // Storage for holding the input and output buffers
    // This will be passed to TensorRT for inference
    m_buffers.resize(m_engine->getNbIOTensors());

    // Create a cuda stream
    cudaStream_t stream;
    checkCudaErrorCode(cudaStreamCreate(&stream));

    // Allocate GPU memory for input and output buffers
    m_outputLengthsFloat.clear();
    for (int i = 0; i < m_engine->getNbIOTensors(); ++i) {
        const auto tensorName = m_engine->getIOTensorName(i);
        m_IOTensorNames.emplace_back(tensorName);
        const auto tensorType = m_engine->getTensorIOMode(tensorName);
        const auto tensorShape = m_engine->getTensorShape(tensorName);
        if (tensorType == TensorIOMode::kINPUT) {
            // Allocate memory for the input
            // Allocate enough to fit the max batch size (we could end up using less later)
            checkCudaErrorCode(cudaMallocAsync(&m_buffers[i], m_options.maxBatchSize * tensorShape.d[1] * tensorShape.d[2] * tensorShape.d[3] * sizeof(float), stream));

            // Store the input dims for later use
            m_inputDims.emplace_back(tensorShape.d[1], tensorShape.d[2], tensorShape.d[3]);
        } else if (tensorType == TensorIOMode::kOUTPUT) {
            // The binding is an output
            uint32_t outputLenFloat = 1;
            m_outputDims.push_back(tensorShape);

            for (int j = 1; j < tensorShape.nbDims; ++j) {
                // We ignore j = 0 because that is the batch size, and we will take that into account when sizing the buffer
                outputLenFloat *= tensorShape.d[j];
            }

            m_outputLengthsFloat.push_back(outputLenFloat);
            // Now size the output buffer appropriately, taking into account the max possible batch size (although we could actually end up using less memory)
            checkCudaErrorCode(cudaMallocAsync(&m_buffers[i], outputLenFloat * m_options.maxBatchSize * sizeof(float), stream));
        } else {
            throw std::runtime_error("Error, IO Tensor is neither an input or output!");
        }
    }

    // Synchronize and destroy the cuda stream
    checkCudaErrorCode(cudaStreamSynchronize(stream));
    checkCudaErrorCode(cudaStreamDestroy(stream));

    return true;
}

void Engine::checkCudaErrorCode(cudaError_t code) {
    if (code != 0) {
        std::string errMsg = "CUDA operation failed with code: " + std::to_string(code) + "(" + cudaGetErrorName(code) + "), with message: " + cudaGetErrorString(code);
        std::cout << errMsg << std::endl;
        throw std::runtime_error(errMsg);
    }
}

bool Engine::runInference(const std::vector<std::vector<cv::Mat>> &inputs,
                          std::vector<std::vector<std::vector<float>>>& featureVectors,
                          const std::array<float, 3>& subVals,
                          const std::array<float, 3>& divVals,
                          bool normalize, bool swap_rb, bool aspect_ratio_pad) {
    if (inputs.empty() || inputs[0].empty()) {
        std::cout << "===== Error =====" << std::endl;
        std::cout << "Provided input vector is empty!" << std::endl;
        return false;
    }

    const auto numInputs = m_inputDims.size();
    if (inputs.size() != numInputs) {
        std::cout << "===== Error =====" << std::endl;
        std::cout << "Incorrect number of inputs provided!" << std::endl;
        return false;
    }

    if (inputs[0].size() > static_cast<size_t>(m_options.maxBatchSize)) {
        std::cout << "===== Error =====" << std::endl;
        std::cout << "The batch size is larger than the model expects!" << std::endl;
        std::cout << "Model max batch size: " << m_options.maxBatchSize << std::endl;
        std::cout << "Batch size provided to call to runInference: " << inputs[0].size() << std::endl;
        return false;
    }

    const auto batchSize = static_cast<int32_t>(inputs[0].size());
    for (size_t i = 1; i < inputs.size(); ++i) {
        if (inputs[i].size() != static_cast<size_t>(batchSize)) {
            std::cout << "===== Error =====" << std::endl;
            std::cout << "The batch size needs to be constant for all inputs!" << std::endl;
            return false;
        }
    }

    cudaStream_t inferenceCudaStream;
    checkCudaErrorCode(cudaStreamCreate(&inferenceCudaStream));

    for (size_t i = 0; i < numInputs; ++i) {
        const auto& batchInput = inputs[i];
        const auto& dims = m_inputDims[i];
        const int channels = dims.d[0];
        const int dst_h = dims.d[1];
        const int dst_w = dims.d[2];

        if (channels != 3) {
            std::cout << "===== Error =====" << std::endl;
            std::cout << "Only 3-channel inputs are supported (got " << channels << ")" << std::endl;
            cudaStreamDestroy(inferenceCudaStream);
            return false;
        }

        nvinfer1::Dims4 inputDims = {batchSize, channels, dst_h, dst_w};
        m_context->setInputShape(m_IOTensorNames[i].c_str(), inputDims);

        float* dst_base = static_cast<float*>(m_buffers[i]);
        const size_t per_image_floats = static_cast<size_t>(channels) * dst_h * dst_w;

        for (int b = 0; b < batchSize; ++b) {
            const cv::Mat& img = batchInput[b];
            if (img.empty() || img.type() != CV_8UC3) {
                std::cout << "===== Error =====" << std::endl;
                std::cout << "Each input must be a non-empty CV_8UC3 cv::Mat" << std::endl;
                cudaStreamDestroy(inferenceCudaStream);
                return false;
            }

            const int src_w = img.cols;
            const int src_h = img.rows;
            const size_t row_bytes = static_cast<size_t>(src_w) * 3;
            const size_t needed = row_bytes * src_h;

            if (needed > m_uploadBufferBytes) {
                if (m_uploadBuffer) {
                    checkCudaErrorCode(cudaFree(m_uploadBuffer));
                    m_uploadBuffer = nullptr;
                }
                checkCudaErrorCode(cudaMalloc(&m_uploadBuffer, needed));
                m_uploadBufferBytes = needed;
            }

            // cudaMemcpy2DAsync handles non-contiguous cv::Mat (e.g. ROI views) via img.step.
            checkCudaErrorCode(cudaMemcpy2DAsync(
                m_uploadBuffer, row_bytes,
                img.data, img.step,
                row_bytes, src_h,
                cudaMemcpyHostToDevice, inferenceCudaStream));

            launchPreprocess(
                static_cast<const uint8_t*>(m_uploadBuffer),
                src_w, src_h, row_bytes,
                dst_base + b * per_image_floats,
                dst_w, dst_h,
                swap_rb, aspect_ratio_pad, normalize,
                subVals[0], subVals[1], subVals[2],
                divVals[0], divVals[1], divVals[2],
                inferenceCudaStream);
        }
    }

    for (size_t i = 0; i < m_buffers.size(); ++i) {
        bool status = m_context->setTensorAddress(m_IOTensorNames[i].c_str(), m_buffers[i]);
        if (!status) {
            cudaStreamDestroy(inferenceCudaStream);
            return false;
        }
    }

    bool status = m_context->enqueueV3(inferenceCudaStream);
    if (!status) {
        cudaStreamDestroy(inferenceCudaStream);
        return false;
    }

    featureVectors.clear();

    for (int batch = 0; batch < batchSize; ++batch) {
        std::vector<std::vector<float>> batchOutputs{};
        for (int32_t outputBinding = numInputs; outputBinding < m_engine->getNbIOTensors(); ++outputBinding) {
            std::vector<float> output;
            auto outputLenFloat = m_outputLengthsFloat[outputBinding - numInputs];
            output.resize(outputLenFloat);
            checkCudaErrorCode(cudaMemcpyAsync(output.data(),
                                               static_cast<char*>(m_buffers[outputBinding]) + (batch * sizeof(float) * outputLenFloat),
                                               outputLenFloat * sizeof(float),
                                               cudaMemcpyDeviceToHost, inferenceCudaStream));
            batchOutputs.emplace_back(std::move(output));
        }
        featureVectors.emplace_back(std::move(batchOutputs));
    }

    checkCudaErrorCode(cudaStreamSynchronize(inferenceCudaStream));
    checkCudaErrorCode(cudaStreamDestroy(inferenceCudaStream));
    return true;
}

std::string Engine::serializeEngineOptions(const Options &options, const std::string& onnxModelPath) {
    const auto filenamePos = onnxModelPath.find_last_of('/') + 1;
    std::string engineName = onnxModelPath.substr(filenamePos, onnxModelPath.find_last_of('.') - filenamePos) + ".engine";

    // Add the GPU device name to the file to ensure that the model is only used on devices with the exact same GPU
    std::vector<std::string> deviceNames;
    getDeviceNames(deviceNames);

    if (static_cast<size_t>(options.deviceIndex) >= deviceNames.size()) {
        throw std::runtime_error("Error, provided device index is out of range!");
    }

    auto deviceName = deviceNames[options.deviceIndex];
    // Remove spaces from the device name
    deviceName.erase(std::remove_if(deviceName.begin(), deviceName.end(), ::isspace), deviceName.end());

    engineName+= "." + deviceName;

    // Serialize the specified options into the filename
    if (options.precision == Precision::FP16) {
        engineName += ".fp16";
    } else {
        engineName += ".fp32";
    }

    engineName += "." + std::to_string(options.maxBatchSize);
    engineName += "." + std::to_string(options.optBatchSize);

    return engineName;
}

void Engine::getDeviceNames(std::vector<std::string>& deviceNames) {
    int numGPUs;
    cudaGetDeviceCount(&numGPUs);

    for (int device=0; device<numGPUs; device++) {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, device);

        deviceNames.push_back(std::string(prop.name));
    }
}

void Engine::transformOutput(std::vector<std::vector<std::vector<float>>>& input, std::vector<std::vector<float>>& output) {
    if (input.size() != 1) {
        throw std::logic_error("The feature vector has incorrect dimensions!");
    }

    output = std::move(input[0]);
}

void Engine::transformOutput(std::vector<std::vector<std::vector<float>>>& input, std::vector<float>& output) {
    if (input.size() != 1 || input[0].size() != 1) {
        throw std::logic_error("The feature vector has incorrect dimensions!");
    }

    output = std::move(input[0][0]);
}
