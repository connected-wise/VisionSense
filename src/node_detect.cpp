#include "ros_compat.h"
#include "common.h"
#include <jetson-utils/videoOutput.h>
#include "image_converter.h"
#include "trtutil.h"
#include "BYTETracker.h"

#include "visionconnect/msg/detect.hpp"
#include "visionconnect/msg/signs.hpp"
#include "visionconnect/msg/track.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

Engine* engine;
#define NMS_THRESH 0.5
#define BATCH_SIZE 1
float m_ratio = 1;
Publisher<visionconnect::msg::Detect> detect_pub = NULL;
Publisher<visionconnect::msg::Signs> signs_pub = NULL;
Publisher<visionconnect::msg::Track> track_pub = NULL;


std::vector<std::string> classes;
std::vector<double> THRS;

double fps = 30.0;
BYTETracker tracker(fps, 30);

std::vector<cv::Scalar> colors = {
  cv::Scalar(10, 230, 240),  // pedestrian
  cv::Scalar(230, 240, 10),  // cyclist
  cv::Scalar(190, 150, 25),  // car
  cv::Scalar(240, 120, 10),  // bus
  cv::Scalar(100, 10, 240),  // truck
  cv::Scalar(240, 130, 10),  // train
  cv::Scalar(160, 90, 160),  // traffic light
  cv::Scalar(120, 55, 70)    // traffic sign
};

// struct Object
// {
//     // The object class.
//     int label{};
//     // The detection's confidence probability.
//     float probability{};
//     // The object bounding box rectangle.
//     cv::Rect_<float> rect;
//     cv::Mat cropped;
// };

void postprocess(std::vector<float>& featureVector, const std::vector<nvinfer1::Dims>& outputDims, cv::Mat& img)
{
  std::vector<cv::Rect> bboxes;
  std::vector<float> scores;
  std::vector<int> labels;
  std::vector<int> indices;
  visionconnect::msg::Box detect_box;
  visionconnect::msg::Detect detect_msg;
  visionconnect::msg::Signs classifier_msg;

  // Tracking
  visionconnect::msg::Track track_msg;
  visionconnect::msg::Box track_box;
  std::vector<float> track_ids;

  // Shrink and send input feed to output stream
  cv::Mat output;
  cv::resize(img, output, img.size());

  std::string labelTxt;
  auto numChannels = outputDims[0].d[1];
  auto numAnchors = outputDims[0].d[2];
  auto numClasses = classes.size();

  float m_imgHeight = img.rows;
  float m_imgWidth = img.cols;

  cv::Mat out = cv::Mat(numChannels, numAnchors, CV_32F, featureVector.data());
  out = out.t();
  int counter = 0;

  for (int i = 0; i < numAnchors; i++)
  {
    auto rowPtr = out.row(i).ptr<float>();
    auto bboxesPtr = rowPtr;
    auto scoresPtr = rowPtr + 4;
    auto maxSPtr = std::max_element(scoresPtr, scoresPtr + numClasses);
    float score = *maxSPtr;
    if (score > THRS[0])
    {
      counter++;

      if (counter > 99)
        continue;
      float x = *bboxesPtr++;
      float y = *bboxesPtr++;
      float w = *bboxesPtr++;
      float h = *bboxesPtr;

      float x0 = std::clamp((x - 0.5f * w) * m_ratio, 0.f, m_imgWidth);
      float y0 = std::clamp((y - 0.5f * h) * m_ratio, 0.f, m_imgHeight);
      float x1 = std::clamp((x + 0.5f * w) * m_ratio, 0.f, m_imgWidth);
      float y1 = std::clamp((y + 0.5f * h) * m_ratio, 0.f, m_imgHeight);

      int label = maxSPtr - scoresPtr;

      cv::Rect_<float> bbox;
      bbox.x = x0;
      bbox.y = y0;
      bbox.width = (x1 - x0);
      bbox.height = (y1 - y0);

      bboxes.push_back(bbox);
      labels.push_back(label);
      scores.push_back(score);
    }
  }
  // NMSBoxesBatched() is available since openCV 4.7
  cv::dnn::NMSBoxesBatched(bboxes, scores, labels, THRS[0], NMS_THRESH, indices);

  // TODO : Track optimize this messy part
  std::vector<Object> objects;
  // TODO: merge two for loops
  for (auto& index : indices)
  {
    cv::Rect tmp = bboxes[index];
    Object obj;
    obj.label = labels[index];
    obj.rect = tmp;
    obj.prob = scores[index];
    objects.push_back(obj);
  }

  vector<STrack> output_stracks = tracker.update(objects);
  for (int i = 0; i < output_stracks.size(); i++)
  {
    int track_id = output_stracks[i].track_id;
    vector<float> tlwh = output_stracks[i].tlwh;
    // Scalar s = tracker.get_color(output_stracks[i].track_id);
    track_msg.classes[i] = output_stracks[i].label;
    track_msg.track_ids[i] = track_id;


    track_box.data = { static_cast<signed short>(output_stracks[i].tlwh[0]),
                       static_cast<signed short>(output_stracks[i].tlwh[1]),
                       static_cast<signed short>(output_stracks[i].tlwh[2]),
                       static_cast<signed short>(output_stracks[i].tlwh[3]) };
    track_msg.boxes[i] = track_box;

    // HISTORY:

  }
  track_msg.num_tracked = output_stracks.size();
  track_pub->publish(track_msg);


  int i = 0;
  for (auto& index : indices)
  {
    // Copy raw detect to ROS message
    detect_msg.classes[i] = labels[index];
    detect_msg.scores[i] = scores[index];

    detect_box.data = { static_cast<signed short>(bboxes[index].x), static_cast<signed short>(bboxes[index].y),
                        static_cast<signed short>(bboxes[index].width),
                        static_cast<signed short>(bboxes[index].height) };
    detect_msg.boxes[i] = detect_box;

    // Sign or Light detected
    if (labels[index] == 6 || labels[index] == 7)
    {
      // Crop and convert through ROS
      sensor_msgs::msg::Image msgClassifyImg;

      convert_frame_to_message(img(bboxes[index]), msgClassifyImg);
      classifier_msg.images.push_back(msgClassifyImg);
      classifier_msg.classes.push_back(labels[index]);
      classifier_msg.scores.push_back(scores[index]);
      classifier_msg.boxes.push_back(detect_box);
    }

    // Redo box and draw for scaled down image
    cv::Rect_<float> bbox = cv::Rect(bboxes[index].x, bboxes[index].y, bboxes[index].width, bboxes[index].height);
    cv::rectangle(output, bbox, colors[(int)labels[index]], 4);

    i++;
  }

  detect_msg.num_detections = i;
  sensor_msgs::msg::Image msg;
  msg.header.stamp = ROS_TIME_NOW();

  // Convert output image to ROS message
  convert_frame_to_message(output, msg);
  detect_msg.image = msg;

  // Publish the messages
  detect_pub->publish(detect_msg);
  signs_pub->publish(classifier_msg);
}

void load_engine(std::string path)
{
  if (!Engine::doesFileExist(path))
  {
    throw std::runtime_error("Error: Unable to find engine at:" + path);
  }

  // Specify our GPU inference configuration options
  Options options;
  options.precision = Precision::FP16;
  options.optBatchSize = 1;
  options.maxBatchSize = 1;

  engine = new Engine(options);

  // Load the TensorRT engine file from disk
  bool success = engine->loadNetwork(path);
  if (!success)
  {
    throw std::runtime_error("Unable to load TRT engine.");
  }
}

void run_engine(cv::Mat img)
{
  // cv::imshow("detection", img);
  // cv::waitKey(1);

  const auto& inputDims = engine->getInputDims();
  std::vector<std::vector<cv::cuda::GpuMat>> inputs;

  for (const auto& inputDim : inputDims)
  {  // For each of the model inputs...
    std::vector<cv::cuda::GpuMat> input;
    for (size_t j = 0; j < BATCH_SIZE; ++j)
    {  // For each element we want to add to the batch...
      cv::cuda::GpuMat gpu_img, resized;
      gpu_img.upload(img);
      // Apply if input is BGR image:
      cv::cuda::cvtColor(gpu_img, gpu_img, cv::COLOR_BGR2RGB);
      resized = engine->resizeKeepAspectRatioPadRightBottom(gpu_img, inputDim.d[2], inputDim.d[1],
                                                            cv::COLOR_BGR2RGB);  // TRT dims are (height, width) whereas
                                                                                 // OpenCV is (width, height)
      input.emplace_back(std::move(resized));
    }
    inputs.emplace_back(std::move(input));
  }

  m_ratio = 1.f / std::min(inputDims[0].d[2] / static_cast<float>(img.cols),
                           inputDims[0].d[1] / static_cast<float>(img.rows));

  std::vector<std::vector<std::vector<float>>> featureVectors;  // Considers a batch output
  std::array<float, 3> subVals{ 0.f, 0.f, 0.f };
  std::array<float, 3> divVals{ 1.f, 1.f, 1.f };
  bool normalize = true;

  bool success = engine->runInference(inputs, featureVectors, subVals, divVals, normalize);
  if (!success)
  {
    throw std::runtime_error("Unable to run inference.");
  }

  // std::cout << "size of output: " << featureVectors[0][0].size() << std::endl;
  const auto outputDims = engine->getOutputDims();

  std::cout << "running detection engine.." << std::endl;

  postprocess(featureVectors[0][0], outputDims, img);
}

// input image subscriber callback
void img_callback(const sensor_msgs::msg::Image::SharedPtr input)
{
  cv::Mat img;
  convert_message_to_frame(input, img);

  if (img.empty())
  {
    ROS_INFO("Failed to convert the input to Opencv image");
    return;
  }

  run_engine(img);
}

// node main loop
int main(int argc, char** argv)
{
  ROS_CREATE_NODE("detect");
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("visionconnect");
  std::string model_str, label_str, engine_path, label_path;

  ROS_DECLARE_PARAMETER("model", model_str);
  ROS_DECLARE_PARAMETER("labels", label_str);
  ROS_DECLARE_PARAMETER("thresholds", THRS);

  ROS_GET_PARAMETER("model", model_str);
  ROS_GET_PARAMETER("labels", label_str);
  ROS_GET_PARAMETER("thresholds", THRS);

  engine_path = package_share_directory + "/graphs/object-detection/" + model_str;
  label_path = package_share_directory + "/graphs/object-detection/" + label_str;

  load_engine(engine_path);
  classes = getClassNames(label_path);

  auto img_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 1, img_callback);

  ROS_CREATE_PUBLISHER(visionconnect::msg::Detect, "detections", 10, detect_pub);
  ROS_CREATE_PUBLISHER(visionconnect::msg::Signs, "signs", 10, signs_pub);
  ROS_CREATE_PUBLISHER(visionconnect::msg::Track, "track", 10, track_pub);

  // start publishing video frames
  ROS_INFO("Detect Node initialized, publishing detections...");
  ROS_SPIN();

  delete engine;
  detect_pub.reset();
  signs_pub.reset();

  return 0;
}
