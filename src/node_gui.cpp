#include "ros_compat.h"
#include "common.h"
#include <jetson-utils/videoOutput.h>
#include "trtutil.h"

#include "visionconnect/msg/detect.hpp"
#include "visionconnect/msg/signs.hpp"
#include "visionconnect/msg/lanes.hpp"
#include "visionconnect/msg/track.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

visionconnect::msg::Detect::SharedPtr detect_msg = NULL;
visionconnect::msg::Signs::SharedPtr signs_msg = NULL;
visionconnect::msg::Lanes::SharedPtr lanes_msg = NULL;
visionconnect::msg::Track::SharedPtr track_msg = NULL;
// sensor_msgs::ImageConstPtr img_msg = NULL;

// Declare publishers
Publisher<sensor_msgs::Image> gui_pub = NULL;

// keeping trail of the track boxes
std::unordered_map<int, std::vector<cv::Point2f>> track_history;
std::unordered_map<int, int> frames_history;
int frame_count = 0;

#define TRACK_HISTORY true;


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

void trackingHistoryCleanup(int frame_id)
{
  std::vector<int> remove_ids;
  for (auto it = track_history.begin(); it != track_history.end();)
  {
    if (it->second.size() > 0)
    {
      if (frame_id - frames_history[it->first] > 300)
      {
        remove_ids.push_back(it->first);
      }
    }
    it++;
  }
  for (int i = 0; i < remove_ids.size(); i++)
  {
    track_history.erase(remove_ids[i]);
    frames_history.erase(remove_ids[i]);
  }
}

void drawTracking(cv::Mat& image, bool update_history = true)
{
  auto boxes = track_msg->boxes;
  auto classes = track_msg->classes;
  auto track_ids = track_msg->track_ids;
  auto num_detections = track_msg->num_tracked;

  for (int i = 0; i < num_detections; i++)
  {
    auto track_box = boxes[i];
    auto track_id = track_ids[i];
    if (classes[i] == 2 && update_history)  // car-vehicle objects only
    {
      cv::Point2f center(track_box.data[0] + track_box.data[2] / 2.0f, track_box.data[1] + track_box.data[3] / 2.0f);
      track_history[track_id].push_back(center);
      frames_history[track_id] = frame_count;

      if (track_history[track_id].size() > 30)
      {
        track_history[track_id].erase(track_history[track_id].begin());
      }
      for (size_t j = 0; j < track_history[track_id].size(); j++)
      {
        // cv::line(image, track_history[track_id][j], track_history[track_id][j + 1], cv::Scalar(230, 230, 230), 2);
        cv::circle(image, track_history[track_id][j], 2, cv::Scalar(230, 230, 230), 1);
      }
    }
    cv::Rect_<float> bbox = cv::Rect(track_box.data[0], track_box.data[1], track_box.data[2], track_box.data[3]);
    cv::rectangle(image, bbox, colors[(int)classes[i]], 4);
  }

  trackingHistoryCleanup(frame_count);
  frame_count++;
}

void drawlane(cv::Mat& image)
{
  // The lane points should be calculated in the postprocess function

  std::vector<cv::Scalar> colors = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
                                     cv::Scalar(255, 255, 0) };

  std::vector<cv::Point> current_lane;
  size_t lane_index = 0;
  for (size_t j = 0; j < lanes_msg->xs.size(); ++j)
  {
    int x = lanes_msg->xs[j];
    int y = lanes_msg->ys[j];

    if (x == -1 && y == -1)
    {
      // This means we've reached the delimiter for a lane
      // Draw the current lane if it has points
      if (!current_lane.empty())
      {
        cv::polylines(image, current_lane, false, colors[lane_index], 7);
        current_lane.clear();  // Clear current_lane for the next set of points
        lane_index++;
      }
    }
    else
    {
      current_lane.push_back(cv::Point(x, y));
    }
  }
  // Now, draw the circles
  for (size_t j = 0; j < lanes_msg->xs.size(); ++j)
  {
    int x = lanes_msg->xs[j];
    int y = lanes_msg->ys[j];

    if (x != -1 && y != -1)
    {
      cv::circle(image, cv::Point(x, y), 2, cv::Scalar(50, 50, 50), -1);
    }
  }
}


void fuse_data()
{

  cv::Mat img;
  auto img_msg = std::make_shared<sensor_msgs::msg::Image>(detect_msg->image);

  convert_message_to_frame(img_msg, img);

  if (detect_msg == NULL || signs_msg == NULL || lanes_msg == NULL)
    return;

  drawTracking(img);
  drawlane(img);

  std::cout << "drawing lanes..." << std::endl;

  auto boxes = detect_msg->boxes;
  auto classes = detect_msg->classes;
  auto scores = detect_msg->scores;
  auto num_detections = detect_msg->num_detections;
  auto signLabels = signs_msg->labels;
  auto signScores = signs_msg->scores;

  size_t j = 0;  // index for sign and light labels
  cv::String label = "";
  int baseline = 0;

  for (int i = 0; i < num_detections; i++)
  {
    int x = static_cast<int>(boxes[i].data[0]);
    int y = static_cast<int>(boxes[i].data[1]);
    int w = static_cast<int>(boxes[i].data[2]);
    int h = static_cast<int>(boxes[i].data[3]);

    // cv::rectangle(img, x, y, w, h, colors[classes[i]], 4);

    if (classes[i] == 0)  // if pedestrian
    {
      label = "pedestrian " + std::to_string(scores[i]).substr(0, 4);
      cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
      cv::Point topLeft(x, y - textSize.height);
      cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline),
                    colors[classes[i]], cv::FILLED);
      cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
      if (scores[i] > 0.7)
        cv::rectangle(img, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 4);
    }

    else if (classes[i] == 1)  // if cyclist
    {
      label = "cyclist " + std::to_string(scores[i]).substr(0, 4);
      cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
      cv::Point topLeft(x, y - textSize.height);
      cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline),
                    colors[classes[i]], cv::FILLED);
      cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
      if (scores[i] > 0.7)
        cv::rectangle(img, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 4);
    }

    else if (classes[i] == 2)  // if vehicle-car
    {
      label = "vehicle-car " + std::to_string(scores[i]).substr(0, 4);
      cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
      cv::Point topLeft(x, y - textSize.height);
      cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline),
                    colors[classes[i]], cv::FILLED);
      cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
    }

    else if (classes[i] == 3)  // if vehicle-bus
    {
      label = "vehicle-bus " + std::to_string(scores[i]).substr(0, 4);
      cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
      cv::Point topLeft(x, y - textSize.height);
      cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline),
                    colors[classes[i]], cv::FILLED);
      cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
    }

    else if (classes[i] == 4)  // if vehicle-truck
    {
      label = "vehicle-truck " + std::to_string(scores[i]).substr(0, 4);
      cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
      cv::Point topLeft(x, y - textSize.height);
      cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline),
                    colors[classes[i]], cv::FILLED);
      cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
    }

    else if (classes[i] == 5)  // if vehicle-train
    {
      label = "vehicle-train " + std::to_string(scores[i]).substr(0, 4);
      cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
      cv::Point topLeft(x, y - textSize.height);
      cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline),
                    colors[classes[i]], cv::FILLED);
      cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
      if (scores[i] > 0.7)
        cv::rectangle(img, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 4);
    }

    else if (classes[i] == 6 && signScores[j] > 0.5 && j < signLabels.size())  // if traffic light
    {
      label = signLabels[j];
      std::cout << "label: " << label << std::endl;
      if (label == "red light")
      {
        // cout << "*******************red light*********" << j << std::endl;
        label = label + " " + std::to_string(signScores[j]).substr(0, 4);  // add score to label
        cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
        cv::Point topLeft(x, y - textSize.height);
        cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline),
                      cv::Scalar(0, 0, 255), cv::FILLED);  // Red background
        cv::rectangle(img, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 4);
        cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
        j++;
      }

      else if (label == "green light")
      {
        label = label + " " + std::to_string(signScores[j]).substr(0, 4);  // add score to label
        cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
        cv::Point topLeft(x, y - textSize.height);
        cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline),
                      cv::Scalar(55, 235, 100), cv::FILLED);  // Green background
        cv::rectangle(img, cv::Rect(x, y, w, h), cv::Scalar(55, 235, 100), 4);
        cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
        j++;
      }

      else if (label == "yellow light")
      {
        label = label + " " + std::to_string(signScores[j]).substr(0, 4);  // add score to label
        cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
        cv::Point topLeft(x, y - textSize.height);
        cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline),
                      cv::Scalar(0, 255, 255), cv::FILLED);  // Yellow background
        cv::rectangle(img, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 255), 4);
        cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
        j++;
      }
      else
      {
        j++;
        continue;
      }
    }

    else if (classes[i] == 7 && signScores[j] > 0.65 && j < signLabels.size())  // if traffic sign
    {
      label = signLabels[j];
      std::cout << "label: " << label << std::endl;

      if (label != "guide sign")
      {
        label = label + " " + std::to_string(signScores[j]).substr(0, 4);  // add score to label
        cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
        cv::Point topLeft(x, y - textSize.height);
        cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline),
                      colors[classes[i]], cv::FILLED);
        cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
      }
      j++;
    }

    // draw tracking history

    // cv::putText(img, std::to_string(classes[i]), cv::Point(boxes[i].data[0]/3, boxes[i].data[1]/3),
    // cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1); cv::putText(img, std::to_string(scores[i]),
    // cv::Point(boxes[i].data[0]/3, boxes[i].data[1]/3 + 5), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1);
  }

  sensor_msgs::msg::Image msg;
  msg.header.stamp = ROS_TIME_NOW();
  // Convert output image to ROS message
  convert_frame_to_message(img, msg);
  gui_pub->publish(msg);

  // cv::resize(img, img, cv::Size(1920, 1080));
  cv::imshow("Perception Fusion", img);
  cv::waitKey(1);
}

// input image subscriber callback
void signs_callback(const visionconnect::msg::Signs::SharedPtr input)
{
  signs_msg = input;
}

// input image subscriber callback
void camera_callback(const sensor_msgs::ImageConstPtr input)
{
  // img_msg = input;
}

// detection overlay callback
void detection_callback(const visionconnect::msg::Detect::SharedPtr input)
{
  detect_msg = input;

  fuse_data();  // Fuse data from all the topics
}

// detection overlay callback
void lanes_callback(const visionconnect::msg::Lanes::SharedPtr input)
{
  lanes_msg = input;
}

void track_callback(const visionconnect::msg::Track::SharedPtr input)
{
  track_msg = input;
}

// node main loop
int main(int argc, char** argv)
{
  ROS_CREATE_NODE("gui");
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("visionconnect");
  cv::namedWindow("Perception Fusion", cv::WND_PROP_AUTOSIZE);
  cv::setWindowProperty("Perception Fusion", cv::WINDOW_NORMAL, cv::WND_PROP_AUTOSIZE);

  auto cam_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 1, camera_callback);
  auto detect_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Detect, "detect_in", 1, detection_callback);
  auto signs_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Signs, "signs_in", 1, signs_callback);
  auto lanes_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Lanes, "lanes_in", 1, lanes_callback);
  auto track_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Track, "track_in", 1, track_callback);

  ROS_CREATE_PUBLISHER(sensor_msgs::Image, "fusion", 5, gui_pub);

  // start publishing video frames
  ROS_INFO("GUI Node initialized, waiting for images");
  ROS_SPIN();

  return 0;
}
