#include "ros_compat.h"
#include "common.h"
#include <jetson-utils/videoOutput.h>
#include "trtutil.h"

#include "visionconnect/msg/detect.hpp"
#include "visionconnect/msg/signs.hpp"
#include "visionconnect/msg/lanes.hpp"
#include "visionconnect/msg/track.hpp"
#include "image_converter.h"
#include <ament_index_cpp/get_package_share_directory.hpp>


visionconnect::msg::Detect::SharedPtr detect_msg = NULL;
visionconnect::msg::Signs::SharedPtr signs_msg = NULL;
visionconnect::msg::Lanes::SharedPtr lanes_msg = NULL;
visionconnect::msg::Track::SharedPtr track_msg = NULL;

videoOutput* stream1 = NULL;
videoOutput* stream2 = NULL;
videoOutput* stream3 = NULL;
imageConverter* image_cvt = NULL;
int display_w, display_h;

void drawlane(cv::Mat &image)
{
    // The lane points should be calculated in the postprocess function

    std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0),
        cv::Scalar(0, 255, 0),
        cv::Scalar(0, 0, 255),
        cv::Scalar(255, 255, 0)
    };

    std::vector<cv::Point> current_lane;
    size_t lane_index = 0;
    for (size_t j = 0; j < lanes_msg->xs.size(); ++j) {
        int x = lanes_msg->xs[j];
        int y = lanes_msg->ys[j];

        if (x == -1 && y == -1) {
            // This means we've reached the delimiter for a lane
            // Draw the current lane if it has points
            if (!current_lane.empty()) {
                cv::polylines(image, current_lane, false, colors[lane_index], 7);
                current_lane.clear();  // Clear current_lane for the next set of points
                lane_index++;
            }
        } else {
            current_lane.push_back(cv::Point(x/3, y/3));
        }
    }
    // Now, draw the circles
    for (size_t j = 0; j < lanes_msg->xs.size(); ++j) {
        int x = lanes_msg->xs[j];
        int y = lanes_msg->ys[j];

        if (x != -1 && y != -1) {
            cv::circle(image, cv::Point(x/3, y/3), 2, cv::Scalar(50,50,50), -1);
        }
    }
}

void fuse_data()
{
    std::vector<cv::Scalar> colors = {
        cv::Scalar(10, 230, 240), //pedestrian
        cv::Scalar(230, 240, 10), //cyclist
        cv::Scalar(190, 150, 25), //car
        cv::Scalar(240, 120, 10), //bus
        cv::Scalar(100, 10, 240), //truck
        cv::Scalar(240, 130, 10), //train
        cv::Scalar(160, 90, 160), //traffic light
        cv::Scalar(120, 55, 70)   //traffic sign
    };
    cv::Mat img;
    auto img_msg = std::make_shared<sensor_msgs::msg::Image>(detect_msg->image);

    convert_message_to_frame(img_msg, img);

    if (detect_msg==NULL || signs_msg==NULL || lanes_msg==NULL)
        return;

    drawlane(img);
    std::cout<<"drawing lanes..."<<std::endl;

    auto boxes = detect_msg->boxes;
    auto classes = detect_msg->classes;
    auto scores = detect_msg->scores;
    auto num_detections = detect_msg->num_detections;
    auto signLabels = signs_msg->labels;
    auto signScores = signs_msg->scores;
    
    size_t j = 0; // index for sign and light labels
    cv::String label = "";
    int baseline = 0;

    for (int i = 0; i < num_detections; i++)
    {
        int x = static_cast<int>(boxes[i].data[0]/3);
        int y = static_cast<int>(boxes[i].data[1]/3);
        int w = static_cast<int>(boxes[i].data[2]/3);
        int h = static_cast<int>(boxes[i].data[3]/3);

        //cv::rectangle(img, x, y, w, h, colors[classes[i]], 4);

        if (classes[i] == 0) // if pedestrian
        {
            label = "pedestrian " + std::to_string(scores[i]).substr(0, 4);
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
            cv::Point topLeft(x, y - textSize.height); 
            cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), colors[classes[i]], cv::FILLED);
            cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
            if (scores[i] > 0.7)
                cv::rectangle(img, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 4);
        }

        else if (classes[i] == 1) // if cyclist
        {
            label = "cyclist " + std::to_string(scores[i]).substr(0, 4);
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
            cv::Point topLeft(x, y - textSize.height); 
            cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), colors[classes[i]], cv::FILLED);
            cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
            if (scores[i] > 0.7)
                cv::rectangle(img, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 4);
        }

        else if (classes[i] == 2) // if vehicle-car
        {
            label = "vehicle-car " + std::to_string(scores[i]).substr(0, 4);
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
            cv::Point topLeft(x, y - textSize.height); 
            cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), colors[classes[i]], cv::FILLED);
            cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);

        }
        
        else if (classes[i] == 3) // if vehicle-bus
        {
            label = "vehicle-bus " + std::to_string(scores[i]).substr(0, 4);
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
            cv::Point topLeft(x, y - textSize.height); 
            cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), colors[classes[i]], cv::FILLED);
            cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);

        }

        else if (classes[i] == 4) // if vehicle-truck
        {
            label = "vehicle-truck " + std::to_string(scores[i]).substr(0, 4);
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
            cv::Point topLeft(x, y - textSize.height); 
            cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), colors[classes[i]], cv::FILLED);
            cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);

        }

        else if (classes[i] == 5) // if vehicle-train
        {
            label = "vehicle-train " + std::to_string(scores[i]).substr(0, 4);
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
            cv::Point topLeft(x, y - textSize.height); 
            cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), colors[classes[i]], cv::FILLED);
            cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
            if (scores[i] > 0.7)
                cv::rectangle(img, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 4);
        }
        
        else if (classes[i] == 6 && signScores[j]>0.5 && j<signLabels.size())  // if traffic light
        {
            label = signLabels[j];
            std::cout << "label: " << label << std::endl;
            if (label == "red light") 
            {
                label = label +  " " + std::to_string(signScores[j]).substr(0, 4);  // add score to label
                cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
                cv::Point topLeft(x, y - textSize.height);           
                cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), cv::Scalar(0, 0, 255), cv::FILLED); // Red background
                cv::rectangle(img, cv::Rect(x, y, w, h), cv::Scalar(0, 0, 255), 4);
                cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
                j++;
            }              

            else if (label == "green light")
            {
                label = label +  " " + std::to_string(signScores[j]).substr(0, 4);  // add score to label
                cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
                cv::Point topLeft(x, y - textSize.height);  
                cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), cv::Scalar(55, 235, 100), cv::FILLED); // Green background
                cv::rectangle(img, cv::Rect(x, y, w, h), cv::Scalar(55, 235, 100), 4);
                cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
                j++;
            }
                
            else if (label == "yellow light")
            {
                label = label +  " " + std::to_string(signScores[j]).substr(0, 4);  // add score to label
                cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
                cv::Point topLeft(x, y - textSize.height);  
                cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), cv::Scalar(0, 255, 255), cv::FILLED); // Yellow background
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

        else if (classes[i] == 7 && signScores[j]>0.65 && j<signLabels.size())  // if traffic sign
        {
            label = signLabels[j];
            std::cout << "label: " << label << std::endl;

            if (label != "guide sign")
            {
                label = label +  " " + std::to_string(signScores[j]).substr(0, 4);  // add score to label
                cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);
                cv::Point topLeft(x, y - textSize.height); 
                cv::rectangle(img, topLeft, cv::Point(topLeft.x + textSize.width, topLeft.y + textSize.height + baseline), colors[classes[i]], cv::FILLED); 
                cv::putText(img, label, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);   
            }
            j++;
        }

        //cv::putText(img, std::to_string(classes[i]), cv::Point(boxes[i].data[0]/3, boxes[i].data[1]/3), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1);
        //cv::putText(img, std::to_string(scores[i]), cv::Point(boxes[i].data[0]/3, boxes[i].data[1]/3 + 5), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1);
    }

    cv::resize(img, img, cv::Size(1920, 1080));
    cv::imshow("Perception Fusion", img);
    cv::waitKey(1);
}

// input image subscriber callback
void gui_callback(const sensor_msgs::ImageConstPtr input)
{

    // convert the image to reside on GPU
	if( !image_cvt || !image_cvt->Convert(input) )
	{
		ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;	
	}

    // render the image
	stream1->Render(image_cvt->ImageGPU(), image_cvt->GetWidth(), image_cvt->GetHeight());
}

// input image subscriber callback
void signs_callback(const visionconnect::msg::Signs::SharedPtr input)
{
    
    ROS_INFO("%u signs and lights detected", input->classes.size());

    signs_msg = input;

}

// input image subscriber callback
void camera_callback(const sensor_msgs::ImageConstPtr input)
{

    // convert the image to reside on GPU
	if( !image_cvt || !image_cvt->Convert(input) )
	{
		ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;	
	}

    // render the image
	stream1->Render(image_cvt->ImageGPU(), image_cvt->GetWidth(), image_cvt->GetHeight());
}

// detection overlay callback
void detection_callback(const visionconnect::msg::Detect::SharedPtr input)
{

    ROS_INFO("Received detection message");

    detect_msg = input;  

    fuse_data(); // Fuse data from all the topics

//     auto img_msg = std::make_shared<sensor_msgs::msg::Image>(input->image);
     
//         // convert the image to reside on GPU
// 	if( !image_cvt || !image_cvt->Convert(img_msg))
// 	{
// 		ROS_INFO("failed to convert %ux%u %s image", img_msg->width, img_msg->height, img_msg->encoding.c_str());
// 		return;	
// 	}

//     stream2->Render(image_cvt->ImageGPU(), image_cvt->GetWidth(), image_cvt->GetHeight());
}

// lanes overlay callback
void lanes_callback(const visionconnect::msg::Lanes::SharedPtr input)
{

    ROS_INFO("Received detection message");

    lanes_msg = input;  

    // auto img_msg = std::make_shared<sensor_msgs::msg::Image>(input->laneimg);
     
    // // convert the image to reside on GPU
	// if( !image_cvt || !image_cvt->Convert(img_msg))
	// {
	// 	ROS_INFO("failed to convert %ux%u %s image", img_msg->width, img_msg->height, img_msg->encoding.c_str());
	// 	return;	
	// }

    // stream3->Render(image_cvt->ImageGPU(), image_cvt->GetWidth(), image_cvt->GetHeight());
}

// node main loop
int main(int argc, char **argv)
{
    ROS_CREATE_NODE("preview");
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("visionconnect");
    std::string output;

    ROS_DECLARE_PARAMETER("output", output);
    ROS_DECLARE_PARAMETER("width", display_w);
    ROS_DECLARE_PARAMETER("height", display_h);

    ROS_GET_PARAMETER("output", output);
    ROS_GET_PARAMETER("width", display_w);
    ROS_GET_PARAMETER("height", display_h);

    // videoOptions video_options;
    // video_options.width = display_w;
	// video_options.height = display_h;

    stream1 = videoOutput::Create(output.c_str());
    stream2 = videoOutput::Create(output.c_str());
    stream3 = videoOutput::Create(output.c_str());  

    if (!stream1 || !stream2 || !stream3)
	{
		ROS_ERROR("failed to open video output");
		return 0;
	}

    image_cvt = new imageConverter();

    //auto cam_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 1, camera_callback);
    auto detect_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Detect, "detect_in", 1, detection_callback);
    auto signs_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Signs, "signs_in", 1, signs_callback);
    auto lanes_sub = ROS_CREATE_SUBSCRIBER(visionconnect::msg::Lanes, "lanes_in", 1, lanes_callback);
    auto gui_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "gui_in", 1, gui_callback);

	// start publishing video frames
    ROS_INFO("Preview Node initialized, waiting for images");
    ROS_SPIN();
    
    //free resources
	delete stream1;
    delete stream2;
    delete stream3;
	delete image_cvt;

    return 0;
}
