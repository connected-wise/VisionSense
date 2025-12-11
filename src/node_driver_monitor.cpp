/*
 * Driver Monitoring Node
 *
 * Monitors driver attention by subscribing to camera/raw topic
 * Features:
 * - Face detection using Haar Cascade
 * - Facial landmark detection using dlib
 * - Eye aspect ratio (EAR) for drowsiness detection
 * - Head pose estimation
 * - Driver state alerts
 *
 * Simplified implementation using OpenCV without DeepStream dependency
 */

#include "ros_compat.h"
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/face.hpp>

// Driver monitoring states
enum DriverState {
    ALERT = 0,
    DROWSY = 1,
    DISTRACTED = 2,
    NO_DRIVER = 3
};

// Globals
Publisher<sensor_msgs::Image> image_pub = NULL;
Publisher<std_msgs::msg::String> state_pub = NULL;
Publisher<std_msgs::msg::Bool> alert_pub = NULL;
Publisher<std_msgs::msg::Float32> ear_pub = NULL;  // Eye Aspect Ratio
rclcpp::Subscription<sensor_msgs::Image>::SharedPtr image_sub;  // Camera subscription

// OpenCV face and eye detection
cv::CascadeClassifier face_cascade;
cv::CascadeClassifier eye_cascade;
cv::Ptr<cv::face::Facemark> facemark;

// Driver monitoring parameters
int frames_no_face = 0;
int frames_eyes_closed = 0;
const int NO_FACE_THRESHOLD = 30;       // ~1 second at 30fps
const int EYES_CLOSED_THRESHOLD = 45;   // ~1.5 seconds at 30fps
const float EAR_THRESHOLD = 0.21;       // Eye Aspect Ratio threshold for closed eyes

// Eye Aspect Ratio calculation
// EAR = (||p2-p6|| + ||p3-p5||) / (2 * ||p1-p4||)
// where p1-p6 are the 6 landmark points around the eye
float calculateEAR(const std::vector<cv::Point2f>& eye_landmarks)
{
    if (eye_landmarks.size() < 6)
        return 1.0;  // Default to "open" if not enough points

    // Vertical eye landmarks
    float vertical1 = cv::norm(eye_landmarks[1] - eye_landmarks[5]);
    float vertical2 = cv::norm(eye_landmarks[2] - eye_landmarks[4]);

    // Horizontal eye landmarks
    float horizontal = cv::norm(eye_landmarks[0] - eye_landmarks[3]);

    // Eye Aspect Ratio
    float ear = (vertical1 + vertical2) / (2.0 * horizontal);

    return ear;
}

const char* getStateName(DriverState state) {
    switch(state) {
        case ALERT: return "ALERT";
        case DROWSY: return "DROWSY";
        case DISTRACTED: return "DISTRACTED";
        case NO_DRIVER: return "NO_DRIVER";
        default: return "UNKNOWN";
    }
}

// Process frame and detect driver state
DriverState processFrame(cv::Mat& frame, cv::Mat& output_frame)
{
    DriverState current_state = ALERT;
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(gray, gray);  // Improve contrast

    // Detect faces
    std::vector<cv::Rect> faces;
    face_cascade.detectMultiScale(gray, faces, 1.1, 3, 0, cv::Size(80, 80));

    if (faces.empty()) {
        frames_no_face++;
        if (frames_no_face > NO_FACE_THRESHOLD) {
            current_state = NO_DRIVER;
        }

        // Draw status
        cv::putText(output_frame, "NO FACE DETECTED", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        return current_state;
    }

    // Reset no-face counter
    frames_no_face = 0;

    // Get largest face (driver)
    cv::Rect main_face = faces[0];
    float max_area = main_face.area();
    for (const auto& face : faces) {
        if (face.area() > max_area) {
            max_area = face.area();
            main_face = face;
        }
    }

    // Check if face is too small (driver distracted/far)
    float face_ratio = max_area / (float)(frame.cols * frame.rows);
    if (face_ratio < 0.05) {  // Face less than 5% of image
        current_state = DISTRACTED;
    }

    // Draw face rectangle
    cv::rectangle(output_frame, main_face, cv::Scalar(255, 0, 0), 2);

    // Detect eyes in face region
    cv::Mat face_roi_gray = gray(main_face);
    cv::Mat face_roi_color = output_frame(main_face);

    std::vector<cv::Rect> eyes;
    eye_cascade.detectMultiScale(face_roi_gray, eyes, 1.1, 3, 0, cv::Size(20, 20));

    // Try facial landmark detection if available
    float avg_ear = 1.0;
    bool landmarks_detected = false;

    if (!facemark.empty()) {
        std::vector<cv::Rect> face_rects;
        face_rects.push_back(main_face);

        std::vector<std::vector<cv::Point2f>> landmarks;
        bool success = facemark->fit(gray, face_rects, landmarks);

        if (success && !landmarks.empty()) {
            landmarks_detected = true;

            // Draw facial landmarks
            for (const auto& point : landmarks[0]) {
                cv::circle(output_frame, point, 2, cv::Scalar(0, 255, 0), -1);
            }

            // Calculate EAR for both eyes (landmarks 36-41 left eye, 42-47 right eye)
            // Using simplified 6-point eye model
            if (landmarks[0].size() >= 48) {
                std::vector<cv::Point2f> left_eye(landmarks[0].begin() + 36,
                                                   landmarks[0].begin() + 42);
                std::vector<cv::Point2f> right_eye(landmarks[0].begin() + 42,
                                                    landmarks[0].begin() + 48);

                float left_ear = calculateEAR(left_eye);
                float right_ear = calculateEAR(right_eye);
                avg_ear = (left_ear + right_ear) / 2.0;

                // Check for closed eyes
                if (avg_ear < EAR_THRESHOLD) {
                    frames_eyes_closed++;
                    if (frames_eyes_closed > EYES_CLOSED_THRESHOLD) {
                        current_state = DROWSY;
                    }
                } else {
                    frames_eyes_closed = 0;
                }

                // Draw EAR value
                char ear_text[50];
                snprintf(ear_text, sizeof(ear_text), "EAR: %.3f", avg_ear);
                cv::putText(output_frame, ear_text, cv::Point(10, 60),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
            }
        }
    }

    // Fallback to simple eye detection if landmarks not available
    if (!landmarks_detected) {
        // Draw detected eyes
        for (const auto& eye : eyes) {
            cv::Point eye_center(main_face.x + eye.x + eye.width/2,
                                main_face.y + eye.y + eye.height/2);
            cv::circle(output_frame, eye_center, eye.width/2,
                      cv::Scalar(0, 255, 0), 2);
        }

        // Simple drowsiness detection based on number of eyes detected
        if (eyes.size() == 0) {
            frames_eyes_closed++;
            if (frames_eyes_closed > EYES_CLOSED_THRESHOLD) {
                current_state = DROWSY;
            }
        } else {
            frames_eyes_closed = 0;
        }
    }

    // Draw state
    cv::Scalar state_color;
    switch(current_state) {
        case ALERT: state_color = cv::Scalar(0, 255, 0); break;
        case DROWSY: state_color = cv::Scalar(0, 0, 255); break;
        case DISTRACTED: state_color = cv::Scalar(0, 165, 255); break;
        default: state_color = cv::Scalar(128, 128, 128); break;
    }

    cv::putText(output_frame, getStateName(current_state), cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, state_color, 2);

    return current_state;
}

// Image callback - processes incoming images from camera/raw topic
void image_callback(const sensor_msgs::Image::SharedPtr msg)
{
    // Convert ROS Image to OpenCV Mat
    cv::Mat cv_image;

    if (msg->encoding == "rgb8") {
        cv_image = cv::Mat(msg->height, msg->width, CV_8UC3,
                          const_cast<uint8_t*>(msg->data.data()), msg->step);
        // Convert RGB to BGR for OpenCV
        cv::cvtColor(cv_image, cv_image, cv::COLOR_RGB2BGR);
    } else if (msg->encoding == "bgr8") {
        cv_image = cv::Mat(msg->height, msg->width, CV_8UC3,
                          const_cast<uint8_t*>(msg->data.data()), msg->step);
    } else {
        ROS_WARN("Unsupported image encoding: %s", msg->encoding.c_str());
        return;
    }

    // Create output image for annotations
    cv::Mat output_image = cv_image.clone();

    // Process frame and determine driver state
    DriverState state = processFrame(cv_image, output_image);

    // Publish state
    std_msgs::msg::String state_msg;
    state_msg.data = getStateName(state);
    state_pub->publish(state_msg);

    // Publish alert if drowsy or no driver
    std_msgs::msg::Bool alert_msg;
    alert_msg.data = (state == DROWSY || state == NO_DRIVER);
    alert_pub->publish(alert_msg);

    // Publish EAR (Eye Aspect Ratio) - placeholder for now
    std_msgs::msg::Float32 ear_msg;
    ear_msg.data = 0.0;  // Will be updated when landmark detection is used
    ear_pub->publish(ear_msg);

    // Convert back to RGB for ROS
    cv::cvtColor(output_image, output_image, cv::COLOR_BGR2RGB);

    // Publish annotated image
    sensor_msgs::Image img_msg;
    img_msg.header.stamp = msg->header.stamp;
    img_msg.header.frame_id = msg->header.frame_id;
    img_msg.height = output_image.rows;
    img_msg.width = output_image.cols;
    img_msg.encoding = "rgb8";
    img_msg.step = output_image.cols * 3;
    size_t size = img_msg.step * img_msg.height;
    img_msg.data.resize(size);
    memcpy(&img_msg.data[0], output_image.data, size);

    image_pub->publish(img_msg);
}

int main(int argc, char** argv)
{
    ROS_CREATE_NODE("driver_monitor");

    ROS_INFO("Driver Monitoring Node - Starting...");

    // Parameters
    std::string face_cascade_path;
    std::string eye_cascade_path;
    std::string landmark_model_path;
    std::string camera_topic;

    ROS_DECLARE_PARAMETER("face_cascade", face_cascade_path);
    ROS_DECLARE_PARAMETER("eye_cascade", eye_cascade_path);
    ROS_DECLARE_PARAMETER("landmark_model", landmark_model_path);
    ROS_DECLARE_PARAMETER("camera_topic", camera_topic);

    ROS_GET_PARAMETER("face_cascade", face_cascade_path);
    ROS_GET_PARAMETER("eye_cascade", eye_cascade_path);
    ROS_GET_PARAMETER("landmark_model", landmark_model_path);
    ROS_GET_PARAMETER("camera_topic", camera_topic);

    // Default values
    if (face_cascade_path.empty())
        face_cascade_path = "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml";

    if (eye_cascade_path.empty())
        eye_cascade_path = "/usr/share/opencv4/haarcascades/haarcascade_eye.xml";

    if (camera_topic.empty())
        camera_topic = "/camera/raw";

    // Load face cascade
    ROS_INFO("Loading face cascade: %s", face_cascade_path.c_str());
    if (!face_cascade.load(face_cascade_path)) {
        ROS_ERROR("Failed to load face cascade from: %s", face_cascade_path.c_str());
        return 1;
    }

    // Load eye cascade
    ROS_INFO("Loading eye cascade: %s", eye_cascade_path.c_str());
    if (!eye_cascade.load(eye_cascade_path)) {
        ROS_WARN("Failed to load eye cascade, continuing without it");
    }

    // Load facial landmark model (optional)
    if (!landmark_model_path.empty()) {
        ROS_INFO("Loading facial landmark model: %s", landmark_model_path.c_str());
        try {
            facemark = cv::face::FacemarkLBF::create();
            facemark->loadModel(landmark_model_path);
            ROS_INFO("Facial landmark detection enabled");
        } catch (const cv::Exception& e) {
            ROS_WARN("Failed to load landmark model: %s", e.what());
            ROS_WARN("Continuing with basic eye detection");
        }
    }

    // Create publishers (ROS2 will automatically add node name as namespace)
    ROS_CREATE_PUBLISHER(sensor_msgs::Image, "image", 2, image_pub);
    ROS_CREATE_PUBLISHER(std_msgs::msg::String, "state", 10, state_pub);
    ROS_CREATE_PUBLISHER(std_msgs::msg::Bool, "alert", 10, alert_pub);
    ROS_CREATE_PUBLISHER(std_msgs::msg::Float32, "ear", 10, ear_pub);

    // Subscribe to camera topic
    ROS_INFO("Subscribing to camera topic: %s", camera_topic.c_str());
    image_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, camera_topic.c_str(), 2, image_callback);

    ROS_INFO("Driver monitoring started");
    ROS_INFO("Subscribing to:");
    ROS_INFO("  - %s (camera input)", camera_topic.c_str());
    ROS_INFO("Publishing to:");
    ROS_INFO("  - /driver_monitor/image (annotated video)");
    ROS_INFO("  - /driver_monitor/state (ALERT/DROWSY/DISTRACTED/NO_DRIVER)");
    ROS_INFO("  - /driver_monitor/alert (bool: true if drowsy/no driver)");
    ROS_INFO("  - /driver_monitor/ear (Eye Aspect Ratio)");

    // Spin
    ROS_SPIN();

    ROS_INFO("Driver monitoring shutting down");

    return 0;
}
