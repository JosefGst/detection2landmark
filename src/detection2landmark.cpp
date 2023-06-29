#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "std_msgs/msg/string.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "apriltag_msgs/msg/april_tag_detection.hpp"
#include "cartographer_ros_msgs/msg/landmark_list.hpp"
#include "cartographer_ros_msgs/msg/landmark_entry.hpp"

using std::placeholders::_1;

class Detection2Landmark : public rclcpp::Node
{
public:
    Detection2Landmark()
        : Node("detection2landmark")
    {
        subscription_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "camF/detections", 10, std::bind(&Detection2Landmark::detection_callback, this, _1));

        publisher_ = this->create_publisher<cartographer_ros_msgs::msg::LandmarkList>("landmark", 10);

        // Declare and acquire `target_frame` parameter
        target_frame_ = this->declare_parameter<std::string>("tag36h11:2", "camF");
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    void detection_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        std::string fromFrameRel = target_frame_.c_str();
        //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->header.frame_id.c_str());

        auto landmark = cartographer_ros_msgs::msg::LandmarkList();
        auto landmark_entry = cartographer_ros_msgs::msg::LandmarkEntry();
        auto april_entry = apriltag_msgs::msg::AprilTagDetection();

        landmark.header = msg->header;
        landmark_entry.id = msg->detections[0].id;

        for (int tag_id = 0; tag_id < msg->detections.size(); tag_id++)
        {
            // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->header.frame_id.c_str());
            landmark_entry.id = std::to_string(msg->detections[tag_id].id);
            // landmark_entry.tracking_from_landmark_transform.position = msg->
            landmark_entry.translation_weight = 1e2;
            landmark_entry.rotation_weight = 1e2;
            landmark.landmarks.push_back(landmark_entry);
        }

        
        publisher_->publish(landmark);
    }

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_;

    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr subscription_;
    rclcpp::Publisher<cartographer_ros_msgs::msg::LandmarkList>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Detection2Landmark>());
    rclcpp::shutdown();
    return 0;
}