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
        // declare params
        this->declare_parameter("family", "36h11");
        this->declare_parameter("camera_frame", "camF");
        this->declare_parameter("translation_weight", 1e2);
        this->declare_parameter("rotation_weight", 1e2);
        rclcpp::Parameter family = this->get_parameter("family");
        rclcpp::Parameter camera_frame = this->get_parameter("camera_frame");
        rclcpp::Parameter translation_weight = this->get_parameter("translation_weight");
        rclcpp::Parameter rotation_weight = this->get_parameter("rotation_weight");
        tag_family_ = "tag" + family.as_string() + ":";
        camera_frame_ = camera_frame.as_string();
        translation_weight_ = translation_weight.as_double();
        rotation_weight_ = rotation_weight.as_double();

        subscription_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "camF/detections", 10, std::bind(&Detection2Landmark::detection_callback, this, _1));

        publisher_ = this->create_publisher<cartographer_ros_msgs::msg::LandmarkList>("landmark", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    void detection_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        auto landmark = cartographer_ros_msgs::msg::LandmarkList();
        auto landmark_entry = cartographer_ros_msgs::msg::LandmarkEntry();
        auto april_entry = apriltag_msgs::msg::AprilTagDetection();

        landmark.header = msg->header;
        landmark_entry.id = msg->detections[0].id;

        for (int tag_id = 0; tag_id < msg->detections.size(); tag_id++)
        {

            std::string toFrameRel = camera_frame_;
            std::string fromFrameRel = tag_family_;
            geometry_msgs::msg::TransformStamped trans;

            landmark_entry.id = std::to_string(msg->detections[tag_id].id);

            // get position and rotation from tf
            try
            {
                fromFrameRel = fromFrameRel.append(landmark_entry.id);
                trans = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), tag_family_.c_str(), ex.what());
                return;
            }
            landmark_entry.tracking_from_landmark_transform.position.x = trans.transform.translation.x;
            landmark_entry.tracking_from_landmark_transform.position.y = trans.transform.translation.y;
            landmark_entry.tracking_from_landmark_transform.position.z = trans.transform.translation.z;
            landmark_entry.tracking_from_landmark_transform.orientation.x = trans.transform.rotation.x;
            landmark_entry.tracking_from_landmark_transform.orientation.y = trans.transform.rotation.y;
            landmark_entry.tracking_from_landmark_transform.orientation.z = trans.transform.rotation.z;
            landmark_entry.tracking_from_landmark_transform.orientation.w = trans.transform.rotation.w;
            landmark_entry.translation_weight = translation_weight_;
            landmark_entry.rotation_weight = rotation_weight_;
            landmark.landmarks.push_back(landmark_entry);
        }

        publisher_->publish(landmark);
    }

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string tag_family_;
    std::string camera_frame_;
    double translation_weight_;
    double rotation_weight_;

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