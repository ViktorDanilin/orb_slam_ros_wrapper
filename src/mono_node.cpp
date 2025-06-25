#include <memory>
#include <string>
#include <vector>

#include "orb_slam3_ros2/common.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

namespace orb_slam3_ros2
{

class MonoNode : public Node
{
public:
    explicit MonoNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MonoNode();

private:
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    
    std::string vocabulary_file_path_;
    std::string settings_file_path_;
    bool use_viewer_;
};

MonoNode::MonoNode(const rclcpp::NodeOptions& options)
    : Node(options)
{

    node_->declare_parameter("vocabulary_file", "");
    node_->declare_parameter("settings_file", "");
    node_->declare_parameter("use_viewer", true);
    

    node_->get_parameter("vocabulary_file", vocabulary_file_path_);
    node_->get_parameter("settings_file", settings_file_path_);
    node_->get_parameter("use_viewer", use_viewer_);
    

    if (vocabulary_file_path_.empty() || settings_file_path_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Vocabulary or settings file path is empty!");
        throw std::runtime_error("Invalid paths to vocabulary or settings files");
    }
    

    ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::MONOCULAR;
    slam_system_ = new ORB_SLAM3::System(
        vocabulary_file_path_,
        settings_file_path_,
        sensor_type,
        use_viewer_
    );
    

    image_subscriber_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw",
        10,
        std::bind(&MonoNode::ImageCallback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(node_->get_logger(), "Monocular node is ready");
}

MonoNode::~MonoNode()
{

}

void MonoNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tcw = slam_system_->TrackMonocular(cv_ptr->image, rclcpp::Time(msg->header.stamp).seconds());
    
    if (!Tcw.matrix().isZero()) {
        auto tracked_points = slam_system_->GetTrackedMapPoints();
        
        PublishPose(Tcw);
        
        if (!tracked_points.empty()) {
            PublishMapPoints(tracked_points);
            
            PublishTrackedMapPoints(tracked_points);
        } else {
            RCLCPP_DEBUG(node_->get_logger(), "No tracked points available");
        }
        
        PublishTrackingState(slam_system_->GetTrackingState());
    } else {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Tracking failed");
    }
}

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<orb_slam3_ros2::MonoNode>();
        rclcpp::spin(node->node_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("mono_node"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}