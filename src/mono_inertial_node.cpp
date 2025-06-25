#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <mutex>

#include "orb_slam3_ros2/common.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>

namespace orb_slam3_ros2
{

class MonoInertialNode : public Node
{
public:
    explicit MonoInertialNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MonoInertialNode();

private:
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    
    std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
    std::mutex imu_mutex_;
    
    std::string vocabulary_file_path_;
    std::string settings_file_path_;
    bool use_viewer_;
    
    double last_image_timestamp_;
};

MonoInertialNode::MonoInertialNode(const rclcpp::NodeOptions& options)
    : Node(options),
      last_image_timestamp_(0.0)
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
    
    ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
    slam_system_ = new ORB_SLAM3::System(
        vocabulary_file_path_,
        settings_file_path_,
        sensor_type,
        use_viewer_
    );
    
    image_subscriber_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", 
        10, 
        std::bind(&MonoInertialNode::ImageCallback, this, std::placeholders::_1)
    );
    
    imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 
        100, 
        std::bind(&MonoInertialNode::ImuCallback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(node_->get_logger(), "Monocular-Inertial node is ready");
}

MonoInertialNode::~MonoInertialNode()
{
}

void MonoInertialNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_queue_.push(msg);
}

void MonoInertialNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    
    std::vector<ORB_SLAM3::IMU::Point> imu_measurements;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        while (!imu_queue_.empty() && rclcpp::Time(imu_queue_.front()->header.stamp).seconds() <= timestamp) {
            auto imu_msg = imu_queue_.front();
            imu_queue_.pop();
            
            double imu_timestamp = rclcpp::Time(imu_msg->header.stamp).seconds();
            
            if (imu_timestamp <= last_image_timestamp_) {
                continue;
            }
            
            ORB_SLAM3::IMU::Point imu_point(
                imu_msg->linear_acceleration.x,
                imu_msg->linear_acceleration.y,
                imu_msg->linear_acceleration.z,
                imu_msg->angular_velocity.x,
                imu_msg->angular_velocity.y,
                imu_msg->angular_velocity.z,
                imu_timestamp
            );
            
            imu_measurements.push_back(imu_point);
        }
    }
    
    last_image_timestamp_ = timestamp;
    
    Sophus::SE3f Tcw = slam_system_->TrackMonocular(cv_ptr->image, timestamp, imu_measurements);
    
    if (!Tcw.matrix().isZero()) {
        PublishPose(Tcw);
        PublishMapPoints(slam_system_->GetTrackedMapPoints());
        PublishTrackedMapPoints(slam_system_->GetTrackedMapPoints());
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
        auto node = std::make_shared<orb_slam3_ros2::MonoInertialNode>();
        rclcpp::spin(node->node_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("mono_inertial_node"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}