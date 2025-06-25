#ifndef COMMON_HPP
#define COMMON_HPP

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// ORB_SLAM3 specific includes
#include "System.h"

namespace orb_slam3_ros2
{

class Node
{
public:
    Node(const rclcpp::NodeOptions& options);
    ~Node();
    
    std::shared_ptr<rclcpp::Node> node_;

protected:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr orb_pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tracked_mappoints_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tracking_state_publisher_;

    std::string world_frame_id_;
    std::string camera_frame_id_;
    std::string map_frame_id_;
    std::string reference_frame_id_;
    bool publish_tf_;
    bool publish_pose_;
    bool publish_orb_pose_;
    bool publish_map_points_;
    bool publish_tracked_points_;
    bool publish_tracking_state_;
    bool apply_world_frame_rotation_;

    void PublishPose(const Sophus::SE3f &Tcw);
    void PublishMapPoints(const std::vector<ORB_SLAM3::MapPoint*>& map_points);
    void PublishTrackedMapPoints(const std::vector<ORB_SLAM3::MapPoint*>& map_points);
    void PublishTrackingState(const int& state);
    tf2::Transform TransformFromSE3(const Sophus::SE3f &se3);
    sensor_msgs::msg::PointCloud2 MapPointsToPointCloud(const std::vector<ORB_SLAM3::MapPoint*>& map_points);

    // ORB_SLAM3 system
    ORB_SLAM3::System* slam_system_;
};

} // namespace orb_slam3_ros2

#endif // COMMON_HPP