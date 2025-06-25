#include "orb_slam3_ros2/common.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace orb_slam3_ros2
{

Node::Node(const rclcpp::NodeOptions& options)
    : node_(std::make_shared<rclcpp::Node>("orb_slam3_ros2", options)),
      tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(node_))
{
    node_->declare_parameter("world_frame_id", "map");
    node_->declare_parameter("camera_frame_id", "camera_link");
    node_->declare_parameter("map_frame_id", "orb_slam3_map");
    node_->declare_parameter("reference_frame_id", "orb_slam3_reference");
    node_->declare_parameter("publish_tf", true);
    node_->declare_parameter("publish_pose", true);
    node_->declare_parameter("publish_orb_pose", true);
    node_->declare_parameter("publish_map_points", true);
    node_->declare_parameter("publish_tracked_points", true);
    node_->declare_parameter("publish_tracking_state", true);
    node_->declare_parameter("apply_world_frame_rotation", true);

    node_->get_parameter("world_frame_id", world_frame_id_);
    node_->get_parameter("camera_frame_id", camera_frame_id_);
    node_->get_parameter("map_frame_id", map_frame_id_);
    node_->get_parameter("reference_frame_id", reference_frame_id_);
    node_->get_parameter("publish_tf", publish_tf_);
    node_->get_parameter("publish_pose", publish_pose_);
    node_->get_parameter("publish_orb_pose", publish_orb_pose_);
    node_->get_parameter("publish_map_points", publish_map_points_);
    node_->get_parameter("publish_tracked_points", publish_tracked_points_);
    node_->get_parameter("publish_tracking_state", publish_tracking_state_);
    node_->get_parameter("apply_world_frame_rotation", apply_world_frame_rotation_);

    if (publish_pose_) {
        pose_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("orb_slam3/camera_pose", 10);
    }
    
    if (publish_orb_pose_) {
        orb_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/orb_pose", 10);
    }
    
    if (publish_map_points_) {
        map_points_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("orb_slam3/map_points", 10);
    }
    
    if (publish_tracked_points_) {
        tracked_mappoints_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("orb_slam3/tracked_points", 10);
    }
    
    if (publish_tracking_state_) {
        tracking_state_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("orb_slam3/tracking_state", 10);
    }
}

Node::~Node()
{
    if (slam_system_) {
        slam_system_->Shutdown();
        delete slam_system_;
        slam_system_ = nullptr;
    }
}

void Node::PublishPose(const Sophus::SE3f &Tcw)
{
    if (!publish_pose_ && !publish_tf_ && !publish_orb_pose_)
        return;

    tf2::Transform tf_transform = TransformFromSE3(Tcw);
    
    if (publish_tf_) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = node_->now();
        transform_stamped.header.frame_id = world_frame_id_;  
        transform_stamped.child_frame_id = camera_frame_id_;
        
        transform_stamped.transform.translation.x = tf_transform.getOrigin().getX();
        transform_stamped.transform.translation.y = tf_transform.getOrigin().getY();
        transform_stamped.transform.translation.z = tf_transform.getOrigin().getZ();
        transform_stamped.transform.rotation.x = tf_transform.getRotation().getX();
        transform_stamped.transform.rotation.y = tf_transform.getRotation().getY();
        transform_stamped.transform.rotation.z = tf_transform.getRotation().getZ();
        transform_stamped.transform.rotation.w = tf_transform.getRotation().getW();
        
        tf_broadcaster_->sendTransform(transform_stamped);
        
        geometry_msgs::msg::TransformStamped map_transform;
        map_transform.header.stamp = node_->now();
        map_transform.header.frame_id = world_frame_id_;
        map_transform.child_frame_id = map_frame_id_;
        
        map_transform.transform.translation.x = 0.0;
        map_transform.transform.translation.y = 0.0;
        map_transform.transform.translation.z = 0.0;
        map_transform.transform.rotation.x = 0.0;
        map_transform.transform.rotation.y = 0.0;
        map_transform.transform.rotation.z = 0.0;
        map_transform.transform.rotation.w = 1.0;
        
        tf_broadcaster_->sendTransform(map_transform);
    }
    
    if (publish_pose_) {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = node_->now();
        odom_msg.header.frame_id = world_frame_id_;
        odom_msg.child_frame_id = camera_frame_id_;
        
        odom_msg.pose.pose.position.x = tf_transform.getOrigin().getX();
        odom_msg.pose.pose.position.y = tf_transform.getOrigin().getY();
        odom_msg.pose.pose.position.z = tf_transform.getOrigin().getZ();
        odom_msg.pose.pose.orientation.x = tf_transform.getRotation().getX();
        odom_msg.pose.pose.orientation.y = tf_transform.getRotation().getY();
        odom_msg.pose.pose.orientation.z = tf_transform.getRotation().getZ();
        odom_msg.pose.pose.orientation.w = tf_transform.getRotation().getW();
        
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;
        
        pose_publisher_->publish(odom_msg);
    }
    
    if (publish_orb_pose_) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = node_->now();
        pose_msg.header.frame_id = world_frame_id_;
        
        pose_msg.pose.position.x = tf_transform.getOrigin().getX();
        pose_msg.pose.position.y = tf_transform.getOrigin().getY();
        pose_msg.pose.position.z = tf_transform.getOrigin().getZ();
        pose_msg.pose.orientation.x = tf_transform.getRotation().getX();
        pose_msg.pose.orientation.y = tf_transform.getRotation().getY();
        pose_msg.pose.orientation.z = tf_transform.getRotation().getZ();
        pose_msg.pose.orientation.w = tf_transform.getRotation().getW();
        
        orb_pose_publisher_->publish(pose_msg);
    }
}

void Node::PublishMapPoints(const std::vector<ORB_SLAM3::MapPoint*>& map_points)
{
    if (!publish_map_points_)
        return;
    
    std::vector<ORB_SLAM3::MapPoint*> valid_map_points;
    for (const auto& map_point : map_points) {
        if (map_point && !map_point->isBad()) {
            valid_map_points.push_back(map_point);
        }
    }
    
    if (valid_map_points.empty()) {
        RCLCPP_DEBUG(node_->get_logger(), "No valid map points to publish");
        return;
    }
        
    sensor_msgs::msg::PointCloud2 cloud = MapPointsToPointCloud(valid_map_points);
    map_points_publisher_->publish(cloud);
}

void Node::PublishTrackedMapPoints(const std::vector<ORB_SLAM3::MapPoint*>& map_points)
{
    if (!publish_tracked_points_)
        return;
    
    std::vector<geometry_msgs::msg::Point> valid_points;
    
    for (const auto& map_point : map_points) {
        if (map_point && !map_point->isBad()) {
            Eigen::Vector3f pos = map_point->GetWorldPos();
            
            geometry_msgs::msg::Point point;
            point.x = pos[0];
            point.y = pos[1];
            point.z = pos[2];
            valid_points.push_back(point);
        }
    }
    
    if (valid_points.empty()) {
        RCLCPP_DEBUG(node_->get_logger(), "No valid tracked points to publish");
        return;
    }
        
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = world_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "tracked_points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.points = valid_points;
    
    tracked_mappoints_publisher_->publish(marker);
}

void Node::PublishTrackingState(const int& state)
{
    if (!publish_tracking_state_)
        return;
        
    std_msgs::msg::Bool msg;
    msg.data = (state == ORB_SLAM3::Tracking::OK);
    tracking_state_publisher_->publish(msg);
}


tf2::Transform Node::TransformFromSE3(const Sophus::SE3f &se3)
{
    Eigen::Matrix3f rotation = se3.rotationMatrix();
    Eigen::Vector3f translation = se3.translation();
    
    tf2::Matrix3x3 tf_camera_rotation(
        rotation(0, 0), rotation(0, 1), rotation(0, 2),
        rotation(1, 0), rotation(1, 1), rotation(1, 2),
        rotation(2, 0), rotation(2, 1), rotation(2, 2)
    );
    
    tf2::Vector3 tf_camera_translation(
        translation(0), translation(1), translation(2)
    );
    
    if (apply_world_frame_rotation_) {
        const tf2::Matrix3x3 world_frame_rotation(
            0, 0, 1,
            -1, 0, 0,
            0, -1, 0
        );
        
        tf_camera_rotation = world_frame_rotation * tf_camera_rotation;
        tf_camera_translation = world_frame_rotation * tf_camera_translation;
        
        tf_camera_rotation = tf_camera_rotation.transpose();
        tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);
        
        tf_camera_rotation = world_frame_rotation * tf_camera_rotation;
        tf_camera_translation = world_frame_rotation * tf_camera_translation;
    } else {
        tf_camera_rotation = tf_camera_rotation.transpose();
        tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);
    }
    
    return tf2::Transform(tf_camera_rotation, tf_camera_translation);
}

sensor_msgs::msg::PointCloud2 Node::MapPointsToPointCloud(const std::vector<ORB_SLAM3::MapPoint*>& map_points)
{
    sensor_msgs::msg::PointCloud2 cloud;
    
    const size_t num_channels = 3;
    cloud.header.stamp = node_->now();
    cloud.header.frame_id = world_frame_id_;
    
    size_t valid_points = 0;
    for (const auto& point : map_points) {
        if (point && !point->isBad()) {
            valid_points++;
        }
    }
    
    if (valid_points == 0) {
        RCLCPP_WARN(node_->get_logger(), "No valid points for point cloud");
        return cloud;
    }
    
    cloud.height = 1;
    cloud.width = valid_points;
    cloud.is_dense = true;
    cloud.is_bigendian = false;
    cloud.fields.resize(num_channels);
    
    cloud.fields[0].name = "x";
    cloud.fields[0].offset = 0;
    cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[0].count = 1;
    
    cloud.fields[1].name = "y";
    cloud.fields[1].offset = 4;
    cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[1].count = 1;
    
    cloud.fields[2].name = "z";
    cloud.fields[2].offset = 8;
    cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[2].count = 1;
    
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step * cloud.height);
    
    unsigned char* cloud_data_ptr = &(cloud.data[0]);
    
    for (const auto& map_point : map_points) {
        if (map_point && !map_point->isBad()) {
            Eigen::Vector3f pos = map_point->GetWorldPos();
            
            if (apply_world_frame_rotation_) {
                float x = pos[0];
                float y = pos[1];
                float z = pos[2];
                
                float temp_x = z;
                float temp_y = -x;
                float temp_z = -y;
                
                pos[0] = temp_x;
                pos[1] = temp_y;
                pos[2] = temp_z;
            }
            
            float* data_ptr = reinterpret_cast<float*>(cloud_data_ptr);
            data_ptr[0] = pos[0];
            data_ptr[1] = pos[1];
            data_ptr[2] = pos[2];
            
            cloud_data_ptr += cloud.point_step;
        }
    }
    
    return cloud;
}

}