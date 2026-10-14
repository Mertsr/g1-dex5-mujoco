#ifndef G1_PERCEPTION__OBJECT_TRACKER_NODE_HPP_
#define G1_PERCEPTION__OBJECT_TRACKER_NODE_HPP_

/**
 * @file object_tracker_node.hpp
 * @brief Model-based 6D tracking of known parts from the head depth camera, at camera rate.
 *
 * Depth in, one Detection3DArray per frame out, in the camera frame and at the image's stamp --
 * the contract g1_object_pose_source's `depth_tracker` source consumes, so the skills read
 * these poses from /objects exactly as they read simulator ground truth.
 *
 * All the tracking is g1_perception::MultiTracker; this node converts messages and answers its
 * questions about frames from TF. See README.md for the configuration.
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <map>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "g1_perception/multi_tracker.hpp"

namespace g1_perception
{

class ObjectTrackerNode : public rclcpp::Node
{
public:
    explicit ObjectTrackerNode(const rclcpp::NodeOptions& options);

private:
    std::vector<ObjectConfig>        readObjects();
    void                             onInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void                             onDepth(const sensor_msgs::msg::Image::SharedPtr msg);
    std::optional<Eigen::Isometry3f> lookup(const std::string& target, const std::string& source,
                                            const rclcpp::Time& stamp, double timeout_s);
    void publishStatus(const std_msgs::msg::Header& header, const std::vector<MultiTracker::Result>& results,
                       double frame_ms);

    std::string world_frame_;
    std::string output_frame_;
    double      depth_scale_;
    double      tf_timeout_s_;
    bool        publish_tf_;

    std::unique_ptr<MultiTracker>                multi_;
    std::optional<Intrinsics>                    intrinsics_;
    std::unique_ptr<tf2_ros::Buffer>             tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>  tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr           depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr      info_sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr   detections_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
    rclcpp::Time                                                       last_status_{ 0, 0, RCL_ROS_TIME };
    // Frame timing over the last status period, for the log line and the diagnostics.
    double sum_ms_ = 0.0, max_ms_ = 0.0;
    int    frames_ = 0;
    std::map<std::string, int> fallback_starts_, valid_frames_;
};

}  // namespace g1_perception

#endif  // G1_PERCEPTION__OBJECT_TRACKER_NODE_HPP_
