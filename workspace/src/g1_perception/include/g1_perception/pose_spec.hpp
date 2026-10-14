#ifndef G1_PERCEPTION__POSE_SPEC_HPP_
#define G1_PERCEPTION__POSE_SPEC_HPP_

/**
 * @file pose_spec.hpp
 * @brief The one-line pose syntax the tracker's YAML uses for priors and offsets.
 *
 *   "<frame> x y z [roll pitch yaw]"   a pose in a named frame (a TF frame, or another tracked
 *                                      object's name), metres and radians, RPY about fixed axes
 *                                      as in ROS (R = Rz(yaw) Ry(pitch) Rx(roll))
 *   "x y z [roll pitch yaw]"           an unframed pose (offsets)
 *   "<axis> start stop step"           a sweep of translations along x, y or z (both ends
 *                                      included), for an object whose position along a known
 *                                      axis is unknown -- a pallet somewhere on its line
 *
 * One string per pose, because ROS 2 parameters cannot hold a list of structures, and a list of
 * strings reads in the YAML much like the poses it holds. No ROS.
 */

#include <Eigen/Geometry>
#include <optional>
#include <string>
#include <vector>

namespace g1_perception
{

struct FramedPose
{
    std::string       frame;
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
};

/// "x y z [roll pitch yaw]". Nothing (with `error` set) on anything else.
std::optional<Eigen::Isometry3f> parsePose(const std::string& text, std::string& error);

/// "<frame> x y z [roll pitch yaw]".
std::optional<FramedPose> parseFramedPose(const std::string& text, std::string& error);

/// "<axis> start stop step" -> translations; nothing on a malformed or runaway (> 10000) sweep.
std::optional<std::vector<Eigen::Isometry3f>> parseSweep(const std::string& text, std::string& error);

Eigen::Isometry3f poseFromXyzRpy(float x, float y, float z, float roll, float pitch, float yaw);

}  // namespace g1_perception

#endif  // G1_PERCEPTION__POSE_SPEC_HPP_
