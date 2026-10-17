#include "g1_manipulation/g1_manipulation_server_node.hpp"

#include <moveit/robot_state/attached_body.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <utility>
#include <vector>

namespace g1_manipulation
{

namespace
{

// A collision box never shrinks below this in any axis. MoveIt treats a zero-extent primitive
// as degenerate and the attach silently does nothing, which then reads as a planner that
// ignored the object.
constexpr double kMinPrimitiveExtent = 0.005;

rclcpp::QoS objectsQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

}  // namespace

bool resolveArm(const std::string& arm, ArmContext& out)
{
    if (arm != "left" && arm != "right")
    {
        return false;
    }
    out.arm_group   = arm + "_arm";
    out.hand_group  = arm + "_hand";
    out.palm_link   = arm + "_hand_palm_link";
    out.grasp_frame = arm + "_hand_grasp_frame";
    out.pinch_frame = arm + "_hand_pinch_frame";
    out.is_left     = arm == "left";
    return true;
}

geometry_msgs::msg::Pose handGoalFromGrasp(const geometry_msgs::msg::Pose& grasp, bool is_left)
{
    if (!is_left)
    {
        return grasp;
    }
    tf2::Transform T;
    tf2::fromMsg(grasp, T);
    tf2::Quaternion flip;
    flip.setRPY(M_PI, 0.0, 0.0);
    T.setRotation(T.getRotation() * flip);
    geometry_msgs::msg::Pose out;
    tf2::toMsg(T, out);
    return out;
}

geometry_msgs::msg::Pose backOff(const geometry_msgs::msg::Pose& pose, double distance)
{
    tf2::Transform T;
    tf2::fromMsg(pose, T);
    const tf2::Vector3 x = tf2::Matrix3x3(T.getRotation()).getColumn(0);
    geometry_msgs::msg::Pose out = pose;
    out.position.x -= distance * x.x();
    out.position.y -= distance * x.y();
    out.position.z -= distance * x.z();
    return out;
}

std::string GraspVariant::suffix() const
{
    if (std::abs(tilt) < 1e-6 && std::abs(lean) < 1e-6 && !flipped)
    {
        return "";
    }
    char buf[80];
    std::snprintf(buf, sizeof buf, " (tilt %+.0f deg, lean %+.0f deg%s)", tilt * 180.0 / M_PI, lean * 180.0 / M_PI,
                  flipped ? ", turned over" : "");
    return buf;
}

std::vector<GraspVariant> graspVariants(const geometry_msgs::msg::Pose& grasp, double max_tilt, double step,
                                        bool allow_flip, double min_descent)
{
    std::vector<double> tilts{ 0.0 };
    if (step > 1e-6)
    {
        for (double t = step; t <= max_tilt + 1e-9; t += step)
        {
            tilts.push_back(t);
            tilts.push_back(-t);
        }
    }
    tf2::Quaternion base;
    tf2::fromMsg(grasp.orientation, base);
    std::vector<GraspVariant> out;
    for (const bool flipped : allow_flip ? std::vector<bool>{ false, true } : std::vector<bool>{ false })
    {
        tf2::Quaternion over;
        over.setRPY(flipped ? M_PI : 0.0, 0.0, 0.0);
        for (const double t : tilts)
        {
            tf2::Quaternion tilt;
            tilt.setRPY(0.0, t, 0.0);  // about the grasp's own y: the closing axis
            const tf2::Quaternion q = (base * over * tilt).normalized();
            if (tf2::Matrix3x3(q).getColumn(0).z() > -min_descent)
            {
                continue;  // fingers level or pointing up: not coming down onto the part
            }
            GraspVariant v;
            v.tilt          = t;
            v.flipped       = flipped;
            v.pose          = grasp;
            v.pose.orientation = tf2::toMsg(q);
            out.push_back(v);
        }
    }
    return out;
}

std::optional<geometry_msgs::msg::Pose> findHypothesis(
    const vision_msgs::msg::Detection3D& detection, const std::string& class_id)
{
    for (const auto& r : detection.results)
    {
        if (r.hypothesis.class_id == class_id)
        {
            return r.pose.pose;
        }
    }
    return std::nullopt;
}

std::optional<std::pair<std::string, geometry_msgs::msg::Pose>> findHypothesisPrefix(
    const vision_msgs::msg::Detection3D& detection, const std::string& prefix)
{
    for (const auto& r : detection.results)
    {
        if (r.hypothesis.class_id.rfind(prefix, 0) == 0)
        {
            return std::make_pair(r.hypothesis.class_id, r.pose.pose);
        }
    }
    return std::nullopt;
}

G1ManipulationServer::G1ManipulationServer(const rclcpp::NodeOptions& options)
  : rclcpp::Node("g1_manipulation_server", options)
{
    object_timeout_s_         = declare_parameter<double>("object_timeout_ms", 1000.0) / 1000.0;
    approach_height_m_        = declare_parameter<double>("approach_height_m", 0.22);
    grasp_height_above_top_m_ = declare_parameter<double>("grasp_height_above_top_m", 0.010);
    lift_height_m_            = declare_parameter<double>("lift_height_m", 0.15);
    // How far a released object may be from where it was aimed before the place is a failure.
    place_tolerance_m_ = declare_parameter<double>("place_tolerance_m", 0.08);
    // Well under the joint limits' own 0.8 rad/s cap. Arm motion disturbs a standing humanoid
    // measurably, and slowing the whole path is preferred over clamping joints, which would
    // bend the path itself.
    velocity_scaling_ = declare_parameter<double>("velocity_scaling", 0.3);
    planning_time_s_  = declare_parameter<double>("planning_time_s", 5.0);

    // How the hand is held at the grasp, for the RIGHT hand; the left mirrors its roll.
    //
    // WHERE the hand grips is not here: it is the {side}_hand_grasp_frame link in the URDF,
    // which is a property of the hand's geometry rather than of this task. Goals are given for
    // that frame, so nothing in this file adds an offset to a palm pose.
    //
    // The fingers close toward the palm's +y, so it is the ROLL that turns the closing axis
    // downward for a grasp off a table, and the pitch that would be wrong.
    grasp_rpy_ = declare_parameter<std::vector<double>>("grasp_rpy", { -M_PI_2, 0.0, 0.0 });
    // For objects that publish their own grasp ("<object>/grasp/<name>" on /objects): the hand
    // goes to a point this far back along the grasp's approach axis, then straight in.
    // 0.20, not less: the RH56E2's straight fingers reach 0.11 m past its grasp point, so a
    // shorter stand-off puts the fingertips at the part's top already at the pregrasp, and the
    // transit there drags them across it.
    grasp_approach_m_    = declare_parameter<double>("grasp_approach_m", 0.20);
    // Re-aiming before the descent follows a part this far at most; further, it was knocked.
    reaim_max_shift_m_   = declare_parameter<double>("reaim_max_shift_m", 0.04);
    locate_timeout_s_    = declare_parameter<double>("locate_timeout_s", 3.0);
    linear_min_fraction_ = declare_parameter<double>("linear_min_fraction", 0.95);
    // A published grasp is tried turned about its closing axis by up to this much, in these
    // steps, and with the hand turned over if allowed; only tilts whose fingers still point down
    // by at least grasp_min_descent (the z of the approach direction) are kept.
    grasp_tilt_max_rad_  = declare_parameter<double>("grasp_tilt_max_deg", 15.0) * M_PI / 180.0;
    grasp_tilt_step_rad_ = declare_parameter<double>("grasp_tilt_step_deg", 15.0) * M_PI / 180.0;
    grasp_allow_flip_    = declare_parameter<bool>("grasp_allow_flip", false);
    // ... and leaned about the pinched edge (the grasp's z) by up to this much.
    grasp_lean_max_rad_  = declare_parameter<double>("grasp_lean_max_deg", 60.0) * M_PI / 180.0;
    // After clearing the octomap to get a stuck arm moving, how long the sensors get to refill
    // it before re-planning (the updaters run at up to 5 Hz).
    octomap_refill_s_    = declare_parameter<double>("octomap_refill_s", 1.0);
    grasp_min_descent_   = declare_parameter<double>("grasp_min_descent", 0.25);
    grasp_max_plans_     = static_cast<int>(declare_parameter<int64_t>("grasp_max_plans", 6));

    objects_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
        "/objects",
        objectsQos(),
        [this](const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg) { onObjects(msg); });

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    get_scene_   = create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    apply_scene_ = create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    // Reach checks go to move_group's IK service rather than an IK solver in this process: the
    // solver plugin is configured, loaded and proven where the planning happens, and an
    // in-process copy failed to instantiate here ("No kinematics solver instantiated").
    compute_ik_ = create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
    clear_octomap_ = create_client<std_srvs::srv::Empty>("/clear_octomap");
}

bool G1ManipulationServer::allowHandContact(
    const ArmContext& arm, const std::vector<std::string>& touchables, bool allowed,
    bool include_links)
{
    // The links that unavoidably enter occupied space during a grasp: the hand itself, and
    // the wrist that carries it. Taken from the robot model rather than listed, so a renamed
    // link is a build-time problem rather than a silently ineffective exemption.
    MoveGroup* hand = groupFor(arm.hand_group);
    if (hand == nullptr)
    {
        return false;
    }
    std::vector<std::string> links =
        hand->getRobotModel()->getJointModelGroup(arm.hand_group)->getLinkModelNames();
    const std::string side = arm.is_left ? "left" : "right";
    links.push_back(arm.palm_link);
    links.push_back(side + "_wrist_pitch_link");
    links.push_back(side + "_wrist_yaw_link");
    // All THREE wrist joints, matching the touch_links the pick attaches the object with. Roll
    // is easy to miss but is the one that reaches during a grasp, so omitting it leaves the
    // exemption silently incomplete.
    links.push_back(side + "_wrist_roll_link");

    // The current matrix has to be read first: ApplyPlanningScene replaces the whole ACM
    // rather than merging into it, so sending only our entries would drop every
    // self-collision rule the SRDF set up.
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components =
        moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = get_scene_->async_send_request(request);
    // Waited on WITHOUT spinning: the executor already owns this node, and spinning it from
    // here would re-enter the executor from inside its own callback.
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "/get_planning_scene did not answer");
        return false;
    }

    moveit_msgs::msg::AllowedCollisionMatrix acm = future.get()->scene.allowed_collision_matrix;

    // A name the matrix has never seen has to be added as a full row and column first,
    // otherwise the indices below run off the end.
    const auto ensure_entry = [&acm](const std::string& name) {
        if (std::find(acm.entry_names.begin(), acm.entry_names.end(), name) != acm.entry_names.end())
        {
            return;
        }
        acm.entry_names.push_back(name);
        for (moveit_msgs::msg::AllowedCollisionEntry& row : acm.entry_values)
        {
            row.enabled.push_back(false);
        }
        moveit_msgs::msg::AllowedCollisionEntry row;
        row.enabled.assign(acm.entry_names.size(), false);
        acm.entry_values.push_back(row);
    };
    const auto index_of = [&acm](const std::string& name) {
        return static_cast<std::size_t>(
            std::find(acm.entry_names.begin(), acm.entry_names.end(), name) -
            acm.entry_names.begin());
    };

    for (const std::string& touchable : touchables)
    {
        ensure_entry(touchable);
    }
    // The exempted things also have to be allowed to touch EACH OTHER, not just the hand.
    // Once the object is attached it stops being a world object and starts moving with the
    // arm, and lifting it out of a surface drags it through the octomap voxels of that
    // surface -- which is a collision between two things the hand is already allowed to
    // touch, and would otherwise fail the lift with the grasp already made.
    for (std::size_t i = 0; i < touchables.size(); ++i)
    {
        for (std::size_t j = i + 1; j < touchables.size(); ++j)
        {
            const std::size_t a            = index_of(touchables[i]);
            const std::size_t b            = index_of(touchables[j]);
            acm.entry_values[a].enabled[b] = allowed;
            acm.entry_values[b].enabled[a] = allowed;
        }
    }
    for (const std::string& touchable : include_links ? touchables : std::vector<std::string>{})
    {
        const std::size_t other = index_of(touchable);
        for (const std::string& link : links)
        {
            const auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), link);
            if (it == acm.entry_names.end())
            {
                continue;
            }
            const auto index = static_cast<std::size_t>(it - acm.entry_names.begin());
            acm.entry_values[index].enabled[other] = allowed;
            acm.entry_values[other].enabled[index] = allowed;
        }
    }

    auto apply           = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    apply->scene.is_diff = true;
    apply->scene.allowed_collision_matrix = acm;
    auto applied                          = apply_scene_->async_send_request(apply);
    if (applied.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "/apply_planning_scene did not answer");
        return false;
    }
    RCLCPP_INFO(
        get_logger(),
        "%s contact between the %s hand and %zu object(s)",
        allowed ? "allowing" : "restoring",
        side.c_str(),
        touchables.size());
    return applied.get()->success;
}

std::optional<geometry_msgs::msg::Pose> G1ManipulationServer::toPlanningFrame(
    const geometry_msgs::msg::Pose& pose, const std::string& frame_id)
{
    if (frame_id.empty() || frame_id == planning_frame_)
    {
        return pose;
    }

    geometry_msgs::msg::PoseStamped in;
    in.header.frame_id = frame_id;
    // Deliberately the latest available rather than a stamp: the source's own stamp can be a
    // few sample periods old, and tf2 would then either extrapolate or refuse. The robot
    // stands still to manipulate, so latest is the right reading.
    in.header.stamp = rclcpp::Time(0);
    in.pose         = pose;

    try
    {
        return tf_buffer_->transform(in, planning_frame_, std::chrono::milliseconds(500)).pose;
    }
    catch (const tf2::TransformException& e)
    {
        RCLCPP_ERROR(
            get_logger(),
            "cannot transform '%s' into the planning frame '%s': %s",
            frame_id.c_str(),
            planning_frame_.c_str(),
            e.what());
        return std::nullopt;
    }
}

void G1ManipulationServer::initialize()
{
    if (grasp_rpy_.size() != 3)
    {
        throw std::runtime_error("grasp_rpy needs exactly 3 entries");
    }

    // both_arms included: the mission tree uses it to tuck both arms in one motion before
    // walking, and a group this node does not build is a goal it rejects outright. It is only
    // ever driven to named postures here -- pose goals for it would need the subgroup IK map,
    // which is why g1.srdf deliberately gives it no kinematics entry.
    for (const std::string& name :
         { "left_arm", "right_arm", "both_arms", "left_hand", "right_hand" })
    {
        auto group = std::make_shared<MoveGroup>(shared_from_this(), name);
        group->setMaxVelocityScalingFactor(velocity_scaling_);
        group->setMaxAccelerationScalingFactor(velocity_scaling_);
        group->setPlanningTime(planning_time_s_);
        groups_.emplace(name, group);
    }
    planning_frame_ = groups_.at("left_arm")->getPlanningFrame();

    // Servers come up only once the groups are usable. Advertising first would accept a goal
    // this node cannot yet act on, and the caller would see a timeout rather than a refusal.
    pick_server_ = rclcpp_action::create_server<Pick>(
        this,
        "~/pick",
        [](const rclcpp_action::GoalUUID&, const std::shared_ptr<const Pick::Goal>&) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle<Pick>>&) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle<Pick>>& handle) {
            std::thread{ [this, handle] { executePick(handle); } }.detach();
        });

    place_server_ = rclcpp_action::create_server<Place>(
        this,
        "~/place",
        [](const rclcpp_action::GoalUUID&, const std::shared_ptr<const Place::Goal>&) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle<Place>>&) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle<Place>>& handle) {
            std::thread{ [this, handle] { executePlace(handle); } }.detach();
        });

    posture_server_ = rclcpp_action::create_server<SetArmPosture>(
        this,
        "~/set_arm_posture",
        [](const rclcpp_action::GoalUUID&, const std::shared_ptr<const SetArmPosture::Goal>&) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle<SetArmPosture>>&) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle<SetArmPosture>>& handle) {
            std::thread{ [this, handle] { executeSetArmPosture(handle); } }.detach();
        });

    RCLCPP_INFO(
        get_logger(),
        "Manipulation skills up, planning in '%s'. Executing needs the arm and hands already "
        "acquired; this node never takes control authority itself.",
        planning_frame_.c_str());
}

void G1ManipulationServer::onObjects(const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(objects_mutex_);
    objects_ = *msg;
}

std::optional<vision_msgs::msg::Detection3D>
G1ManipulationServer::lookUpObject(const std::string& object_id)
{
    vision_msgs::msg::Detection3DArray snapshot;
    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        snapshot = objects_;
    }

    // Age is judged here rather than at the source, because only the thing about to commit an
    // arm to a grasp knows how old a pose is too old. g1_object_pose_source deliberately
    // forwards the sample's own stamp so this check means something.
    const double age = (now() - rclcpp::Time(snapshot.header.stamp)).seconds();
    if (age > object_timeout_s_)
    {
        RCLCPP_ERROR(
            get_logger(),
            "No usable object poses: newest %.2f s old (limit %.2f). Is g1_object_pose_source "
            "active, and is its source publishing?",
            age,
            object_timeout_s_);
        return std::nullopt;
    }

    for (const vision_msgs::msg::Detection3D& detection : snapshot.detections)
    {
        if (!detection.results.empty() && detection.results.front().hypothesis.class_id == object_id)
        {
            return detection;
        }
    }
    // Several of one part are reported as <id>_1, <id>_2, ... (the tracker's `instances`): asked
    // for <id>, take the one nearest the pelvis, which is the one the arm is likeliest to reach.
    {
        std::optional<vision_msgs::msg::Detection3D> nearest;
        double                                       best = 1e9;
        const std::string                            prefix = object_id + "_";
        for (const vision_msgs::msg::Detection3D& detection : snapshot.detections)
        {
            if (detection.results.empty())
            {
                continue;
            }
            const std::string& id = detection.results.front().hypothesis.class_id;
            if (id.size() <= prefix.size() || id.rfind(prefix, 0) != 0 ||
                id.find_first_not_of("0123456789", prefix.size()) != std::string::npos)
            {
                continue;
            }
            const std::string frame = detection.header.frame_id.empty() ? snapshot.header.frame_id
                                                                        : detection.header.frame_id;
            const auto pose = toPlanningFrame(detection.results.front().pose.pose, frame);
            if (!pose)
            {
                continue;
            }
            const double d = std::hypot(pose->position.x, pose->position.y);
            if (d < best)
            {
                best    = d;
                nearest = detection;
            }
        }
        if (nearest)
        {
            RCLCPP_INFO(get_logger(), "'%s': taking %s, %.2f m from the pelvis", object_id.c_str(),
                        nearest->results.front().hypothesis.class_id.c_str(), best);
            return nearest;
        }
    }
    // Fresh, but without it: a measuring source (the depth tracker) reports only what it is
    // tracking right now. Say what IS there, which is usually the answer.
    std::string seen;
    for (const vision_msgs::msg::Detection3D& detection : snapshot.detections)
    {
        if (!detection.results.empty())
        {
            seen += (seen.empty() ? "" : ", ") + detection.results.front().hypothesis.class_id;
        }
    }
    RCLCPP_ERROR(
        get_logger(),
        "No object called '%s' is being reported right now (reported: %s). With the depth "
        "tracker, the part has to be in the head camera's view and tracked.",
        object_id.c_str(),
        seen.empty() ? "nothing" : seen.c_str());
    return std::nullopt;
}

moveit_msgs::msg::CollisionObject G1ManipulationServer::publishCollisionObject(
    const vision_msgs::msg::Detection3D& detection,
    const geometry_msgs::msg::Pose&      in_planning_frame)
{
    moveit_msgs::msg::CollisionObject object;
    object.id              = detection.results.front().hypothesis.class_id;
    object.header.frame_id = planning_frame_;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    // A box for every object, whatever its real shape. The planner only needs a conservative
    // volume to route around, and the bounding box the pose source reports is exactly that.
    primitive.dimensions = { std::max(detection.bbox.size.x, kMinPrimitiveExtent),
                             std::max(detection.bbox.size.y, kMinPrimitiveExtent),
                             std::max(detection.bbox.size.z, kMinPrimitiveExtent) };

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(in_planning_frame);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // ADD on an id that already exists replaces it, so a re-plan against a moved object does
    // not need a remove first.
    planning_scene_.applyCollisionObjects({ object });
    // Returned so the attach can rebuild the same geometry: by then the world copy has been
    // removed, and an attached body has to carry its own shape.
    return object;
}

geometry_msgs::msg::Pose G1ManipulationServer::graspFrameGoal(
    const geometry_msgs::msg::Pose& object_pose, double object_height_m, const ArmContext& arm) const
{
    // Horizontally the grasp frame goes straight to the object: that frame is defined as the
    // point the hand closes on, so putting it at the object IS the grasp.
    geometry_msgs::msg::Pose goal;
    goal.position = object_pose.position;

    // Vertically it does NOT. Measured from the object's TOP FACE, not its centre.
    //
    // The roll points the closing axis at the floor and the fingertips sit about 24 mm beyond
    // the grasp frame along it, so this offset decides where the fingers end up. Aiming at the
    // centre would put them inside the octomap's 20 mm padding around the surface, closing
    // through it rather than around the object's upper half.
    const double top = object_pose.position.z + 0.5 * object_height_m;
    goal.position.z  = top + grasp_height_above_top_m_;

    // Only the orientation is a choice, and it mirrors: the two hands close in opposite
    // directions, so the roll that points the closing axis at the floor flips sign.
    tf2::Quaternion rotation;
    rotation.setRPY((arm.is_left ? -1.0 : 1.0) * grasp_rpy_[0], grasp_rpy_[1], grasp_rpy_[2]);
    goal.orientation = tf2::toMsg(rotation);
    return goal;
}

bool G1ManipulationServer::moveTo(
    MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
    const std::string& what)
{
    group.setStartStateToCurrentState();
    // Named explicitly rather than relying on the group's default tip: the goal is for the
    // grasp frame, which hangs off the palm and is not what the group ends at.
    group.setPoseTarget(pose, link);

    MoveGroup::Plan plan;
    if (!planWithRecovery(group, plan, what))
    {
        return false;
    }
    const auto executed = group.execute(plan);
    if (executed != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(
            get_logger(),
            "%s: execution failed (%d). The usual cause is the arm not being acquired -- run "
            "g1_bringup's activate_arm first.",
            what.c_str(),
            executed.val);
        return false;
    }
    return true;
}

bool G1ManipulationServer::moveToNamed(MoveGroup& group, const std::string& named_target)
{
    group.setStartStateToCurrentState();
    if (!group.setNamedTarget(named_target))
    {
        RCLCPP_ERROR(
            get_logger(),
            "'%s' is not a named pose of group '%s'",
            named_target.c_str(),
            group.getName().c_str());
        return false;
    }

    // Plan then execute, NOT move(). They are not equivalent: move() runs through MoveIt's
    // PlanExecution, which re-checks the whole remaining path against every planning-scene
    // update and aborts the moment one invalidates it. Against a live octomap fed by a chest
    // camera staring at the arm, the arm's own freshly integrated voxels invalidate it
    // constantly, so a plan that started fine can die partway through against itself.
    //
    // execute() checks nothing during the motion, matching every other move in this file: Pick
    // and Place plan and execute for the pregrasp, descent and lift, all fully collision-checked
    // at PLAN time. This only drops a during-flight recheck that, on this stack, reports the
    // robot's own arm.
    MoveGroup::Plan plan;
    if (!planWithRecovery(group, plan, "'" + named_target + "'"))
    {
        return false;
    }
    return group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool G1ManipulationServer::moveLinear(
    MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
    const std::string& what)
{
    group.setStartStateToCurrentState();
    // computeCartesianPath moves the group's end-effector link; the goal is for the grasp
    // frame, so it stands in as the end effector for this one call.
    const std::string previous = group.getEndEffectorLink();
    group.setEndEffectorLink(link);
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = group.computeCartesianPath({ pose }, 0.005, 0.0, trajectory, true);
    group.setEndEffectorLink(previous);
    if (fraction < linear_min_fraction_)
    {
        RCLCPP_ERROR(
            get_logger(),
            "%s: only %.0f%% of the straight line is reachable without collision",
            what.c_str(),
            100.0 * fraction);
        return false;
    }
    // A Cartesian path comes back with timing for full speed; slow it to the skills' scaling,
    // the same reason every other move here is scaled.
    robot_trajectory::RobotTrajectory timed(group.getRobotModel(), group.getName());
    timed.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(timed, velocity_scaling_, velocity_scaling_))
    {
        RCLCPP_ERROR(get_logger(), "%s: could not time the straight-line path", what.c_str());
        return false;
    }
    timed.getRobotTrajectoryMsg(trajectory);
    const auto executed = group.execute(trajectory);
    if (executed != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "%s: execution failed (%d)", what.c_str(), executed.val);
        return false;
    }
    return true;
}

G1ManipulationServer::MoveGroup* G1ManipulationServer::groupFor(const std::string& name)
{
    const auto it = groups_.find(name);
    return it == groups_.end() ? nullptr : it->second.get();
}

void G1ManipulationServer::usePinchFrame(ArmContext& arm)
{
    // Published grasps are pinches, aimed at where the thumb meets the index finger. Falls back
    // to grasp_frame on a description without the pinch frame.
    if (groupFor(arm.arm_group)->getRobotModel()->hasLinkModel(arm.pinch_frame))
    {
        arm.grasp_frame = arm.pinch_frame;
    }
}

std::optional<bool> G1ManipulationServer::hasIk(const std::string& group, const std::string& link,
                                                const geometry_msgs::msg::Pose& pose,
                                                std::map<std::string, double>* joints)
{
    ik_broken_code_ = 0;
    if (!compute_ik_->wait_for_service(std::chrono::seconds(2)))
    {
        return std::nullopt;
    }
    auto request                          = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    request->ik_request.group_name        = group;
    request->ik_request.ik_link_name      = link;
    request->ik_request.robot_state.is_diff = true;  // seeded from the current state
    request->ik_request.avoid_collisions  = false;  // reach only; the planner checks collisions
    request->ik_request.pose_stamped.header.frame_id = planning_frame_;
    request->ik_request.pose_stamped.pose = pose;
    request->ik_request.timeout           = rclcpp::Duration::from_seconds(0.05);
    auto future = compute_ik_->async_send_request(request);
    // Waited on without spinning, like the planning-scene calls: the executor owns this node.
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
    {
        return std::nullopt;
    }
    const auto response = future.get();
    const int  code     = response->error_code.val;
    if (code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        if (joints)
        {
            // Only the group's own joints: the rest of the state is what it already is.
            const auto& names = groupFor(group)->getActiveJoints();
            const auto& js    = response->solution.joint_state;
            joints->clear();
            for (std::size_t i = 0; i < js.name.size() && i < js.position.size(); ++i)
            {
                if (std::find(names.begin(), names.end(), js.name[i]) != names.end())
                {
                    (*joints)[js.name[i]] = js.position[i];
                }
            }
        }
        return true;
    }
    if (code == moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION ||
        code == moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT)
    {
        return false;
    }
    // Anything else is move_group unable to run IK at all -- typically no solver loaded for the
    // group -- which must not be reported as "out of reach".
    RCLCPP_ERROR(get_logger(), "/compute_ik for '%s' failed with MoveIt error code %d", group.c_str(), code);
    ik_broken_code_ = code;
    return std::nullopt;
}

bool G1ManipulationServer::planWithRecovery(MoveGroup& group, MoveGroup::Plan& plan, const std::string& what)
{
    auto planned = group.plan(plan);
    if (planned == moveit::core::MoveItErrorCode::SUCCESS)
    {
        return true;
    }
    // The usual cause after a pick that got part-way: the hand is now where the octomap still
    // has voxels from before it got there (the self-filter only stops NEW points of the robot
    // being mapped), so the start state is in collision and every plan fails, including the one
    // to move away. Clearing the map and letting the sensors refill it -- without the hand, which
    // they filter out -- is the fix. Waited for, so the retry does not plan in an empty world.
    RCLCPP_WARN(get_logger(), "%s: planning failed (%d); clearing the octomap and retrying once", what.c_str(),
                planned.val);
    if (clear_octomap_->wait_for_service(std::chrono::seconds(1)))
    {
        auto future = clear_octomap_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
        future.wait_for(std::chrono::seconds(2));
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(octomap_refill_s_ * 1000)));
    }
    group.setStartStateToCurrentState();
    planned = group.plan(plan);
    if (planned != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "%s: planning failed again (%d)", what.c_str(), planned.val);
        return false;
    }
    return true;
}

bool G1ManipulationServer::moveToJoints(MoveGroup& group, const std::map<std::string, double>& joints,
                                        const std::string& what)
{
    group.setStartStateToCurrentState();
    if (!group.setJointValueTarget(joints))
    {
        RCLCPP_ERROR(get_logger(), "%s: joint target rejected", what.c_str());
        return false;
    }
    MoveGroup::Plan plan;
    if (!planWithRecovery(group, plan, what))
    {
        return false;
    }
    if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "%s: execution failed", what.c_str());
        return false;
    }
    return true;
}

void G1ManipulationServer::executePick(const std::shared_ptr<GoalHandle<Pick>>& goal_handle)
{
    const auto goal     = goal_handle->get_goal();
    auto       result   = std::make_shared<Pick::Result>();
    auto       feedback = std::make_shared<Pick::Feedback>();

    if (goal->arm != "left" && goal->arm != "right" && goal->arm != "auto")
    {
        result->success = false;
        result->message = "arm must be 'left', 'right' or 'auto', got '" + goal->arm + "'";
        goal_handle->abort(result);
        return;
    }
    // Located before the arm is chosen: with arm "auto" the grasp point decides it. Nothing has
    // moved yet, so a failure here needs no clean-up.
    const auto abort_locating = [&](const std::string& why) {
        result->success = false;
        result->message = std::string(Pick::Feedback::PHASE_LOCATING) + ": " + why;
        goal_handle->abort(result);
    };

    feedback->phase = Pick::Feedback::PHASE_LOCATING;
    goal_handle->publish_feedback(feedback);
    // Waited for, briefly: a tracked part can be between frames, or still being searched for
    // the moment the robot has turned to it.
    auto detection = lookUpObject(goal->object_id);
    const auto locate_deadline = now() + rclcpp::Duration::from_seconds(locate_timeout_s_);
    while (!detection && now() < locate_deadline && rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        detection = lookUpObject(goal->object_id);
    }
    if (!detection)
    {
        abort_locating("no fresh pose for '" + goal->object_id + "' within " +
                       std::to_string(locate_timeout_s_) + " s");
        return;
    }
    // The instance actually being picked ("panel_2" when asked for "panel"): what the grasps,
    // the collision object and the attachment are named after from here on.
    const std::string object_id = detection->results.front().hypothesis.class_id;
    // /objects is in odom and the planner works in pelvis; the two differ by wherever the
    // robot is standing, so every measured pose goes through TF before it is planned against.
    const std::string object_frame =
        detection->header.frame_id.empty() ? objects_.header.frame_id : detection->header.frame_id;
    const auto object_pose = toPlanningFrame(detection->results.front().pose.pose, object_frame);
    if (!object_pose)
    {
        abort_locating("could not transform the object pose");
        return;
    }
    // Where the grasp is aimed at, updated if the part is seen to move before the descent.
    geometry_msgs::msg::Pose aimed_object_pose = *object_pose;
    // An object that publishes its own grasps ("<object>/grasp/<name>", from g1_perception's
    // tracker) is taken at one of them, straight in along the grasp's approach axis. Anything
    // else gets the top grasp graspFrameGoal() builds from its bounding box.
    struct Candidate
    {
        std::string                             name;
        std::optional<geometry_msgs::msg::Pose> pose;  ///< planning frame; empty = bounding box
    };
    std::vector<Candidate> candidates;
    for (const auto& r : detection->results)
    {
        if (r.hypothesis.class_id.rfind(object_id + "/grasp/", 0) != 0)
        {
            continue;
        }
        const auto pose = toPlanningFrame(r.pose.pose, object_frame);
        if (!pose)
        {
            abort_locating("could not transform " + r.hypothesis.class_id);
            return;
        }
        candidates.push_back({ r.hypothesis.class_id, pose });
    }
    // Fingers pointing most AWAY from the robot first: a part's grasps usually come as mirror
    // pairs tipped toward either end of it, and the one leaning away is the one an arm reaching
    // out from the pelvis can take. The others are still tried, in that order.
    const auto forwardness = [](const geometry_msgs::msg::Pose& pose) {
        tf2::Quaternion q;
        tf2::fromMsg(pose.orientation, q);
        return tf2::Matrix3x3(q).getColumn(0).x();  // fingers' component along pelvis +x
    };
    std::stable_sort(candidates.begin(), candidates.end(), [&](const Candidate& a, const Candidate& b) {
        return forwardness(*a.pose) > forwardness(*b.pose);
    });
    if (candidates.empty())
    {
        candidates.push_back({ "the bounding-box top grasp", std::nullopt });
    }

    // Filled for whichever attempt's pregrasp is planned to.
    ArmContext                              arm;
    MoveGroup*                              arm_group  = nullptr;
    MoveGroup*                              hand_group = nullptr;
    std::string                             arm_name;
    std::optional<geometry_msgs::msg::Pose> grasp_pose;
    geometry_msgs::msg::Pose                grasp_goal, pregrasp_goal;

    // Every failure below leaves the hand open and nothing attached, so a retry starts from a
    // defined state rather than part-way into a grasp. That is what makes the behavior tree's
    // retry meaningful rather than a replay.
    const auto fail = [&](const std::string& phase, const std::string& why) {
        if (hand_group)
        {
            moveToNamed(*hand_group, "open");
        }
        if (arm_group)
        {
            arm_group->detachObject(object_id);
            allowHandContact(arm, { "<octomap>", object_id }, false);
        }
        result->success = false;
        result->message = phase + ": " + why;
        goal_handle->abort(result);
    };

    // The arms to try, in order. "auto": the hand on the side the grasp point is on first (the
    // planning frame is the pelvis, +y to the robot's left), then the other one.
    const auto arms_for = [&](double y) -> std::vector<std::string> {
        if (goal->arm != "auto")
        {
            return { goal->arm };
        }
        return y >= 0.0 ? std::vector<std::string>{ "left", "right" }
                        : std::vector<std::string>{ "right", "left" };
    };

    // A published grasp fixes WHERE the hand closes and across what (its closing axis), not
    // exactly how the hand is turned: a pinch across a wall is the same grasp with the fingers
    // tipped anywhere in the wall's plane, i.e. turned about the closing axis. Which tilt the arm
    // can reach depends on where the robot stands, so every candidate is expanded into tilts
    // (and, optionally, the hand turned over), checked with IK first -- cheap -- and only the
    // reachable ones are planned. The configured tilt comes first, then the nearest ones.
    struct Attempt
    {
        std::string              label;
        std::string              arm;
        std::optional<geometry_msgs::msg::Pose> grasp_pose;
        geometry_msgs::msg::Pose grasp, pregrasp;
        /// The pregrasp as joint values, from IK seeded at the current arm: planned to directly,
        /// so the arm goes to the solution nearest where it is, not whichever one OMPL samples
        /// for a pose goal (with a 7-joint arm, often a twisted elbow and wrist).
        std::map<std::string, double> pregrasp_joints;
    };
    std::vector<Attempt> attempts;
    std::string          unreachable;
    int                  ik_checked = 0;
    for (const Candidate& c : candidates)
    {
        const double y = c.pose ? c.pose->position.y : object_pose->position.y;
        if (!c.pose)
        {
            for (const std::string& a : arms_for(y))
            {
                ArmContext ctx;
                resolveArm(a, ctx);
                Attempt at{ c.name, a, std::nullopt, graspFrameGoal(*object_pose, detection->bbox.size.z, ctx),
                            geometry_msgs::msg::Pose() };
                at.pregrasp = at.grasp;
                at.pregrasp.position.z += approach_height_m_;
                attempts.push_back(at);
            }
            continue;
        }
        int reachable_here = 0;
        // Most vertical first: the hand comes down onto the edge from above when it can, and
        // tips only as far as reach needs.
        // Also leaned about the grasp's z, the axis along the edge being pinched: the hand comes
        // in over the edge at an angle instead of straight down, as a person reaching for the far
        // side of a box does. Straight down needs the wrist right above the point, which costs
        // the arm 15-20 cm of reach; leaned, the fingers reach out to it.
        std::vector<double> leans{ 0.0 };
        for (double l = grasp_tilt_step_rad_; l <= grasp_lean_max_rad_ + 1e-9 && grasp_tilt_step_rad_ > 1e-6;
             l += grasp_tilt_step_rad_)
        {
            leans.push_back(l);
            leans.push_back(-l);
        }
        std::vector<GraspVariant> variants;
        for (const double lean : leans)
        {
            geometry_msgs::msg::Pose leaned = *c.pose;
            tf2::Quaternion          q, rz;
            tf2::fromMsg(leaned.orientation, q);
            rz.setRPY(0.0, 0.0, lean);
            leaned.orientation = tf2::toMsg((q * rz).normalized());
            for (GraspVariant v :
                 graspVariants(leaned, grasp_tilt_max_rad_, grasp_tilt_step_rad_, grasp_allow_flip_, grasp_min_descent_))
            {
                v.lean = lean;
                variants.push_back(v);
            }
        }
        std::stable_sort(variants.begin(), variants.end(), [](const GraspVariant& a, const GraspVariant& b) {
            tf2::Quaternion qa, qb;
            tf2::fromMsg(a.pose.orientation, qa);
            tf2::fromMsg(b.pose.orientation, qb);
            return tf2::Matrix3x3(qa).getColumn(0).z() < tf2::Matrix3x3(qb).getColumn(0).z();
        });
        for (const GraspVariant& v : variants)
        {
            if (static_cast<int>(attempts.size()) >= grasp_max_plans_)
            {
                break;  // enough reachable ones to plan to
            }
            for (const std::string& a : arms_for(y))
            {
                ArmContext ctx;
                resolveArm(a, ctx);
                usePinchFrame(ctx);
                Attempt    at{ c.name + v.suffix(), a, v.pose, handGoalFromGrasp(v.pose, ctx.is_left),
                               geometry_msgs::msg::Pose() };
                at.pregrasp = backOff(at.grasp, grasp_approach_m_);
                // Both ends of the straight-line approach must have an IK solution (collisions
                // ignored here; the planner checks them).
                ++ik_checked;
                const auto grasp_ik = hasIk(ctx.arm_group, ctx.grasp_frame, at.grasp);
                if (!grasp_ik)
                {
                    fail(Pick::Feedback::PHASE_PREGRASP,
                         ik_broken_code_ != 0
                             ? "move_group cannot run IK for " + ctx.arm_group + " (MoveIt error " +
                                   std::to_string(ik_broken_code_) +
                                   "): its kinematics plugin did not load. kinematics.yaml uses "
                                   "pick_ik: sudo apt install ros-humble-pick-ik, then restart"
                             : "move_group's /compute_ik did not answer; is move_group running?");
                    return;
                }
                if (*grasp_ik &&
                    hasIk(ctx.arm_group, ctx.grasp_frame, at.pregrasp, &at.pregrasp_joints).value_or(false))
                {
                    attempts.push_back(at);
                    ++reachable_here;
                }
                if (static_cast<int>(attempts.size()) >= grasp_max_plans_)
                {
                    break;
                }
            }
        }
        if (reachable_here == 0 && static_cast<int>(attempts.size()) < grasp_max_plans_)
        {
            char where[160];
            std::snprintf(where, sizeof where, "%s: no tilt reachable by %s, grasp at pelvis (%.2f, %.2f, %.2f) m",
                          c.name.c_str(), goal->arm == "auto" ? "either arm" : goal->arm.c_str(),
                          c.pose->position.x, c.pose->position.y, c.pose->position.z);
            unreachable += (unreachable.empty() ? "" : "; ") + std::string(where);
        }
    }
    RCLCPP_INFO(get_logger(), "pick %s: %zu reachable grasp poses out of %d IK checks",
                object_id.c_str(), attempts.size(), ik_checked);

    if (attempts.empty())
    {
        fail(Pick::Feedback::PHASE_PREGRASP,
             "out of reach at every tilt of every grasp (" + unreachable +
                 "). Stand closer: for a grasp near pelvis height the arm reaches about 0.28-0.36 m "
                 "ahead of the pelvis (x) at 0.1 m to the side, and nothing past ~0.43 m.");
        return;
    }

    // The box goes where the detection's bounding box is: for a part whose own frame is not at
    // its middle (the panel's body frame is not), the object pose would put the box off to one
    // side and leave part of the panel uncovered for the planner to swing the hand through.
    const auto box_pose = toPlanningFrame(detection->bbox.center, object_frame);
    const moveit_msgs::msg::CollisionObject object =
        publishCollisionObject(*detection, box_pose && (detection->bbox.size.x > 0.0) ? *box_pose : *object_pose);

    feedback->phase = Pick::Feedback::PHASE_PREGRASP;
    goal_handle->publish_feedback(feedback);
    bool        reached = false;
    std::string tried;
    int         planned = 0;
    for (const Attempt& at : attempts)
    {
        if (planned >= grasp_max_plans_)
        {
            break;
        }
        ++planned;
        arm_name = at.arm;
        resolveArm(arm_name, arm);
        if (at.grasp_pose)
        {
            usePinchFrame(arm);
        }
        arm_group     = groupFor(arm.arm_group);
        hand_group    = groupFor(arm.hand_group);
        grasp_pose    = at.grasp_pose;
        grasp_goal    = at.grasp;
        pregrasp_goal = at.pregrasp;
        RCLCPP_INFO(get_logger(), "pick %s: trying %s with the %s hand", object_id.c_str(),
                    at.label.c_str(), arm_name.c_str());

        // Opened before the approach, not after: closing on the way in would knock the object
        // off the table before the hand is around it. Reported separately from the arm move: a
        // hand that will not open is an unacquired or unpowered hand, not geometry.
        // A published grasp is a pinch: thumb across from the fingers before going down, if the
        // hand has that shape ("pinch_open" in g1.srdf). The bounding-box grasp opens fully.
        const auto& shapes = hand_group->getNamedTargets();
        const std::string open_shape =
            grasp_pose && std::find(shapes.begin(), shapes.end(), "pinch_open") != shapes.end() ? "pinch_open"
                                                                                                : "open";
        if (!moveToNamed(*hand_group, open_shape))
        {
            fail(Pick::Feedback::PHASE_PREGRASP, "the " + arm_name + " hand would not open");
            return;
        }
        const std::string what = "pregrasp (" + at.label + ")";
        if (at.pregrasp_joints.empty() ? moveTo(*arm_group, pregrasp_goal, arm.grasp_frame, what)
                                       : moveToJoints(*arm_group, at.pregrasp_joints, what))
        {
            reached = true;
            break;
        }
        // IK said reachable, so this one is blocked: the open hand or the arm on the way hits the
        // octomap or the part's box, or no path was found in time.
        RCLCPP_WARN(get_logger(), "pick %s: %s with the %s hand is reachable but the plan was blocked",
                    object_id.c_str(), at.label.c_str(), arm_name.c_str());
        tried += (tried.empty() ? "" : "; ") + at.label + " (" + arm_name + ")";
    }
    if (!reached)
    {
        fail(Pick::Feedback::PHASE_PREGRASP,
             "reachable but blocked: " + tried +
                 (unreachable.empty() ? "" : "; out of reach: " + unreachable) +
                 ". The open hand at the pregrasp hits the octomap or the part's box, or the arm "
                 "has no collision-free path there.");
        return;
    }

    feedback->phase = Pick::Feedback::PHASE_APPROACH;
    goal_handle->publish_feedback(feedback);

    // Only NOW is contact allowed, and only for the last few centimetres.
    //
    // Turning it on earlier is what let a plan route the arm straight through the table on the
    // way to the pregrasp pose: with the hand exempt from the octomap for the whole skill, a
    // path through the surface costs the planner nothing. The transit above therefore runs
    // fully collision-checked, and the exemption covers only the short descent that has to end
    // in contact.
    //
    // The object's own collision geometry is REMOVED rather than exempted. It was added so the
    // pregrasp plan would route around it; from here it is in the way, and MoveIt's documented
    // pick sequence is remove, close, attach. Attaching re-adds it as an attached body, which
    // is also what gets PointCloudOctomapUpdater's ShapeMask to stop feeding it back into the
    // octomap -- world collision objects get no such filtering, attached bodies do.
    allowHandContact(arm, { "<octomap>" }, true);
    planning_scene_.removeCollisionObjects({ object_id });

    // Re-aimed at where the part is NOW, not where it was when the pick started: the tracker
    // keeps measuring it while the arm moves, and a part nudged on the way in (or simply seen
    // better from closer) moves the grasp with it. A part that moved a lot was knocked, and
    // grasping at a guess would knock it further.
    if (grasp_pose)
    {
        if (const auto fresh = lookUpObject(object_id))
        {
            const std::string fresh_frame =
                fresh->header.frame_id.empty() ? object_frame : fresh->header.frame_id;
            if (const auto now_pose = toPlanningFrame(fresh->results.front().pose.pose, fresh_frame))
            {
                tf2::Transform then_T, now_T, grasp_T;
                tf2::fromMsg(aimed_object_pose, then_T);
                tf2::fromMsg(*now_pose, now_T);
                const tf2::Transform moved = now_T * then_T.inverse();
                const double         shift = moved.getOrigin().length();
                const double         turn  = moved.getRotation().getAngle();
                if (shift > reaim_max_shift_m_)
                {
                    fail(Pick::Feedback::PHASE_APPROACH,
                         "the part moved " + std::to_string(static_cast<int>(shift * 1000)) +
                             " mm since the pick started (knocked?); not grasping at a guess");
                    return;
                }
                if (shift > 0.003 || turn > 0.02)
                {
                    RCLCPP_INFO(get_logger(), "pick %s: part moved %.1f mm / %.1f deg; re-aiming the grasp",
                                object_id.c_str(), shift * 1000.0, turn * 180.0 / M_PI);
                    tf2::fromMsg(grasp_goal, grasp_T);
                    tf2::toMsg(moved * grasp_T, grasp_goal);
                    aimed_object_pose = *now_pose;
                }
            }
        }
    }

    // Straight in for a published grasp: the parts beside it are close, and a free-space plan
    // may swing the fingers through them.
    if (!(grasp_pose ? moveLinear(*arm_group, grasp_goal, arm.grasp_frame, "approach")
                     : moveTo(*arm_group, grasp_goal, arm.grasp_frame, "approach")))
    {
        fail(Pick::Feedback::PHASE_APPROACH, "could not reach the grasp pose");
        return;
    }

    feedback->phase = Pick::Feedback::PHASE_GRASP;
    goal_handle->publish_feedback(feedback);
    if (!moveToNamed(*hand_group, "closed"))
    {
        fail(Pick::Feedback::PHASE_GRASP, "the hand did not close");
        return;
    }
    // Attached AFTER the hand closes, so the planner starts treating the object as part of the
    // arm at the same moment the hand does.
    //
    // Built explicitly rather than via attachObject(id, link), which promotes an object that is
    // still in the world -- and this one was removed before the approach, so there is nothing
    // left to promote. An attached body carries its own geometry, so the shape and pose come
    // from what publishCollisionObject built.
    //
    // touch_links is what tells the planner the fingers are SUPPOSED to be in contact with it.
    // Without them the attach itself reads as a collision the moment it takes effect.
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name        = arm.palm_link;
    attached.object           = object;
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    // The hand's own links, plus the palm and the WRIST: a cube gripped just under its top face
    // reaches past the fingers to touch it, and without the wrist in touch_links every later
    // plan starts with the attached object already in collision.
    //
    // Same set allowHandContact() exempts, and for the same reason: these are the links that
    // unavoidably touch what is being carried.
    attached.touch_links =
        hand_group->getRobotModel()->getJointModelGroup(arm.hand_group)->getLinkModelNames();
    attached.touch_links.push_back(arm.palm_link);
    const std::string touch_side = arm.is_left ? "left" : "right";
    attached.touch_links.push_back(touch_side + "_wrist_pitch_link");
    attached.touch_links.push_back(touch_side + "_wrist_yaw_link");
    attached.touch_links.push_back(touch_side + "_wrist_roll_link");
    planning_scene_.applyAttachedCollisionObject(attached);

    // Extended to the object only now that it is attached and about to be lifted OUT of the
    // surface it was resting on. Its own voxels, and the table's underneath it, are still in
    // the octomap from before the grasp -- the ShapeMask stops new clouds re-adding it, but it
    // does not erase what is already there. Without this exemption the lift starts with the
    // attached cube already in collision against the table it is still sitting on.
    allowHandContact(arm, { "<octomap>", object_id }, true);

    feedback->phase = Pick::Feedback::PHASE_LIFT;
    goal_handle->publish_feedback(feedback);
    geometry_msgs::msg::Pose lifted = grasp_goal;
    lifted.position.z += lift_height_m_;
    if (!(grasp_pose ? moveLinear(*arm_group, lifted, arm.grasp_frame, "lift")
                     : moveTo(*arm_group, lifted, arm.grasp_frame, "lift")))
    {
        fail(Pick::Feedback::PHASE_LIFT, "could not lift clear of the surface");
        return;
    }

    allowHandContact(arm, { "<octomap>", object_id }, false);
    // How it is held, so a place can put the OBJECT at a pose: grasp_frame_T_object, from the
    // grasp the hand was sent to and the object pose it was planned against.
    {
        tf2::Transform world_T_grasp, world_T_object;
        tf2::fromMsg(grasp_goal, world_T_grasp);
        tf2::fromMsg(aimed_object_pose, world_T_object);
        std::lock_guard<std::mutex> lock(held_mutex_);
        held_grasp_[object_id] = world_T_grasp.inverse() * world_T_object;
        held_arm_[object_id]   = arm_name;
    }
    result->success = true;
    result->message = "picked " + object_id + " with the " + arm_name + " hand";
    goal_handle->succeed(result);
}

void G1ManipulationServer::executePlace(const std::shared_ptr<GoalHandle<Place>>& goal_handle)
{
    const auto goal     = goal_handle->get_goal();
    auto       result   = std::make_shared<Place::Result>();
    auto       feedback = std::make_shared<Place::Feedback>();

    // What is held, read from the scene rather than remembered, because the attachment is the
    // authority on what the hand actually has. Needed BEFORE the exemption is applied: the
    // object has to be named in it.
    std::string held_id;
    double      held_height = 0.0;
    for (const auto& [id, attached] : planning_scene_.getAttachedObjects())
    {
        if (!attached.object.primitives.empty() &&
            attached.object.primitives.front().dimensions.size() == 3)
        {
            held_id     = id;
            held_height = attached.object.primitives.front().dimensions[2];
            break;
        }
    }
    // How it is held, if this server picked it at a published grasp: then the place puts the
    // OBJECT at the target pose, and the hand goes wherever that puts it.
    std::optional<tf2::Transform> grasp_T_object;
    std::string                   picked_with;
    {
        std::lock_guard<std::mutex> lock(held_mutex_);
        if (const auto it = held_grasp_.find(held_id); it != held_grasp_.end())
        {
            grasp_T_object = it->second;
            picked_with    = held_arm_[held_id];
        }
    }

    std::string arm_name = goal->arm;
    if (arm_name == "auto")
    {
        arm_name = picked_with.empty() ? "right" : picked_with;
    }
    ArmContext arm;
    if (!resolveArm(arm_name, arm))
    {
        result->success = false;
        result->message = "arm must be 'left', 'right' or 'auto', got '" + goal->arm + "'";
        goal_handle->abort(result);
        return;
    }
    if (grasp_T_object)
    {
        usePinchFrame(arm);  // it was picked at a published grasp, i.e. pinched
    }
    MoveGroup* arm_group  = groupFor(arm.arm_group);
    MoveGroup* hand_group = groupFor(arm.hand_group);

    // The octomap AND the object being carried: an attached body is part of the robot for
    // kinematics but is still its own collision entity, so exempting the palm it hangs off
    // does nothing for it.
    //
    // Pick already exempts both for the lift, which is the same motion in reverse.
    const std::vector<std::string> touchables =
        held_id.empty() ? std::vector<std::string>{ "<octomap>" } :
                          std::vector<std::string>{ "<octomap>", held_id };

    const auto fail = [&](const std::string& phase, const std::string& why) {
        allowHandContact(arm, touchables, false);
        result->success = false;
        result->message = phase + ": " + why;
        goal_handle->abort(result);
    };

    // The carried object may pass through the surface's voxels on the way in; the ARM may not.
    // Exempting the hand this early let plans route the arm into the bench, and the collision
    // shoved the base off its stance, putting the re-aimed descent out of reach. Same hazard
    // the pick defers its exemption for; the descent below gets the full one.
    allowHandContact(arm, touchables, true, /*include_links=*/false);

    // Where the OBJECT goes, from /objects when the goal names a detected thing.
    //
    // Prefer that over the caller's coordinate. A tree writes its drop point in the MAP frame,
    // while the base approach that parked the robot drove against /objects, published in ODOM --
    // and those two frames are only as close as AMCL is right. A realistic map->odom drift is
    // bigger than the arm's 0.04 m lateral window. Resolving the target from the stream the
    // approach used makes the two agree by construction, however far localization has drifted.
    //
    // Two kinds: "<object>/slot/<name>", a pose a detection publishes for something to sit in
    // (the object goes exactly there), or a plain object id, a surface the held object is
    // stood on top of. Returns the target in the planning frame and in the /objects frame.
    const std::string& name     = goal->surface_object_id;
    const auto         slot_at  = name.find("/slot/");
    const bool         is_slot  = slot_at != std::string::npos;
    const std::string  owner_id = is_slot ? name.substr(0, slot_at) : name;
    const auto resolve = [&]() -> std::optional<std::pair<geometry_msgs::msg::Pose, geometry_msgs::msg::Pose>> {
        const auto detection = lookUpObject(owner_id);
        if (!detection)
        {
            return std::nullopt;
        }
        const std::string frame =
            detection->header.frame_id.empty() ? objects_.header.frame_id : detection->header.frame_id;
        std::optional<geometry_msgs::msg::Pose> in_objects =
            is_slot ? findHypothesis(*detection, name) : detection->results.front().pose.pose;
        if (!in_objects)
        {
            RCLCPP_ERROR(get_logger(), "'%s' publishes no '%s'", owner_id.c_str(), name.c_str());
            return std::nullopt;
        }
        if (!is_slot)
        {
            // A detected surface reports its own centre, so the held object goes on TOP of it:
            // half the surface's height to reach its face, half the object's to stand it there.
            in_objects->position.z += 0.5 * (detection->bbox.size.z + held_height);
        }
        const auto in_planning = toPlanningFrame(*in_objects, frame);
        if (!in_planning)
        {
            return std::nullopt;
        }
        return std::make_pair(*in_planning, *in_objects);
    };

    // The grasp frame's goal for an object target, and the point the straight descent starts.
    const auto hand_goals = [&](const geometry_msgs::msg::Pose& target) {
        geometry_msgs::msg::Pose place_goal, preplace;
        if (grasp_T_object)
        {
            // world_T_grasp = world_T_object * object_T_grasp: the object exactly at the target,
            // held as it was picked. The approach is the grasp's own, in reverse for the retreat.
            tf2::Transform world_T_object;
            tf2::fromMsg(target, world_T_object);
            tf2::toMsg(world_T_object * grasp_T_object->inverse(), place_goal);
            preplace = backOff(place_goal, grasp_approach_m_);
        }
        else
        {
            place_goal = graspFrameGoal(target, held_height, arm);
            preplace   = place_goal;
            preplace.position.z += approach_height_m_;
        }
        return std::make_pair(place_goal, preplace);
    };

    std::optional<geometry_msgs::msg::Pose> target;
    std::optional<geometry_msgs::msg::Point> expected;  // in the /objects frame
    if (!name.empty())
    {
        const auto resolved = resolve();
        if (!resolved)
        {
            fail(Place::Feedback::PHASE_PREPLACE, "nothing usable called '" + name + "' on /objects");
            return;
        }
        target   = resolved->first;
        expected = resolved->second.position;
    }
    else
    {
        // An empty frame means the goal is already in the planning frame. Anything else is
        // transformed rather than assumed: a target in odom treated as pelvis lands metres away,
        // and by the time that is visible the arm is already moving.
        target = toPlanningFrame(goal->pose.pose, goal->pose.header.frame_id);
    }
    if (!target)
    {
        fail(Place::Feedback::PHASE_PREPLACE, "could not transform the target pose");
        return;
    }
    auto [place_goal, preplace] = hand_goals(*target);

    feedback->phase = Place::Feedback::PHASE_PREPLACE;
    goal_handle->publish_feedback(feedback);
    if (!moveTo(*arm_group, preplace, arm.grasp_frame, "preplace"))
    {
        fail(Place::Feedback::PHASE_PREPLACE, "could not reach the pose above the target");
        return;
    }

    // Re-resolve before descending. Everything above was computed in the pelvis frame before the
    // arm reached out, and the reach moves the base: extending a loaded arm shifts the COM and
    // the balancing gait steps to keep up, measured at 0.165 m.
    if (!name.empty())
    {
        if (const auto fresh = resolve())
        {
            const auto [regoal, repre] = hand_goals(fresh->first);
            const double shift = std::hypot(regoal.position.x - place_goal.position.x,
                                            regoal.position.y - place_goal.position.y);
            if (shift > 0.01)
            {
                RCLCPP_INFO(get_logger(), "the base moved %.3f m during the reach; re-aiming", shift);
            }
            target     = fresh->first;
            expected   = fresh->second.position;
            place_goal = regoal;
            // The retreat returns here, so it moves with the re-aim. Left stale, the lift is
            // diagonal by however far the base walked.
            preplace = repre;
        }
    }

    feedback->phase = Place::Feedback::PHASE_LOWER;
    goal_handle->publish_feedback(feedback);
    // Now the hand may touch the surface: the descent ends in contact by definition.
    allowHandContact(arm, touchables, true);
    if (!(grasp_T_object ? moveLinear(*arm_group, place_goal, arm.grasp_frame, "lower")
                         : moveTo(*arm_group, place_goal, arm.grasp_frame, "lower")))
    {
        fail(Place::Feedback::PHASE_LOWER, "could not lower onto the target");
        return;
    }

    feedback->phase = Place::Feedback::PHASE_RELEASE;
    goal_handle->publish_feedback(feedback);
    if (!moveToNamed(*hand_group, "open"))
    {
        fail(Place::Feedback::PHASE_RELEASE, "the hand did not open");
        return;
    }
    // Detached only after the hand is open. Detaching first would let the planner route the
    // arm through a volume the object is still occupying.
    //
    // What is held is read from the scene rather than remembered from the pick, so a place
    // still works against a server that did not do the picking.
    const moveit::core::RobotStatePtr              state = arm_group->getCurrentState();
    std::vector<const moveit::core::AttachedBody*> attached;
    state->getAttachedBodies(attached, state->getJointModelGroup(arm.arm_group));
    for (const moveit::core::AttachedBody* body : attached)
    {
        arm_group->detachObject(body->getName());
    }
    {
        std::lock_guard<std::mutex> lock(held_mutex_);
        held_grasp_.erase(held_id);
        held_arm_.erase(held_id);
    }

    feedback->phase = Place::Feedback::PHASE_RETREAT;
    goal_handle->publish_feedback(feedback);
    // Restored BEFORE the retreat is planned, or the lift may route the open hand through what
    // was just set down. The object only: the hand is inside the surface's voxels at this height
    // by construction, so restoring `<octomap>` here would put the start state in collision.
    if (!held_id.empty())
    {
        allowHandContact(arm, { held_id }, false);
    }
    if (!(grasp_T_object ? moveLinear(*arm_group, preplace, arm.grasp_frame, "retreat")
                         : moveTo(*arm_group, preplace, arm.grasp_frame, "retreat")))
    {
        fail(Place::Feedback::PHASE_RETREAT, "could not retreat clear of the object");
        return;
    }

    allowHandContact(arm, touchables, false);

    // Did it actually land there? A successful plan says nothing about where the object ended
    // up: one release short dropped the cube on the floor with every leaf reporting success.
    //
    // Against `expected` when /objects gave the target, so both sides come from /objects and
    // the walking base cancels. A caller-supplied pose falls back to the planning frame.
    if (!held_id.empty())
    {
        if (const auto landed = lookUpObject(held_id))
        {
            const geometry_msgs::msg::Pose&  pose = landed->results.front().pose.pose;
            const geometry_msgs::msg::Point& aim  = expected ? *expected : target->position;

            std::optional<geometry_msgs::msg::Point> where = pose.position;
            if (!expected)
            {
                const std::string frame       = landed->header.frame_id.empty() ?
                                                    objects_.header.frame_id :
                                                    landed->header.frame_id;
                const auto        in_planning = toPlanningFrame(pose, frame);
                where = in_planning ? std::optional(in_planning->position) : std::nullopt;
            }
            if (where)
            {
                const double off = std::hypot(where->x - aim.x, where->y - aim.y, where->z - aim.z);
                if (off > place_tolerance_m_)
                {
                    result->success = false;
                    result->message = std::string(Place::Feedback::PHASE_RETREAT) + ": " + held_id +
                                      " ended up " + std::to_string(off) +
                                      " m from where it was placed";
                    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
                    goal_handle->abort(result);
                    return;
                }
                RCLCPP_INFO(
                    get_logger(),
                    "%s came to rest %.3f m from the target",
                    held_id.c_str(),
                    off);
            }
        }
    }

    result->success = true;
    result->message = "placed with the " + arm_name + " hand";
    goal_handle->succeed(result);
}

void G1ManipulationServer::executeSetArmPosture(
    const std::shared_ptr<GoalHandle<SetArmPosture>>& goal_handle)
{
    const auto goal   = goal_handle->get_goal();
    auto       result = std::make_shared<SetArmPosture::Result>();

    MoveGroup* group = groupFor(goal->group);
    if (group == nullptr)
    {
        result->success = false;
        result->message = "'" + goal->group + "' is not a group this node drives";
        goal_handle->abort(result);
        return;
    }
    if (!moveToNamed(*group, goal->named_target))
    {
        result->success = false;
        result->message = "could not reach '" + goal->named_target + "'";
        goal_handle->abort(result);
        return;
    }

    result->success = true;
    result->message = goal->group + " is at " + goal->named_target;
    goal_handle->succeed(result);
}

}  // namespace g1_manipulation
