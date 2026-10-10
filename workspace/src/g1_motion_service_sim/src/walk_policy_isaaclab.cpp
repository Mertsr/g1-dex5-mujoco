#include "g1_motion_service_sim/walk_policy_isaaclab.hpp"

namespace g1_motion_service_sim
{

std::array<float, kIlObsStepDim> ilAssembleObservationStep(
    const IlPolicyInputs& in, const IlPolicyConfig& cfg,
    const std::array<float, kIlNumJoints>& last_full_action)
{
    std::array<float, kIlObsStepDim> obs{};
    std::size_t                      o = 0;

    for (double v : in.base_ang_vel_body) obs[o++] = static_cast<float>(v);
    for (double v : in.projected_gravity) obs[o++] = static_cast<float>(v);

    obs[o++] = static_cast<float>(in.cmd_vx);
    obs[o++] = static_cast<float>(in.cmd_vy);
    obs[o++] = static_cast<float>(in.cmd_vyaw);
    obs[o++] = static_cast<float>(cfg.command_height);

    // joint_pos: legs then arms, relative to default_joint_pos.
    for (int idx : cfg.leg_action_indices)
        obs[o++] = static_cast<float>(in.joint_pos[static_cast<std::size_t>(idx)] - cfg.default_joint_pos[static_cast<std::size_t>(idx)]);
    for (int idx : cfg.arm_obs_indices)
        obs[o++] = static_cast<float>(in.joint_pos[static_cast<std::size_t>(idx)] - cfg.default_joint_pos[static_cast<std::size_t>(idx)]);

    // joint_vel: same order, relative to default_joint_vel.
    for (int idx : cfg.leg_action_indices)
        obs[o++] = static_cast<float>(in.joint_vel[static_cast<std::size_t>(idx)] - cfg.default_joint_vel[static_cast<std::size_t>(idx)]);
    for (int idx : cfg.arm_obs_indices)
        obs[o++] = static_cast<float>(in.joint_vel[static_cast<std::size_t>(idx)] - cfg.default_joint_vel[static_cast<std::size_t>(idx)]);

    for (float v : last_full_action) obs[o++] = v;

    return obs;  // o == kIlObsStepDim
}

std::array<double, kIlNumLegs> ilActionToLegTargets(
    const std::array<float, kIlNumLegs>& raw_action, const IlPolicyConfig& cfg)
{
    std::array<double, kIlNumLegs> targets{};
    for (std::size_t i = 0; i < kIlNumLegs; ++i)
    {
        const std::size_t idx = static_cast<std::size_t>(cfg.leg_action_indices[i]);
        targets[i]            = cfg.default_joint_pos[idx] + static_cast<double>(raw_action[i]) * cfg.action_scale;
    }
    return targets;
}

std::array<float, kIlNumJoints> ilBuildFullAction(
    const std::array<float, kIlNumLegs>& raw_action, const IlPolicyConfig& cfg)
{
    std::array<float, kIlNumJoints> full{};
    for (std::size_t i = 0; i < kIlNumJoints; ++i) full[i] = static_cast<float>(cfg.default_joint_pos[i]);
    for (std::size_t i = 0; i < kIlNumLegs; ++i)
        full[static_cast<std::size_t>(cfg.leg_action_indices[i])] = raw_action[i];
    return full;
}

}  // namespace g1_motion_service_sim
