#include "g1_perception/pose_spec.hpp"

#include <cmath>
#include <sstream>

namespace g1_perception
{
namespace
{

// Whitespace-separated tokens; a parse that leaves anything over is an error, so a typo such as
// a missing space ("0.1 0.2,0.3") cannot silently drop a value.
std::vector<std::string> tokens(const std::string& text)
{
    std::istringstream       in(text);
    std::vector<std::string> out;
    std::string              t;
    while (in >> t)
    {
        out.push_back(t);
    }
    return out;
}

bool toFloat(const std::string& s, float& out)
{
    try
    {
        std::size_t used = 0;
        out              = std::stof(s, &used);
        return used == s.size() && std::isfinite(out);
    }
    catch (const std::exception&)
    {
        return false;
    }
}

std::optional<Eigen::Isometry3f> poseFromTokens(const std::vector<std::string>& t, std::size_t first,
                                                const std::string& text, std::string& error)
{
    const std::size_t n = t.size() - first;
    if (n != 3 && n != 6)
    {
        error = "'" + text + "': expected x y z or x y z roll pitch yaw";
        return std::nullopt;
    }
    float v[6] = { 0, 0, 0, 0, 0, 0 };
    for (std::size_t i = 0; i < n; ++i)
    {
        if (!toFloat(t[first + i], v[i]))
        {
            error = "'" + text + "': '" + t[first + i] + "' is not a number";
            return std::nullopt;
        }
    }
    return poseFromXyzRpy(v[0], v[1], v[2], v[3], v[4], v[5]);
}

}  // namespace

Eigen::Isometry3f poseFromXyzRpy(float x, float y, float z, float roll, float pitch, float yaw)
{
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.linear()          = (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                  Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                  Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()))
                     .toRotationMatrix();
    T.translation() = Eigen::Vector3f(x, y, z);
    return T;
}

std::optional<Eigen::Isometry3f> parsePose(const std::string& text, std::string& error)
{
    return poseFromTokens(tokens(text), 0, text, error);
}

std::optional<FramedPose> parseFramedPose(const std::string& text, std::string& error)
{
    const auto t = tokens(text);
    float      dummy;
    if (t.empty() || toFloat(t[0], dummy))
    {
        error = "'" + text + "': expected a frame name first, then x y z [roll pitch yaw]";
        return std::nullopt;
    }
    const auto pose = poseFromTokens(t, 1, text, error);
    if (!pose)
    {
        return std::nullopt;
    }
    return FramedPose{ t[0], *pose };
}

std::optional<std::vector<Eigen::Isometry3f>> parseSweep(const std::string& text, std::string& error)
{
    const auto t = tokens(text);
    if (t.size() != 4 || (t[0] != "x" && t[0] != "y" && t[0] != "z"))
    {
        error = "'" + text + "': expected <x|y|z> start stop step";
        return std::nullopt;
    }
    float start, stop, step;
    if (!toFloat(t[1], start) || !toFloat(t[2], stop) || !toFloat(t[3], step) || step <= 0.0f || stop < start)
    {
        error = "'" + text + "': start <= stop and step > 0 are required";
        return std::nullopt;
    }
    const long n = static_cast<long>(std::floor((stop - start) / step + 1e-4)) + 1;
    if (n > 10000)
    {
        error = "'" + text + "': " + std::to_string(n) + " steps; that many hypotheses would never finish";
        return std::nullopt;
    }
    const int                      axis = t[0][0] - 'x';
    std::vector<Eigen::Isometry3f> out;
    for (long i = 0; i < n; ++i)
    {
        Eigen::Isometry3f T  = Eigen::Isometry3f::Identity();
        T.translation()[axis] = start + step * static_cast<float>(i);
        out.push_back(T);
    }
    return out;
}

}  // namespace g1_perception
