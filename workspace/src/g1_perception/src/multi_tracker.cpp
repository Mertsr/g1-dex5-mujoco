#include "g1_perception/multi_tracker.hpp"

#include <map>
#include <set>
#include <stdexcept>

namespace g1_perception
{

MultiTracker::MultiTracker(std::vector<ObjectConfig> objects)
{
    std::set<std::string> names;
    for (const auto& c : objects)
    {
        if (c.name.empty() || !names.insert(c.name).second)
        {
            throw std::runtime_error("object names must be non-empty and unique ('" + c.name + "')");
        }
    }
    for (auto& c : objects)
    {
        for (const auto& p : c.priors)
        {
            if (p.frame == c.name)
            {
                throw std::runtime_error("object '" + c.name + "' has a prior relative to itself");
            }
        }
        Object o;
        std::shared_ptr<const ObjectModel> model;
        try
        {
            model = ObjectModel::load(c.model_path, c.model);
        }
        catch (const std::exception& e)
        {
            throw std::runtime_error("object '" + c.name + "': " + e.what());
        }
        // The extent in the output frame: the model's points carried into it, boxed there.
        Eigen::AlignedBox3f     box;
        const Eigen::Isometry3f output_T_model = c.model_T_output.inverse();
        for (const auto& p : model->points())
        {
            box.extend(output_T_model * p);
        }
        o.box_centre = box.center();
        o.box_size   = box.sizes();
        o.tracker    = std::make_unique<ObjectTracker>(c.name, model, c.params);
        o.config     = std::move(c);
        objects_.push_back(std::move(o));
    }
}

void MultiTracker::reset()
{
    for (auto& o : objects_)
    {
        o.tracker->reset();
        o.motion.reset();
        o.world_T_model.reset();
        o.was_tracking = false;
    }
}

std::vector<MultiTracker::Result> MultiTracker::update(const DepthImage& depth, const Intrinsics& k, double stamp,
                                                       const Eigen::Isometry3f& world_T_cam,
                                                       const std::string& camera_frame, const FrameLookup& lookup)
{
    const Eigen::Isometry3f cam_T_world = world_T_cam.inverse();
    // One lookup per frame name per frame: several priors usually share a frame.
    std::map<std::string, std::optional<Eigen::Isometry3f>> frame_cache;
    const auto world_T = [&](const std::string& frame) -> std::optional<Eigen::Isometry3f> {
        for (const auto& o : objects_)
        {
            if (o.config.name == frame)
            {
                return o.world_T_model;
            }
        }
        auto it = frame_cache.find(frame);
        if (it == frame_cache.end())
        {
            it = frame_cache.emplace(frame, lookup ? lookup(frame, stamp) : std::nullopt).first;
        }
        return it->second;
    };

    std::vector<Result> results;
    results.reserve(objects_.size());
    for (auto& o : objects_)
    {
        FrameContext ctx;
        ctx.stamp = stamp;
        if (const auto p = o.motion.predict(stamp))
        {
            ctx.predicted = cam_T_world * *p;
        }
        if (o.world_T_model)
        {
            ctx.fallbacks.push_back(cam_T_world * *o.world_T_model);  // as last seen, no velocity
        }
        ctx.motion = [&](double then) -> std::optional<Eigen::Isometry3f> {
            if (!lookup)
            {
                return std::nullopt;
            }
            const auto world_T_cam_then = lookup(camera_frame, then);
            if (!world_T_cam_then)
            {
                return std::nullopt;
            }
            return cam_T_world * *world_T_cam_then;  // cam_now_T_cam_then
        };
        for (const auto& other : objects_)
        {
            if (&other != &o && other.world_T_model)
            {
                ctx.others.push_back({ &other.tracker->model(), cam_T_world * *other.world_T_model });
            }
        }
        if (o.tracker->state() == ObjectTracker::State::kSearching)
        {
            for (const auto& p : o.config.priors)
            {
                if (const auto parent = world_T(p.frame))
                {
                    ctx.priors.push_back(cam_T_world * *parent * p.pose);
                }
            }
        }

        Result r;
        r.name         = o.config.name;
        r.out          = o.tracker->update(depth, k, ctx);
        const bool tracking = r.out.state == ObjectTracker::State::kTracking;
        r.valid        = tracking && r.out.valid;
        r.box_centre   = o.box_centre;
        r.box_size     = o.box_size;
        if (r.valid)
        {
            o.world_T_model = world_T_cam * r.out.cam_T_model;
            o.motion.update(stamp, *o.world_T_model);
            r.world_T_output = *o.world_T_model * o.config.model_T_output;
            r.cam_T_output   = r.out.cam_T_model * o.config.model_T_output;
        }
        else if (!tracking)
        {
            // Lost: stop explaining depth with a pose nobody has confirmed, and stop predicting.
            o.world_T_model.reset();
            o.motion.reset();
        }
        r.just_locked  = tracking && !o.was_tracking;
        r.just_lost    = !tracking && o.was_tracking;
        o.was_tracking = tracking;
        results.push_back(std::move(r));
    }
    return results;
}

}  // namespace g1_perception
