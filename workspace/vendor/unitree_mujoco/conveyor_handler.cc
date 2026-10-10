#include "conveyor_handler.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace grove_g1
{
namespace
{

// Belt geoms this manager owns are named with this prefix in the scene (an exact match, or
// `conveyor_belt_<anything>` for more than one belt). Anything else in the model is left
// alone.
constexpr const char* kBeltPrefix = "conveyor_belt";

constexpr int kTickRateHz = 500;  // matches dex3_handler's own rate; this is a force, not a DDS write

struct Config
{
    bool enabled = false;

    // Target speed along the belt's local +x, m/s. Positive only: a belt that should run the
    // other way is turned around in the scene, not given a negative speed here, so the geom's
    // own pose stays the single source of truth for direction.
    double speed_mps = 0.08;

    // N per (m/s) of velocity error on the driven body's linear dofs. Large enough to reach
    // speed_mps well inside one capture pass at a typical small-part mass (tens of grams to a
    // few hundred), not tuned against any real belt's drive torque.
    double gain = 40.0;

    // How far above the belt's own top surface a body's origin may sit and still count as
    // "on" it, in metres. Covers a settling bounce without reaching so high that a part lifted
    // off by the arm is still dragged.
    double capture_height_m = 0.09;
};

// One belt zone: a static geom's body, half-extents, and the direction to drive toward.
struct Belt
{
    int    body_id = -1;
    double half_x  = 0.0;
    double half_y  = 0.0;
    double half_z  = 0.0;
    std::string name;
};

// One body this manager may drive: resolved once from the model's free joints, independent of
// whether it is currently on a belt -- that is re-checked every tick, since parts move on and
// off a belt as the scene runs.
struct Draggable
{
    int body_id = -1;
    int dof_adr = -1;  // first of 6 free-joint dofs; [0..2] are world-frame linear velocity
};

struct State
{
    std::thread       thread;
    std::atomic<bool> running{ false };
};

State& state()
{
    // Deliberately leaked, for the same reason the other handlers leak theirs: the physics
    // thread ends in exit(0), which runs static destructors while this thread may still be
    // alive, and destroying a joinable std::thread calls std::terminate.
    static State* s = new State();
    return *s;
}

Config loadConfig()
{
    Config      cfg;
    const char* path = std::getenv("GROVE_G1_SENSOR_CONFIG");
    if (path == nullptr || *path == '\0')
    {
        return cfg;
    }
    try
    {
        const YAML::Node root = YAML::LoadFile(path);
        const YAML::Node node = root["conveyor"];
        if (!node)
        {
            return cfg;
        }
        cfg.enabled = node["enabled"] ? node["enabled"].as<bool>() : true;
        if (node["speed_mps"])
        {
            cfg.speed_mps = node["speed_mps"].as<double>();
        }
        if (node["gain"])
        {
            cfg.gain = node["gain"].as<double>();
        }
        if (node["capture_height_m"])
        {
            cfg.capture_height_m = node["capture_height_m"].as<double>();
        }
    }
    catch (const std::exception& e)
    {
        std::fprintf(
            stderr, "[grove_g1] conveyor config '%s' failed to load (%s); DISABLED\n", path,
            e.what());
        cfg.enabled = false;
    }
    if (cfg.speed_mps <= 0.0 || cfg.gain <= 0.0 || cfg.capture_height_m <= 0.0)
    {
        std::fprintf(
            stderr,
            "[grove_g1] conveyor needs speed_mps, gain and capture_height_m all > 0; DISABLED\n");
        cfg.enabled = false;
    }
    return cfg;
}

bool resolve(const mjModel* model, std::vector<Belt>& belts, std::vector<Draggable>& bodies)
{
    belts.clear();
    bodies.clear();
    const std::size_t prefix_len = std::char_traits<char>::length(kBeltPrefix);

    for (int g = 0; g < model->ngeom; ++g)
    {
        const char* name = mj_id2name(model, mjOBJ_GEOM, g);
        if (name == nullptr || std::string(name).compare(0, prefix_len, kBeltPrefix) != 0)
        {
            continue;
        }
        if (model->geom_type[g] != mjGEOM_BOX)
        {
            std::fprintf(stderr, "[grove_g1] conveyor geom '%s' is not a box; not managed\n", name);
            continue;
        }
        Belt belt;
        belt.body_id = model->geom_bodyid[g];
        belt.half_x  = model->geom_size[3 * g + 0];
        belt.half_y  = model->geom_size[3 * g + 1];
        belt.half_z  = model->geom_size[3 * g + 2];
        belt.name    = name;
        belts.push_back(belt);
    }
    if (belts.empty())
    {
        return false;
    }

    for (int j = 0; j < model->njnt; ++j)
    {
        if (model->jnt_type[j] != mjJNT_FREE)
        {
            continue;
        }
        Draggable body;
        body.body_id = model->jnt_bodyid[j];
        body.dof_adr = model->jnt_dofadr[j];
        bodies.push_back(body);
    }

    std::fprintf(
        stderr, "[grove_g1] conveyor: %zu belt(s), %zu free body(ies) eligible to drag\n",
        belts.size(), bodies.size());
    return !bodies.empty();
}

// `world_pos` expressed in `belt`'s own body frame, using that body's current world pose --
// so a belt the scene rotates or (one day) moves still drags along its own local +x, not a
// world axis baked in at compile time.
void toBeltFrame(const mjData* data, const Belt& belt, const double world_pos[3], double out[3])
{
    double delta[3];
    mju_sub3(delta, world_pos, data->xpos + 3 * belt.body_id);
    const mjtNum* R = data->xmat + 9 * belt.body_id;  // row-major 3x3, world <- local
    // out = R^T * delta
    out[0] = R[0] * delta[0] + R[3] * delta[1] + R[6] * delta[2];
    out[1] = R[1] * delta[0] + R[4] * delta[1] + R[7] * delta[2];
    out[2] = R[2] * delta[0] + R[5] * delta[1] + R[8] * delta[2];
}

void run(const Config cfg, mjModel** model, mjData** data, std::recursive_mutex* sim_mtx)
{
    auto& s = state();
    while (s.running.load(std::memory_order_relaxed) && (*model == nullptr || *data == nullptr))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!s.running.load(std::memory_order_relaxed))
    {
        return;
    }

    mjModel*               m = *model;
    std::vector<Belt>      belts;
    std::vector<Draggable> bodies;
    if (!resolve(m, belts, bodies))
    {
        std::fprintf(stderr, "[grove_g1] no conveyor belts (or no free bodies) in this scene; conveyor is OFF\n");
        return;
    }
    const auto period = std::chrono::nanoseconds(std::chrono::seconds(1)) / kTickRateHz;
    auto       next    = std::chrono::steady_clock::now();

    while (s.running.load(std::memory_order_relaxed))
    {
        next += period;
        std::this_thread::sleep_until(next);

        std::lock_guard<std::recursive_mutex> lock(*sim_mtx);
        if (*model != m || *data == nullptr)
        {
            std::fprintf(stderr, "[grove_g1] model replaced; conveyor is OFF\n");
            return;
        }
        mjData* d = *data;

        for (const Draggable& body : bodies)
        {
            const double* pos = d->xpos + 3 * body.body_id;
            bool          on_belt = false;
            double        dir_world[3] = { 0.0, 0.0, 0.0 };

            for (const Belt& belt : belts)
            {
                double local[3];
                toBeltFrame(d, belt, pos, local);
                const double height_above_top = local[2] - belt.half_z;
                if (std::abs(local[0]) <= belt.half_x && std::abs(local[1]) <= belt.half_y &&
                    height_above_top >= 0.0 && height_above_top <= cfg.capture_height_m)
                {
                    const mjtNum* R = d->xmat + 9 * belt.body_id;
                    dir_world[0]    = R[0];  // belt body's local +x, in world frame
                    dir_world[1]    = R[3];
                    dir_world[2]    = R[6];
                    on_belt         = true;
                    break;  // first matching belt wins; scenes should not overlap belts
                }
            }

            double* force = d->qfrc_applied + body.dof_adr;
            if (!on_belt)
            {
                mju_zero3(force);
                continue;
            }
            const double* vel = d->qvel + body.dof_adr;
            for (int k = 0; k < 3; ++k)
            {
                const double target = dir_world[k] * cfg.speed_mps;
                force[k]            = cfg.gain * (target - vel[k]);
            }
        }
    }

    std::lock_guard<std::recursive_mutex> lock(*sim_mtx);
    if (*model == m && *data != nullptr)
    {
        for (const Draggable& body : bodies)
        {
            mju_zero3((*data)->qfrc_applied + body.dof_adr);
        }
    }
}

}  // namespace

void StartConveyorHandler(mjModel** model, mjData** data, std::recursive_mutex* sim_mtx)
{
    auto& s = state();
    if (s.running.load(std::memory_order_relaxed))
    {
        return;
    }
    const Config cfg = loadConfig();
    if (!cfg.enabled)
    {
        return;
    }
    s.running.store(true, std::memory_order_relaxed);
    s.thread = std::thread(run, cfg, model, data, sim_mtx);
}

void StopConveyorHandler()
{
    auto& s = state();
    if (!s.thread.joinable())
    {
        s.running.store(false, std::memory_order_relaxed);
        return;
    }
    s.running.store(false, std::memory_order_relaxed);
    s.thread.join();
}

}  // namespace grove_g1
