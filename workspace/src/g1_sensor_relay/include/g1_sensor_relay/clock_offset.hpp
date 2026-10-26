#ifndef G1_SENSOR_RELAY__CLOCK_OFFSET_HPP_
#define G1_SENSOR_RELAY__CLOCK_OFFSET_HPP_

#include <deque>
#include <utility>

namespace g1_sensor_relay
{

/**
 * @brief Offset from the simulator's clock to this node's, from frames' (arrival, capture) pairs.
 *
 * Every frame carries sim_time_s, MuJoCo's clock at the snapshot, and arrives some latency
 * later. arrival - sim_time is the clock offset plus that latency, and latency is never
 * negative, so the smallest value recently seen is the best estimate of the offset.
 *
 * RECENTLY is the point. The two clocks drift apart whenever the simulator cannot hold real
 * time: at a real-time factor of 0.5, arrival - sim_time grows by half a second every second.
 * A minimum over all time (or one that only leaks upward slowly) then lags further and further
 * behind, every frame is stamped that far in the past, and once that is more than tf2's buffer
 * nothing can be transformed: seen as "extrapolation into the past ... requested time T but the
 * earliest data is at T + 90 s" from the object tracker after minutes of walking, with the
 * pallet and its slots gone from /objects. Over a short window the minimum still drops the
 * latency of the frames in it, and follows any drift within the window's length.
 */
class ClockOffset
{
public:
    explicit ClockOffset(double window_s = 0.25) : window_s_(window_s) {}

    /// Adds a frame captured at @p sim_time_s that arrived at @p arrival_s; returns the offset.
    double update(double arrival_s, double sim_time_s)
    {
        const double delta = arrival_s - sim_time_s;
        // A monotonic queue: each entry smaller than everything after it, so the front is the
        // window's minimum and every frame is pushed and popped once.
        while (!window_.empty() && window_.back().second >= delta)
        {
            window_.pop_back();
        }
        window_.emplace_back(arrival_s, delta);
        while (window_.front().first < arrival_s - window_s_)
        {
            window_.pop_front();
        }
        return window_.front().second;
    }

private:
    double                                  window_s_;
    std::deque<std::pair<double, double>>   window_;  ///< (arrival, arrival - sim_time)
};

}  // namespace g1_sensor_relay

#endif  // G1_SENSOR_RELAY__CLOCK_OFFSET_HPP_
