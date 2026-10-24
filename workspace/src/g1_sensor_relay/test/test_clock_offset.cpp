#include <gmock/gmock.h>

#include <cmath>

#include "g1_sensor_relay/clock_offset.hpp"

using g1_sensor_relay::ClockOffset;

// IMU frames at 200 Hz, each arriving 0.2-3 ms after capture, the two clocks 1000 s apart.
TEST(ClockOffset, SteadyClockGivesTheOffsetLessTheLatencyNoise)
{
    ClockOffset c;
    double      off = 0.0;
    for (int i = 0; i < 2000; ++i)
    {
        const double sim     = i * 0.005;
        const double latency = 0.0002 + 0.0028 * std::fabs(std::sin(i * 1.7));
        off                  = c.update(sim + 1000.0 + latency, sim);
    }
    EXPECT_NEAR(off, 1000.0, 0.001);
}

// The simulator at half real time for five minutes: arrival - sim_time grows 0.5 s per second.
// The estimate must keep up (it fell 90 s behind before), within the window's share of the drift.
TEST(ClockOffset, FollowsASimulatorSlowerThanRealTime)
{
    ClockOffset c(0.25);
    double      off = 0.0, truth = 0.0;
    for (int i = 0; i < 60000; ++i)
    {
        const double wall = i * 0.005;   // this node's clock
        const double sim  = 0.5 * wall;  // MuJoCo's, at RTF 0.5
        truth             = wall - sim;  // the offset now, latency aside
        off               = c.update(wall + 0.001, sim);
    }
    EXPECT_NEAR(off, truth, 0.13);
    EXPECT_GT(truth, 140.0);
}

// A camera frame held up 300 ms (a render stall) while the IMU frames keep arriving on time must
// not drag the stamps: the estimate stays the IMU's.
TEST(ClockOffset, ADelayedFrameDoesNotRaiseIt)
{
    ClockOffset c;
    for (int i = 0; i < 200; ++i)
    {
        c.update(i * 0.005 + 5.001, i * 0.005);  // IMU, 1 ms latency
    }
    EXPECT_NEAR(c.update(1.0 + 5.001, 1.0), 5.001, 1e-9);
    // The late camera frame, captured at 0.7, arriving at 1.0 + 5.0: its own lag does not count.
    EXPECT_NEAR(c.update(1.0 + 5.0 + 0.0005, 0.7), 5.001, 1e-9);
}

// A simulation reset puts sim_time back to 0: the offset jumps up and the estimate follows
// within one window.
TEST(ClockOffset, FollowsASimulationReset)
{
    ClockOffset c(0.25);
    for (int i = 0; i < 1000; ++i)
    {
        c.update(i * 0.005 + 5.001, i * 0.005);
    }
    double off = 0.0;
    for (int i = 0; i < 100; ++i)
    {
        off = c.update(5.0 + i * 0.005 + 5.001, i * 0.005);  // sim restarted at wall 10 s
    }
    EXPECT_NEAR(off, 10.001, 1e-6);
}
