// Real-time SCHEDULING for the control node's cyclic loop thread: which CPU it
// is pinned to and which SCHED_FIFO priority it runs at.
//
// Companion to rt_memory.hpp (memory locking) and modeled on it: everything
// here is free of rclcpp, performs no logging, and reports raw facts in a
// result struct so the caller (control_node.cpp) decides how to log them. The
// pure helpers are unit-tested in test/test_rt_sched.cpp; configure_realtime_sched()
// is exercised there too, asserting that it *reports* its outcome (CI is not
// privileged, so the SCHED_FIFO request is expected to fail there).
//
// The syscalls themselves come from realtime_tools (configure_sched_fifo,
// set_current_thread_affinity). This file is the node's POLICY around them:
// which values are legal, what to fall back to, in which order to apply them,
// and how to report the result -- none of which realtime_tools decides.
//
// Why this exists: the loop's priority used to be a compiled-in FIFO 50 and the
// thread floated over every core. On the robot both cyclic loops (arm and hose
// reel EtherCAT) run in one container whose other ~50 threads outranked them,
// and both loops tied with every threaded IRQ handler. Priority and CPU are
// therefore ROS2 parameters (rt_sched.priority / rt_sched.cpu, see
// config/rt_memory.yaml) so each deployed node can be placed in the host's
// priority ladder from its launch file, without a recompile. The defaults
// reproduce the old behaviour exactly: FIFO 50, no pin.
#pragma once

#include <string>

namespace ELITE_CS_ROBOT_ROS_DRIVER {
namespace rt_sched {

// Compiled-in default when no rt_sched.priority parameter is given. Matches the
// value the node hard-coded before the parameter existed.
inline constexpr int kDefaultPriority = 50;

// rt_sched.cpu value meaning "do not touch the loop thread's CPU affinity".
inline constexpr int kNoCpuPin = -1;

// Accepted SCHED_FIFO priority range. 99 is deliberately excluded: the kernel's
// per-CPU migration threads run there and nothing in user space should tie
// with them.
inline constexpr int kMinPriority = 1;
inline constexpr int kMaxPriority = 98;

inline bool priority_is_valid(int priority) {
    return priority >= kMinPriority && priority <= kMaxPriority;
}

// The priority configure_realtime_sched() will actually request for a given
// parameter value: the value itself when valid, otherwise kDefaultPriority. A
// scheduling tuning value must never take the node down, so out-of-range input
// degrades to the old behaviour instead of aborting.
inline int effective_priority(int requested) {
    return priority_is_valid(requested) ? requested : kDefaultPriority;
}

// Raw facts from one configure_realtime_sched() call. No interpretation and no
// logging here; describe() renders them and control_node.cpp logs them.
struct RtSchedSetup {
    // CPU pinning (skipped entirely when requested_cpu < 0).
    int requested_cpu = kNoCpuPin;
    bool affinity_attempted = false;
    bool affinity_succeeded = false;
    std::string affinity_error;      // non-empty only when attempted && !succeeded

    // SCHED_FIFO priority.
    int requested_priority = kDefaultPriority;
    bool priority_valid = true;      // priority_is_valid(requested_priority)
    int applied_priority = kDefaultPriority;  // effective_priority(requested_priority)
    bool sched_succeeded = false;
    int sched_errno = 0;             // valid only when !sched_succeeded

    // Where the calling thread was running once both steps were done, from
    // sched_getcpu(); -1 if that call failed.
    int running_cpu = -1;
};

// One human-readable line summarising a setup result: policy, applied (and, if
// different, requested) priority, requested CPU and pin outcome, and the CPU the
// thread is actually on. Pure; used for the node's single INFO summary line.
std::string describe(const RtSchedSetup& setup);

// Configure the CALLING thread for real-time cyclic work:
//   1. if cpu >= 0, pin the thread to exactly that CPU via
//      realtime_tools::set_current_thread_affinity. Guards kept on this side:
//      a negative cpu is never forwarded (realtime_tools reads -1 as "reset
//      affinity to every core", the opposite of "leave it alone"), and a cpu at
//      or beyond CPU_SETSIZE is rejected before any call. The pin runs first so
//      a failure is on record before the thread becomes hard to preempt; a
//      failed pin never blocks step 2 (an unpinned RT loop is still better than
//      a non-RT one). A CPU outside the process's allowed set (e.g. a container
//      cpuset) fails and is reported with realtime_tools' message.
//   2. request SCHED_FIFO at effective_priority(priority) via
//      realtime_tools::configure_sched_fifo. Failure (typically EPERM without
//      CAP_SYS_NICE / an rtprio ulimit) is reported, not fatal.
// Never throws, never aborts, never logs. Call it from the thread to configure.
RtSchedSetup configure_realtime_sched(int cpu, int priority);

}  // namespace rt_sched
}  // namespace ELITE_CS_ROBOT_ROS_DRIVER
