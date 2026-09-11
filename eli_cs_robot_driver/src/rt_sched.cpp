// Effectful real-time SCHEDULING setup for the control loop thread: CPU pin and
// SCHED_FIFO priority. No rclcpp dependency (see rt_sched.hpp); the only ROS
// package touched is realtime_tools, whose configure_sched_fifo() and
// set_current_thread_affinity() are thin wrappers over sched_setscheduler() and
// pthread_setaffinity_np().
#include "eli_cs_robot_driver/rt_sched.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>

#include <sched.h>  // sched_getcpu, CPU_SETSIZE

// This include directive triggers a compilation warning to use <realtime_tools/realtime_helpers.hpp> instead.
// However, while that change is a drop-in replacement that clears the warning, something about it introduced
// a breaking change of the worse kind -- silent failures: arm motions failed in real life without explicit
// errors. (This note moved here from control_node.cpp with the configure_sched_fifo call.) In the
// realtime_tools shipped today, thread_priority.hpp is a one-line include of realtime_helpers.hpp plus the
// warning, so the two are equivalent; the deprecated spelling stays until that history is re-verified
// rather than assumed.
#include <realtime_tools/thread_priority.hpp>

namespace ELITE_CS_ROBOT_ROS_DRIVER {
namespace rt_sched {

RtSchedSetup configure_realtime_sched(int cpu, int priority) {
    RtSchedSetup setup{};
    setup.requested_cpu = cpu;
    setup.requested_priority = priority;
    setup.priority_valid = priority_is_valid(priority);
    setup.applied_priority = effective_priority(priority);

    // 1. Pin first (see rt_sched.hpp for why the order matters). A negative cpu
    // means "leave affinity alone" and is deliberately NOT passed on:
    // realtime_tools::set_current_thread_affinity(-1) resets the thread to every
    // core, which is a change, not a no-op.
    if (cpu >= 0) {
        setup.affinity_attempted = true;
        if (cpu >= CPU_SETSIZE) {
            // Beyond what a cpu_set_t can hold; reject before any library call.
            char msg[96];
            std::snprintf(msg, sizeof(msg), "cpu %d is outside 0..%d", cpu, CPU_SETSIZE - 1);
            setup.affinity_error = msg;
        } else {
            const auto [ok, message] = realtime_tools::set_current_thread_affinity(cpu);
            setup.affinity_succeeded = ok;
            if (!ok) {
                setup.affinity_error = message.empty() ? "set_current_thread_affinity failed" : message;
            }
        }
    }

    // 2. SCHED_FIFO at the effective priority. configure_sched_fifo() returns
    // the negated sched_setscheduler() result and does nothing else after the
    // syscall, so errno is still that call's when it reports failure.
    errno = 0;
    if (realtime_tools::configure_sched_fifo(setup.applied_priority)) {
        setup.sched_succeeded = true;
    } else {
        setup.sched_errno = errno;
    }

    setup.running_cpu = sched_getcpu();
    return setup;
}

std::string describe(const RtSchedSetup& setup) {
    char priority_part[96];
    if (setup.priority_valid) {
        std::snprintf(priority_part, sizeof(priority_part), "priority %d", setup.applied_priority);
    } else {
        std::snprintf(priority_part, sizeof(priority_part),
            "priority %d (requested %d is outside %d..%d, using default)",
            setup.applied_priority, setup.requested_priority, kMinPriority, kMaxPriority);
    }

    char policy_part[160];
    if (setup.sched_succeeded) {
        std::snprintf(policy_part, sizeof(policy_part), "SCHED_FIFO %s", priority_part);
    } else {
        std::snprintf(policy_part, sizeof(policy_part),
            "SCHED_FIFO %s REFUSED (%s), loop keeps inherited scheduling",
            priority_part, std::strerror(setup.sched_errno));
    }

    char cpu_part[256];
    if (!setup.affinity_attempted) {
        std::snprintf(cpu_part, sizeof(cpu_part), "cpu pin none requested");
    } else if (setup.affinity_succeeded) {
        std::snprintf(cpu_part, sizeof(cpu_part), "cpu pin %d", setup.requested_cpu);
    } else {
        std::snprintf(cpu_part, sizeof(cpu_part), "cpu pin %d FAILED (%s), affinity unchanged",
            setup.requested_cpu, setup.affinity_error.c_str());
    }

    char line[768];
    std::snprintf(line, sizeof(line), "RT sched (control thread): %s; %s; running on cpu %d",
        policy_part, cpu_part, setup.running_cpu);
    return std::string(line);
}

}  // namespace rt_sched
}  // namespace ELITE_CS_ROBOT_ROS_DRIVER
