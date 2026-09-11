// Unit tests for the real-time SCHEDULING helpers in rt_sched.hpp: priority
// validation/fallback, the summary line, and configure_realtime_sched() itself.
// The SCHED_FIFO request needs CAP_SYS_NICE or an rtprio ulimit, neither of
// which CI has, so those tests assert that the outcome is *reported* faithfully
// (and that a refusal never throws), not that it succeeds. CPU pinning needs no
// privilege and is asserted for real against sched_getcpu().
// No rclcpp dependency, like rt_sched.hpp/.cpp.
#include <sched.h>

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "eli_cs_robot_driver/rt_sched.hpp"

namespace rt = ELITE_CS_ROBOT_ROS_DRIVER::rt_sched;

namespace {

// CPUs this process may run on, in ascending order (honours a container cpuset,
// unlike the host CPU count).
std::vector<int> allowed_cpus() {
    std::vector<int> cpus;
    cpu_set_t set;
    CPU_ZERO(&set);
    if (sched_getaffinity(0, sizeof(set), &set) != 0) {
        return cpus;
    }
    for (int i = 0; i < CPU_SETSIZE; ++i) {
        if (CPU_ISSET(i, &set)) {
            cpus.push_back(i);
        }
    }
    return cpus;
}

}  // namespace

// --- priority_is_valid / effective_priority ---------------------------------

TEST(PriorityIsValid, BoundsAreOneToNinetyEight) {
    EXPECT_FALSE(rt::priority_is_valid(0));
    EXPECT_TRUE(rt::priority_is_valid(1));
    EXPECT_TRUE(rt::priority_is_valid(rt::kDefaultPriority));
    EXPECT_TRUE(rt::priority_is_valid(98));
    EXPECT_FALSE(rt::priority_is_valid(99));  // kernel migration threads live here
    EXPECT_FALSE(rt::priority_is_valid(-1));
}

TEST(EffectivePriority, ValidValuesPassThrough) {
    EXPECT_EQ(rt::effective_priority(1), 1);
    EXPECT_EQ(rt::effective_priority(70), 70);
    EXPECT_EQ(rt::effective_priority(98), 98);
}

TEST(EffectivePriority, InvalidValuesFallBackToDefault) {
    EXPECT_EQ(rt::effective_priority(0), rt::kDefaultPriority);
    EXPECT_EQ(rt::effective_priority(99), rt::kDefaultPriority);
    EXPECT_EQ(rt::effective_priority(-5), rt::kDefaultPriority);
}

TEST(Defaults, MatchTheOldHardCodedBehaviour) {
    // A node launched without rt_sched.* parameters must behave exactly as it
    // did when FIFO 50 / no pin were compiled in.
    EXPECT_EQ(rt::kDefaultPriority, 50);
    EXPECT_EQ(rt::kNoCpuPin, -1);
}

// --- describe ------------------------------------------------------------------

TEST(Describe, SuccessMentionsPolicyPriorityCpuAndRunningCpu) {
    rt::RtSchedSetup s{};
    s.requested_cpu = 6;
    s.affinity_attempted = true;
    s.affinity_succeeded = true;
    s.requested_priority = 80;
    s.priority_valid = true;
    s.applied_priority = 80;
    s.sched_succeeded = true;
    s.running_cpu = 6;
    const std::string line = rt::describe(s);
    EXPECT_NE(line.find("SCHED_FIFO"), std::string::npos) << line;
    EXPECT_NE(line.find("priority 80"), std::string::npos) << line;
    EXPECT_NE(line.find("cpu pin 6"), std::string::npos) << line;
    EXPECT_NE(line.find("running on cpu 6"), std::string::npos) << line;
    EXPECT_EQ(line.find("FAILED"), std::string::npos) << line;
    EXPECT_EQ(line.find("REFUSED"), std::string::npos) << line;
}

TEST(Describe, InvalidPriorityNamesRequestedAndApplied) {
    rt::RtSchedSetup s{};
    s.requested_priority = 99;
    s.priority_valid = false;
    s.applied_priority = rt::kDefaultPriority;
    s.sched_succeeded = true;
    const std::string line = rt::describe(s);
    EXPECT_NE(line.find("priority 50"), std::string::npos) << line;
    EXPECT_NE(line.find("requested 99"), std::string::npos) << line;
    EXPECT_NE(line.find("using default"), std::string::npos) << line;
}

TEST(Describe, NoPinRequestedIsSaidExplicitly) {
    rt::RtSchedSetup s{};
    s.sched_succeeded = true;
    EXPECT_NE(rt::describe(s).find("cpu pin none requested"), std::string::npos);
}

TEST(Describe, FailuresAreLoud) {
    rt::RtSchedSetup s{};
    s.requested_cpu = 3;
    s.affinity_attempted = true;
    s.affinity_succeeded = false;
    s.affinity_error = "Invalid argument";
    s.sched_succeeded = false;
    s.sched_errno = EPERM;
    const std::string line = rt::describe(s);
    EXPECT_NE(line.find("cpu pin 3 FAILED (Invalid argument)"), std::string::npos) << line;
    EXPECT_NE(line.find("REFUSED"), std::string::npos) << line;
    EXPECT_NE(line.find("inherited scheduling"), std::string::npos) << line;
}

// --- configure_realtime_sched --------------------------------------------------

TEST(ConfigureRealtimeSched, NoPinLeavesAffinityAloneAndReportsPriority) {
    const auto before = allowed_cpus();
    const rt::RtSchedSetup s = rt::configure_realtime_sched(rt::kNoCpuPin, 70);
    EXPECT_EQ(s.requested_cpu, rt::kNoCpuPin);
    EXPECT_FALSE(s.affinity_attempted);
    EXPECT_FALSE(s.affinity_succeeded);
    EXPECT_EQ(s.requested_priority, 70);
    EXPECT_TRUE(s.priority_valid);
    EXPECT_EQ(s.applied_priority, 70);
    // Unprivileged CI: refused with EPERM. Privileged (a robot container): ok.
    if (!s.sched_succeeded) {
        EXPECT_EQ(s.sched_errno, EPERM);
    }
    EXPECT_GE(s.running_cpu, 0);
    EXPECT_EQ(allowed_cpus(), before);
}

TEST(ConfigureRealtimeSched, InvalidPriorityIsClampedAndFlagged) {
    const rt::RtSchedSetup s = rt::configure_realtime_sched(rt::kNoCpuPin, 99);
    EXPECT_FALSE(s.priority_valid);
    EXPECT_EQ(s.requested_priority, 99);
    EXPECT_EQ(s.applied_priority, rt::kDefaultPriority);
}

TEST(ConfigureRealtimeSched, PinToAllowedCpuLandsThere) {
    const auto cpus = allowed_cpus();
    ASSERT_FALSE(cpus.empty());
    const int target = cpus.back();
    const rt::RtSchedSetup s = rt::configure_realtime_sched(target, rt::kDefaultPriority);
    EXPECT_TRUE(s.affinity_attempted);
    ASSERT_TRUE(s.affinity_succeeded) << s.affinity_error;
    EXPECT_TRUE(s.affinity_error.empty());
    EXPECT_EQ(s.requested_cpu, target);
    // Once pinned to a single CPU this thread can only be running there.
    EXPECT_EQ(s.running_cpu, target);
    EXPECT_EQ(sched_getcpu(), target);
    // The affinity change is real, not just reported.
    EXPECT_EQ(allowed_cpus(), std::vector<int>{target});
}

TEST(ConfigureRealtimeSched, PinToDisallowedCpuIsReportedAndPriorityStillRequested) {
    // CPU_SETSIZE - 1 is far beyond any allowed set we run on; realtime_tools
    // (or the kernel) refuses and the pin must be reported as failed with a
    // message, not throw, and must not block the priority step.
    const auto before = allowed_cpus();
    const rt::RtSchedSetup s = rt::configure_realtime_sched(CPU_SETSIZE - 1, 60);
    EXPECT_TRUE(s.affinity_attempted);
    EXPECT_FALSE(s.affinity_succeeded);
    EXPECT_FALSE(s.affinity_error.empty());
    EXPECT_EQ(s.applied_priority, 60);  // step 2 still ran
    EXPECT_EQ(allowed_cpus(), before);   // and the affinity really was left alone
}

TEST(ConfigureRealtimeSched, OutOfRangeCpuIndexIsRejectedBeforeAnyCall) {
    const rt::RtSchedSetup s = rt::configure_realtime_sched(CPU_SETSIZE + 10, rt::kDefaultPriority);
    EXPECT_TRUE(s.affinity_attempted);
    EXPECT_FALSE(s.affinity_succeeded);
    EXPECT_NE(s.affinity_error.find("outside"), std::string::npos) << s.affinity_error;
}

TEST(ConfigureRealtimeSched, NegativeCpuIsNeverForwardedAsAReset) {
    // realtime_tools reads core -1 as "reset affinity to all cores". Our -1
    // means "leave it alone", so first narrow this thread to one CPU, then ask
    // for kNoCpuPin and check the narrowing survived.
    const auto cpus = allowed_cpus();
    ASSERT_FALSE(cpus.empty());
    const int target = cpus.front();
    ASSERT_TRUE(rt::configure_realtime_sched(target, rt::kDefaultPriority).affinity_succeeded);
    const rt::RtSchedSetup s = rt::configure_realtime_sched(rt::kNoCpuPin, rt::kDefaultPriority);
    EXPECT_FALSE(s.affinity_attempted);
    EXPECT_EQ(allowed_cpus(), std::vector<int>{target});
}
