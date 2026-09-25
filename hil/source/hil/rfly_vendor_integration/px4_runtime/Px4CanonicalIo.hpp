#pragma once

// PX4 v1.16 uORB I/O component. Publication requires a configured Authority.
#include "../SlimSnapshotExecutor.hpp"
#include "ExecutionAuthority.hpp"
#include "CommittedFeedback.hpp"
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>

ORB_DECLARE(actuator_outputs_rfly);

namespace gpenmpc_rfly_px4 {
using Ticket=gpenmpc_rfly_state_execution::SnapshotTicket;

// The module owner supplies boot/UID identity and validates output ownership.
// A lease binds output bits and expiry for MAVLink while preserving physical
// output isolation. Revoke retires the lease; plant stop requires observation.
enum class Fault:uint8_t {None,Configuration,AuthorityUnavailable,IdentityMismatch,
    Pending,Source,Command,Telemetry,ControlOwnership,Expired,Lease,Publish,Commit,Stopped,Feedback};
enum class Capture:uint8_t {NoUpdate,Accepted,Rejected};
struct Diagnostics {
    uint64_t capture_calls{0},source_updates{0},captured{0},kernel_calls{0};
    uint64_t publish_attempts{0},publish_succeeded{0},numerical_commits{0};
    uint64_t first_fault_us{0},last_publication_us{0},original_valid_until_us{0};
    uint64_t last_commit_completed_us{0}; // separate actual post-commit HRT
    uint64_t disarmed_observations_released{0};
    uint32_t last_source_generation{0};
    Fault fault{Fault::None};
    // None of these counters proves MAVLink/USB/CopterSim consumption.
};

class Px4CanonicalIo final {
public:
    Px4CanonicalIo(const gpenmpc_odometry::Configuration &source,
                   const gpenmpc_rfly_execution::Configuration &execution,
                   uint64_t telemetry_max_age_us,Authority *authority) noexcept;
    ~Px4CanonicalIo();
    Px4CanonicalIo(const Px4CanonicalIo &)=delete;
    Px4CanonicalIo &operator=(const Px4CanonicalIo &)=delete;

    // Exactly one original subscription update and its original generation;
    // no LP/attitude stitching, timestamp renewal or synthetic source tick.
    Capture capture_next(Ticket &ticket) noexcept;
    Capture capture_disarmed_observation(Ticket &ticket) noexcept;
    bool release_disarmed_observation(const Ticket &ticket) noexcept;
    const gpenmpc_odometry::Snapshot *snapshot(const Ticket &ticket) const noexcept;
    // Same task, read-only original private snapshot; no HOST state, capture,
    // timestamp refresh or control-history advance. May be ObservationReleased.
    const gpenmpc_odometry::Snapshot *latest_captured_snapshot()const noexcept{return snapshot(pending_ticket_);}
    bool execute(const gpenmpc_rfly_slim::Command &command) noexcept;
    // Same pump owner only, never called directly by the MAVLink thread.
    // One-shot raw observation; HistoricalExpired/Revoked are not live commits.
    FeedbackDisposition take_committed_feedback(CommittedFeedback &out) noexcept;
    void stop() noexcept;
    const Diagnostics &diagnostics()const noexcept{return diagnostics_;}
    const gpenmpc_rfly_state_execution::NumericalReceipt &last_receipt()const noexcept{return receipt_;}

private:
    static uint64_t clock(void *) noexcept{return hrt_absolute_time();}
    static uint64_t minimum(uint64_t a,uint64_t b) noexcept{return a<b?a:b;}
    static uint64_t expiry(uint64_t origin,uint64_t age) noexcept;
    bool fresh(uint64_t stamp,uint64_t now)const noexcept;
    bool refresh_control(uint64_t &original_receipt_us) noexcept;
    bool refresh_disarmed() noexcept;
    bool fail(Fault reason) noexcept;
    bool identity_matches() noexcept;

    const Identity expected_identity_;
    const gpenmpc_consumption::Limits limits_;
    const uint64_t telemetry_max_age_us_;
    Authority *const authority_;
    // Persistent, single-work-queue owner: never put this >12 KiB object on
    // the control tick stack or call capture/execute concurrently.
    gpenmpc_rfly_slim::SnapshotExecutor<4> executor_;
    uORB::Subscription odometry_;
    uORB::Subscription status_sub_{ORB_ID(vehicle_status)};
    uORB::Subscription mode_sub_{ORB_ID(vehicle_control_mode)};
    uORB::Subscription offboard_sub_{ORB_ID(offboard_control_mode)};
    uORB::Publication<actuator_outputs_s> output_{ORB_ID(actuator_outputs_rfly)};
    vehicle_odometry_s raw_{};
    vehicle_status_s status_{};
    vehicle_control_mode_s mode_{};
    offboard_control_mode_s offboard_{};
    actuator_outputs_s output_message_{};
    Ticket pending_ticket_{};
    bool pending_{false};
    bool observation_pending_{false};
    gpenmpc_rfly_state_execution::NumericalPrepared prepared_{};
    gpenmpc_rfly_execution::BackendRflyAck ack_{};
    gpenmpc_rfly_state_execution::NumericalReceipt receipt_{};
    CommittedFeedbackLatch feedback_{};
    Diagnostics diagnostics_{};
};
} // namespace gpenmpc_rfly_px4
