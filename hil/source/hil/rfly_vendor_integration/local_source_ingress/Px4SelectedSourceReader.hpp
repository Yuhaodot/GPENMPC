#pragma once
#include "Px4OriginalHilReceiptReader.hpp"
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/sensors_status_imu.h>
#include <uORB/topics/sensor_combined.h>

// Read-only single-EKF endpoint evidence. No sensors, parameters, HRT, estimator,
// publication or control action is written. This is NOT full EKF input-history
// lineage, physical/DLL association, authentication, or a controller permission.
// Caller serializes all access with its existing source-reader owner.
namespace gpenmpc_selected_source {
enum class Result : std::uint8_t { Matched, Missing, Unsupported, Invalid };
enum class Reason : std::uint32_t { None, ParameterMissing, ParameterType, ParameterRead,
    ParameterConfiguration, InvalidSnapshot, Identity, OriginalReader, OriginalMetadata,
    MissingTopic, IncompleteHistory, RetentionFull, Malformed, Regression,
    AmbiguousDevice, Selection, Endpoint, IntegratedValues, Retirement, Stopped };
template<class T> struct Observed { T value{}; std::uint32_t generation{}; };
struct Parameters { std::int32_t sens_imu_mode{}, ekf2_multi_imu{}, ekf2_multi_mag{}; };
struct Receipt {
    gpenmpc_hil_endpoint_reader::Receipt original{};
    Observed<sensor_gyro_s> gyro{};
    Observed<sensor_accel_s> accel{};
    Observed<vehicle_imu_s> imu{};
    Observed<sensor_selection_s> selection{};
    Observed<sensors_status_imu_s> status{};
    Observed<sensor_combined_s> combined{};
    Parameters actual_parameters{};
    // vehicle_odometry publication instance is NOT a selector's internal EKF
    // instance.
    std::uint8_t gyro_instance{}, accel_instance{}, imu_instance{}, vehicle_odometry_instance{};
    std::uint64_t snapshot_sample_us{}, snapshot_generation{}, snapshot_publication_us{};
    bool unique_selected_endpoint_observed{};
    bool full_integration_history_proven{},full_ekf_history_proven{},dll_association_proven{},control_authority{};
};
struct Diagnostics {
    std::uint64_t drained{},matched{},missing{},retired{},generation_gaps{};
    Reason first_reason{Reason::None}; Result latched_result{Result::Missing};
    bool unknown_prebaseline{true}; // never converted into a full-history claim
};
class Px4SelectedSourceReader final {
public:
    // Engineering retention only; no time/freshness allowance. No eviction of
    // unconsumed values. Raw driver queues are 8; stock IMU/metadata queue is 1.
    static constexpr std::size_t capacity=8, imu_instances=4;
    static constexpr std::size_t raw_instances=ORB_MULTI_MAX_INSTANCES;
    explicit Px4SelectedSourceReader(gpenmpc_hil_endpoint_reader::Px4OriginalHilReceiptReader&,
        const gpenmpc_consumption::Identity&) noexcept;
    Px4SelectedSourceReader(const Px4SelectedSourceReader&)=delete;
    Px4SelectedSourceReader&operator=(const Px4SelectedSourceReader&)=delete;
    Result drain() noexcept;
    Result lookup(const gpenmpc_odometry::Snapshot&,Receipt&) noexcept;
    // Owner separately retires the original reader. This only reclaims our
    // consumed records; selected/status baseline is retained, not re-timestamped.
    bool retire_through(std::uint64_t successful_endpoint) noexcept;
    void stop() noexcept;
    const Diagnostics& diagnostics()const noexcept{return diagnostics_;}
private:
    template<class T> struct History {
        Observed<T> records[capacity]{}; std::size_t count{};
        std::uint32_t last_generation{};std::uint64_t last_timestamp{};
        bool observed{},advertised{};
    };
    template<class T> bool collect(uORB::Subscription&,History<T>&) noexcept;
    template<class T> static const Observed<T>* exact(const History<T>&,std::uint64_t) noexcept;
    template<class T> static void retire(History<T>&,std::uint64_t,bool keep_baseline) noexcept;
    bool parameters(Parameters&) noexcept;
    Result latch(Reason,Result) noexcept;
    Result missing() noexcept;
    gpenmpc_hil_endpoint_reader::Px4OriginalHilReceiptReader& original_;
    const gpenmpc_consumption::Identity identity_;
    uORB::Subscription gyro_sub_[raw_instances]{};
    uORB::Subscription accel_sub_[raw_instances]{};
    uORB::Subscription imu_sub_[imu_instances]{};
    uORB::Subscription selection_sub_{ORB_ID(sensor_selection)};
    uORB::Subscription status_sub_{ORB_ID(sensors_status_imu)};
    uORB::Subscription combined_sub_{ORB_ID(sensor_combined)};
    History<sensor_gyro_s> gyro_[raw_instances]{};
    History<sensor_accel_s> accel_[raw_instances]{};
    History<vehicle_imu_s> imu_[imu_instances]{};
    History<sensor_selection_s> selection_{};
    History<sensors_status_imu_s> status_{};
    History<sensor_combined_s> combined_{};
    Diagnostics diagnostics_{};
    Parameters parameters_{};
    std::uint64_t last_success_{},retired_through_{};
};
}
