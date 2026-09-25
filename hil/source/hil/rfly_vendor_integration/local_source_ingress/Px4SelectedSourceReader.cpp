#include "Px4SelectedSourceReader.hpp"
#include <parameters/param.h>
#include <cmath>
#include <cstring>
namespace gpenmpc_selected_source {
namespace {
template<class T> std::uint64_t endpoint(const T&v) noexcept{return v.timestamp_sample;}
std::uint64_t endpoint(const sensor_selection_s&v) noexcept{return v.timestamp;}
std::uint64_t endpoint(const sensors_status_imu_s&v) noexcept{return v.timestamp;}
std::uint64_t endpoint(const sensor_combined_s&v) noexcept{return v.timestamp;}
bool finite3(const float*v)noexcept{return std::isfinite(v[0])&&std::isfinite(v[1])&&std::isfinite(v[2]);}
bool valid(const sensor_gyro_s&v)noexcept{return v.timestamp_sample&&v.timestamp>=v.timestamp_sample&&v.device_id&&std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
bool valid(const sensor_accel_s&v)noexcept{return v.timestamp_sample&&v.timestamp>=v.timestamp_sample&&v.device_id&&std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
bool valid(const vehicle_imu_s&v)noexcept{return v.timestamp_sample&&v.timestamp>=v.timestamp_sample&&v.accel_device_id&&v.gyro_device_id&&v.delta_angle_dt&&v.delta_velocity_dt&&finite3(v.delta_angle)&&finite3(v.delta_velocity);}
bool valid(const sensor_selection_s&v)noexcept{return v.timestamp&&v.accel_device_id&&v.gyro_device_id;}
// sensors_poll publishes a timestamped zero-selection status during startup.
bool valid(const sensors_status_imu_s&v)noexcept{return v.timestamp!=0;}
bool valid(const sensor_combined_s&v)noexcept{return v.timestamp&&v.gyro_integral_dt&&v.accelerometer_integral_dt&&finite3(v.gyro_rad)&&finite3(v.accelerometer_m_s2);}
bool bits(float a,float b)noexcept{return std::memcmp(&a,&b,sizeof a)==0;}
}
Px4SelectedSourceReader::Px4SelectedSourceReader(gpenmpc_hil_endpoint_reader::Px4OriginalHilReceiptReader&r,
    const gpenmpc_consumption::Identity&id)noexcept:original_(r),identity_(id){
    for(std::size_t i=0;i<raw_instances;++i){gyro_sub_[i]=uORB::Subscription{ORB_ID(sensor_gyro),static_cast<std::uint8_t>(i)};accel_sub_[i]=uORB::Subscription{ORB_ID(sensor_accel),static_cast<std::uint8_t>(i)};}
    for(std::size_t i=0;i<imu_instances;++i)imu_sub_[i]=uORB::Subscription{ORB_ID(vehicle_imu),static_cast<std::uint8_t>(i)};
}
Result Px4SelectedSourceReader::latch(Reason why,Result result)noexcept{
    if(diagnostics_.first_reason==Reason::None){diagnostics_.first_reason=why;diagnostics_.latched_result=result;}return diagnostics_.latched_result;
}
Result Px4SelectedSourceReader::missing()noexcept{++diagnostics_.missing;return Result::Missing;}
bool Px4SelectedSourceReader::parameters(Parameters&out)noexcept{
    const char*names[]{"SENS_IMU_MODE","EKF2_MULTI_IMU","EKF2_MULTI_MAG"};std::int32_t*values[]{&out.sens_imu_mode,&out.ekf2_multi_imu,&out.ekf2_multi_mag};
    for(unsigned i=0;i<3;++i){const param_t p=param_find_no_notification(names[i]);
        if(p==PARAM_INVALID){latch(Reason::ParameterMissing,Result::Unsupported);return false;}
        if(param_type(p)!=PARAM_TYPE_INT32){latch(Reason::ParameterType,Result::Unsupported);return false;}
        if(param_get(p,values[i])!=0){latch(Reason::ParameterRead,Result::Unsupported);return false;}}
    if(out.sens_imu_mode!=1||out.ekf2_multi_imu!=0||out.ekf2_multi_mag!=0){latch(Reason::ParameterConfiguration,Result::Unsupported);return false;}return true;
}
template<class T> bool Px4SelectedSourceReader::collect(uORB::Subscription&sub,History<T>&h)noexcept{
    h.advertised=sub.advertised();T message{};
    for(std::size_t n=0;n<capacity;++n){if(!sub.update(&message))return true;++diagnostics_.drained;
        const auto g=sub.get_last_generation();const auto t=endpoint(message);
        if(!valid(message)){latch(Reason::Malformed,Result::Invalid);return false;}
        if(h.observed&&(h.last_generation==UINT32_MAX||g!=h.last_generation+1)){
            ++diagnostics_.generation_gaps;latch(Reason::IncompleteHistory,Result::Missing);return false;}
        if(h.observed&&t<=h.last_timestamp){latch(Reason::Regression,Result::Invalid);return false;}
        if(h.count==capacity){latch(Reason::RetentionFull,Result::Missing);return false;}
        h.records[h.count++]={message,g};h.observed=true;h.last_generation=g;h.last_timestamp=t;
    }
    // Leave unread pending updates visible after a bounded drain.
    if(sub.updated()){latch(Reason::IncompleteHistory,Result::Missing);return false;}return true;
}
Result Px4SelectedSourceReader::drain()noexcept{
    if(diagnostics_.first_reason!=Reason::None)return diagnostics_.latched_result;
    if(!parameters(parameters_))return diagnostics_.latched_result;
    for(std::size_t i=0;i<raw_instances;++i)if(!collect(gyro_sub_[i],gyro_[i])||!collect(accel_sub_[i],accel_[i]))return diagnostics_.latched_result;
    for(std::size_t i=0;i<imu_instances;++i)if(!collect(imu_sub_[i],imu_[i]))return diagnostics_.latched_result;
    if(!collect(selection_sub_,selection_)||!collect(status_sub_,status_)||!collect(combined_sub_,combined_))return diagnostics_.latched_result;
    return Result::Missing; // draining alone never supplies a matched receipt
}
template<class T> const Observed<T>* Px4SelectedSourceReader::exact(const History<T>&h,std::uint64_t t)noexcept{
    for(std::size_t i=0;i<h.count;++i){if(endpoint(h.records[i].value)==t)return &h.records[i];}
    return nullptr;
}
Result Px4SelectedSourceReader::lookup(const gpenmpc_odometry::Snapshot&s,Receipt&out)noexcept{
    out=Receipt{};if(diagnostics_.first_reason!=Reason::None)return diagnostics_.latched_result;
    if(!s.valid())return latch(Reason::InvalidSnapshot,Result::Invalid);
    const auto&state=s.estimator();if(!(state.identity==identity_)||s.source_topic()!=ORB_ID(vehicle_odometry)||s.source_instance()!=0)return latch(Reason::Identity,Result::Invalid);
    if(!parameters(parameters_))return diagnostics_.latched_result;
    const auto t=state.timestamp_sample_us;
    if(t<=retired_through_)return latch(Reason::Retirement,Result::Invalid);
    const auto*c=exact(combined_,t);if(!c)return missing();
    // Obtain the receipt from the source reader.
    gpenmpc_hil_endpoint_reader::Receipt origin{};
    if(!original_.lookup(s,origin))return latch(Reason::OriginalReader,Result::Invalid);
    const auto&o=origin.original.topic;
    if(!origin.endpoint.exact_unique_endpoint||origin.endpoint.snapshot_sample_us!=t||origin.endpoint.snapshot_generation!=state.generation||o.timestamp!=t||
       !o.gyro_update_called||!o.accel_update_called||o.gyro_topic_instance<0||o.accel_topic_instance<0||
       o.gyro_topic_instance>=static_cast<int>(imu_instances)||o.accel_topic_instance>=static_cast<int>(imu_instances)||
       o.gyro_topic_instance!=o.accel_topic_instance||!o.gyro_device_id||!o.accel_device_id)
        return latch(Reason::OriginalMetadata,Result::Invalid);
    const auto gi=static_cast<std::size_t>(o.gyro_topic_instance),ai=static_cast<std::size_t>(o.accel_topic_instance);
    const auto*g=exact(gyro_[gi],t);const auto*a=exact(accel_[ai],t);const auto*v=exact(imu_[gi],t);
    if(!g||!a||!v)return missing();
    if(g->value.device_id!=o.gyro_device_id||a->value.device_id!=o.accel_device_id||v->value.gyro_device_id!=o.gyro_device_id||v->value.accel_device_id!=o.accel_device_id||
       v->value.timestamp<g->value.timestamp||v->value.timestamp<a->value.timestamp||v->value.timestamp>state.publication_us)
        return latch(Reason::Endpoint,Result::Invalid);
    // Reject duplicate devices and advertised instances without a retained baseline.
    for(std::size_t i=0;i<raw_instances;++i){
        if((gyro_[i].advertised&&!gyro_[i].observed)||(accel_[i].advertised&&!accel_[i].observed))return missing();
        for(std::size_t j=0;j<gyro_[i].count;++j)if(i!=gi&&gyro_[i].records[j].value.device_id==o.gyro_device_id)return latch(Reason::AmbiguousDevice,Result::Invalid);
        for(std::size_t j=0;j<accel_[i].count;++j)if(i!=ai&&accel_[i].records[j].value.device_id==o.accel_device_id)return latch(Reason::AmbiguousDevice,Result::Invalid);
    }
    const Observed<sensors_status_imu_s>*status=nullptr;
    // imuPoll creates this status after converting the IMU. Keep its original
    // publication HRT; it must fit inside this estimator publication bracket.
    for(std::size_t i=0;i<status_.count;++i){const auto&r=status_.records[i];if(r.value.timestamp>=v->value.timestamp&&r.value.timestamp<=state.publication_us){status=&r;break;}}
    if(!status)return missing();
    const Observed<sensor_selection_s>*selection=nullptr;
    for(std::size_t i=0;i<selection_.count;++i)if(selection_.records[i].value.timestamp<=status->value.timestamp)selection=&selection_.records[i];
    if(!selection)return missing();
    if(!status->value.gyro_device_id_primary||!status->value.accel_device_id_primary)return missing();
    if(selection->value.gyro_device_id!=o.gyro_device_id||selection->value.accel_device_id!=o.accel_device_id||
       status->value.gyro_device_id_primary!=o.gyro_device_id||status->value.accel_device_id_primary!=o.accel_device_id)
        return latch(Reason::Selection,Result::Invalid);
    for(std::size_t i=0;i<imu_instances;++i){const auto&z=status->value;
        if(i!=gi&&(z.gyro_device_ids[i]==o.gyro_device_id||z.accel_device_ids[i]==o.accel_device_id))return latch(Reason::AmbiguousDevice,Result::Invalid);
        // Health/priority remain original diagnostic fields. This observer
        // must not add a control-admission threshold on top of stock voting.
        if(i==gi&&(z.gyro_device_ids[i]!=o.gyro_device_id||z.accel_device_ids[i]!=o.accel_device_id))return latch(Reason::Selection,Result::Invalid);
    }
    // A later observed selection change before the estimator publication
    // invalidates this bracket. Do not cherry-pick an earlier matching record.
    for(std::size_t i=0;i<selection_.count;++i){const auto&z=selection_.records[i].value;
        if(z.timestamp>=selection->value.timestamp&&z.timestamp<=state.publication_us&&
           (z.gyro_device_id!=o.gyro_device_id||z.accel_device_id!=o.accel_device_id))return latch(Reason::Selection,Result::Invalid);}
    for(std::size_t i=0;i<status_.count;++i){const auto&z=status_.records[i].value;
        if(z.timestamp<status->value.timestamp||z.timestamp>state.publication_us)continue;
        if(z.gyro_device_id_primary!=o.gyro_device_id||z.accel_device_id_primary!=o.accel_device_id||
           z.gyro_device_ids[gi]!=o.gyro_device_id||z.accel_device_ids[ai]!=o.accel_device_id)return latch(Reason::Selection,Result::Invalid);
        for(std::size_t j=0;j<imu_instances;++j)if(j!=gi&&(z.gyro_device_ids[j]==o.gyro_device_id||z.accel_device_ids[j]==o.accel_device_id))return latch(Reason::AmbiguousDevice,Result::Invalid);
    }
    const auto&im=v->value;const auto&co=c->value;
    if(co.accelerometer_timestamp_relative!=0||co.gyro_integral_dt!=im.delta_angle_dt||co.accelerometer_integral_dt!=im.delta_velocity_dt||
       co.gyro_clipping!=im.delta_angle_clipping||co.accelerometer_clipping!=im.delta_velocity_clipping||
       co.gyro_calibration_count!=im.gyro_calibration_count||co.accel_calibration_count!=im.accel_calibration_count)
        return latch(Reason::IntegratedValues,Result::Invalid);
    // Exactly the original voted_sensors_update.cpp float32 operation order.
    const float gyro_inverse=1.e6f/static_cast<float>(im.delta_angle_dt),accel_inverse=1.e6f/static_cast<float>(im.delta_velocity_dt);
    for(unsigned k=0;k<3;++k)if(!bits(im.delta_angle[k]*gyro_inverse,co.gyro_rad[k])||!bits(im.delta_velocity[k]*accel_inverse,co.accelerometer_m_s2[k]))return latch(Reason::IntegratedValues,Result::Invalid);
    out.original=origin;out.gyro=*g;out.accel=*a;out.imu=*v;out.selection=*selection;out.status=*status;out.combined=*c;out.actual_parameters=parameters_;
    out.gyro_instance=static_cast<std::uint8_t>(gi);out.accel_instance=static_cast<std::uint8_t>(ai);out.imu_instance=static_cast<std::uint8_t>(gi);out.vehicle_odometry_instance=s.source_instance();
    out.snapshot_sample_us=t;out.snapshot_generation=state.generation;out.snapshot_publication_us=state.publication_us;out.unique_selected_endpoint_observed=true;
    last_success_=t;++diagnostics_.matched;return Result::Matched;
}
template<class T> void Px4SelectedSourceReader::retire(History<T>&h,std::uint64_t t,bool keep)noexcept{
    std::size_t n=0;while(n<h.count&&endpoint(h.records[n].value)<=t)++n;
    if(keep&&n)--n;
    for(std::size_t i=n;i<h.count;++i){h.records[i-n]=h.records[i];}
    h.count-=n;
}
bool Px4SelectedSourceReader::retire_through(std::uint64_t t)noexcept{
    if(diagnostics_.first_reason!=Reason::None)return false;
    if(!t||t!=last_success_||t<=retired_through_){latch(Reason::Retirement,Result::Invalid);return false;}
    for(std::size_t i=0;i<raw_instances;++i){retire(gyro_[i],t,true);retire(accel_[i],t,true);}
    for(std::size_t i=0;i<imu_instances;++i)retire(imu_[i],t,true);
    retire(combined_,t,false);retire(selection_,t,true);retire(status_,t,true);
    retired_through_=t;++diagnostics_.retired;return true;
}
void Px4SelectedSourceReader::stop()noexcept{latch(Reason::Stopped,Result::Invalid);}
}
