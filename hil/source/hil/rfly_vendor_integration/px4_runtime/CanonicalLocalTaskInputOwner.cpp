#include "CanonicalLocalTaskInputOwner.hpp"
#include "../CanonicalOperatorReference.hpp"
namespace gpenmpc_rfly_px4 {
namespace tw=gpenmpc_local_task_wire;
CanonicalLocalTaskInputOwner::CanonicalLocalTaskInputOwner(const LocalTaskInputConfiguration&c)noexcept:c_(c),window_(c.window){
    if(window_.failed()||!c.input_assembly_max_us||!c.outer_max_age_us||
       gpenmpc_full_inner_window_scratch_bytes()>scratch_capacity)
        fail(LocalTaskInputFault::Configuration,0);
}
LocalTaskInputPort CanonicalLocalTaskInputOwner::port()noexcept{
    return {this,&read_entry,&window_entry,&window_,this,&observe_entry};
}
bool CanonicalLocalTaskInputOwner::fail(LocalTaskInputFault f,std::uint64_t now)noexcept{
    if(d_.first_fault==LocalTaskInputFault::None){d_.first_fault=f;d_.first_fault_us=now;
        d_.fault_sequence=sequence_;d_.fault_first_arrival_us=first_arrival_;d_.fault_last_arrival_us=last_arrival_;
        d_.fault_next_fragment=next_;d_.fault_ready=ready_;
        if(inside_arrival_)fault_arrival_=*inside_arrival_;}
    return false;
}
void CanonicalLocalTaskInputOwner::stop(std::uint64_t now)noexcept{(void)fail(LocalTaskInputFault::Stopped,now);}
bool CanonicalLocalTaskInputOwner::tick(std::uint64_t now,bool disarmed_observation)noexcept{
    if(d_.first_fault!=LocalTaskInputFault::None)return false;
    if(!now||now<d_.last_processing_us)return fail(LocalTaskInputFault::Clock,now);
    d_.last_processing_us=now;
    const bool manual_stream=c_.runtime_state_only&&c_.component_initialization&&c_.operator_reference;
    // Manual input uses source and command age checks without an additional
    // packet-assembly deadline. Complete-message order and source binding apply.
    if((assembling_||ready_)&&(!first_arrival_||now<first_arrival_||
       (!manual_stream&&now-first_arrival_>c_.input_assembly_max_us))){
        // A complete observation can await a new uORB sample; capture's
        // NoUpdate must not prevent the existing disarmed retirement rule.
        // Mode comes from THIS core poll, never the previous observed mode.
        // Partial packets, active control and unknown sources remain fatal.
        if(disarmed_observation&&c_.runtime_state_only&&ready_&&!assembling_&&
           first_arrival_&&now>=first_arrival_){
            bool known=false;
            for(const auto&a:anchors_)known=known||gpenmpc_local_input::same_source(a,staged_.source);
            if(!known)return fail(LocalTaskInputFault::Source,now);
            ready_=false;++d_.missing_inputs;return true;
        }
        return fail(LocalTaskInputFault::Expired,now);
    }
    return true;
}
bool CanonicalLocalTaskInputOwner::receive(const gpenmpc_argument_transport::Arrival&a,std::uint64_t now)noexcept{
    inside_arrival_=&a;
    const auto done=[&](bool ok){inside_arrival_=nullptr;return ok;};
    if(!tick(now))return done(false);
    if(ready_)return done(fail(LocalTaskInputFault::Busy,now));
    const auto&f=a.fields;const unsigned index=f.payload[0]&15;
    if((f.payload[0]>>4)!=tw::schema||index>=tw::fragment_count||!f.timestamp||f.timestamp>now||f.timestamp<last_arrival_)
        return done(fail(LocalTaskInputFault::Fragment,now));
    const auto generation=gpenmpc_argument_transport::get64(f.payload+1);
    if(!assembling_){
        if(index||!generation||generation<=highest_started_)return done(fail(LocalTaskInputFault::Regression,now));
        sequence_=generation;highest_started_=generation;first_arrival_=f.timestamp;next_=0;assembling_=true;
    }
    if(index!=next_||generation!=sequence_)return done(fail(LocalTaskInputFault::Fragment,now));
    const unsigned offset=index*119,n=unsigned(bytes_.size()-offset)<119?unsigned(bytes_.size()-offset):119;
    if(f.payload_length!=n+9)return done(fail(LocalTaskInputFault::Fragment,now));
    std::memcpy(bytes_.data()+offset,f.payload+9,n);last_arrival_=f.timestamp;++next_;++d_.fragments;
    if(next_!=tw::fragment_count)return done(true);
    if(!tw::decode(bytes_,staged_)||staged_.source.source_generation!=sequence_)
        return done(fail(LocalTaskInputFault::Integrity,now));
    const auto&e=c_.window.expected;
    if(!(staged_.source.identity==e.identity)||staged_.session_sha!=e.execution_session_sha256||
       staged_.configuration_sha!=e.configuration_sha256||staged_.task_sha!=e.task_sha256||
       staged_.reference_sha!=e.reference_asset_sha256||staged_.leg!=e.leg_index)
        return done(fail(LocalTaskInputFault::IdentityMismatch,now));
    if(staged_.source.original_receipt_us>first_arrival_)
        return done(fail(LocalTaskInputFault::Clock,now));
    const auto span=now-first_arrival_;if(span>d_.maximum_assembly_span_us)d_.maximum_assembly_span_us=span;
    ready_arrival_=f.timestamp;assembling_=false;ready_=true;++d_.messages;return done(true);
}
bool CanonicalLocalTaskInputOwner::retain_source(const gpenmpc_local_input::SnapshotKey&key)noexcept{
    for(auto&a:anchors_){
        if(a.source_generation==key.source_generation)return gpenmpc_local_input::same_source(a,key)||fail(LocalTaskInputFault::Source,d_.last_processing_us);
        if(a.source_generation&&d_.last_processing_us>=a.sample_us&&d_.last_processing_us-a.sample_us>c_.outer_max_age_us)a={};
    }
    for(auto&a:anchors_)if(!a.source_generation){a=key;++d_.sources_retained;return true;}
    return fail(LocalTaskInputFault::OuterAnchor,d_.last_processing_us);
}
bool CanonicalLocalTaskInputOwner::accept_outer(const tw::Message&m)noexcept{
    const auto now=d_.last_processing_us;
    if(have_outer_&&m.outer_generation<outer_.generation)return fail(LocalTaskInputFault::Regression,now);
    if(have_outer_&&m.outer_generation==outer_.generation){
        const auto&p=outer_original_;
        if(m.outer_source_generation!=p.outer_source_generation||m.outer_sample_us!=p.outer_sample_us||
           m.outer_original_host_source_rx_ns!=p.outer_original_host_source_rx_ns||m.outer_creation_ns!=p.outer_creation_ns||
           m.outer_expiry_ns!=p.outer_expiry_ns||std::memcmp(m.outer_target4,p.outer_target4,sizeof m.outer_target4))
            return fail(LocalTaskInputFault::OuterMutation,now);
        return now<=outer_.valid_until_us||fail(LocalTaskInputFault::OuterExpired,now);
    }
    const gpenmpc_local_input::SnapshotKey*anchor=nullptr;
    for(const auto&a:anchors_)if(a.source_generation==m.outer_source_generation&&a.sample_us==m.outer_sample_us)anchor=&a;
    if(!anchor||!(anchor->identity==m.source.identity)||anchor->original_receipt_us>ready_arrival_)
        return fail(LocalTaskInputFault::OuterAnchor,now);
    // Same conservative original-source anchoring as RCT1; HOST ns remain
    // HOST ns. An updated sensor input NEVER renews a held outer generation.
    const auto elapsed=m.outer_creation_ns-m.outer_original_host_source_rx_ns;
    if(elapsed/1000>c_.outer_max_age_us||(elapsed/1000==c_.outer_max_age_us&&elapsed%1000))
        return fail(LocalTaskInputFault::OuterExpired,now);
    auto horizon=(m.outer_expiry_ns-m.outer_original_host_source_rx_ns)/1000;
    if(horizon>c_.outer_max_age_us)horizon=c_.outer_max_age_us;
    if(!horizon||anchor->sample_us>UINT64_MAX-horizon||now>anchor->sample_us+horizon||
       (have_outer_&&(ready_arrival_<=outer_.board_rx_us||m.outer_creation_ns<=outer_original_.outer_creation_ns)))
        return fail(LocalTaskInputFault::OuterExpired,now);
    outer_={};outer_.identity=anchor->identity;outer_.generation=m.outer_generation;
    outer_.based_on_sample_generation=anchor->source_generation;outer_.based_on_timestamp_sample_us=anchor->sample_us;
    outer_.board_rx_us=ready_arrival_;outer_.valid_until_us=anchor->sample_us+horizon;
    gpenmpc_consumption::CanonicalSha256 h;for(auto v:m.outer_target4)h.real(v);outer_.payload_sha256=h.finish();
    outer_original_=m;have_outer_=true;++d_.outer_installs;d_.last_outer=m.outer_generation;return true;
}
LocalTaskRead CanonicalLocalTaskInputOwner::window_entry(void*p,LocalTaskView&v)noexcept{
    return static_cast<CanonicalLocalTaskInputOwner*>(p)->window(v);
}
LocalTaskRead CanonicalLocalTaskInputOwner::window(LocalTaskView&v)noexcept{
    v={};if(!tick(hrt_absolute_time())||window_.failed())return LocalTaskRead::Invalid;
    const auto*w=window_.ready_window(d_.last_processing_us);
    if(!w)return LocalTaskRead::Missing;
    v.window=w;v.window_scratch=scratch_;v.window_scratch_bytes=sizeof scratch_;
    v.inputs.loaded_window_generation=w->window_generation;return LocalTaskRead::Ready;
}
LocalTaskRead CanonicalLocalTaskInputOwner::read_entry(void*p,const gpenmpc_odometry::Snapshot&s,
    const gpenmpc_selected_source::Receipt&r,LocalTaskView&v)noexcept{
    return static_cast<CanonicalLocalTaskInputOwner*>(p)->read(s,r,v);
}
LocalTaskRead CanonicalLocalTaskInputOwner::observe_entry(void*p,const gpenmpc_odometry::Snapshot&s,
    const gpenmpc_selected_source::Receipt&r,bool exported_snapshot)noexcept{
    return static_cast<CanonicalLocalTaskInputOwner*>(p)->observe(s,r,exported_snapshot);
}
LocalTaskRead CanonicalLocalTaskInputOwner::observe(const gpenmpc_odometry::Snapshot&s,
    const gpenmpc_selected_source::Receipt&r,bool exported_snapshot)noexcept{
    if(!tick(hrt_absolute_time()))return LocalTaskRead::Invalid;
    gpenmpc_local_input::SnapshotKey key{};
    if(!gpenmpc_local_input::snapshot_key(s,key)||(!c_.runtime_state_only&&(!r.unique_selected_endpoint_observed||
       !r.original.endpoint.exact_unique_endpoint||!gpenmpc_local_input::same_source(key,r.original.original_snapshot_key)))||
       ((!c_.runtime_state_only||exported_snapshot)&&!retain_source(key))){
        fail(LocalTaskInputFault::Source,d_.last_processing_us);return LocalTaskRead::Invalid;
    }
    // The fixed anchor set serves original HOST responses, not every native
    // uORB update. Fast read/receive still validates current state and the
    // held input's exact source/age; only exported RLS needs long retention.
    if(c_.runtime_state_only&&ready_){
        bool known=false;
        for(const auto&a:anchors_)known=known||gpenmpc_local_input::same_source(a,staged_.source);
        if(!known||staged_.source.sample_us>key.sample_us){
            fail(LocalTaskInputFault::Source,d_.last_processing_us);return LocalTaskRead::Invalid;
        }
        if(key.sample_us-staged_.source.sample_us>gpenmpc_local_input::maximum_held_input_age_us){
            // A complete known candidate has not installed any numerical
            // state yet. Retire it without renewing current_ or outer_, even
            // while armed, so the next fresh packet can still be received.
            // read() independently requires the bounded current input;
            // otherwise no control executes. Actual control-dt/lease expiry,
            // partial assemblies and unknown/future sources remain terminal.
            ready_=false;++d_.missing_inputs;return LocalTaskRead::Ready;
        }
        // Runtime-state inputs carry slow environment/rotor initialization
        // and an independently aged outer command. Their source is retained,
        // not relabelled as the current fast state. Only the explicit inner
        // component test requires a zero outer command.
        if(c_.operator_reference){
            const auto*v=staged_.outer_target4;
            // RC target4 is [yaw rate, world vx, world vy, upward speed],
            // with bounds from the operator-reference configuration.
            if(!c_.component_initialization||std::fabs(v[0])>gpenmpc_operator_reference::yaw_rate_rad_s||std::fabs(v[1])>gpenmpc_operator_reference::horizontal_speed_mps||std::fabs(v[2])>gpenmpc_operator_reference::horizontal_speed_mps||std::fabs(v[3])>gpenmpc_operator_reference::vertical_speed_mps){
                fail(LocalTaskInputFault::OuterMutation,d_.last_processing_us);return LocalTaskRead::Invalid;
            }
        }else if(c_.component_initialization)for(double v:staged_.outer_target4)if(!(v>=0.0&&v<=0.0)){fail(LocalTaskInputFault::OuterMutation,d_.last_processing_us);return LocalTaskRead::Invalid;}
        if(!accept_outer(staged_))return LocalTaskRead::Invalid;
        current_=staged_;have_current_=true;ready_=false;++d_.bindings;d_.last_source=staged_.source.source_generation;
    }
    return LocalTaskRead::Ready;
}
LocalTaskRead CanonicalLocalTaskInputOwner::read(const gpenmpc_odometry::Snapshot&s,
    const gpenmpc_selected_source::Receipt&r,LocalTaskView&v)noexcept{
    v={};if(observe(s,r,false)!=LocalTaskRead::Ready)return LocalTaskRead::Invalid;
    gpenmpc_local_input::SnapshotKey key{};
    if(!gpenmpc_local_input::snapshot_key(s,key)){
        fail(LocalTaskInputFault::Source,d_.last_processing_us);return LocalTaskRead::Invalid;
    }
    if(ready_){
        if(!gpenmpc_local_input::same_source(key,staged_.source)||
           (!c_.runtime_state_only&&std::memcmp(staged_.original_sensor52,r.original.endpoint.original.original_sensor52,52))){
            fail(LocalTaskInputFault::Sensor,d_.last_processing_us);return LocalTaskRead::Invalid;
        }
        if(!accept_outer(staged_))return LocalTaskRead::Invalid;
        current_=staged_;have_current_=true;ready_=false;++d_.bindings;d_.last_source=key.source_generation;
    }
    const bool held=c_.runtime_state_only&&have_current_&&key.identity==current_.source.identity&&
        (key.reset_counter==current_.source.reset_counter||s.heading_reset_from(current_.source.reset_counter))&&key.sample_us>=current_.source.sample_us&&
        key.sample_us-current_.source.sample_us<=gpenmpc_local_input::maximum_held_input_age_us;
    if(!have_current_||(!held&&!gpenmpc_local_input::same_source(key,current_.source))){
        ++d_.missing_inputs;return LocalTaskRead::Missing;
    }
    if(!have_outer_||d_.last_processing_us>outer_.valid_until_us){
        fail(LocalTaskInputFault::OuterExpired,d_.last_processing_us);return LocalTaskRead::Invalid;
    }
    const auto&input_key=held?current_.source:key;
    rotor_={};rotor_.source=input_key;rotor_.value_kind=gpenmpc_local_input::RotorValueKind::OriginalPlantLagState;
    auto&rv=rotor_.original_observation;for(unsigned j=0;j<6;++j)rv.observed_thrust_n[j]=current_.rotor_n[j];
    rv.dll_generation=current_.rotor_generation;rv.dll_session=current_.rotor_session;
    rv.original_host_receive_ns=current_.rotor_host_receive_ns;rv.original_board_ingress_us=current_.source.original_receipt_us;
    rv.original_sim_time_s=current_.rotor_sim_time_s;rv.original_observation_sha=current_.rotor_observation_sha;
    rotor_.verified_association_receipt_sha256=current_.rotor_association_sha;
    payload_={input_key,current_.task_sha,current_.payload_evidence_sha,current_.payload_generation,current_.payload_kg};
    wind_.source=input_key;wind_.original_estimate_evidence_sha256=current_.wind_evidence_sha;
    wind_.original_estimate_generation=current_.wind_generation;
    for(unsigned j=0;j<2;++j)wind_.estimate_xy_mps[j]=current_.estimated_wind_xy[j];
    v.inputs.rotor=&rotor_;v.inputs.payload=&payload_;v.inputs.wind=&wind_;v.inputs.outer=outer_;
    std::memcpy(v.inputs.target4,current_.outer_target4,sizeof v.inputs.target4);
    const auto*w=window_.ready_window(d_.last_processing_us);
    v.inputs.loaded_window_generation=w?w->window_generation:window_.diagnostics().released_generation;
    if(w){v.window=w;v.window_scratch=scratch_;v.window_scratch_bytes=sizeof scratch_;}
    return v.inputs.loaded_window_generation?LocalTaskRead::Ready:LocalTaskRead::Missing;
}
}
