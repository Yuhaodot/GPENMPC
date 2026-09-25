#include "CanonicalLocalExchangeCore.hpp"
namespace gpenmpc_rfly_px4 {
LocalWireConfiguration CanonicalLocalExchangeCore::wire_config(const LocalExchangeConfiguration&c,const void*p)noexcept{
    LocalWireConfiguration w{};w.expected_session=c.identity;w.producer_identity=p;
    w.gp_request_transport_max_age_us=c.gp_request_transport_max_age_us;w.snapshot_transport_max_age_us=c.snapshot_transport_max_age_us;
    w.target_system=c.transport.source_system;w.target_component=c.transport.source_component;return w;
}
CanonicalLocalExchangeCore::CanonicalLocalExchangeCore(Px4CanonicalLocalIo&i,CanonicalLocalExecutionCycle&c,
    gpenmpc_selected_source::Px4SelectedSourceReader&s,const LocalTaskInputPort&t,const LocalExchangeConfiguration&cfg)noexcept:
    io_(i),cycle_(c),selected_(s),task_(t),export_active_snapshots_(cfg.export_active_snapshots),
    export_committed_state_(cfg.export_committed_state),
    pending_(c),ingress_(cfg.transport,t.window_receiver,t.task_receiver,c.runtime_state_only()),outbox_(wire_config(cfg,this)),
    subscription_(ORB_ID(gpenmpc_full_inner_ingress),cfg.ingress_topic_instance){
    if(!cfg.identity.uid||!cfg.identity.boot_generation||cfg.identity.system!=cfg.transport.target_system||
       cfg.identity.component!=cfg.transport.target_component||!cfg.gp_request_transport_max_age_us||!cfg.snapshot_transport_max_age_us||
       cfg.ingress_topic_instance>=ORB_MULTI_MAX_INSTANCES||!subscription_.valid()||ingress_.failed()||outbox_.fault()!=LocalWireFault::None||
       ((task_.owner==nullptr)!=(task_.read==nullptr))||((task_.owner==nullptr)!=(task_.window==nullptr))||
       ((task_.window_receiver||task_.task_receiver)&&!task_.owner))fail(LocalExchangeFault::Configuration);
#ifndef GPENMPC_CANONICAL_CLOSED_EVIDENCE
    // Fixture libraries require an explicit fixture configuration. The
    // application context requires the closed-evidence ABI before export.
    if(export_committed_state_)fail(LocalExchangeFault::Configuration);
#endif
}
bool CanonicalLocalExchangeCore::enqueue_committed()noexcept{
    if(!committed_pending_)return true;
    // Buffer the immutable component-history RLC until a newer RLS has been sent.
    // The outbox checks its retained source timestamp and expiry.
    if(cycle_.component_initialization()&&export_active_snapshots_&&
       (last_snapshot_export_us_<=last_committed_export_us_||
        last_component_input_sample_us_<=committed_observation_.source_timestamp_ns/1000))return true;
    if(cycle_.runtime_state_only()&&!cycle_.component_initialization()&&export_active_snapshots_&&
       last_component_input_sample_us_<=committed_observation_.source_timestamp_ns/1000)return true;
    // Export a buffered RLC after a newer input has supported a commit.
    // Component history is lower priority than control input; full-method RLC
    // also supplies the outer calculation and uses the bounded burst path.
    // RLS, RLI, GP and environment deadlines are checked independently.
    const auto result=outbox_.publish_committed(committed_bytes_,this,cycle_.runtime_state_only(),
        cycle_.runtime_state_only()&&!cycle_.component_initialization());
    if(result==LocalWirePublish::Unavailable)return fail(LocalExchangeFault::Outbox);
    if(result==LocalWirePublish::Published){committed_pending_=false;++d_.committed_states_enqueued;}
    return true;
}
bool CanonicalLocalExchangeCore::load_task_window(const LocalTaskView&v)noexcept{
    if(!v.window)return fail(LocalExchangeFault::Window);
    const auto generation=v.window->window_generation;
    if(generation==loaded_window_generation_)return true;
    if(generation<=loaded_window_generation_||
       (task_.window_receiver&&v.window!=task_.window_receiver->ready_window(hrt_absolute_time()))||
       !io_.load_window(*v.window,v.window_scratch,v.window_scratch_bytes))return fail(LocalExchangeFault::Window);
    // Convert and install before releasing the completed receive buffer. Latch failure.
    loaded_window_generation_=generation;++d_.window_loads;
    if(task_.window_receiver&&!task_.window_receiver->release(generation))return fail(LocalExchangeFault::Window);
    return true;
}
bool CanonicalLocalExchangeCore::fail(LocalExchangeFault f)noexcept{
    if(d_.first_fault==LocalExchangeFault::None){d_.first_fault=f;d_.first_fault_us=hrt_absolute_time();}
    outbox_.close();ingress_.stop(hrt_absolute_time());pending_.stop();selected_.stop();return false;
}
bool CanonicalLocalExchangeCore::enqueue_gp()noexcept{
    if(!pending_.pending()||gp_enqueued_)return true;
    const auto*b=pending_.pending_bytes();if(!b)return fail(LocalExchangeFault::Cycle);
    const auto result=outbox_.publish_gp(*b,this);
    if(result==LocalWirePublish::Unavailable)return fail(LocalExchangeFault::Outbox);
    if(result==LocalWirePublish::Published){gp_enqueued_=true;++d_.gp_requests_enqueued;}
    return true;
}
bool CanonicalLocalExchangeCore::receive(bool disarmed_observation)noexcept{
    // The generated uORB queue holds one 16-fragment prearm window burst.
    // Bounded work per poll, no reset or extra subscription to silently
    // drain/skip traffic. Queue capacity is not an age/deadline extension.
    for(unsigned i=0;i<gpenmpc_full_inner_ingress_s::ORB_QUEUE_LENGTH;++i){
        if(!subscription_.update(&topic_))break;
        arrival_=gpenmpc_argument_transport::from_topic(topic_,subscription_.get_last_generation());
        ++d_.ingress_updates;
        if(!ingress_.receive(arrival_,hrt_absolute_time(),disarmed_observation))return fail(LocalExchangeFault::Ingress);
        if(!retire_unanswered_gp())return false;
        // Install a complete component input against its retained exported source
        // before receiving another message. Validate source, identity, age and command.
        if(cycle_.runtime_state_only()&&task_.observe_source){
            const auto*s=io_.retained_latest_snapshot();
            if(s){
                selected_receipt_={};
                if(task_.observe_source(task_.owner,*s,selected_receipt_,false)!=LocalTaskRead::Ready)
                    return fail(LocalExchangeFault::TaskInput);
            }
        }
        if(ingress_.ready()){
            if(!gp_enqueued_||!pending_.pending()||!ingress_.take_gp_reply(completed_,hrt_absolute_time()))
                return fail(LocalExchangeFault::Ingress);
            if(!pending_.accept_reply(completed_.bytes,completed_.last_original_arrival_us,hrt_absolute_time()))
                return fail(LocalExchangeFault::Cycle);
            gp_enqueued_=false;++d_.gp_replies_filled;
        }
    }
    if(!ingress_.tick(hrt_absolute_time(),disarmed_observation))return fail(LocalExchangeFault::Ingress);
    return retire_unanswered_gp();
}
bool CanonicalLocalExchangeCore::retire_unanswered_gp()noexcept{
    const auto generation=ingress_.take_unanswered_generation();if(!generation)return true;
    if(!pending_.retire_unanswered(generation,hrt_absolute_time()))return fail(LocalExchangeFault::Cycle);
    gp_enqueued_=false;return true;
}
LocalExchangePoll CanonicalLocalExchangeCore::poll(bool disarmed)noexcept{
    using R=LocalExchangePoll;if(d_.first_fault!=LocalExchangeFault::None)return R::Fault;
    ++d_.polls;const auto now=hrt_absolute_time();
    if(!now||now<d_.last_processing_us){fail(LocalExchangeFault::Clock);return R::Fault;}d_.last_processing_us=now;
    if(outbox_.closed()||outbox_.fault()!=LocalWireFault::None){fail(LocalExchangeFault::Outbox);return R::Fault;}
    // Same outbox/stream; prioritize the actual numerical dependency over
    // its 13-fragment diagnostic observation. Neither message is dropped,
    // retimestamped or considered delivered by being copied to the outbox.
    // Runtime ingress precedes read-only history. A real GP request remains
    // in flight after its numerical deadline, without holding the fast loop.
    if(cycle_.component_initialization()||cycle_.runtime_state_only()){
        if(!receive(disarmed)||!enqueue_gp())return R::Fault;
    }else if(!enqueue_gp()||!enqueue_committed()||!receive(disarmed))return R::Fault;
    if(cycle_.runtime_state_only()&&!pending_.service_control_deadline(now))return R::Fault;
    if(pending_.numerical_pending())return R::Idle;
    if(!cycle_.component_initialization()&&!cycle_.runtime_state_only()&&(committed_pending_||(export_committed_state_&&outbox_.pending())))return R::Idle;
    // The 1-ms poll services ingress; control follows the nominal 100-Hz cadence.
    // Read the latest uORB state when due and execute once per retained state.
    // Input/window work and source-age and elapsed-dt checks remain active.
    if(!disarmed&&!capture_pending_&&(cycle_.component_initialization()||cycle_.runtime_state_only())&&d_.commits&&
       now-last_control_start_us_<gpenmpc_consumption::canonical_dt_us)return R::Idle;
    if(!cycle_.runtime_state_only()&&(selected_.drain()==gpenmpc_selected_source::Result::Invalid||
       selected_.diagnostics().latched_result==gpenmpc_selected_source::Result::Unsupported)){fail(LocalExchangeFault::SelectedSource);return R::Fault;}
    // Replace unexecuted runtime captures while awaiting HOST input. Reference
    // or window retries stay bound to their starting sample; Io validates
    // commit-to-sample dt and held-input ages.
    const bool refresh_unexecuted=capture_pending_&&!disarmed&&(cycle_.component_initialization()||cycle_.runtime_state_only())&&
        !io_.diagnostics().awaiting_window;
    if(!capture_pending_||refresh_unexecuted){
        if(!disarmed&&!task_.owner){++d_.missing_task_inputs;return R::Idle;}
        if(task_.owner){
            task_view_={};const auto available=task_.window(task_.owner,task_view_);
            if(available==LocalTaskRead::Missing){
                // A partial future window cannot suspend the already loaded
                // valid reference. A true query miss retains its source below.
                if(!loaded_window_generation_&&!disarmed){++d_.missing_task_inputs;return R::Idle;}
            }else if(available!=LocalTaskRead::Ready||!load_task_window(task_view_)){
                fail(LocalExchangeFault::Window);return R::Fault;}
        }
        const auto c=disarmed?cycle_.capture_disarmed():cycle_.capture();
        // A yaw reset with its native attitude companion still in flight
        // invalidates the old unexecuted capture; never execute it meanwhile.
        if(c==LocalCapture::NoUpdate){if(!refresh_unexecuted||!cycle_.snapshot())return R::Idle;}
        else{
            if(c!=LocalCapture::Accepted){fail(LocalExchangeFault::Cycle);return R::Fault;}
            capture_pending_=true;capture_disarmed_=disarmed;snapshot_enqueued_=false;
        }
    }
    if(capture_disarmed_!=disarmed){fail(LocalExchangeFault::Cycle);return R::Fault;}
    const auto*s=cycle_.snapshot();const auto*e=cycle_.original_endpoint();
    if(!s||!e){fail(LocalExchangeFault::Cycle);return R::Fault;}
    selected_receipt_={};
    const auto match=cycle_.runtime_state_only()?gpenmpc_selected_source::Result::Matched:selected_.lookup(*s,selected_receipt_);
    if(match==gpenmpc_selected_source::Result::Missing){++d_.missing_selected_endpoints;return R::Idle;}
    if(match!=gpenmpc_selected_source::Result::Matched||(!cycle_.runtime_state_only()&&!selected_receipt_.unique_selected_endpoint_observed)){fail(LocalExchangeFault::SelectedSource);return R::Fault;}
    if(task_.observe_source&&task_.observe_source(task_.owner,*s,selected_receipt_,false)!=LocalTaskRead::Ready){
        fail(LocalExchangeFault::TaskInput);return R::Fault;}
    // Disarmed RLS uses a 20-ms observation cadence. Active full-method sources
    // remain exact-source bound. RC export waits for the previous input return,
    // while capture and control proceed independently. Retry a retired unsent
    // input with a fresh snapshot after two 20-ms export slots.
    const bool rc_waiting_input=!disarmed&&cycle_.runtime_state_only()&&cycle_.component_initialization()&&
        last_snapshot_sample_us_&&last_component_input_sample_us_<last_snapshot_sample_us_;
    const bool export_now=(!disarmed&&!cycle_.runtime_state_only()&&!cycle_.component_initialization())||
        (!outbox_.numerical_pending()&&(!last_snapshot_export_us_||
         now-last_snapshot_export_us_>=(rc_waiting_input?40000ULL:20000ULL)));
    if((disarmed||export_active_snapshots_)&&!snapshot_enqueued_&&export_now){
        // Wire remains endpoint-only. This Context separately retains actual
        // selected-endpoint matching; neither is full integration history.
        if(outbox_.numerical_pending())return R::Idle;
        const bool encoded=cycle_.runtime_state_only()?gpenmpc_local_snapshot_wire::encode_runtime_state(*s,snapshot_bytes_):
            gpenmpc_local_snapshot_wire::encode_from_actual(*s,*e,snapshot_bytes_);
        if(!encoded){fail(LocalExchangeFault::Cycle);return R::Fault;}
        const auto sent=outbox_.publish_snapshot(snapshot_bytes_,this);
        if(sent==LocalWirePublish::Busy)return R::Idle;
        if(sent!=LocalWirePublish::Published){fail(LocalExchangeFault::Outbox);return R::Fault;}
        ++d_.snapshots_enqueued;snapshot_enqueued_=true;last_snapshot_export_us_=now;
        last_snapshot_sample_us_=s->estimator().timestamp_sample_us;
        if(task_.observe_source&&task_.observe_source(task_.owner,*s,selected_receipt_,true)!=LocalTaskRead::Ready){
            fail(LocalExchangeFault::TaskInput);return R::Fault;}
    }
    if(disarmed){
        const auto original=s->estimator().timestamp_sample_us;
        if(!cycle_.release_disarmed()){fail(LocalExchangeFault::Cycle);return R::Fault;}
        if(!cycle_.runtime_state_only()&&!selected_.retire_through(original)){fail(LocalExchangeFault::SelectedSource);return R::Fault;}
        capture_pending_=false;return R::Progress;
    }
    if(!task_.owner||!task_.read){++d_.missing_task_inputs;return R::Idle;}
    task_view_={};const auto read=task_.read(task_.owner,*s,selected_receipt_,task_view_);
    if(read==LocalTaskRead::Missing){++d_.missing_task_inputs;return R::Idle;}
    if(read!=LocalTaskRead::Ready){fail(LocalExchangeFault::TaskInput);return R::Fault;}
    const auto requested=task_view_.inputs.loaded_window_generation;
    if(requested!=loaded_window_generation_){
        if(requested<=loaded_window_generation_||!task_view_.window||
           task_view_.window->window_generation!=requested||!load_task_window(task_view_)){
            fail(LocalExchangeFault::Window);return R::Fault;}
    }
    if(!loaded_window_generation_){fail(LocalExchangeFault::Window);return R::Fault;}
    if(export_active_snapshots_&&!cycle_.component_initialization()&&!snapshot_enqueued_&&!cycle_.runtime_state_only()){
        // Retain the fast source for the subsequent RLC.
        if(outbox_.numerical_pending())return R::Idle;
        const bool encoded=cycle_.runtime_state_only()?gpenmpc_local_snapshot_wire::encode_runtime_state(*s,snapshot_bytes_):
            gpenmpc_local_snapshot_wire::encode_from_actual(*s,*e,snapshot_bytes_);
        if(!encoded||outbox_.publish_snapshot(snapshot_bytes_,this)!=LocalWirePublish::Published){fail(LocalExchangeFault::Outbox);return R::Fault;}
        snapshot_enqueued_=true;last_snapshot_export_us_=now;++d_.snapshots_enqueued;
    }
    const auto original=s->estimator().timestamp_sample_us;
    // Anchor the control schedule to execution after input and encoding work.
    // This prevents a catch-up burst before actuator-publication consumption.
    // Reference retries retain their execution timestamp and deadline.
    if(!io_.diagnostics().awaiting_window)last_control_start_us_=hrt_absolute_time();
    const auto step=cycle_.execute(task_view_.inputs);
    if(step==LocalCycleExecute::NeedReferenceWindow)return R::Idle; // source/outer/age retained by actual Cycle
    if(step!=LocalCycleExecute::Committed){fail(LocalExchangeFault::Cycle);return R::Fault;}
    capture_pending_=false;++d_.commits;
    if(cycle_.component_initialization()||cycle_.runtime_state_only())
        last_component_input_sample_us_=task_view_.inputs.rotor->source.sample_us;
    if(!cycle_.runtime_state_only()&&!selected_.retire_through(original)){fail(LocalExchangeFault::SelectedSource);return R::Fault;}
    const auto observation_interval=cycle_.component_initialization()?500000ULL:200000ULL;
    if(export_committed_state_&&(!cycle_.runtime_state_only()||
       (!committed_pending_&&(cycle_.component_initialization()||snapshot_enqueued_)&&
        (!last_committed_export_us_||now-last_committed_export_us_>=observation_interval)))){
        // Component history is exported at 2 Hz and full-method history at 5 Hz.
        // GP requests follow commits; the outer calculation retains its 0.30-s clock.
        // When RLS or GP owns the outbox, omit this RLC and report the cumulative
        // commit count in the next paired observation.
        if(!io_.copy_local_numerical_state(committed_numerical_)||
           !gpenmpc_local_committed_wire::from_actual(committed_numerical_,cycle_.committed_phase_observation(),committed_observation_)||
           !gpenmpc_local_committed_wire::encode(committed_observation_,committed_bytes_)){
            fail(LocalExchangeFault::Cycle);return R::Fault;}
        committed_pending_=true;last_committed_export_us_=now;
    }
    const auto next=pending_.observe_actual_commit();
    if(next==LocalGpBegin::Rejected||next==LocalGpBegin::Empty){fail(LocalExchangeFault::Cycle);return R::Fault;}
    if(next==LocalGpBegin::RequestReady){
        const auto*b=pending_.pending_bytes();
        if(!b||!ingress_.expect_gp_reply(*b,pending_.retained_actual_feedback().original_cycle_read_us,hrt_absolute_time())||!enqueue_gp()){
            fail(LocalExchangeFault::Ingress);return R::Fault;}
    }
    // The original GP query, if required, has already acquired the single
    // outbox. Complete closed evidence waits in the owner-resident buffer.
    // Runtime observations use the existing separate bounded history slot;
    // their drain is not permission for the next numerical control step.
    if(!enqueue_committed())return R::Fault;
    return R::Progress;
}
void CanonicalLocalExchangeCore::stop()noexcept{fail(LocalExchangeFault::Stopped);}
}
