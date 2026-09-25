#pragma once
// Host-testable scheduling prototype, separate from the PX4 application.
// Only local_source() invokes the numerical/output port. GP replies and
// reference-window arrivals update stored inputs. Ports default to absent;
// their implementation supplies canonical pre/post and reference transitions.
#include "../px4_full_inner/consumption/ConsumptionBinding.hpp"
#include <cstddef>
#include <cstdint>
#include <cmath>

namespace gpenmpc_local_schedule {
using gpenmpc_consumption::Identity;
using gpenmpc_consumption::Estimator;
using Hash=gpenmpc_portable::Array<std::uint32_t,8>;
template<std::size_t N>using Vector=gpenmpc_portable::Array<double,N>;
enum class Mode:std::uint8_t{Paused,Running,Faulted,Stopped};
enum class Fault:std::uint8_t{None,Configuration,Authority,LocalSource,Identity,Clock,
    SourceAge,Generation,Reset,Interval,MissingGp,GpIdentity,GpLate,GpEarlyClosure,
    Reference,ReferenceCapacity,Rotor,Execution,Commit,Stopped};
enum class Event:std::uint8_t{Idle,Stored,Duplicate,Committed,Rejected};
struct Configuration {
    Identity observed_session{};Hash task_sha{},configuration_sha{},gp_model_sha{};
    // Explicit caller limits only: the prototype provides NO live defaults.
    std::uint64_t source_max_age_us{},gp_reply_max_age_us{};
    std::uint64_t initial_global_leg{}; // frozen existing task ordinal, not always leg 1 after a new session
    double initial_dt_s{},maximum_reference_lookahead_s{};
    std::uint8_t original_reset_counter{};
};
// Bounded descriptors reference immutable trajectory storage owned by the reference port.
struct ReferenceWindow {
    Hash task_sha{},configuration_sha{},asset_sha{};
    std::uint64_t leg{},generation{},original_asset_token{};
    double first_phase_s{},last_phase_s{};
};
struct ReferenceCandidate {
    Vector<3> position_m{},velocity_mps{},acceleration_mps2{},jerk_mps3{};
    std::uint64_t leg{},window_generation{},original_source_generation{},candidate_token{};
    double phase_before_s{},phase_after_s{};
};
struct RotorLag {
    Vector<6> observed_thrust_n{};
    std::uint64_t dll_generation{},dll_session{},original_host_receive_ns{},original_board_ingress_us{};
    double original_sim_time_s{};
    // Hash of the retained original DLL observation AND independent-clock
    // association, to be checked by rotor_for(). Never inferred from outputs.
    Hash original_observation_sha{};
};
struct GpQuery {
    Vector<17> features_f17{};Hash request_sha{},model_sha{};
    Identity session{};std::uint64_t source_generation{},source_sample_us{},original_commit_us{},original_valid_until_us{};
    bool prediction_required{};
};
struct GpReply {
    Hash request_sha{},model_sha{};Identity session{};
    std::uint64_t source_generation{},source_sample_us{},original_commit_us{},original_valid_until_us{};
    // Original canonicalSparseGpFixedInput 18-double order: mean3,std3,
    // calibrated halfwidth3,latent variance3,distance,trust,hardInvalid,applied3.
    // NaNs/hard-invalid numerical results stay intact for canonical fallback;
    // missing transport and an explicit hard-invalid GP result are different.
    Vector<18> original_output18{};
    bool prediction_sample_closed{},observed_innovation_available{};
};
struct CommitResult {
    bool kernel_called{},output_published{},output_committed{};
    std::uint64_t original_commit_us{};
    bool prediction_required{};Vector<17> features_f17{};
};
class Ports {
public:
    virtual ~Ports()=default;
    virtual std::uint64_t actual_now_us()noexcept{return 0;}
    virtual bool authority(const Identity&,std::uint64_t)noexcept{return false;}
    virtual bool observed_disarmed(std::uint64_t)noexcept{return false;}
    // Must originate from the actual AtomicOdometryAdapter/Snapshot; not a
    // HOST RSP/RCT reconstructed estimator or expected-identity assertion.
    virtual bool local_source_verified(const Estimator&)noexcept{return false;}
    // Must bind original DLL lag6 + independent sim-clock observation to this
    // actual source. The current production provider has NO such board port.
    virtual bool rotor_for(const Estimator&,RotorLag&)noexcept{return false;}
    // Must execute original stepBoardReferenceAdapter/jerk-bounded transition
    // over its retained trajectory window; p/v/a/j and candidate phase all
    // belong to one transaction and are installed only on real output commit.
    virtual bool reference_for(const Estimator&,const ReferenceWindow&,double,bool,double,ReferenceCandidate&)noexcept{return false;}
    // A matching previous reply is an OPEN k prediction, closed by numerical
    // precontrol using this local k+1 observation, never by reply reception.
    // nullptr means the original numerical early-return skipped prediction.
    // This must own candidate rollback, actual publication, postcontrol and
    // atomic installation; the prototype provides no implementation/authority.
    virtual CommitResult local_step(const Estimator&,const RotorLag&,const ReferenceCandidate&,
                                    const GpReply*,double,bool)noexcept{return {};}
    virtual void revoke()noexcept{}
};
struct Diagnostics {
    std::uint64_t local_source_events{},duplicate_sources{},kernel_calls{},output_publications{},commits{};
    std::uint64_t gp_requests{},gp_replies{},prior_predictions_passed_to_next_source{},leg_resets{};
    std::uint64_t last_committed_source{},last_committed_sample_us{},first_fault_us{};
    Fault first_fault{Fault::None};
};

template<std::size_t WindowCapacity=2>class BoardLocalInnerSchedule final {
    static_assert(WindowCapacity>0,"Explicit bounded reference storage");
public:
    explicit BoardLocalInnerSchedule(Configuration c,Ports*ports=nullptr)noexcept:config_(c),ports_(ports){
        if(!c.observed_session.uid||!c.observed_session.boot_generation||!c.observed_session.system||!c.observed_session.component||
           empty(c.task_sha)||empty(c.configuration_sha)||empty(c.gp_model_sha)||!c.source_max_age_us||!c.gp_reply_max_age_us||!c.initial_global_leg||
           !std::isfinite(c.initial_dt_s)||c.initial_dt_s<=0||c.initial_dt_s>0.0100001||
           !std::isfinite(c.maximum_reference_lookahead_s)||c.maximum_reference_lookahead_s<=0)fail(Fault::Configuration,0);
    }
    bool begin_leg(std::uint64_t leg,std::uint64_t now)noexcept{
        if(mode_!=Mode::Paused||!event_time(now))return false;
        if(!authorized(now)||!ports_->observed_disarmed(now))return fail(Fault::Authority,now);
        if(!leg||leg_==UINT64_MAX||leg!=(leg_?leg_+1:config_.initial_global_leg))return fail(Fault::Reference,now);
        // Explicit disarmed sequential-leg reset only. Does not reset global
        // uORB watermark/boot/reset identity or recover a first fault.
        leg_=leg;phase_s_=0;new_leg_=true;window_count_=0;clear_gp();++diagnostics_.leg_resets;return true;
    }
    bool add_reference_window(const ReferenceWindow&w,std::uint64_t now)noexcept{
        if(terminal()||!event_time(now))return false;
        if(!authorized(now))return fail(Fault::Authority,now);
        if(w.task_sha!=config_.task_sha||w.configuration_sha!=config_.configuration_sha||empty(w.asset_sha)||
           w.leg!=leg_||!w.generation||w.generation<=last_window_generation_||!w.original_asset_token||!std::isfinite(w.first_phase_s)||!std::isfinite(w.last_phase_s)||
           w.first_phase_s<0||w.last_phase_s<=w.first_phase_s||w.last_phase_s-phase_s_>config_.maximum_reference_lookahead_s||
           (window_count_&&(w.generation<=windows_[window_count_-1].generation||
                            !same_phase(w.first_phase_s,windows_[window_count_-1].last_phase_s))))return fail(Fault::Reference,now);
        if(window_count_==WindowCapacity)return fail(Fault::ReferenceCapacity,now);
        windows_[window_count_++]=w;last_window_generation_=w.generation;return true;
    }
    bool resume(std::uint64_t now)noexcept{
        if(mode_!=Mode::Paused||!event_time(now))return false;
        if(!authorized(now))return fail(Fault::Authority,now);
        if(!leg_||!current_window())return fail(Fault::Reference,now);
        mode_=Mode::Running;return true;
    }
    bool pause(std::uint64_t now)noexcept{
        if(mode_!=Mode::Running||!event_time(now))return false;
        mode_=Mode::Paused;return true; // Retain phase and pending work.
    }
    Event local_source(const Estimator&s,std::uint64_t now)noexcept{
        ++diagnostics_.local_source_events;
        if(terminal())return Event::Rejected;
        if(!event_time(now))return Event::Rejected;
        if(mode_!=Mode::Running)return Event::Idle;
        if(!authorized(now))return reject(Fault::Authority,now);
        if(!ports_->local_source_verified(s))return reject(Fault::LocalSource,now);
        if(s.pose_frame!=1||s.velocity_frame!=1||!finite(s.p)||!finite(s.v)||!finite(s.q)||!finite(s.body_rates))return reject(Fault::LocalSource,now);
        if(!(s.identity==config_.observed_session))return reject(Fault::Identity,now);
        if(s.reset_counter!=config_.original_reset_counter)return reject(Fault::Reset,now);
        const Hash source_hash=source_digest(s);
        if(have_source_&&s.generation==last_source_.generation&&source_hash==last_source_sha_){++diagnostics_.duplicate_sources;return Event::Duplicate;}
        if(!s.generation||(have_source_&&s.generation<=last_source_.generation))return reject(Fault::Generation,now);
        if(!s.timestamp_sample_us||!s.publication_us||!s.board_rx_us||s.timestamp_sample_us>s.publication_us||
           s.publication_us>s.board_rx_us||s.board_rx_us>now)return reject(Fault::Clock,now);
        if(now-s.timestamp_sample_us>config_.source_max_age_us)return reject(Fault::SourceAge,now);
        double dt=config_.initial_dt_s;
        if(have_source_&&!new_leg_){
            if(s.timestamp_sample_us<=last_source_.timestamp_sample_us||s.board_rx_us<=last_source_.board_rx_us||
               s.timestamp_sample_us-last_source_.timestamp_sample_us>gpenmpc_consumption::canonical_dt_us)return reject(Fault::Interval,now);
            dt=double(s.timestamp_sample_us-last_source_.timestamp_sample_us)*1e-6;
        }
        if(have_source_&&s.timestamp_sample_us<=last_source_.timestamp_sample_us)return reject(Fault::Interval,now);
        // No wait for a HOST inner command. Required k reply must already be
        // present when k+1 local source arrives; holding it .3s is forbidden.
        if(query_.prediction_required&&(!reply_ready_||now>query_.original_valid_until_us))return reject(Fault::MissingGp,now);
        auto*w=current_window();if(!w)return reject(Fault::Reference,now);
        RotorLag rotor{};
        if(!ports_->rotor_for(s,rotor)||!rotor.dll_generation||!rotor.dll_session||!rotor.original_host_receive_ns||
           !rotor.original_board_ingress_us||rotor.original_board_ingress_us>now||empty(rotor.original_observation_sha)||
           !std::isfinite(rotor.original_sim_time_s)||rotor.original_sim_time_s<0||!finite(rotor.observed_thrust_n))return reject(Fault::Rotor,now);
        ReferenceCandidate reference{};
        if(!ports_->reference_for(s,*w,phase_s_,new_leg_,dt,reference)||reference.leg!=leg_||
           reference.window_generation!=w->generation||reference.original_source_generation!=s.generation||!reference.candidate_token||
           !same_phase(reference.phase_before_s,phase_s_)||!std::isfinite(reference.phase_after_s)||reference.phase_after_s<phase_s_||
           reference.phase_after_s>w->last_phase_s||!finite(reference.position_m)||!finite(reference.velocity_mps)||
           !finite(reference.acceleration_mps2)||!finite(reference.jerk_mps3))return reject(Fault::Reference,now);
        const auto before=ports_->actual_now_us();
        if(before<now||before-s.timestamp_sample_us>config_.source_max_age_us||!authorized(before))return reject(Fault::SourceAge,now);
        const bool closes=query_.prediction_required;
        const auto result=ports_->local_step(s,rotor,reference,closes?&reply_:nullptr,dt,new_leg_);
        diagnostics_.kernel_calls+=result.kernel_called;diagnostics_.output_publications+=result.output_published;
        diagnostics_.commits+=result.output_committed; // preserve actually reported actions on a later fault
        const auto after=ports_->actual_now_us();
        if(after<before)return reject(Fault::Clock,after);
        last_event_us_=after;
        if(!result.kernel_called||!result.output_published||!result.output_committed)return reject(Fault::Execution,after);
        if(!result.original_commit_us||result.original_commit_us<before||result.original_commit_us>after)return reject(Fault::Commit,after);
        diagnostics_.last_committed_source=s.generation;diagnostics_.last_committed_sample_us=s.timestamp_sample_us;
        if(after<before||after-s.timestamp_sample_us>config_.source_max_age_us||!authorized(after))return reject(Fault::SourceAge,after);
        if(result.prediction_required&&!finite(result.features_f17))return reject(Fault::Execution,after);
        if(closes)++diagnostics_.prior_predictions_passed_to_next_source;
        clear_gp();query_.prediction_required=result.prediction_required;
        if(result.prediction_required){
            if(s.timestamp_sample_us>UINT64_MAX-config_.gp_reply_max_age_us)return reject(Fault::Clock,after);
            query_.session=config_.observed_session;query_.source_generation=s.generation;query_.source_sample_us=s.timestamp_sample_us;
            query_.original_commit_us=result.original_commit_us;query_.original_valid_until_us=s.timestamp_sample_us+config_.gp_reply_max_age_us;
            query_.features_f17=result.features_f17;query_.model_sha=config_.gp_model_sha;query_.request_sha=query_digest(query_);
            if(after>query_.original_valid_until_us)return reject(Fault::GpLate,after);
        }
        phase_s_=reference.phase_after_s;last_source_=s;last_source_sha_=source_hash;have_source_=true;new_leg_=false;
        retire_windows();return Event::Committed;
    }
    bool take_gp_request(GpQuery&out,std::uint64_t now)noexcept{
        if(terminal()||!query_.prediction_required||query_sent_)return false;
        if(!event_time(now))return false;
        if(now>query_.original_valid_until_us)return fail(Fault::GpLate,now);
        out=query_;query_sent_=true;++diagnostics_.gp_requests;return true;
    }
    Event gp_reply(const GpReply&r,std::uint64_t original_ingress_us,std::uint64_t now)noexcept{
        if(terminal()||!event_time(now))return Event::Rejected;
        if(!query_.prediction_required||!query_sent_||!(r.session==query_.session)||r.model_sha!=query_.model_sha||
           r.request_sha!=query_.request_sha||r.source_generation!=query_.source_generation||r.source_sample_us!=query_.source_sample_us||
           r.original_commit_us!=query_.original_commit_us||r.original_valid_until_us!=query_.original_valid_until_us)return reject(Fault::GpIdentity,now);
        if(r.prediction_sample_closed||r.observed_innovation_available)return reject(Fault::GpEarlyClosure,now);
        if(original_ingress_us<query_.original_commit_us||original_ingress_us>now||now>query_.original_valid_until_us)return reject(Fault::GpLate,now);
        if(reply_ready_){
            if(reply_digest(r)==reply_digest(reply_))return Event::Duplicate;
            return reject(Fault::GpIdentity,now);
        }
        reply_=r;reply_ready_=true;reply_original_ingress_us_=original_ingress_us;++diagnostics_.gp_replies;
        return Event::Stored; // never closes innovation or calls local_step
    }
    bool tick(std::uint64_t now)noexcept{
        if(terminal()||!event_time(now))return false;
        if(query_.prediction_required&&now>query_.original_valid_until_us)return fail(Fault::GpLate,now);
        return true; // no reference / kernel / heartbeat credit on polling
    }
    void stop()noexcept{if(!terminal())mode_=Mode::Stopped;clear_pending();if(ports_)ports_->revoke();}
    Mode mode()const noexcept{return mode_;}const Diagnostics&diagnostics()const noexcept{return diagnostics_;}
    double phase_s()const noexcept{return phase_s_;}std::size_t reference_windows()const noexcept{return window_count_;}
    bool gp_pending()const noexcept{return query_.prediction_required;}bool reply_ready()const noexcept{return reply_ready_;}
    std::uint64_t reply_original_ingress_us()const noexcept{return reply_original_ingress_us_;}
private:
    // Exact equality with NaN rejection and signed-zero equality.
    static bool same_phase(double a,double b)noexcept{return a<=b&&a>=b;}
    static bool empty(const Hash&h)noexcept{for(auto v:h)if(v)return false;return true;}
    template<std::size_t N>static bool finite(const Vector<N>&v)noexcept{for(auto x:v)if(!std::isfinite(x))return false;return true;}
    static void identity_hash(gpenmpc_consumption::CanonicalSha256&h,const Identity&i)noexcept{h.u64(i.uid);h.u64(i.boot_generation);h.byte(i.system);h.byte(i.component);}
    static Hash source_digest(const Estimator&s)noexcept{gpenmpc_consumption::CanonicalSha256 h;identity_hash(h,s.identity);h.u64(s.generation);h.u64(s.timestamp_sample_us);h.u64(s.publication_us);h.u64(s.board_rx_us);h.byte(s.pose_frame);h.byte(s.velocity_frame);h.byte(s.reset_counter);h.reals(s.p);h.reals(s.v);h.reals(s.q);h.reals(s.body_rates);return h.finish();}
    static Hash query_digest(const GpQuery&q)noexcept{gpenmpc_consumption::CanonicalSha256 h;h.u32(0x51475031);identity_hash(h,q.session);h.u64(q.source_generation);h.u64(q.source_sample_us);h.u64(q.original_commit_us);h.u64(q.original_valid_until_us);h.words(q.model_sha);h.reals(q.features_f17);return h.finish();}
    static Hash reply_digest(const GpReply&r)noexcept{gpenmpc_consumption::CanonicalSha256 h;h.words(r.request_sha);h.reals(r.original_output18);return h.finish();}
    bool terminal()const noexcept{return mode_==Mode::Stopped||mode_==Mode::Faulted;}
    bool authorized(std::uint64_t now)noexcept{return ports_&&ports_->authority(config_.observed_session,now);}
    bool event_time(std::uint64_t now)noexcept{if(!now||now<last_event_us_)return fail(Fault::Clock,now);last_event_us_=now;return true;}
    ReferenceWindow*current_window()noexcept{retire_windows();return window_count_&&phase_s_>=windows_[0].first_phase_s&&phase_s_<windows_[0].last_phase_s?&windows_[0]:nullptr;}
    void retire_windows()noexcept{while(window_count_&&phase_s_>=windows_[0].last_phase_s){for(std::size_t i=1;i<window_count_;++i)windows_[i-1]=windows_[i];windows_[--window_count_]={};}}
    void clear_gp()noexcept{query_={};reply_={};reply_ready_=query_sent_=false;reply_original_ingress_us_=0;}
    void clear_pending()noexcept{clear_gp();for(auto&w:windows_)w={};window_count_=0;}
    bool fail(Fault reason,std::uint64_t now)noexcept{if(diagnostics_.first_fault==Fault::None){diagnostics_.first_fault=reason;diagnostics_.first_fault_us=now;}mode_=Mode::Faulted;clear_pending();if(ports_)ports_->revoke();return false;}
    Event reject(Fault reason,std::uint64_t now)noexcept{fail(reason,now);return Event::Rejected;}
    const Configuration config_;Ports*const ports_;Mode mode_{Mode::Paused};Diagnostics diagnostics_{};
    ReferenceWindow windows_[WindowCapacity]{};std::size_t window_count_{};
    Estimator last_source_{};Hash last_source_sha_{};GpQuery query_{};GpReply reply_{};
    std::uint64_t last_event_us_{},leg_{},reply_original_ingress_us_{},last_window_generation_{};double phase_s_{};
    bool have_source_{},new_leg_{},query_sent_{},reply_ready_{};
};
} // namespace gpenmpc_local_schedule
