// Explicit MOCK ports only: no generated kernel, uORB, HRT, plant or network.
// Tests scheduling eligibility and causality, never production flight facts.
#include "BoardLocalInnerSchedule.hpp"
#include <cstdio>
#include <cstdlib>
#include <limits>
using namespace gpenmpc_local_schedule;
namespace {
unsigned checks=0;
void check(bool ok,const char*name){++checks;if(!ok){std::fprintf(stderr,"FAIL %u %s\n",checks,name);std::exit(1);}}
Hash hash(std::uint32_t value){Hash h{};h[0]=value;return h;}
Configuration config(){Configuration c{};c.observed_session={71,9,1,1};c.task_sha=hash(11);c.configuration_sha=hash(12);c.gp_model_sha=hash(13);
    c.source_max_age_us=20000;c.gp_reply_max_age_us=20000;c.initial_global_leg=1;c.initial_dt_s=.01;c.maximum_reference_lookahead_s=1;return c;}
ReferenceWindow window(double first=0,double last=1,std::uint64_t generation=1,std::uint64_t leg=1){
    ReferenceWindow w{};w.task_sha=hash(11);w.configuration_sha=hash(12);w.asset_sha=hash(14);w.leg=leg;w.generation=generation;
    w.original_asset_token=17;w.first_phase_s=first;w.last_phase_s=last;return w;
}
Estimator source(std::uint64_t generation,std::uint64_t sample){Estimator s{};s.identity=config().observed_session;s.generation=generation;
    s.timestamp_sample_us=sample;s.publication_us=sample+1;s.board_rx_us=sample+2;s.pose_frame=s.velocity_frame=1;s.q[0]=1;return s;}
struct MockPorts final:Ports {
    std::uint64_t now=1000,steps=0,closes=0,revokes=0,last_reply_source=0;
    bool has_authority=true,disarmed=true,local=true,rotor=true,reference=true;
    bool bad_jerk=false,bad_phase=false,publish=true,commit=true,expire_on_return=false;
    bool last_new_leg=false;double last_dt=0,last_phase=0;Vector<6> last_rotor{};
    std::uint64_t actual_now_us()noexcept override{return now;}
    bool authority(const Identity&i,std::uint64_t)noexcept override{return has_authority&&i==config().observed_session;}
    bool observed_disarmed(std::uint64_t)noexcept override{return disarmed;}
    bool local_source_verified(const Estimator&)noexcept override{return local;}
    bool rotor_for(const Estimator&s,RotorLag&r)noexcept override{
        if(!rotor)return false;
        r.dll_generation=s.generation;r.dll_session=4;r.original_host_receive_ns=UINT64_C(9007199254741111);
        r.original_board_ingress_us=s.board_rx_us;r.original_sim_time_s=double(s.timestamp_sample_us)*1e-6;
        r.original_observation_sha=hash(23);for(unsigned k=0;k<6;++k)r.observed_thrust_n[k]=20+double(k);return true;
    }
    bool reference_for(const Estimator&s,const ReferenceWindow&w,double phase,bool new_leg,double dt,ReferenceCandidate&r)noexcept override{
        if(!reference)return false;
        r.leg=w.leg;r.window_generation=w.generation;r.original_source_generation=s.generation;r.candidate_token=s.generation;
        r.phase_before_s=phase;r.phase_after_s=phase+dt;r.position_m[0]=phase;r.velocity_mps[0]=1;
        r.acceleration_mps2[1]=.25;r.jerk_mps3[2]=.125; // fixture values, not canonical interpolation
        if(bad_jerk)r.jerk_mps3[0]=std::numeric_limits<double>::quiet_NaN();
        if(bad_phase)r.phase_after_s=w.last_phase_s+1;
        last_new_leg=new_leg;return true;
    }
    CommitResult local_step(const Estimator&s,const RotorLag&r,const ReferenceCandidate&reference,
                           const GpReply*previous,double dt,bool new_leg)noexcept override{
        ++steps;last_dt=dt;last_phase=reference.phase_before_s;last_rotor=r.observed_thrust_n;
        if(previous){++closes;last_reply_source=previous->source_generation;
            check(!previous->prediction_sample_closed&&!previous->observed_innovation_available,"reply reaches precontrol still open");
            check(previous->source_sample_us<s.timestamp_sample_us,"only prior sample prediction may close");}
        CommitResult out{};out.kernel_called=true;out.output_published=publish;out.output_committed=commit;
        out.original_commit_us=++now;out.prediction_required=!new_leg;
        for(unsigned i=0;i<17;++i)out.features_f17[i]=double(s.generation)+double(i)*.25;
        if(expire_on_return)now+=config().source_max_age_us;
        return out;
    }
    void revoke()noexcept override{++revokes;}
};
using Schedule=BoardLocalInnerSchedule<2>;
void setup(Schedule&s,MockPorts&p){check(s.begin_leg(1,p.now),"explicit disarmed leg reset");check(s.add_reference_window(window(),p.now),"original window descriptor retained");check(s.resume(p.now),"resume without arming");}
Event event(Schedule&s,MockPorts&p,std::uint64_t generation,std::uint64_t stamp){p.now=stamp+3;return s.local_source(source(generation,stamp),p.now);}
GpReply reply(const GpQuery&q){GpReply r{};r.request_sha=q.request_sha;r.model_sha=q.model_sha;r.session=q.session;r.source_generation=q.source_generation;
    r.source_sample_us=q.source_sample_us;r.original_commit_us=q.original_commit_us;r.original_valid_until_us=q.original_valid_until_us;
    for(unsigned k=0;k<18;++k)r.original_output18[k]=double(k)*.1;return r;}
GpQuery first_query(Schedule&s,MockPorts&p){setup(s,p);check(event(s,p,10,10000)==Event::Committed,"first local sample without GP");
    check(event(s,p,13,20000)==Event::Committed,"raw generation gap not synthesized");GpQuery q{};check(s.take_gp_request(q,p.now),"k query after commit");return q;}
}
int main(){
    {Schedule absent(config());check(!absent.begin_leg(1,1000),"no authority no leg/execution");check(absent.diagnostics().kernel_calls==0,"absent ports zero execution");}
    {Ports unavailable;Schedule absent(config(),&unavailable);check(!absent.begin_leg(1,1000),"default methods grant nothing");}
    {MockPorts p;auto c=config();c.initial_global_leg=3;Schedule s(c,&p);check(s.begin_leg(3,1000),"new session retains original global leg three");
        check(s.add_reference_window(window(0,1,1,3),1000)&&s.resume(1000),"global leg three own original reference window");
        check(event(s,p,1,10000)==Event::Committed&&p.last_new_leg,"initial global leg ordinal does not create earlier control history");}
    {MockPorts p;Schedule s(config(),&p);p.disarmed=false;check(!s.begin_leg(1,1000),"leg reset needs actual disarmed port");}
    // Sixty source-driven scheduling events.
    {MockPorts p;Schedule s(config(),&p);setup(s,p);
        for(std::uint64_t k=1;k<=60;++k){const auto prior=p.steps;check(event(s,p,k*3,k*10000)==Event::Committed,"fresh local source triggers one step");
            check(p.steps==prior+1,"single local step per source");const auto committed=s.diagnostics().commits;
            GpQuery q{};if(k==1){check(!s.take_gp_request(q,p.now),"first canonical early return no GP query");}
            else {check(s.take_gp_request(q,p.now),"one query every required inner commit");check(!s.take_gp_request(q,p.now),"query taken exactly once");
                check(q.source_generation==k*3&&q.original_valid_until_us==k*10000+20000,"source/expiry not renewed");
                const auto g=reply(q);const auto first=++p.now;
                check(s.gp_reply(g,first,first)==Event::Stored,"original k GP response stored");
                check(s.diagnostics().commits==committed&&p.steps==prior+1,"GP receive never triggers step/commit");
                ++p.now;check(s.gp_reply(g,p.now,p.now)==Event::Duplicate,"same reply cannot refresh receipt");
                check(s.reply_original_ingress_us()==first,"duplicate retains original ingress");
            }
            const auto phase=s.phase_s();check(s.tick(++p.now),"poll no numerical tick");check(s.phase_s()==phase&&p.steps==prior+1,"poll cannot advance phase");
        }
        check(s.diagnostics().commits==60&&s.diagnostics().gp_requests==59&&s.diagnostics().gp_replies==59,"60/59/59 causal actions");
        check(p.closes==58&&s.diagnostics().prior_predictions_passed_to_next_source==58,"k+1 closes only 58 prior required predictions");
        check(p.last_rotor[0]==20&&p.last_rotor[5]==25,"observed lag6 distinct from controller outputs");
        s.stop();GpQuery q{};check(s.reference_windows()==0&&!s.gp_pending()&&!s.reply_ready()&&!s.take_gp_request(q,p.now),"stop clears all pending buffers");
        check(event(s,p,181,610000)==Event::Rejected&&!s.resume(p.now),"stop cannot restart or execute");
        check(s.diagnostics().commits==60,"stop preserves executed accounting");
    }
    {MockPorts p;Schedule s(config(),&p);setup(s,p);auto raw=source(1,10000);p.now=10003;check(s.local_source(raw,p.now)==Event::Committed,"first source");
        ++p.now;check(s.local_source(raw,p.now)==Event::Duplicate&&p.steps==1,"duplicate original source is no second execution");
        raw.p[0]=1;check(s.local_source(raw,++p.now)==Event::Rejected&&s.diagnostics().first_fault==Fault::Generation,"same source generation changed bits fault");}
    {MockPorts p;Schedule s(config(),&p);setup(s,p);event(s,p,9,10000);check(event(s,p,1,20000)==Event::Rejected,"generation reset forbidden");}
    {MockPorts p;Schedule s(config(),&p);setup(s,p);auto raw=source(1,10000);raw.reset_counter=1;p.now=10003;
        check(s.local_source(raw,p.now)==Event::Rejected&&p.steps==0,"EKF reset not a new-leg label");}
    {MockPorts p;Schedule s(config(),&p);setup(s,p);event(s,p,1,10000);check(event(s,p,2,20001)==Event::Rejected,"10001us source interval rejected");}
    {MockPorts p;Schedule s(config(),&p);setup(s,p);p.now=40001;check(s.local_source(source(1,10000),p.now)==Event::Rejected&&p.steps==0,"caller explicit source age");}
    {MockPorts p;Schedule s(config(),&p);setup(s,p);p.now=10001;check(s.local_source(source(1,10000),p.now)==Event::Rejected,"future original receipt rejected");}
    {MockPorts p;Schedule s(config(),&p);setup(s,p);p.local=false;check(event(s,p,1,10000)==Event::Rejected&&p.steps==0,"HOST reconstructed source cannot be verified local");}
    {MockPorts p;Schedule s(config(),&p);setup(s,p);p.has_authority=false;check(event(s,p,1,10000)==Event::Rejected&&p.steps==0,"revoked authority blocks source step");}
    {MockPorts p;Schedule s(config(),&p);setup(s,p);p.rotor=false;check(event(s,p,1,10000)==Event::Rejected&&p.steps==0,"no real rotor/source-clock binding no command substitution");}
    {MockPorts p;Schedule s(config(),&p);setup(s,p);p.reference=false;check(event(s,p,1,10000)==Event::Rejected&&p.steps==0,"no exact reference provider no interpolation substitute");}
    {MockPorts p;Schedule s(config(),&p);setup(s,p);p.bad_jerk=true;check(event(s,p,1,10000)==Event::Rejected&&p.steps==0,"missing finite jerk blocks execution");}
    {MockPorts p;Schedule s(config(),&p);setup(s,p);p.bad_phase=true;check(event(s,p,1,10000)==Event::Rejected&&p.steps==0,"cannot extrapolate beyond original window");}
    {MockPorts p;Schedule s(config(),&p);s.begin_leg(1,1000);check(s.add_reference_window(window(0,.1,1),1000),"first bounded window");
        check(s.add_reference_window(window(.1,.2,2),1000),"second bounded window");
        check(!s.add_reference_window(window(.2,.3,3),1000)&&s.diagnostics().first_fault==Fault::ReferenceCapacity,"no overwrite at window capacity");}
    {MockPorts p;Schedule s(config(),&p);s.begin_leg(1,1000);check(!s.add_reference_window(window(0,1.01),1000),"caller lookahead not exceeded");}
    {MockPorts p;Schedule s(config(),&p);setup(s,p);event(s,p,1,10000);const auto phase=s.phase_s();check(s.pause(++p.now),"explicit task pause");
        check(event(s,p,2,20000)==Event::Idle&&s.phase_s()==phase&&p.steps==1,"paused source does not advance reference/observer");
        check(!s.begin_leg(1,++p.now),"same leg cannot reset continuity");}
    {MockPorts p;Schedule s(config(),&p);setup(s,p);event(s,p,1,10000);s.pause(++p.now);check(s.begin_leg(2,++p.now),"sequential disarmed new leg");
        check(s.phase_s()==0&&s.reference_windows()==0,"new leg resets phase/window explicitly");s.add_reference_window(window(0,1,2,2),++p.now);s.resume(++p.now);
        check(event(s,p,2,90000)==Event::Committed&&p.last_new_leg,"fresh leg first source is explicit reset not fake historical ticks");
        check(s.diagnostics().commits==2,"no catch-up execution during ground gap");}
    {MockPorts p;Schedule s(config(),&p);first_query(s,p);check(event(s,p,14,30000)==Event::Rejected&&p.steps==2,"missing k GP reply fails closed at k+1");
        const auto fault=s.diagnostics().first_fault;GpReply r{};check(s.gp_reply(r,p.now,p.now)==Event::Rejected&&s.diagnostics().first_fault==fault,"late reply cannot wash first fault");}
    {MockPorts p;Schedule s(config(),&p);const auto q=first_query(s,p);const auto r=reply(q);p.now=q.original_valid_until_us+1;
        check(s.gp_reply(r,p.now,p.now)==Event::Rejected&&p.steps==2,"late GP explicit original deadline");}
    {MockPorts p;Schedule s(config(),&p);const auto q=first_query(s,p);p.now=q.original_valid_until_us+1;
        check(!s.tick(p.now)&&p.steps==2,"missing GP idle poll expires without a new source");}
    {MockPorts p;Schedule s(config(),&p);const auto q=first_query(s,p);auto r=reply(q);r.source_generation+=1;
        ++p.now;check(s.gp_reply(r,p.now,p.now)==Event::Rejected,"future k reply cannot match prior query");}
    {MockPorts p;Schedule s(config(),&p);const auto q=first_query(s,p);auto r=reply(q);r.model_sha[0]^=1;
        ++p.now;check(s.gp_reply(r,p.now,p.now)==Event::Rejected,"wrong GP model identity");}
    {MockPorts p;Schedule s(config(),&p);const auto q=first_query(s,p);auto r=reply(q);r.request_sha[0]^=1;
        ++p.now;check(s.gp_reply(r,p.now,p.now)==Event::Rejected,"wrong feature/request digest");}
    {MockPorts p;Schedule s(config(),&p);const auto q=first_query(s,p);auto r=reply(q);r.prediction_sample_closed=true;
        ++p.now;check(s.gp_reply(r,p.now,p.now)==Event::Rejected&&p.closes==0,"HOST must not close k using k measurement");}
    {MockPorts p;Schedule s(config(),&p);const auto q=first_query(s,p);auto r=reply(q);r.original_output18[0]=std::numeric_limits<double>::quiet_NaN();r.original_output18[14]=1;
        ++p.now;check(s.gp_reply(r,p.now,p.now)==Event::Stored,"explicit canonical hard invalid retained not missing transport");
        check(event(s,p,14,30000)==Event::Committed&&p.closes==1,"hard-invalid GP passed to exact math for original fallback");}
    {MockPorts p;Schedule s(config(),&p);setup(s,p);p.commit=false;check(event(s,p,1,10000)==Event::Rejected,"failed local commit fault");
        check(s.diagnostics().kernel_calls==1&&s.diagnostics().output_publications==1&&s.diagnostics().commits==0,"publication evidence not erased on commit failure");}
    {MockPorts p;Schedule s(config(),&p);setup(s,p);p.expire_on_return=true;check(event(s,p,1,10000)==Event::Rejected,"post-commit original source expiry fault");
        check(s.diagnostics().commits==1&&!s.gp_pending(),"late committed action retained but no GP/control continuation");}
    {MockPorts p;Schedule s(config(),&p);s.begin_leg(1,1000);s.add_reference_window(window(0,.01),1000);s.resume(1000);event(s,p,1,10000);
        check(s.reference_windows()==0,"consumed window retired");check(!s.add_reference_window(window(.01,.02,1),++p.now),"retirement cannot reset window generation watermark");}
    {MockPorts p;Schedule s(config(),&p);setup(s,p);event(s,p,1,10000);event(s,p,2,20000);GpQuery q{};p.now=40001;
        check(!s.take_gp_request(q,p.now)&&s.diagnostics().first_fault==Fault::GpLate,"query dequeue preserves original expiry");}
    std::printf("{\"status\":\"PASS_PURE_HOST_LOCAL_SOURCE_SCHEDULER_MOCK_PORTS\",\"checks\":%u,\"scheduler_bytes\":%zu,\"reference_window_capacity\":2,\"positive_local_source_steps\":60,\"gp_requests\":59,\"gp_replies\":59,\"k_plus_one_prediction_inputs\":58,\"test_source_max_age_us\":20000,\"test_gp_reply_max_age_us\":20000,\"test_reference_lookahead_s\":1,\"source_dt_upper_us_from_existing_core\":10000,\"actual_kernel_steps\":0,\"actual_uORB_or_HRT\":false,\"exact_reference_provider_implemented\":false,\"actual_rotor_source_time_binding\":false,\"GP_wire_implemented\":false,\"ARM_compile\":false,\"live_ready\":false}\n",checks,sizeof(Schedule));
}
