// Reuse fixture helpers and broker/HRT/authority shims from the cycle test.
// Its entry function is renamed so only this test's main is executed.
#define GPENMPC_CYCLE_TEST_MAIN retained_cycle_fixture_main
#include "test_local_execution_cycle.cpp"
#undef GPENMPC_CYCLE_TEST_MAIN
#include "px4_runtime/CanonicalLocalGpPending.hpp"
namespace gw=gpenmpc_local_gp_wire;
static unsigned gp_packets{},gp_queries{};
template<std::size_t N>static bool gp_roundtrip(const gpenmpc_portable::Array<uint8_t,N>&bytes,
    gpenmpc_portable::Array<uint8_t,N>&joined,unsigned injection_kind=0){
    joined={};unsigned offset=0;const bool request=N==gw::request_bytes;
    for(unsigned k=0;k<gw::fragment_count;++k){gw::Fragment f{};if(!gw::fragment(bytes,k,f))return false;
        mavlink_message_t sent{},got{},rx{};mavlink_status_t state{},reported{};
        uint8_t sys=request?1:255,comp=request?1:190,tsys=request?255:1,tcomp=request?190:1;
        if(injection_kind==1)sys=200;if(injection_kind==2){tsys=sys;tcomp=comp;}
        mavlink_msg_tunnel_pack(sys,comp,&sent,tsys,tcomp,injection_kind==3?200:42002,f.length,f.payload);
        uint8_t wire[MAVLINK_MAX_PACKET_LEN]{};const auto length=mavlink_msg_to_send_buffer(wire,&sent);unsigned accepted=0;
        for(unsigned j=0;j<length;++j)if(mavlink_frame_char_buffer(&rx,&state,wire[j],&got,&reported)==MAVLINK_FRAMING_OK)++accepted;
        if(accepted!=1||got.msgid!=MAVLINK_MSG_ID_TUNNEL)return false;
        mavlink_tunnel_t t{};mavlink_msg_tunnel_decode(&got,&t);++gp_packets;
        if(got.sysid!=(request?1:255)||got.compid!=(request?1:190)||t.target_system!=(request?255:1)||t.target_component!=(request?190:1)||
            t.payload_type!=42002||t.payload_length!=f.length||!same(t.payload,f.payload,f.length)||
            t.payload[0]!=static_cast<uint8_t>(((request?gw::request_schema:gw::reply_schema)<<4)|k))return false;
        const unsigned n=f.length-9;if(offset+n>N)return false;std::memcpy(joined.data()+offset,t.payload+9,n);offset+=n;
    }
    return offset==N&&same(bytes.data(),joined.data(),N);
}
template<std::size_t N>static void repair_checksum(gpenmpc_portable::Array<uint8_t,N>&bytes){
    gpenmpc_snapshot_wire::Writer w(bytes.data()+N-32);gw::write_hash(w,gw::digest(bytes.data(),N-32));
}
static bool actual_reply(li::CanonicalLocalGpPending&p,int(*predict)(const double*,double*),gw::ReplyBytes&wire,gw::Request*request_out=nullptr){
    const auto*b=p.pending_bytes();if(!b)return false;gw::RequestBytes arrived{};gw::Request q{};
    if(!gp_roundtrip(*b,arrived)||!gw::decode(arrived,q))return false;
    const auto&f=p.retained_actual_feedback();
    check(same(q.request19,f.actual.request19,152)&&q.source_timestamp_ns==f.actual.token.lease_envelope.timestamp_sample_us*1000&&
        q.source_generation==f.actual.token.lease_envelope.sample_generation&&q.output_generation==f.actual.token.lease_envelope.output_generation&&
        q.original_publication_us==f.actual.original_publication_us&&q.publication_valid_until_us==f.actual.original_valid_until_us,
        "actual owner feedback source/request/publication metadata survive RGP1 exactly");
    gw::Reply r{};r.identity=q.identity;r.source_timestamp_ns=q.source_timestamp_ns;r.source_generation=q.source_generation;r.output_generation=q.output_generation;
    r.original_request_sha256=gw::digest(arrived.data(),arrived.size());r.gp_model_sha256=q.gp_model_sha256;
    if(predict(q.request19+1,r.result18)!=0)return false;++gp_queries;
    gw::ReplyBytes sent{};if(!gw::encode(r,sent)||!gp_roundtrip(sent,wire))return false;
    if(request_out)*request_out=q;return true;
}
static bool two_commits(CycleOwner&o,li::CanonicalLocalGpPending&p){
    for(unsigned i=0;i<2;++i){StepInputs in{};if(!o.capture(i,in)||o.cycle->execute(in.value)!=li::LocalCycleExecute::Committed)return false;
        if(p.observe_actual_commit()!=(i?li::LocalGpBegin::RequestReady:li::LocalGpBegin::NotRequired))return false;}
    return true;
}
#ifndef GPENMPC_GP_PENDING_TEST_MAIN
#define GPENMPC_GP_PENDING_TEST_MAIN wmain
#endif
int GPENMPC_GP_PENDING_TEST_MAIN(int argc,wchar_t**argv){
    if((argc!=5&&argc!=6)||!load(argv[1],argv[2])||!load_private(argv[2])||!parse_hash(argv[4]))return 2;
    // Actual fixed canonical configuration digest; the retained private input,
    // session/source/Authority/publication remain explicitly HOST fixtures.
    const auto frozen=gw::canonical_configuration();
    for(unsigned j=0;j<32;++j)config.configuration_sha256[j]=static_cast<uint8_t>(frozen[j/4]>>(24-8*(j%4)));
    if(argc==6){
        CycleOwner o(true,false,nullptr,true);li::CanonicalLocalGpPending p(*o.cycle);
        for(unsigned i=0;i<60;++i){
            StepInputs in{};
            if(!o.capture(i,in)||o.cycle->execute(in.value)!=li::LocalCycleExecute::Committed||
                p.observe_actual_commit()!=li::LocalGpBegin::NotRequired)return 7;
            gpenmpc_full_inner_diagnostics d{};
            if(!o.cycle->numerical_diagnostics(d)||d.numeric_failure||d.joint_failure||p.pending())return 8;
        }
        std::printf("COMPONENT 60 actual generated-C SE3 steps, GP inference=0, GP wire=0, joint_installs=%llu; HOST fixture only\n",
            static_cast<unsigned long long>(o.phase->diagnostics().installs));
        return o.phase->diagnostics().installs==60?0:9;
    }
    HMODULE dll=LoadLibraryExW(argv[3],nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);if(!dll)return 3;
    using Predict=int(*)(const double*,double*);const auto predict=reinterpret_cast<Predict>(GetProcAddress(dll,"gpenmpc_gp256_predict"));if(!predict)return 3;
    unsigned completed=0,main_gp=0,main_packets=0;
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);
        for(unsigned i=0;i<60;++i){StepInputs in{};check(o.capture(i,in)&&o.cycle->execute(in.value)==li::LocalCycleExecute::Committed,"actual Cycle source/full-C publication before GP observer");
            const auto phase=o.phase->diagnostics();const auto pubs=publish_calls;const auto result=p.observe_actual_commit();
            if(result==li::LocalGpBegin::Rejected)std::fprintf(stderr,"pending row%u fault%u cycle%u\n",i,unsigned(p.diagnostics().first_fault),unsigned(o.cycle->diagnostics().first_fault));
            check(result==(i?li::LocalGpBegin::RequestReady:li::LocalGpBegin::NotRequired),"actual first not-required then each real pending query");
            if(result==li::LocalGpBegin::Rejected)break;
            check(p.retained_actual_feedback().actual.publication_succeeded&&p.retained_actual_feedback().phase_installed&&
                p.retained_actual_feedback().source_receipt_retired,"GP owner reads actual same-Cycle full completion");
            if(i){gw::ReplyBytes reply{};gw::Request q{};check(actual_reply(p,predict,reply,&q),"actual GP request and reply through six real MAVLink frames");
                const auto arrival=p.retained_actual_feedback().original_cycle_read_us+20;const auto processing=arrival+10;
                check(p.accept_reply(reply,arrival,processing)&&!p.pending()&&!p.pending_bytes(),"matched original numerical reply fills exactly one same owner pending");
                clock_us=processing+1;gpenmpc_full_inner_diagnostics d{};
                check(o.cycle->numerical_diagnostics(d)&&d.prediction_ready&&d.prediction_fills==i&&p.diagnostics().fills==i&&
                    p.retained_reply_bytes()&&same(p.retained_reply_bytes()->data(),reply.data(),reply.size()),
                    "actual numerical pending ready and original full reply bytes retained");
                check(p.diagnostics().last_original_arrival_us==arrival&&p.diagnostics().last_processing_us==processing&&
                    q.publication_valid_until_us==p.retained_actual_feedback().actual.original_valid_until_us,"original arrival processing and publication expiry stay separate");
                ++main_gp;
            }else check(!p.pending()&&!p.pending_bytes()&&!p.retained_request_bytes()&&gp_queries==0,"not-required first commit fabricates no GP packet");
            check(p.observe_actual_commit()==li::LocalGpBegin::Empty&&o.phase->diagnostics().installs==phase.installs&&
                same(&o.phase->diagnostics().phase_s,&phase.phase_s,8)&&same(&o.phase->diagnostics().phase_rate,&phase.phase_rate,8)&&publish_calls==pubs,
                "empty poll and real GP fill cause no source capture reference control or phase advancement");
            ++completed;
        }
        main_packets=gp_packets;check(completed==60&&main_gp==59&&main_packets==354&&publish_calls==60,"sixty real Cycle commits and59 actualGP queries over354 correct-direction packets");
    }
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);
        check(observe(o,0)==li::LocalCapture::Accepted&&o.cycle->release_disarmed(),"disarmed real source preparation");
        check(p.observe_actual_commit()==li::LocalGpBegin::Empty&&!p.pending_bytes()&&p.diagnostics().actual_feedback_reads==0&&
            no_math_or_publication(o),"disarmed observation has no control feedback or GP query");}
    for(unsigned kind=0;kind<11;++kind){CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);check(two_commits(o,p),"wire mismatch real pending setup");
        gw::ReplyBytes raw{};check(actual_reply(p,predict,raw),"actual predictor negative reply seed");gw::Reply decoded{};check(gw::decode(raw,decoded),"original reply decoded for bounded negative");
        if(kind<8){if(kind==0)decoded.source_timestamp_ns++;if(kind==1)decoded.source_generation++;if(kind==2)decoded.output_generation++;
            if(kind==3)decoded.identity.uid++;if(kind==4)decoded.identity.boot_generation++;if(kind==5)decoded.identity.system++;
            if(kind==6)decoded.identity.component++;if(kind==7)decoded.original_request_sha256[0]^=1;
            check(gw::encode(decoded,raw),"well-formed wrong causal tags retain valid transport checksum");
        }else if(kind==8){raw[78]^=1;repair_checksum(raw);} // wrong model, valid framing/checksum
        else if(kind==9)raw[254]^=1; // actual checksum corruption
        else{gpenmpc_snapshot_wire::Writer w(raw.data()+110);w.real(std::numeric_limits<double>::quiet_NaN());repair_checksum(raw);}
        const auto original=p.retained_actual_feedback();const auto arrival=original.original_cycle_read_us+20;
        check(!p.accept_reply(raw,arrival,arrival+10)&&p.diagnostics().first_fault==li::LocalGpFault::Wire&&p.diagnostics().fill_attempts==0&&
            p.diagnostics().fills==0&&publish_calls==2,"wrong tag identity model request digest checksum or nonfinite reply never reaches fill");
        check(p.retained_reply_bytes()&&same(p.retained_reply_bytes()->data(),raw.data(),raw.size())&&
            p.diagnostics().retained_reply_original_arrival_us==arrival&&p.retained_request_bytes()&&!p.pending_bytes()&&
            p.retained_actual_feedback().actual.original_publication_us==original.actual.original_publication_us&&
            p.retained_actual_feedback().actual.publication_succeeded,"rejected raw query reply arrival and actual prior publication remain auditable");
    }
    for(unsigned kind=0;kind<4;++kind){CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);check(two_commits(o,p),"clock negative real pending setup");
        gw::ReplyBytes reply{};check(actual_reply(p,predict,reply),"clock negative actual reply");const auto&f=p.retained_actual_feedback();
        uint64_t arrival=f.original_cycle_read_us+20,processing=arrival+10;
        if(kind==0)arrival=0;if(kind==1)arrival=f.actual.original_publication_us-1;
        if(kind==2){arrival=f.actual.original_publication_us;processing=f.original_cycle_read_us+10;} // after publication, before request existed
        if(kind==3)processing=arrival-1;
        check(!p.accept_reply(reply,arrival,processing)&&p.diagnostics().first_fault==li::LocalGpFault::Clock&&p.diagnostics().fill_attempts==0&&
            p.retained_reply_bytes()&&same(p.retained_reply_bytes()->data(),reply.data(),reply.size())&&
            p.diagnostics().retained_reply_original_arrival_us==arrival&&p.diagnostics().retained_reply_processing_us==processing,
            "zero regressed or pre-request arrival and reversed processing reject with exact original raw retained");
    }
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);check(two_commits(o,p),"old publication expiry not GP lifetime setup");gw::ReplyBytes reply{};
        check(actual_reply(p,predict,reply),"actual reply after original actuator lease age");
        const auto expiry=p.retained_actual_feedback().actual.original_valid_until_us;const auto pubs=publish_calls;
        check(p.accept_reply(reply,expiry+1,expiry+2)&&publish_calls==pubs&&o.phase->diagnostics().installs==2,
            "GP fill after old publication expiry grants no new output, no arbitrary five-ms GP deadline");
        clock_us=expiry+3;StepInputs next{};check(o.capture(2,next)&&o.cycle->execute(next.value)==li::LocalCycleExecute::Committed,
            "next real new source still must satisfy original age dt and whole actual Cycle gates");}
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);check(two_commits(o,p),"duplicate actual reply setup");gw::ReplyBytes reply{};actual_reply(p,predict,reply);
        const auto time=p.retained_actual_feedback().original_cycle_read_us+20;check(p.accept_reply(reply,time,time+1),"first matching reply actual fill");
        check(!p.accept_reply(reply,time+2,time+3)&&p.diagnostics().first_fault==li::LocalGpFault::Pending&&p.diagnostics().fills==1&&
            p.diagnostics().fill_attempts==1&&publish_calls==2,"duplicate reply cannot perform a second numerical fill");}
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);check(two_commits(o,p),"pending polling failure setup");const auto original=p.retained_actual_feedback();
        check(p.observe_actual_commit()==li::LocalGpBegin::Rejected&&p.diagnostics().first_fault==li::LocalGpFault::Pending&&p.retained_request_bytes()&&
            p.retained_actual_feedback().actual.original_publication_us==original.actual.original_publication_us,"new observe call cannot overwrite outstanding pending request");}
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);check(two_commits(o,p),"different schedule pending source setup");gw::ReplyBytes reply{};actual_reply(p,predict,reply);
        StepInputs next{};check(o.capture(2,next),"new private source pending before GP arrival");const auto time=p.retained_actual_feedback().original_cycle_read_us+20;
        check(!p.accept_reply(reply,time,time+1)&&p.diagnostics().first_fault==li::LocalGpFault::Numeric&&p.diagnostics().fill_attempts==1&&
            p.diagnostics().fills==0&&publish_calls==2&&o.phase->diagnostics().installs==2,"late GP with next source already pending cannot fill or execute");}
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);check(two_commits(o,p),"unobserved new actual commit pending-owner conflict setup");gw::ReplyBytes reply{};actual_reply(p,predict,reply);
        gw::Reply decoded{};gw::decode(reply,decoded);const uint64_t tags[2]={decoded.source_timestamp_ns,decoded.source_generation};
        check(o.cycle->fill_gp(tags,decoded.result18),"explicit test-only competing same-owner fill, not another Io");
        StepInputs next{};check(o.capture(2,next)&&o.cycle->execute(next.value)==li::LocalCycleExecute::Committed,"new actual third publication occurred before old pending owner advanced");
        check(p.observe_actual_commit()==li::LocalGpBegin::Rejected&&p.diagnostics().first_fault==li::LocalGpFault::Pending,"pending owner refuses silently adopting new commit");
        li::LocalCycleFeedback raw{};check(o.cycle->take_feedback(raw)&&raw.actual.publication_succeeded&&raw.actual.numerical_reference_installed&&
            raw.phase_installed&&raw.actual.token.lease_envelope.output_generation==3&&!raw.usable_at_read,
            "actual new publication joint and phase evidence remains in original Cycle after conflict");}
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);check(two_commits(o,p),"regressed later commit clock setup");gw::ReplyBytes reply{};actual_reply(p,predict,reply);
        const auto future=raw_fixture[2].timestamp_sample+1000;check(p.accept_reply(reply,future,future+1),"monotonic earlier reply processing retained");
        StepInputs next{};check(o.capture(2,next)&&o.cycle->execute(next.value)==li::LocalCycleExecute::Committed,"synthetic next-cycle clock intentionally regressed relative to recorded processing");
        check(p.observe_actual_commit()==li::LocalGpBegin::Rejected&&p.diagnostics().first_fault==li::LocalGpFault::Clock&&
            p.retained_actual_feedback().actual.publication_succeeded,"original cycle-read clock cannot go backward across replies");}
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);check(two_commits(o,p),"stop retains actual query setup");gw::ReplyBytes reply{};actual_reply(p,predict,reply);
        p.stop();check(!p.accept_reply(reply,clock_us,clock_us)&&p.diagnostics().first_fault==li::LocalGpFault::Stopped&&
            !p.pending_bytes()&&p.retained_request_bytes()&&p.retained_actual_feedback().actual.publication_succeeded&&p.diagnostics().fill_attempts==0,
            "stop preserves historical actual commit/query and cannot revive pending");}
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);StepInputs in{};o.capture(0,in);publish_success=false;
        check(o.cycle->execute(in.value)==li::LocalCycleExecute::Rejected&&p.observe_actual_commit()==li::LocalGpBegin::Rejected&&
            p.diagnostics().first_fault==li::LocalGpFault::Feedback&&p.retained_actual_feedback().actual.publication_attempted&&
            !p.retained_actual_feedback().actual.publication_succeeded&&!p.pending_bytes(),"failed publication leaves the GP query empty");}
    {CycleOwner o;li::CanonicalLocalGpPending p(*o.cycle);check(two_commits(o,p),"MAVLink direction negative setup");
        const auto request=*p.pending_bytes();gw::RequestBytes joined{};
        for(unsigned kind=1;kind<=3;++kind)check(!gp_roundtrip(request,joined,kind),"wrong sender target or payload_type cannot masquerade as registered direction");}
    FreeLibrary(dll);
    std::printf("{\"checks\":%u,\"failed\":%u,\"actual_cycle_rows\":%u,\"main_actual_GP_calls\":%u,\"main_actual_MAVLink_packets\":%u,\"all_actual_GP_calls\":%u,\"all_MAVLink_packets\":%u,\"sizeof_GP_pending\":%zu,\"payload_type\":42002,\"request_direction\":\"board1/1_to_HOST255/190\",\"reply_direction\":\"HOST255/190_to_board1/1\",\"source_HRT_arrival_Authority_uORB_MOCK\":true,\"actual_live_route\":false,\"COM\":0}\n",
        checks,failed,completed,main_gp,main_packets,gp_queries,gp_packets,sizeof(li::CanonicalLocalGpPending));
    return failed?1:0;
}
