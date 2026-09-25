#define main command_transport_fixture_main
#include "../test_slim_argument_transport.cpp"
#undef main
#include "../px4_runtime/CanonicalExchangePump.hpp"
#include <deque>
#include <thread>
#include <atomic>
namespace px=gpenmpc_rfly_px4;
namespace cw=gpenmpc_context_wire;
namespace sw=gpenmpc_snapshot_wire;
const orb_metadata __orb_vehicle_odometry{0,sizeof(vehicle_odometry_s),1};
const orb_metadata __orb_vehicle_status{1,sizeof(vehicle_status_s),1};
const orb_metadata __orb_vehicle_control_mode{2,sizeof(vehicle_control_mode_s),1};
const orb_metadata __orb_offboard_control_mode{3,sizeof(offboard_control_mode_s),1};
const orb_metadata __orb_actuator_outputs_rfly{4,sizeof(actuator_outputs_s),1};
const orb_metadata __orb_gpenmpc_full_inner_ingress{5,sizeof(gpenmpc_full_inner_ingress_s),8};
struct BusRecord{std::uint32_t generation{};std::vector<std::uint8_t>bytes;};
std::deque<BusRecord>bus[8];std::uint32_t bus_generation[8]{};std::uint64_t actual_test_hrt=0;bool publish_failure=false;
std::uint64_t simulated_ocm_publish_delay_us=0; // labelled HOST latency injection only
extern "C" std::uint64_t hrt_absolute_time()noexcept{return actual_test_hrt;}
bool gpenmpc_test_topic_update(const orb_metadata*meta,void*out,std::uint32_t&generation,std::uint8_t instance)noexcept{
    if(instance)return false;
    for(const auto&r:bus[meta->id])if(r.generation>generation){std::memcpy(out,r.bytes.data(),meta->size);generation=r.generation;return true;}return false;
}
bool gpenmpc_test_topic_publish(const orb_metadata*meta,const void*in)noexcept{
    if(publish_failure&&meta->id==4)return false;
    BusRecord record{};record.generation=++bus_generation[meta->id];record.bytes.resize(meta->size);std::memcpy(record.bytes.data(),in,meta->size);
    auto&q=bus[meta->id];q.push_back(record);while(q.size()>meta->queue)q.pop_front();
    if(meta->id==3)actual_test_hrt+=simulated_ocm_publish_delay_us;return true;
}
void clear_bus(){for(auto&q:bus)q.clear();for(auto&v:bus_generation)v=0;actual_test_hrt=0;publish_failure=false;simulated_ocm_publish_delay_us=0;}
class SimulatedAuthority final:public px::Authority {
public:
    std::uint64_t until{},installs{},confirms{},revokes{},confirm_delay_us{};
    bool observed_identity(Identity&out)noexcept override{out=id();return true;}
    bool validate(const Token&,const vehicle_status_s&,const vehicle_control_mode_s&,const offboard_control_mode_s&,
        std::uint64_t now,std::uint64_t&expiry)noexcept override{expiry=until;return now<=until;}
    bool install_lease(const Token&,const actuator_outputs_s&,std::uint64_t valid_until)noexcept override{++installs;return valid_until<=until;}
    bool confirm_publication(const Token&,std::uint64_t pub,std::uint64_t committed)noexcept override{++confirms;actual_test_hrt+=confirm_delay_us;return pub<=committed&&committed<=until;}
    void revoke()noexcept override{++revokes;}
};
void telemetry(std::uint64_t sample){vehicle_status_s status{};status.timestamp=sample+200;status.hil_state=vehicle_status_s::HIL_STATE_ON;
    status.arming_state=vehicle_status_s::ARMING_STATE_ARMED;status.nav_state=vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
    vehicle_control_mode_s mode{};mode.timestamp=sample+200;mode.flag_armed=true;mode.flag_control_offboard_enabled=true;
    offboard_control_mode_s offboard{};offboard.timestamp=sample+200;offboard.direct_actuator=true;
    gpenmpc_test_topic_publish(ORB_ID(vehicle_status),&status);gpenmpc_test_topic_publish(ORB_ID(vehicle_control_mode),&mode);gpenmpc_test_topic_publish(ORB_ID(offboard_control_mode),&offboard);
}
struct PumpOutcome{bool passed{},sticky{};unsigned frames{},steps{},publications{},exports{},captures{},feedback_frames{},wire_bytes{};std::uint64_t source_generation{};px::ExchangeFault fault{};};
PumpOutcome run_pump(const std::vector<Fixture>&fixtures,const std::vector<Raw>&states,unsigned mutation=0){
    clear_bus();PumpOutcome result{};const auto execution=execution_config(fixtures[0]);auto source=source_config();source.vehicle_odometry_topic=ORB_ID(vehicle_odometry);
    SimulatedAuthority authority;px::Px4CanonicalIo io(source,execution,5000,&authority);
    px::ExchangeConfiguration config{};config.execution=execution;config.transport={42,191,1,1,3,5000};config.retained_anchor_capacity=64;
    px::CanonicalExchangePump pump(config);ingress::Receiver receiver;SnapshotTicket first_ticket{};bool ok=true;
    std::ofstream feedback_file;if(!mutation){feedback_file.open("PRODUCTION_IO_FEEDBACK_PACKETS.bin",std::ios::binary);feedback_file.write("RFC5",4);const char count[4]={0,0,0,2};feedback_file.write(count,4);}
    for(unsigned round=0;round<2&&ok;++round){const std::uint64_t sample=1000000+10000*round;actual_test_hrt=sample+300;authority.until=sample+4000;
        auto odometry=raw(states[round],sample);
        // This is a simulated atomic source update with its original generation;
        // increment2 proves a slow roundtrip cannot manufacture missed samples.
        bus_generation[0]=std::uint32_t(states[round].generation-1+(mutation==8&&round==1?1:0));
        gpenmpc_test_topic_publish(ORB_ID(vehicle_odometry),&odometry);telemetry(sample);
        if(pump.poll(io)!=px::ExchangePoll::Progress){ok=false;break;}
        sw::Bytes exported{};unsigned offset=0;
        for(unsigned j=0;j<3;++j){px::SnapshotFragment out{};if(pump.snapshot_outbox().take(out,actual_test_hrt)!=px::ExportTake::Fragment){ok=false;break;}
            const auto n=out.fragment.length-9;std::memcpy(exported.data()+offset,out.fragment.payload+9,n);offset+=n;++result.exports;}
        if(!ok)break;SnapshotTicket ticket{};std::memcpy(ticket.data(),exported.data()+4,32);if(!round)first_ticket=ticket;
        cw::Context context{};context.configuration_sha256=old::bytes_of(execution.configuration_payload_sha256);context.reference_generation=round+1;context.outer_generation=1;
        context.reference_source_ticket=ticket;context.outer_source_ticket=first_ticket;
        context.reference_time={9000000000ULL+round*10000000ULL,9000100000ULL+round*10000000ULL,9400000000ULL+round*10000000ULL};
        context.outer_time={9000000000ULL,9000050000ULL,9400000000ULL};context.outer_payload={0.125,0.01,-0.02,0.03};
        const auto&f=fixtures[round];for(unsigned j=0;j<3;++j){const double sign=j==2?-1:1;
            context.reference_ned[j]=sign*f.k.refP[j];context.reference_ned[j+3]=sign*f.k.refV[j];context.reference_ned[j+6]=sign*f.k.refA[j];}
        cw::Bytes context_bytes{};cw::encode(context,context_bytes);auto md=metadata(f,ticket);md.reference_generation=round+1;
        for(unsigned j=0;j<3;++j){old::Fragment part{};sw::fragment(exported,j,part);mavlink_message_t m{};mavlink_status_t tx{};
            mavlink_msg_tunnel_pack_status(1,1,&tx,&m,42,191,ingress::payload_type,part.length,part.payload);std::uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};result.wire_bytes+=mavlink_msg_to_send_buffer(bytes,&m);}
        t::SlimMessage numerical{};const auto abi=gpenmpc_rfly_slim::encode(f.k);t::encode_slim(md,abi.data(),abi.size(),numerical);
        // Actual generated packets -> byte parser -> original receiver -> the
        // simulated uORB test adapter. All7 already queued before ONE real poll.
        for(unsigned j=0;j<7;++j){old::Fragment part{};if(j<3)cw::fragment(context_bytes,j,part);else t::fragment_slim(numerical,j-3,part);
            if(mutation==1&&round==1&&j==2)part.payload[0]=0x20;
            if(mutation==2&&round==1&&j==3)part.payload[0]=0x40;
            if(mutation==3&&round==1&&j==1)part.payload[0]=0x70;
            mavlink_message_t message{},buffer{},parsed{};mavlink_status_t tx{},parser{},status{};tx.current_tx_seq=std::uint8_t(round*7+j);
            mavlink_msg_tunnel_pack_status(42,191,&tx,&message,1,1,ingress::payload_type,part.length,part.payload);
            std::uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};const auto n=mavlink_msg_to_send_buffer(bytes,&message);unsigned good=0;
            for(unsigned k=0;k<n;++k)good+=mavlink_frame_char_buffer(&buffer,&parser,bytes[k],&parsed,&status)==MAVLINK_FRAMING_OK;
            if(good!=1){ok=false;break;}++result.frames;result.wire_bytes+=n;
            receiver.receive(parsed,sample+500+100*j,1,1,3);
        }
        receiver.drain(sample+1150,[&](const ingress::Fields&fields){gpenmpc_full_inner_ingress_s m{};ingress::copy_to_topic(fields,receiver.counters(),m);
            if(mutation==4&&round==1&&fields.reception_sequence==9){++bus_generation[5];return true;}
            if(mutation==5&&round==1)m.timestamp=sample-10000;
            if(mutation==6&&round==1)m.ingress_first_fault=1;
            return gpenmpc_test_topic_publish(ORB_ID(gpenmpc_full_inner_ingress),&m);});
        actual_test_hrt=sample+(mutation==7&&round==1?6001:1500);
        if(mutation==9&&round==1)publish_failure=true;
        if(pump.poll(io)!=px::ExchangePoll::Progress){ok=false;break;}
        const auto&actual=io.last_receipt();
        if(!actual.receipt_valid){ok=false;break;}
        // Read the actual generated step's result in this isolated HOST test;
        // no second kernel call or reconstruction supplies a feedback oracle.
        for(unsigned j=0;j<61;++j)if(std::abs(GPENMPC_Rfly_Canonical_Control_Y.FullKernel61[j]-f.expected[j])>1e-10)ok=false;
        if(!ok)break;
        const auto&r=bus[4].back();actuator_outputs_s output{};std::memcpy(&output,r.bytes.data(),sizeof output);
        float expected[16]{};const unsigned map[6]={4,0,3,5,1,2};for(unsigned j=0;j<6;++j)expected[map[j]]=static_cast<float>(f.expected[j+4]/32.145727009134916);
        if(std::memcmp(expected,output.output,sizeof expected)!=0){ok=false;break;}
        gpenmpc_feedback_wire::Bytes complete{};std::vector<std::vector<std::uint8_t>>actual_frames;unsigned feedback_offset=0;
        for(unsigned j=0;j<10;++j){px::FeedbackFragment part{};
            if(pump.feedback_outbox().copy_next(part,actual_test_hrt)!=px::FeedbackExport::Fragment||!pump.feedback_outbox().allows_send(part.generation,part.index,actual_test_hrt)){ok=false;break;}
            mavlink_message_t message{},buffer{},parsed{};mavlink_status_t tx{},parser{},status{};
            mavlink_msg_tunnel_pack_status(1,1,&tx,&message,42,191,ingress::payload_type,part.fragment.length,part.fragment.payload);
            std::uint8_t wire[MAVLINK_MAX_PACKET_LEN]{};const auto n=mavlink_msg_to_send_buffer(wire,&message);unsigned good=0;
            for(unsigned k=0;k<n;++k)good+=mavlink_frame_char_buffer(&buffer,&parser,wire[k],&parsed,&status)==MAVLINK_FRAMING_OK;
            mavlink_tunnel_t decoded{};mavlink_msg_tunnel_decode(&parsed,&decoded);
            if(good!=1||decoded.payload[0]!=0x50+j||!pump.feedback_outbox().retire_copy(part.generation,part.index)){ok=false;break;}
            const unsigned ndata=decoded.payload_length-9;std::memcpy(complete.data()+feedback_offset,decoded.payload+9,ndata);feedback_offset+=ndata;
            actual_frames.emplace_back(wire,wire+n);++result.feedback_frames;result.wire_bytes+=n;
        }
        px::CommittedFeedback feedback{},duplicate{};
        if(!ok||!gpenmpc_feedback_wire::decode(complete,feedback)||feedback.disposition!=px::FeedbackDisposition::Fresh||feedback.snapshot_ticket!=ticket||
           !(feedback.token==actual.execution.binding.consumed)||feedback.commit_completed_us!=io.diagnostics().last_commit_completed_us||
           feedback.publication_us!=io.diagnostics().last_publication_us||feedback.original_valid_until_us!=io.diagnostics().original_valid_until_us||
           std::memcmp(feedback.actual61.data(),GPENMPC_Rfly_Canonical_Control_Y.FullKernel61,488)!=0||
           std::memcmp(feedback.published_control16.data(),output.output,64)!=0||io.take_committed_feedback(duplicate)!=px::FeedbackDisposition::Empty){ok=false;break;}
        if(!mutation){feedback_file.write(reinterpret_cast<const char*>(complete.data()),complete.size());
            gpenmpc_portable::Array<std::uint8_t,488>oracle{};sw::Writer w(oracle.data());for(double value:feedback.actual61)w.real(value);
            feedback_file.write(reinterpret_cast<const char*>(oracle.data()),oracle.size());
            for(const auto&frame:actual_frames){const std::uint8_t length[2]={std::uint8_t(frame.size()>>8),std::uint8_t(frame.size())};feedback_file.write(reinterpret_cast<const char*>(length),2);feedback_file.write(reinterpret_cast<const char*>(frame.data()),frame.size());}}
    }
    result.steps=unsigned(io.diagnostics().kernel_calls);result.publications=unsigned(io.diagnostics().publish_succeeded);result.captures=unsigned(io.diagnostics().captured);
    result.source_generation=io.diagnostics().last_source_generation;result.passed=ok&&pump.diagnostics().executed==2&&authority.confirms==2;
    result.fault=pump.diagnostics().fault;
    if(!result.passed){const auto before=pump.diagnostics().first_fault_us;actual_test_hrt+=1;result.sticky=pump.poll(io)==px::ExchangePoll::Fault&&pump.diagnostics().fault==result.fault&&pump.diagnostics().first_fault_us==before;}
    return result;
}
void outbox_tests(){
    sw::Bytes bytes{};bytes[58]=1;px::SnapshotOutbox box;px::SnapshotFragment fragment{};
    check(box.publish(bytes,100,200,42,191),"outbox producer publishes immutable snapshot once");
    check(!box.publish(bytes,100,200,42,191),"pending snapshot cannot be overwritten");
    for(unsigned j=0;j<3;++j)check(box.take(fragment,150)==px::ExportTake::Fragment&&fragment.fragment.payload[0]==0x30+j&&fragment.original_valid_until_us==200,"outbox exactly3 original deadline fragments");
    check(box.take(fragment,150)==px::ExportTake::Empty,"outbox empty after third copy");
    check(box.publish(bytes,100,200,42,191)&&box.take(fragment,201)==px::ExportTake::Expired&&box.failed(),"expiry never refreshed by consumption");
    check(!box.publish(bytes,100,300,42,191),"expired outbox cannot be republished to wash fault");
    box.close();check(box.take(fragment,150)==px::ExportTake::Stopped&&!box.publish(bytes,100,200,42,191),"closed outbox no reset or new producer");
    // Two-thread SPSC byte-handoff test using synthetic queue records.
    px::SnapshotOutbox concurrent;std::atomic<unsigned>errors{0},received{0};
    std::thread consumer([&]{for(unsigned message=1;message<=1000;++message){sw::Bytes rebuilt{};unsigned offset=0;
        for(unsigned part=0;part<3;++part){px::SnapshotFragment value{};px::ExportTake state;
            do{state=concurrent.take(value,150);if(state==px::ExportTake::Empty)std::this_thread::yield();}while(state==px::ExportTake::Empty);
            if(state!=px::ExportTake::Fragment){++errors;return;}
            const unsigned n=value.fragment.length-9;std::memcpy(rebuilt.data()+offset,value.fragment.payload+9,n);offset+=n;}
        for(unsigned j=0;j<rebuilt.size();++j)if(rebuilt[j]!=std::uint8_t(message+j)){++errors;break;}++received;}});
    std::thread producer([&]{for(unsigned message=1;message<=1000;++message){sw::Bytes value{};
        for(unsigned j=0;j<value.size();++j)value[j]=std::uint8_t(message+j);
        while(!concurrent.publish(value,100,200,42,191))std::this_thread::yield();}});
    producer.join();consumer.join();check(errors.load()==0&&received.load()==1000,"actual concurrent producer/consumer1000 snapshots no overwrite/torn read");
}
void actual_io_feedback_tests(const Fixture&fixture,const Raw&state){
    for(unsigned mutation=0;mutation<6;++mutation){clear_bus();actual_test_hrt=1000300;
        const auto execution=execution_config(fixture);auto source=source_config();source.vehicle_odometry_topic=ORB_ID(vehicle_odometry);
        SimulatedAuthority authority;authority.until=1004000;px::Px4CanonicalIo io(source,execution,5000,&authority);
        auto odometry=raw(state);bus_generation[0]=std::uint32_t(state.generation-1);gpenmpc_test_topic_publish(ORB_ID(vehicle_odometry),&odometry);telemetry(1000000);
        SnapshotTicket ticket{};check(io.capture_next(ticket)==px::Capture::Accepted,"actual Io private capture for receipt lifetime negative");
        const auto command=slim_command(fixture,ticket);actual_test_hrt=1000900;if(mutation==4)publish_failure=true;
        if(mutation==5)authority.confirm_delay_us=5000;
        const bool executed=io.execute(command);px::CommittedFeedback f{};
        if(mutation==4){check(!executed&&io.take_committed_feedback(f)==px::FeedbackDisposition::Empty&&io.diagnostics().publish_succeeded==0,"failed actual publication emits no successful feedback");continue;}
        if(mutation==5){check(!executed&&io.diagnostics().fault==px::Fault::Expired&&io.diagnostics().kernel_calls==1&&io.diagnostics().publish_succeeded==1&&io.diagnostics().numerical_commits==1&&
            io.take_committed_feedback(f)==px::FeedbackDisposition::Revoked&&f.commit_completed_us==1000900&&f.original_valid_until_us==1004000&&std::memcmp(f.actual61.data(),GPENMPC_Rfly_Canonical_Control_Y.FullKernel61,488)==0,
            "confirm crosses original expiry: actual counters/raw retained, execute fails, no fresh receipt");continue;}
        check(executed&&io.diagnostics().kernel_calls==1&&io.diagnostics().numerical_commits==1,"actual Io single execute creates one observation");
        const auto until=io.diagnostics().original_valid_until_us;
        if(mutation==1)actual_test_hrt=until+1;
        if(mutation==2)io.stop();
        if(mutation==3)check(!io.execute(command)&&io.diagnostics().fault==px::Fault::Feedback&&io.diagnostics().kernel_calls==1&&io.diagnostics().publish_succeeded==1,"unconsumed feedback refuses second action before kernel and preserves first counts");
        const auto disposition=io.take_committed_feedback(f);
        check(disposition==(mutation==0?px::FeedbackDisposition::Fresh:mutation==1?px::FeedbackDisposition::HistoricalExpired:px::FeedbackDisposition::Revoked)&&
            f.original_valid_until_us==until&&std::memcmp(f.actual61.data(),GPENMPC_Rfly_Canonical_Control_Y.FullKernel61,488)==0,"production raw61 retained through original expiry/revoke/full latch");
        px::CommittedFeedback duplicate{};check(io.take_committed_feedback(duplicate)==px::FeedbackDisposition::Empty,"production observation exactly once");
        if(mutation)continue;
        for(unsigned kind=0;kind<6;++kind){px::CommittedFeedbackOutbox outbox;check(outbox.publish(f,42,191),"feedback outbox original commit one slot");
            if(kind==1)outbox.revoke();
            px::FeedbackFragment fragment{};gpenmpc_feedback_wire::Bytes bytes{};unsigned offset=0;
            for(unsigned j=0;j<10;++j){const auto now=kind==0?until+1:f.commit_completed_us;
                if(kind==5&&j==1){check(outbox.copy_next(fragment,until+1)==px::FeedbackExport::Interrupted,"Fresh expiry after emitted first fragment stops batch without rewriting prefix");
                    px::CommittedFeedback audit{};check(outbox.take_interrupted_raw(audit)==px::FeedbackDisposition::HistoricalExpired&&audit.token==f.token&&audit.original_valid_until_us==until,"partial-message expiry preserves original raw token and deadline");break;}
                const auto copied=outbox.copy_next(fragment,now);check(copied==px::FeedbackExport::Fragment,"feedback bounded immutable fragment copy");
                if(copied!=px::FeedbackExport::Fragment)break;
                if(j==0&&(kind==2||kind==3)){
                    if(kind==2)outbox.revoke();
                    check(!outbox.allows_send(fragment.generation,fragment.index,kind==3?until+1:now),"post-copy fresh revocation/expiry rejected before void send");
                    outbox.interrupt();px::CommittedFeedback audit{};
                    check(outbox.take_interrupted_raw(audit)==(kind==2?px::FeedbackDisposition::Revoked:px::FeedbackDisposition::HistoricalExpired)&&std::memcmp(audit.actual61.data(),f.actual61.data(),488)==0,"incomplete wire raw retained for explicit audit flush");
                    check(!outbox.publish(f,42,191),"audit flush cannot wash failed batch or retry");break;
                }
                const unsigned n=fragment.fragment.length-9;std::memcpy(bytes.data()+offset,fragment.fragment.payload+9,n);offset+=n;
                check(outbox.allows_send(fragment.generation,fragment.index,now)&&outbox.retire_copy(fragment.generation,fragment.index),"copy retirement is bounded ownership not transport ACK");
            }
            if(kind<2||kind==4){px::CommittedFeedback decoded{};
                check(gpenmpc_feedback_wire::decode(bytes,decoded)&&decoded.disposition==(kind==0?px::FeedbackDisposition::HistoricalExpired:kind==1?px::FeedbackDisposition::Revoked:px::FeedbackDisposition::Fresh)&&!outbox.pending(),"classification frozen before first fragment and whole SHA remains valid");}
        }
        px::CommittedFeedbackOutbox concurrent;std::atomic<unsigned>errors{0},received{0};std::atomic<bool>cancel{false};
        std::thread consumer([&]{for(unsigned generation=1;generation<=100&&!cancel.load();++generation){gpenmpc_feedback_wire::Bytes bytes{};unsigned offset=0;
            for(unsigned j=0;j<10;++j){px::FeedbackFragment part{};px::FeedbackExport result;
                do{result=concurrent.copy_next(part,f.commit_completed_us);if(result==px::FeedbackExport::Empty)std::this_thread::yield();}while(result==px::FeedbackExport::Empty&&!cancel.load());
                if(result!=px::FeedbackExport::Fragment){++errors;cancel=true;return;}
                const auto n=part.fragment.length-9;std::memcpy(bytes.data()+offset,part.fragment.payload+9,n);offset+=n;
                if(!concurrent.retire_copy(part.generation,part.index)){++errors;cancel=true;return;}}
            px::CommittedFeedback decoded{};if(!gpenmpc_feedback_wire::decode(bytes,decoded)||decoded.token.output_generation!=generation||decoded.actual61[0]!=generation){++errors;cancel=true;return;}++received;}});
        std::thread producer([&]{for(unsigned generation=1;generation<=100&&!cancel.load();++generation){auto synthetic=f;synthetic.token.output_generation=generation;synthetic.actual61[0]=generation;
            while(!concurrent.publish(synthetic,42,191)&&!cancel.load())std::this_thread::yield();}});
        producer.join();consumer.join();check(errors.load()==0&&received.load()==100,"SPSC handoff preserves 100 records across 1000 fragments without torn reads");
    }
}
void disarmed_telemetry(std::uint64_t sample){telemetry(sample);vehicle_status_s status{};status.timestamp=sample+200;
    status.hil_state=vehicle_status_s::HIL_STATE_ON;status.arming_state=vehicle_status_s::ARMING_STATE_DISARMED;
    vehicle_control_mode_s mode{};mode.timestamp=sample+200;
    gpenmpc_test_topic_publish(ORB_ID(vehicle_status),&status);gpenmpc_test_topic_publish(ORB_ID(vehicle_control_mode),&mode);}
sw::Bytes take_snapshot(px::CanonicalExchangePump&pump){sw::Bytes bytes{};unsigned offset=0;
    for(unsigned j=0;j<3;++j){px::SnapshotFragment f{};if(pump.snapshot_outbox().take(f,actual_test_hrt)!=px::ExportTake::Fragment)throw std::runtime_error("expected original snapshot export");
        const unsigned n=f.fragment.length-9;std::memcpy(bytes.data()+offset,f.fragment.payload+9,n);offset+=n;}return bytes;}
void disarmed_observation_tests(const Fixture&fixture,const Raw&state){
    for(unsigned mutation=0;mutation<3;++mutation){clear_bus();actual_test_hrt=1000300;auto source=source_config();source.vehicle_odometry_topic=ORB_ID(vehicle_odometry);
        SimulatedAuthority authority;authority.until=1004000;px::Px4CanonicalIo io(source,execution_config(fixture),5000,&authority);auto odometry=raw(state);
        bus_generation[0]=std::uint32_t(state.generation-1);gpenmpc_test_topic_publish(ORB_ID(vehicle_odometry),&odometry);disarmed_telemetry(1000000);
        if(mutation==1)telemetry(1000000);
        SnapshotTicket ticket{};const auto result=io.capture_disarmed_observation(ticket);
        if(mutation==1){check(result==px::Capture::Rejected&&io.diagnostics().kernel_calls==0&&authority.installs==0,"armed actual status rejects observation entry without kernel/lease");continue;}
        check(result==px::Capture::Accepted,"real disarmed original source observation");
        if(mutation==2)telemetry(1000000);
        const bool released=io.release_disarmed_observation(ticket);
        check(released==(mutation==0),"release rechecks actual disarmed state and does not grant authority");
        check(!io.execute(slim_command(fixture,ticket))&&io.diagnostics().kernel_calls==0&&io.diagnostics().publish_attempts==0&&io.diagnostics().numerical_commits==0,"released or rejected observation ticket cannot execute");
    }
    {std::uint64_t now=1000300;gpenmpc_rfly_slim::SnapshotExecutor<> store(source_config(),execution_config(fixture),{clock_now,&now});SnapshotTicket ticket{};
        check(store.capture(raw(state),std::uint32_t(state.generation),now,id(),&topic,0,ticket)&&store.releaseObservation(ticket),"private core Available to ObservationReleased without execution history");
        now=1000900;NumericalPrepared prepared{};check(!store.prepareNumericalOnly(slim_command(fixture,ticket),prepared)&&store.kernel_calls()==0,"private core independently rejects replay of released observation");}
    clear_bus();auto source=source_config();source.vehicle_odometry_topic=ORB_ID(vehicle_odometry);const auto execution=execution_config(fixture);
    SimulatedAuthority authority;px::Px4CanonicalIo io(source,execution,5000,&authority);px::ExchangeConfiguration config{};
    config.execution=execution;config.transport={42,191,1,1,3,5000};config.retained_anchor_capacity=64;px::CanonicalExchangePump pump(config);
    SnapshotTicket original_outer_ticket{};
    for(unsigned j=0;j<30;++j){const std::uint64_t sample=1000000+10000*j;actual_test_hrt=sample+300;authority.until=sample+4000;
        auto odometry=raw(state,sample);gpenmpc_test_topic_publish(ORB_ID(vehicle_odometry),&odometry);disarmed_telemetry(sample);
        check(pump.poll_disarmed_observation(io)==px::ExchangePoll::Progress,"30 actual 10ms disarmed observations continue during asynchronous preparation");
        const auto bytes=take_snapshot(pump);if(!j)std::memcpy(original_outer_ticket.data(),bytes.data()+4,32);
    }
    check(io.diagnostics().disarmed_observations_released==30&&io.diagnostics().kernel_calls==0&&io.diagnostics().publish_attempts==0&&io.diagnostics().numerical_commits==0&&authority.installs==0,
        "300ms preparation observations have zero kernel/lease/publication/commit");
    const std::uint64_t sample=1300000;actual_test_hrt=sample+300;authority.until=sample+4000;auto odometry=raw(state,sample);
    gpenmpc_test_topic_publish(ORB_ID(vehicle_odometry),&odometry);telemetry(sample);
    check(pump.poll(io)==px::ExchangePoll::Progress,"first direct source is fresh actual generation31, no reset or fake source tick");
    const auto bytes=take_snapshot(pump);SnapshotTicket ticket{};std::memcpy(ticket.data(),bytes.data()+4,32);
    cw::Context context{};context.configuration_sha256=old::bytes_of(execution.configuration_payload_sha256);context.reference_generation=context.outer_generation=1;
    context.reference_source_ticket=ticket;context.outer_source_ticket=original_outer_ticket;
    context.reference_time={9300000000ULL,9300100000ULL,9700000000ULL};context.outer_time={9000000000ULL,9000050000ULL,9400000000ULL};context.outer_payload={0.125,0.01,-0.02,0.03};
    for(unsigned j=0;j<3;++j){const double sign=j==2?-1:1;context.reference_ned[j]=sign*fixture.k.refP[j];context.reference_ned[j+3]=sign*fixture.k.refV[j];context.reference_ned[j+6]=sign*fixture.k.refA[j];}
    cw::Bytes context_bytes{};cw::encode(context,context_bytes);old::Metadata md{};md.command_generation=31;md.reference_generation=md.outer_generation=1;
    md.snapshot_ticket=ticket;md.configuration_sha256=old::bytes_of(execution.configuration_payload_sha256);auto arguments=fixture.k;
    arguments.augmentation_state_generation=arguments.continuity_state_generation=31;const auto abi=gpenmpc_rfly_slim::encode(arguments);t::SlimMessage numerical{};t::encode_slim(md,abi.data(),abi.size(),numerical);
    ingress::Receiver receiver;
    for(unsigned j=0;j<7;++j){old::Fragment part{};if(j<3)cw::fragment(context_bytes,j,part);else t::fragment_slim(numerical,j-3,part);
        mavlink_message_t message{},buffer{},parsed{};mavlink_status_t tx{},parser{},status{};mavlink_msg_tunnel_pack_status(42,191,&tx,&message,1,1,ingress::payload_type,part.length,part.payload);
        std::uint8_t packet[MAVLINK_MAX_PACKET_LEN]{};const auto n=mavlink_msg_to_send_buffer(packet,&message);unsigned good=0;
        for(unsigned k=0;k<n;++k)good+=mavlink_frame_char_buffer(&buffer,&parser,packet[k],&parsed,&status)==MAVLINK_FRAMING_OK;
        check(good==1,"prepared-context actual generated MAVLink frame");receiver.receive(parsed,sample+500+100*j,1,1,3);}
    receiver.drain(sample+1150,[&](const ingress::Fields&fields){gpenmpc_full_inner_ingress_s m{};ingress::copy_to_topic(fields,receiver.counters(),m);return gpenmpc_test_topic_publish(ORB_ID(gpenmpc_full_inner_ingress),&m);});
    actual_test_hrt=sample+1500;check(pump.poll(io)==px::ExchangePoll::Progress&&io.diagnostics().kernel_calls==1&&io.diagnostics().publish_succeeded==1,"first direct execution accepts original300ms-old outer anchor within unchanged400ms limit");
    const auto&token=io.last_receipt().execution.binding.consumed;
    check(token.sample_generation==31&&token.source_generation_delta==0&&token.sample_delta_us==0&&token.actual_tick_delta_us==0&&token.outer_based_on_sample_generation==1&&token.outer_based_on_timestamp_sample_us==1000000,
        "observation history is not control history; original prepared outer lineage retained");
    for(unsigned j=0;j<61;++j)check(std::abs(GPENMPC_Rfly_Canonical_Control_Y.FullKernel61[j]-fixture.expected[j])<=1e-10,"first direct full61 unchanged under released observation phase");
    check(pump.poll_disarmed_observation(io)==px::ExchangePoll::Fault&&io.diagnostics().kernel_calls==1,"observation mode cannot reopen after direct execution");
}
#ifndef GPENMPC_PUMP_TEST_HELPERS_ONLY
int main(int argc,char**argv){try{if(argc!=3)return 2;const auto fixtures=read_kernel(argv[1]);const auto states=read_state(argv[2]);const auto positive=run_pump(fixtures,states);
    check(positive.passed&&positive.frames==14&&positive.exports==6,"production Pump+Io actual2x7 queued frames and6 private snapshot fragments");
    check(positive.steps==2&&positive.publications==2&&positive.captures==2,"two actual canonical steps, simulated uORB publications, only2 captures");
    check(positive.feedback_frames==20,"actual production Io readonly61 receipt to outbox and20 actual MAVLink frames");
    for(unsigned mutation=1;mutation<=9;++mutation){const auto negative=run_pump(fixtures,states,mutation);
        check(!negative.passed&&negative.sticky,"production pump bad order/schema/gap/time/fault/late poll/source generation/publish failure latches");
        check(negative.publications<2,"invalid second transaction not counted as successful publication");}
    outbox_tests();actual_io_feedback_tests(fixtures[0],states[0]);disarmed_observation_tests(fixtures[0],states[0]);
    std::ofstream out("EXCHANGE_PUMP_RESULT.json");out<<"{\"checks\":"<<checks<<",\"failed\":"<<failures<<",\"actual_production_pump_cpp\":true,\"actual_production_io_cpp\":true,\"positive_generated_mavlink_frames\":"<<positive.frames<<",\"positive_private_export_fragments\":"<<positive.exports<<",\"actual_simulink_steps\":"<<positive.steps<<",\"simulated_uorb_publications\":"<<positive.publications<<",\"feedback_frames\":"<<positive.feedback_frames<<",\"two_complete_exchange_wire_bytes\":"<<positive.wire_bytes<<",\"negative_scenarios\":9,\"same_poll_context3_numeric4\":true,\"outbox_concurrent_synthetic_messages\":1000,\"feedback_spsc_synthetic_messages\":100,\"disarmed_observations_released\":30,\"disarmed_kernel_lease_publish_commit_counts\":[0,0,0,0],\"bootstrap_first_direct_actual_kernel_steps\":1,\"bootstrap_first_direct_control_history_deltas\":[0,0,0],\"original_outer_anchor_age_us_at_first_direct\":301500,\"uorb_hrt_authority\":\"EXPLICIT_HOST_TEST_ADAPTER_NOT_REAL_BOARD\",\"live_clock_binding\":false,\"board_access\":0}\n";
    std::cout<<"checks="<<checks<<" failed="<<failures<<" frames="<<positive.frames<<" exports="<<positive.exports<<" steps="<<positive.steps<<" pubs="<<positive.publications<<'\n';return failures?1:0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
#endif
