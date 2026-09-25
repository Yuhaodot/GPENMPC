#define main command_transport_fixture_main
#include "../test_slim_argument_transport.cpp"
#undef main
#include "RflyIngressDispatch.hpp"
namespace cw=gpenmpc_context_wire;
namespace sw=gpenmpc_snapshot_wire;
struct Outcome{unsigned frames{},wire_bytes{},steps{},commits{};bool passed{},sticky{};gpenmpc_argument_transport::Fault fault{};std::uint64_t sequence{},uorb{};};
Outcome run_dispatch(const std::vector<Fixture>&fixtures,const std::vector<Raw>&states,unsigned mutation=0){
    Outcome result{};const auto config=execution_config(fixtures[0]);const auto source=source_config();
    std::uint64_t now=1001500,uorb_generation=0;gpenmpc_rfly_slim::SnapshotExecutor<>store(source,config,{clock_now,&now});
    cw::AnchorStore<2>anchors;cw::ContextBinding binding(config);gpenmpc_rfly_wire::IngressDispatch dispatch({42,191,1,1,3,5000});
    ingress::Receiver receiver;SnapshotTicket first_ticket{};bool ok=true;unsigned frame_index=0;
    auto emit=[&](old::Fragment f,std::uint64_t stamp){
        if(mutation==3&&frame_index==7)f.payload[0]=0x70; // unknown next context
        if(mutation==4&&frame_index==4)f.payload[0]=0x41; // context in numeric fragment middle
        if(mutation==5&&frame_index==1)f.payload[0]=0x21; // numeric in context fragment middle
        mavlink_message_t message{},buffer{},parsed{};mavlink_status_t tx{},parser{},status{};tx.current_tx_seq=std::uint8_t(240+frame_index);
        mavlink_msg_tunnel_pack_status(42,191,&tx,&message,1,1,ingress::payload_type,f.length,f.payload);
        std::uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};const auto n=mavlink_msg_to_send_buffer(bytes,&message);++result.frames;result.wire_bytes+=n;
        unsigned good=0;for(unsigned j=0;j<n;++j)good+=mavlink_frame_char_buffer(&buffer,&parser,bytes[j],&parsed,&status)==MAVLINK_FRAMING_OK;
        if(good!=1)return false;
        receiver.receive(parsed,stamp,1,1,3);bool accepted=true;
        receiver.drain(stamp+10,[&](const ingress::Fields&fields){++uorb_generation;gpenmpc_full_inner_ingress_s m{};
            ingress::copy_to_topic(fields,receiver.counters(),m);
            if(mutation==1&&frame_index==7)return true; // actual published frame omitted by consumer, original counters remain
            auto a=old::from_topic(m,uorb_generation);
            if(mutation==2&&frame_index==8)++a.uorb_generation;
            if(mutation==6&&frame_index==7)a.ingress_first_fault=1;
            if(mutation==7&&frame_index==7)a.fields.timestamp=1000100;
            if(mutation==8&&frame_index==7)a.ingress_queue_overflows=1;
            if(mutation==9&&frame_index==7)++a.fields.source_system;
            accepted=dispatch.receive(a,stamp+10);return true;});++frame_index;return accepted;
    };
    for(unsigned round=0;round<2&&ok;++round){const auto sample=1000000+round*10000;now=sample+1500;const auto&f=fixtures[round];SnapshotTicket ticket{};
        if(!store.capture(raw(states[round],sample),std::uint32_t(states[round].generation),sample+300,id(),&topic,0,ticket)){ok=false;break;}
        if(!round)first_ticket=ticket;sw::Bytes down{};if(!anchors.record(store,ticket,config,now,down)){ok=false;break;}
        cw::Context c{};c.configuration_sha256=old::bytes_of(config.configuration_payload_sha256);c.reference_generation=round+1;c.outer_generation=1;
        c.reference_source_ticket=ticket;c.outer_source_ticket=first_ticket;
        c.reference_time={9000000000ULL+round*10000000ULL,9000100000ULL+round*10000000ULL,9400000000ULL+round*10000000ULL};
        c.outer_time={9000000000ULL,9000050000ULL,9400000000ULL};c.outer_payload={0.125,0.01,-0.02,0.03};
        for(unsigned j=0;j<3;++j){const double sign=j==2?-1:1;c.reference_ned[j]=sign*f.k.refP[j];c.reference_ned[j+3]=sign*f.k.refV[j];c.reference_ned[j+6]=sign*f.k.refA[j];}
        cw::Bytes encoded{};if(!cw::encode(c,encoded)||!dispatch.expect_context(c.reference_generation,sample+400)){ok=false;break;}
        for(unsigned j=0;j<3&&ok;++j){old::Fragment fragment{};cw::fragment(encoded,j,fragment);ok=emit(fragment,sample+500+j*100);}
        cw::Context decoded{};std::uint64_t original_context=0;
        if(ok)ok=dispatch.take_context(decoded,original_context,sample+750)&&binding.accept(decoded,anchors,original_context,sample+750);
        auto md=metadata(f,ticket);md.reference_generation=round+1;t::SlimMessage numerical{};const auto abi=gpenmpc_rfly_slim::encode(f.k);
        if(ok)ok=t::encode_slim(md,abi.data(),abi.size(),numerical)&&dispatch.expect_numerical(md,sample+750);
        for(unsigned j=0;j<4&&ok;++j){old::Fragment fragment{};t::fragment_slim(numerical,j,fragment);ok=emit(fragment,sample+800+j*100);}
        t::SlimCompleted done{};if(ok)ok=dispatch.take_numerical(done,sample+1150);
        gpenmpc_rfly_slim::Command command{};NumericalPrepared prepared{};
        if(ok)ok=t::bind_command(done,binding.reference(),binding.outer(),config,command)&&store.prepareNumericalOnly(command,prepared);
        if(ok){++result.steps;double error=0;for(unsigned j=0;j<61;++j)error=std::max(error,std::abs(prepared.execution.actual61[j]-f.expected[j]));
            float expected[16]{};const unsigned map[6]={4,0,3,5,1,2};for(unsigned j=0;j<6;++j)expected[map[j]]=static_cast<float>(f.expected[j+4]/32.145727009134916);
            ok=error<=1e-10&&std::memcmp(expected,prepared.execution.rfly_controls16.data(),sizeof expected)==0;
            NumericalReceipt receipt{};if(ok)ok=store.commitNumericalReceipt(ticket,ack(prepared,now),receipt);if(ok)++result.commits;
        }
    }
    result.passed=ok&&result.steps==2&&result.commits==2;result.fault=dispatch.fault();
    result.sequence=dispatch.last_reception_sequence();result.uorb=dispatch.last_uorb_generation();
    result.sticky=dispatch.failed()&&!dispatch.expect_context(3,1021000)&&dispatch.fault()==result.fault;
    return result;
}
int main(int argc,char**argv){try{if(argc!=3)return 2;const auto fixtures=read_kernel(argv[1]);const auto states=read_state(argv[2]);
    const auto positive=run_dispatch(fixtures,states);check(positive.passed,"two context3+numeric4 batches actual private context+Simulink+numeric ACK");
    check(positive.frames==14&&positive.sequence==14&&positive.uorb==14,"unaltered global actual reception/uORB sequence across context batches");
    for(unsigned mutation=1;mutation<=9;++mutation){const auto negative=run_dispatch(fixtures,states,mutation);
        check(!negative.passed&&negative.sticky,"global omission/gap/unknown/interleaving/fault/time/overflow/source rejected permanently");
        check(negative.steps<2,"second numerical call not executed after bad context/ingress");}
    std::ofstream out("INGRESS_DISPATCH_RESULT.json");out<<"{\"checks\":"<<checks<<",\"failed\":"<<failures<<",\"positive_actual_frames\":"<<positive.frames<<",\"actual_wire_bytes\":"<<positive.wire_bytes<<",\"actual_simulink_steps\":"<<positive.steps<<",\"mock_numerical_ack_commits\":"<<positive.commits<<",\"negative_scenarios\":9,\"protocol\":\"CONTIGUOUS_CONTEXT3_THEN_NUMERIC4_NO_MID_BATCH_INTERLEAVING\",\"sequence_renumbered\":false,\"live_clock_binding\":false,\"board_access\":0}\n";
    std::cout<<"checks="<<checks<<" failed="<<failures<<" frames="<<positive.frames<<" steps="<<positive.steps<<" commits="<<positive.commits<<" bytes="<<positive.wire_bytes<<'\n';return failures?1:0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
