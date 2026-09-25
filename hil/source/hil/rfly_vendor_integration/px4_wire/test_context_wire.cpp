// Context-wire tests reuse the schema-2 fixture reader and types.
// The included command-transport test entry is not executed.
#define main command_transport_fixture_main
#include "../test_slim_argument_transport.cpp"
#undef main
#include "RflyContextAssembler.hpp"
namespace cw=gpenmpc_context_wire;
namespace sw=gpenmpc_snapshot_wire;
struct Transfer{cw::Context context{};std::uint64_t ingress{};unsigned bytes{},frames{};bool ok{},sticky{};};
std::vector<std::uint8_t> packet(const old::Fragment&part,std::uint8_t seq,std::uint8_t system=42,std::uint8_t component=191){
    mavlink_message_t msg{};mavlink_status_t tx{};tx.current_tx_seq=seq;
    mavlink_msg_tunnel_pack_status(system,component,&tx,&msg,system==42?1:42,system==42?1:191,ingress::payload_type,part.length,part.payload);
    std::vector<std::uint8_t>b(MAVLINK_MAX_PACKET_LEN);b.resize(mavlink_msg_to_send_buffer(b.data(),&msg));return b;
}
bool parse_packet(const std::vector<std::uint8_t>&b,mavlink_message_t&out){mavlink_message_t buffer{};mavlink_status_t parser{},status{};unsigned good=0;
    for(auto v:b)good+=mavlink_frame_char_buffer(&buffer,&parser,v,&out,&status)==MAVLINK_FRAMING_OK;return good==1;}
Transfer transfer(const cw::Bytes&bytes,unsigned mutation=0,const std::vector<std::vector<std::uint8_t>>*matlab=nullptr){
    Transfer result{};cw::Assembler assembler({42,191,1,1,3,5000});assembler.expect(1,1000400);ingress::Receiver receiver;
    for(unsigned j=0;j<(mutation==1?2:3);++j){old::Fragment f{};cw::fragment(bytes,j,f);if(mutation==2&&j==1)f.payload[0]=64;
        if(mutation==3&&j==1)f.payload[17]^=1;
        if(mutation==4&&j==0)f.payload[0]=32;
        auto b=packet(f,std::uint8_t(j));if(matlab){check((*matlab)[j]==b,"MATLAB context full frame bit exact");b=(*matlab)[j];}
        mavlink_message_t parsed{};if(!parse_packet(b,parsed))return result;
        receiver.receive(parsed,1000500+j*100,1,1,3);result.bytes+=unsigned(b.size());++result.frames;
    }
    std::uint64_t generation=0;receiver.drain(1000850,[&](const ingress::Fields&f){gpenmpc_full_inner_ingress_s m{};
        ingress::copy_to_topic(f,receiver.counters(),m);++generation;auto a=old::from_topic(m,generation);
        if(mutation==5&&generation==2)++a.uorb_generation;
        if(mutation==6)a.ingress_first_fault=1;
        assembler.receive(a,1000900);return true;});
    result.ok=assembler.take(result.context,result.ingress,1000900);
    if(mutation==1)assembler.tick(1005401);
    const auto first=assembler.fault();result.sticky=assembler.failed()&&!assembler.expect(2,1005500)&&assembler.fault()==first;return result;
}
cw::Context context(const Fixture&f,SnapshotTicket ticket){cw::Context c{};
    c.configuration_sha256=old::bytes_of(gpenmpc_rfly_execution::kCanonicalConfigurationSha);c.reference_generation=c.outer_generation=1;
    c.reference_source_ticket=c.outer_source_ticket=ticket;
    c.reference_time={9000000000ULL,9000100000ULL,9400000000ULL};c.outer_time={9000000000ULL,9000050000ULL,9400000000ULL};
    for(unsigned j=0;j<3;++j){double sign=j==2?-1:1;c.reference_ned[j]=sign*f.k.refP[j];c.reference_ned[j+3]=sign*f.k.refV[j];c.reference_ned[j+6]=sign*f.k.refA[j];}
    c.outer_payload={0.125,0.01,-0.02,0.03}; // Codec fixture.
    return c;
}
int main(int argc,char**argv){try{
    if(argc<3)return 2;auto fixtures=read_kernel(argv[1]);auto states=read_state(argv[2]);const auto&f=fixtures[0];
    std::uint64_t now=1000900;const auto config=execution_config(f);const auto source=source_config();
    gpenmpc_rfly_slim::SnapshotExecutor<>store(source,config,{clock_now,&now});SnapshotTicket ticket{};
    check(store.capture(raw(states[0]),std::uint32_t(states[0].generation),1000300,id(),&topic,0,ticket),"private snapshot capture");
    cw::AnchorStore<2>anchors;sw::Bytes snapshot{};check(anchors.record(store,ticket,config,now,snapshot),"export records exact private anchor");
    const auto sent=context(f,ticket);cw::Bytes bytes{};check(cw::encode(sent,bytes),"context encode");
    std::vector<std::vector<std::uint8_t>>matlab;
    if(argc==4){std::ifstream in(argv[3],std::ios::binary);if(!in)throw std::runtime_error("MATLAB context missing");
        for(unsigned j=0;j<3;++j){std::uint8_t length[2]{};read(in,length,2);matlab.emplace_back((unsigned(length[0])<<8)|length[1]);read(in,matlab.back().data(),matlab.back().size());}}
    const auto wire=transfer(bytes,0,matlab.empty()?nullptr:&matlab);check(wire.ok&&wire.ingress==1000700,"generated MAVLink parse actual ingress preserves original HRT");
    cw::ContextBinding binding(config);check(binding.accept(wire.context,anchors,wire.ingress,now),"private source anchored context accepted");
    check(binding.reference().timestamp_us==wire.ingress&&binding.reference().valid_until_us==1400000,"board binding event not mapped HOST creation; conservative source expiry");
    check(binding.outer().based_on_timestamp_sample_us==1000000&&binding.outer().based_on_sample_generation==states[0].generation,"actual outer private source lineage");
    const auto old_reference=binding.reference();const auto old_outer=binding.outer();
    check(binding.accept(sent,anchors,1000800,now)&&binding.reference().board_rx_us==old_reference.board_rx_us&&binding.outer().valid_until_us==old_outer.valid_until_us,"repeat exact generation does not renew or refresh");
    // Actual numerical command uses the decoded/admitted context, not fixture-created Reference/Outer.
    t::SlimMessage slim_message{};const auto abi=gpenmpc_rfly_slim::encode(f.k);const auto md=metadata(f,ticket);
    check(t::encode_slim(md,abi.data(),abi.size(),slim_message),"unchanged schema2 encode");const auto numerical=transport(abi,md);
    gpenmpc_rfly_slim::Command cmd{};check(t::bind_command(numerical.done,binding.reference(),binding.outer(),config,cmd),"wire context binds unchanged schema2");
    NumericalPrepared prepared{};check(store.prepareNumericalOnly(cmd,prepared)&&store.kernel_calls()==1,"context to actual Simulink single step");
    double error=0;for(unsigned j=0;j<61;++j)error=std::max(error,std::abs(prepared.execution.actual61[j]-f.expected[j]));
    check(error<=1e-10,"context controller outputs within tolerance");
    const unsigned mapping[6]={4,0,3,5,1,2};bool bits=true;
    float expected16[16]{};for(unsigned j=0;j<6;++j)expected16[mapping[j]]=static_cast<float>(f.expected[j+4]/32.145727009134916);
    for(unsigned j=0;j<16;++j)bits=bits&&std::memcmp(&expected16[j],&prepared.execution.rfly_controls16[j],4)==0;
    check(bits,"context actuator encoding bit parity");
    for(unsigned mode=1;mode<=6;++mode){const auto bad=transfer(bytes,mode);check(!bad.ok&&bad.sticky,"lost/reordered/corrupt/schema/uorb/firstfault fail closed");}
    for(unsigned mode=0;mode<11;++mode){cw::ContextBinding negative(config);check(negative.accept(sent,anchors,1000700,1000900),"negative baseline");auto bad=sent;
        if(mode==0)++bad.reference_time.creation_ns;if(mode==1)++bad.outer_time.expiry_ns;
        if(mode==2)bad.reference_source_ticket={};if(mode==3)bad.configuration_sha256[0]^=1;
        if(mode==4){bad.reference_generation=2;bad.reference_time.creation_ns++;}
        if(mode==5){bad.reference_generation=2;bad.reference_time.creation_ns++;}
        if(mode==6)bad.outer_payload[2]+=1;
        if(mode==7){bad.reference_generation=2;bad.reference_time.creation_ns=bad.reference_time.source_receipt_ns-1;}
        if(mode==8){bad.reference_generation=2;bad.outer_generation=2;bad.reference_source_ticket[3]^=1;bad.outer_source_ticket=bad.reference_source_ticket;}
        const auto ingress_time=mode==4?1000700:(mode==5?1000600:1000800);
        const auto test_now=mode==9?1400001:1000900;
        if(mode==10){bad.reference_time.expiry_ns=UINT64_MAX;bad.outer_time.expiry_ns=UINT64_MAX;}
        const bool rejected=!negative.accept(bad,anchors,ingress_time,test_now);
        check(rejected,"mutation/missing source/config/repeated or reversed ingress/stale/host-expiry mutation rejected");
        const auto failure=negative.failure();check(!negative.accept(sent,anchors,1000900,1000900)&&negative.failure()==failure,"context first fault cannot wash");
    }
    // A second valid source event tests .30s reference renewal.
    std::uint64_t later=1300900;gpenmpc_rfly_slim::SnapshotExecutor<>later_store(source,config,{clock_now,&later});SnapshotTicket later_ticket{};
    check(later_store.capture(raw(states[0],1300000),std::uint32_t(states[0].generation+30),1300300,id(),&topic,0,later_ticket),"isolated later source event");
    sw::Bytes later_export{};check(anchors.record(later_store,later_ticket,config,later,later_export),"retain later source alongside held outer basis");
    cw::ContextBinding renewal(config);check(renewal.accept(sent,anchors,1000700,1000900),"renewal baseline");auto renewed=sent;
    renewed.reference_generation=2;renewed.reference_source_ticket=later_ticket;renewed.reference_time={9300000000ULL,9300100000ULL,9700000000ULL};
    check(renewal.accept(renewed,anchors,1300700,later)&&renewal.outer().board_rx_us==1000700&&renewal.outer().valid_until_us==1400000,".30s new reference holds original outer expiry");
    renewed.reference_generation=3;renewed.outer_generation=2;renewed.outer_source_ticket=later_ticket;renewed.outer_time={9300000000ULL,9300050000ULL,9700000000ULL};renewed.reference_time.creation_ns++;
    check(renewal.accept(renewed,anchors,1300800,later)&&renewal.outer().valid_until_us==1700000,"new outer generation +new private source renews at .30s");
    cw::ContextBinding missing(config);cw::AnchorStore<1>empty;check(!missing.accept(sent,empty,1000700,now),"all HOST SHA/ticket claims without private observation rejected");
    cw::ContextBinding expired(config);check(!expired.accept(sent,anchors,1400001,1400001),"old anchor on first receive rejected not arrival+TTL");
    std::ofstream out("PRIVATE_SNAPSHOT_AND_CONTEXT.bin",std::ios::binary);out.write(reinterpret_cast<const char*>(snapshot.data()),snapshot.size());out.write(reinterpret_cast<const char*>(bytes.data()),bytes.size());
    unsigned snapshot_wire_bytes=0;for(unsigned j=0;j<3;++j){old::Fragment part{};check(sw::fragment(snapshot,j,part),"snapshot fragment");const auto b=packet(part,std::uint8_t(j),1,1);
        mavlink_message_t p{};check(parse_packet(b,p),"board snapshot actual MAVLink parser");snapshot_wire_bytes+=unsigned(b.size());const std::uint8_t n[2]={std::uint8_t(b.size()>>8),std::uint8_t(b.size())};out.write(reinterpret_cast<const char*>(n),2);out.write(reinterpret_cast<const char*>(b.data()),b.size());}out.close();
    std::ofstream json("CONTEXT_WIRE_RESULT.json");json<<"{\"checks\":"<<checks<<",\"failed\":"<<failures<<",\"actual_simulink_steps\":1,\"context_bytes\":316,\"snapshot_bytes\":246,\"context_mavlink_bytes\":"<<wire.bytes<<",\"snapshot_mavlink_bytes\":"<<snapshot_wire_bytes<<",\"numerical_mavlink_bytes\":"<<numerical.bytes<<",\"combined_per_exchange_wire_bytes\":"<<wire.bytes+snapshot_wire_bytes+numerical.bytes<<",\"live_clock_binding\":false,\"board_access\":0}\n";
    std::cout<<"checks="<<checks<<" failed="<<failures<<" context="<<wire.bytes<<" snapshot="<<snapshot_wire_bytes<<" numerical="<<numerical.bytes<<'\n';return failures?1:0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
