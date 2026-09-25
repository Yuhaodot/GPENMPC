#include "SlimArgumentTransport.hpp"
#include <sstream>
#include "../px4_full_inner/argument_transport/CanonicalArgumentTransport.hpp"
#include <uORB/topics/gpenmpc_full_inner_ingress.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <stdexcept>
using namespace gpenmpc_rfly_state_execution;
namespace {
unsigned checks=0,failures=0;int topic=0;
void check(bool b,const char*n){++checks;if(!b){++failures;std::cerr<<"FAIL:"<<n<<'\n';}}
void read(std::ifstream&f,void*p,std::size_t n){f.read(static_cast<char*>(p),static_cast<std::streamsize>(n));if(!f)throw std::runtime_error("truncated fixture");}
std::uint64_t little(std::ifstream&f,unsigned n){std::uint64_t x=0;for(unsigned j=0;j<n;++j){std::uint8_t b=0;read(f,&b,1);x|=std::uint64_t(b)<<(8*j);}return x;}
Digest parse(const char*s){if(std::strlen(s)!=64)throw std::runtime_error("bad digest length");Digest d{};for(unsigned j=0;j<64;++j){unsigned v=0;
    if(s[j]>='0'&&s[j]<='9')v=unsigned(s[j]-'0');else if(s[j]>='A'&&s[j]<='F')v=unsigned(s[j]-'A'+10);else if(s[j]>='a'&&s[j]<='f')v=unsigned(s[j]-'a'+10);else throw std::runtime_error("bad digest");d[j/8]=(d[j/8]<<4)|v;}return d;}
struct Fixture{gpenmpc_portable::Array<std::uint8_t,829>abi{};Digest digest{};KernelArguments k{};gpenmpc_portable::Array<double,61>expected{};};
std::vector<Fixture> read_kernel(const char*path){std::ifstream f(path,std::ios::binary);if(!f)throw std::runtime_error("kernel fixture missing");
    std::uint8_t b[4]{};read(f,b,4);unsigned n=0;for(auto v:b)n=(n<<8)|v;if(n!=76)throw std::runtime_error("kernel denominator not76");std::vector<Fixture>a(n);
    for(auto&r:a){read(f,r.abi.data(),r.abi.size());for(auto&w:r.digest){read(f,b,4);for(auto v:b)w=(w<<8)|v;}
        gpenmpc_kernel_abi::DecodeFailure error{};if(!gpenmpc_kernel_abi::decode(r.abi.data(),r.abi.size(),r.digest,r.k,error))throw std::runtime_error("fixture decode");
        gpenmpc_portable::Array<std::uint8_t,61*8>x{};read(f,x.data(),x.size());gpenmpc_kernel_abi::Reader reader(x.data());reader.reals(r.expected);std::uint8_t valid=0;read(f,&valid,1);if(valid!=1)throw std::runtime_error("invalid oracle");}
    if(f.peek()!=std::char_traits<char>::eof())throw std::runtime_error("trailing kernel bytes");return a;}
struct Raw{std::uint64_t generation=0;gpenmpc_portable::Array<double,13>ned{},mapped{};};
std::vector<Raw> read_state(const char*path){std::ifstream f(path,std::ios::binary);if(!f||little(f,4)!=76)throw std::runtime_error("state denominator not76");std::vector<Raw>a(76);
    for(auto&r:a){r.generation=little(f,8);for(auto*p:{&r.ned,&r.mapped})for(auto&v:*p){const auto b=little(f,8);std::memcpy(&v,&b,8);}}
    if(f.peek()!=std::char_traits<char>::eof())throw std::runtime_error("trailing state bytes");return a;}
std::uint64_t clock_now(void*p)noexcept{return *static_cast<std::uint64_t*>(p);}
Identity id(){return{0x1122334455667788ULL,42,1,1};} // HOST fixture identity, not queried board.
gpenmpc_odometry::Configuration source_config(){gpenmpc_odometry::Configuration c{};c.identity=id();c.vehicle_odometry_topic=&topic;
    c.coordinates=gpenmpc_odometry::Coordinates::ExplicitTranslatedNed;c.sample_max_age_us=5000;return c;}
gpenmpc_rfly_execution::Configuration execution_config(const Fixture&f){gpenmpc_rfly_execution::Configuration c{};c.identity=id();
    c.limits={5000,400000,400000,4000,PublicationPath::DirectCanonicalMotors};c.coordinates=gpenmpc_rfly_execution::Coordinates::StateAndReferenceAreTaskLocalNed;
    c.approved_parameters=f.k.parameters;c.approved_parameter_sha256=gpenmpc_rfly_execution::parameter_sha256(f.k.parameters);
    c.configuration_payload_sha256=gpenmpc_rfly_execution::kCanonicalConfigurationSha;c.kernel_source_sha256=gpenmpc_rfly_execution::kGeneratedArmSourceSha;
    c.matlab_extraction_source_sha256=gpenmpc_rfly_execution::kMatlabExtractionSha;c.wrapper_matlab_source_sha256=gpenmpc_rfly_execution::kWrapperMatlabSourceSha;
    c.encoding=gpenmpc_rfly_execution::RflyEncoding::OfficialHIL16CtrlsNorm;return c;}
vehicle_odometry_s raw(const Raw&r,std::uint64_t t=1000000){vehicle_odometry_s s{};s.timestamp_sample=t;s.timestamp=t+200;s.pose_frame=s.velocity_frame=1;
    for(unsigned j=0;j<3;++j){s.position[j]=static_cast<float>(r.ned[j]);s.velocity[j]=static_cast<float>(r.ned[j+3]);s.angular_velocity[j]=static_cast<float>(r.ned[j+10]);}
    for(unsigned j=0;j<4;++j)s.q[j]=static_cast<float>(r.ned[j+6]);s.position_variance[0]=std::numeric_limits<float>::quiet_NaN();return s;}
NumericalCommand command(const Fixture&f,SnapshotTicket t,std::uint64_t sample=1000000){NumericalCommand c{};
    c.snapshot_ticket=t;c.arguments=f.abi;c.argument_sha256=f.digest;c.configuration_sha256=gpenmpc_rfly_execution::kCanonicalConfigurationSha;
    c.matlab_source_sha256=gpenmpc_rfly_execution::kMatlabExtractionSha;c.generated_source_sha256=gpenmpc_rfly_execution::kGeneratedArmSourceSha;
    c.wrapper_matlab_source_sha256=gpenmpc_rfly_execution::kWrapperMatlabSourceSha;
    auto&r=c.reference;auto&o=c.outer;r.identity=o.identity=id();r.generation=1;r.outer_generation=1;r.timestamp_us=sample+200;r.board_rx_us=sample+400;r.valid_until_us=sample+300000;
    r.p={f.k.refP[0],f.k.refP[1],-f.k.refP[2]};r.v={f.k.refV[0],f.k.refV[1],-f.k.refV[2]};r.a={f.k.refA[0],f.k.refA[1],-f.k.refA[2]};
    o.generation=1;o.based_on_sample_generation=f.k.augmentation_state_generation;o.based_on_timestamp_sample_us=sample;
    o.board_rx_us=sample+350;o.valid_until_us=sample+300000;o.payload_sha256=f.digest;c.original_board_ingress_us=sample+400;return c;}
gpenmpc_rfly_execution::BackendRflyAck ack(const NumericalPrepared&p,std::uint64_t now=1000800){return {p.execution.token,true,p.execution.token.output_generation,now,p.execution.rfly_controls16,p.execution.encoding,
    p.execution.generated_c_source_sha256,p.execution.wrapper_matlab_source_sha256};}

gpenmpc_rfly_slim::Command slim_command(const Fixture&f,SnapshotTicket ticket){
    const auto legacy=command(f,ticket);gpenmpc_rfly_slim::Command c{};
    c.snapshot_ticket=ticket;c.arguments=gpenmpc_rfly_slim::encode(f.k);c.argument_sha256=gpenmpc_rfly_slim::digest(c.arguments);
    c.configuration_sha256=legacy.configuration_sha256;c.matlab_source_sha256=legacy.matlab_source_sha256;
    c.generated_source_sha256=legacy.generated_source_sha256;c.wrapper_matlab_source_sha256=legacy.wrapper_matlab_source_sha256;
    c.reference=legacy.reference;c.outer=legacy.outer;c.original_board_ingress_us=legacy.original_board_ingress_us;return c;
}
}

namespace t=gpenmpc_slim_transport;
namespace old=gpenmpc_argument_transport;
namespace ingress=gpenmpc_ingress;
struct WireResult {
    t::SlimCompleted done{};old::Fault fault{old::Fault::None};
    bool accepted=false,sticky=false;unsigned packets=0,crc_rejects=0,bytes=0;
    ingress::Result first_ingress_fault{ingress::Result::Accepted};std::uint64_t first_ingress_fault_hrt=0;
};
old::Metadata metadata(const Fixture&f,const SnapshotTicket&ticket){
    old::Metadata m{};m.command_generation=f.k.augmentation_state_generation;
    m.reference_generation=m.outer_generation=1;m.snapshot_ticket=ticket;
    m.configuration_sha256=old::bytes_of(gpenmpc_rfly_execution::kCanonicalConfigurationSha);return m;
}
void repair_message_sha(t::SlimMessage&m){
    const auto sha=old::digest(m.bytes,old::binding_size+t::slim_abi_size);
    old::copy_bytes(sha.data(),32,m.bytes+old::binding_size+t::slim_abi_size);
}
WireResult transport(const gpenmpc_rfly_slim::Bytes&abi,old::Metadata expected,unsigned mutation=0,
                     const std::vector<std::vector<std::uint8_t>>*matlab_packets=nullptr){
    WireResult result{};auto sent=expected;
    if(mutation==1)sent.snapshot_ticket[0]^=1;
    if(mutation==2)sent.configuration_sha256[0]^=1;
    t::SlimMessage message{};if(!t::encode_slim(sent,abi.data(),abi.size(),message))return result;
    if(mutation==3)message.bytes[100]^=1;
    if(mutation==4){message.bytes[old::binding_size+124]=2;repair_message_sha(message);}
    if(mutation==5){gpenmpc_rfly_slim::Writer w(message.bytes+old::binding_size+4);w.real(std::numeric_limits<double>::quiet_NaN());repair_message_sha(message);}
    t::SlimAssembler assembler({42,191,1,1,3,5000});
    if(!assembler.expect(expected,1000400))return result;
    ingress::Receiver receiver;unsigned planned=mutation==6?3:4;
    if(mutation==16)planned=9;
    for(unsigned j=0;j<planned;++j){
        old::Fragment part{};unsigned index=j%4;
        if(mutation==7&&j==1)index=2;
        if(mutation==8&&j==1)index=0;
        if(!t::fragment_slim(message,index,part))return result;
        if(mutation==9&&j==0)part.payload[0]=0x10;
        if(mutation==10&&j==0)--part.length;
        if(mutation==11&&j==3)part.payload[127]=1;
        if(mutation==12&&j==0)part.payload[8]^=1;
        mavlink_message_t frame{},buffer{},parsed{};mavlink_status_t tx{},parser{},report{};
        tx.current_tx_seq=matlab_packets?(*matlab_packets)[j][4]:std::uint8_t(253+j);
        mavlink_msg_tunnel_pack_status(mutation==13?43:42,191,&tx,&frame,mutation==14&&j==0?2:1,1,ingress::payload_type,part.length,part.payload);
        std::uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};
        const auto n=mavlink_msg_to_send_buffer(bytes,&frame);result.bytes+=n;++result.packets;
        if(matlab_packets){
            const auto&actual=(*matlab_packets)[j];
            check(actual.size()==n&&std::memcmp(actual.data(),bytes,n)==0,"actual MATLAB serializer equals independent generated MAVLink bytes");
            if(actual.size()!=n||std::memcmp(actual.data(),bytes,n)!=0)return result;
            std::memcpy(bytes,actual.data(),n); // parse actual MATLAB bytes
        }
        if(mutation==15&&j==0)bytes[n-1]^=1;
        unsigned good=0,bad=0;
        for(unsigned b=0;b<n;++b){const auto state=mavlink_frame_char_buffer(&buffer,&parser,bytes[b],&parsed,&report);good+=state==MAVLINK_FRAMING_OK;bad+=state==MAVLINK_FRAMING_BAD_CRC;}
        result.crc_rejects+=bad;
        if(good!=1){if(mutation==15&&j==0&&bad==1)continue;return result;}
        std::uint64_t stamp=1000500+100*j;
        if(mutation==17&&j==1)stamp=1000499;
        if(mutation==18&&j==0)stamp=1002000;
        receiver.receive(parsed,stamp,1,1,3);
    }
    if(mutation==19)receiver.drain(1000850,[](const ingress::Fields&){return false;});
    std::uint64_t uorb=0;unsigned drained_fields=0;bool all=true;
    receiver.drain(1000900,[&](const ingress::Fields&fields){
        ++drained_fields;++uorb;
        if(mutation==20&&drained_fields==2)++uorb;
        if(mutation==21&&drained_fields==2)return true;
        gpenmpc_full_inner_ingress_s message_topic{};
        ingress::copy_to_topic(fields,receiver.counters(),message_topic);
        all=assembler.receive(old::from_topic(message_topic,uorb),mutation==22?1005401:1000900)&&all;return true;
    });
    if(mutation==6)assembler.tick(1005401);
    result.first_ingress_fault=receiver.counters().first_fault;
    result.first_ingress_fault_hrt=receiver.counters().first_fault_hrt;
    result.accepted=all&&assembler.take(result.done,1001000)&&result.done.complete&&!result.done.control_authority;
    if(mutation==23&&result.accepted){
        result.accepted=assembler.expect(expected,1001100); // same command cannot replay
    }
    result.fault=assembler.fault();
    if(assembler.failed()){
        const auto first=assembler.fault();const auto original=assembler.fault_hrt();
        result.sticky=!assembler.expect(expected,1001200)&&!assembler.tick(1001300)&&
            assembler.fault()==first&&assembler.fault_hrt()==original&&!assembler.ready();
    }
    return result;
}
unsigned old_serialized_bytes(const Fixture&f,const old::Metadata&m){
    old::Message message{};if(!old::encode(m,f.abi.data(),f.abi.size(),message))throw std::runtime_error("old schema fixture encode");
    unsigned count=0;for(unsigned j=0;j<8;++j){old::Fragment part{};old::fragment(message,j,part);
        mavlink_message_t frame{};mavlink_status_t tx{};tx.current_tx_seq=std::uint8_t(253+j);
        mavlink_msg_tunnel_pack_status(42,191,&tx,&frame,1,1,ingress::payload_type,part.length,part.payload);
        std::uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};count+=mavlink_msg_to_send_buffer(bytes,&frame);
    }return count;
}
std::uint64_t big(std::ifstream&f,unsigned count){std::uint64_t value=0;for(unsigned j=0;j<count;++j){std::uint8_t b=0;read(f,&b,1);value=(value<<8)|b;}return value;}
int main(int argc,char**argv){try{
    if(argc!=7&&argc!=8)throw std::runtime_error("76 ABI/raw fixtures and actual config/science/generated/wrapper SHA; optional actual MATLAB packet file");
    if(std::ifstream("HOST_SLIM_TRANSPORT_RESULT.json").good())throw std::runtime_error("do not overwrite result");
    check(parse(argv[3])==gpenmpc_rfly_execution::kCanonicalConfigurationSha&&parse(argv[4])==gpenmpc_rfly_execution::kMatlabExtractionSha&&
          parse(argv[5])==gpenmpc_rfly_execution::kGeneratedArmSourceSha&&parse(argv[6])==gpenmpc_rfly_execution::kWrapperMatlabSourceSha,"actual separate source identities");
    const auto fixtures=read_kernel(argv[1]);const auto states=read_state(argv[2]);
    std::ifstream matlab;const bool matlab_tested=argc==8;
    if(matlab_tested){matlab.open(argv[7],std::ios::binary);if(!matlab||big(matlab,4)!=0x52534D32||big(matlab,4)!=76)throw std::runtime_error("actual MATLAB packet fixture header");}
    std::ofstream bindings("SLIM_TRANSPORT_METADATA.bin",std::ios::binary);
    const std::uint8_t count_bytes[4]={0,0,0,76};bindings.write(reinterpret_cast<const char*>(count_bytes),4);
    unsigned prepared_count=0,committed_count=0,exact16=0,positive_packets=0,positive_bytes=0,old_bytes=0,min_bytes=10000,max_bytes=0;
    double max61=0;std::uint64_t actual_calls=0;std::ofstream rows("SLIM_TRANSPORT_ROWS.csv");
    rows<<"row,raw_generation,schema2_packets,schema2_serialized_bytes,schema1_serialized_bytes,max_error61,official16_exact\n";
    for(unsigned row=0;row<76;++row){
        const auto&f=fixtures[row];const auto&r=states[row];const auto cfg=execution_config(f);std::uint64_t now=1000300;
        gpenmpc_rfly_slim::SnapshotExecutor<> executor(source_config(),cfg,{clock_now,&now});SnapshotTicket ticket{};
        check(executor.capture(raw(r),unsigned(r.generation),now,id(),&topic,0,ticket),"real float32 private source capture");
        const auto baseline=slim_command(f,ticket);const auto meta=metadata(f,ticket);
        t::SlimMessage expected_message{};t::encode_slim(meta,baseline.arguments.data(),261,expected_message);
        bindings.write(reinterpret_cast<const char*>(expected_message.bytes),88);
        std::vector<std::vector<std::uint8_t>> matlab_packets;
        if(matlab_tested){
            std::uint8_t actual_message[381]{};read(matlab,actual_message,381);
            check(std::memcmp(actual_message,expected_message.bytes,381)==0,"MATLAB all381 assembled bytes equal independent C++ encoding");
            for(unsigned j=0;j<4;++j){const auto n=big(matlab,2);if(n<12||n>MAVLINK_MAX_PACKET_LEN)throw std::runtime_error("MATLAB packet length");
                matlab_packets.emplace_back(std::size_t(n));read(matlab,matlab_packets.back().data(),std::size_t(n));}
        }
        const auto wire=transport(baseline.arguments,meta,0,matlab_tested?&matlab_packets:nullptr);check(wire.accepted,"pack parse ingress four fragments complete");
        check(wire.packets==4&&wire.crc_rejects==0,"exactly four actual MAVLink2 parsed frames");
        positive_packets+=wire.packets;positive_bytes+=wire.bytes;min_bytes=std::min(min_bytes,wire.bytes);max_bytes=std::max(max_bytes,wire.bytes);
        const auto previous_bytes=old_serialized_bytes(f,meta);old_bytes+=previous_bytes;
        check(wire.done.trace.original_hrt[0]==1000500&&wire.done.trace.original_hrt[3]==1000800&&
              wire.done.trace.completed_hrt==1000900&&wire.done.trace.mavlink_sequence[3]==(matlab_tested?matlab_packets[3][4]:0),"original HRT and actual MAVLink sequence retained");
        check(std::memcmp(wire.done.abi,baseline.arguments.data(),261)==0,"all261 payload bytes exact through real parser");
        gpenmpc_rfly_slim::Command received{};
        check(t::bind_command(wire.done,baseline.reference,baseline.outer,cfg,received),"already admitted reference and outer metadata bind");
        check(received.original_board_ingress_us==1000800&&received.snapshot_ticket==ticket,"no HOST clock or ticket substitution");
        now=1001100;NumericalPrepared p{};const bool ok=executor.prepareNumericalOnly(received,p);
        check(ok&&executor.kernel_calls()==1,"actual Simulink step exactly once");double error=0;bool equal16=false;
        if(ok){++prepared_count;
            for(unsigned j=0;j<61;++j)error=std::max(error,std::abs(p.execution.actual61[j]-f.expected[j]));
            max61=std::max(max61,error);check(error<=1e-10,"inherited61 numerical tolerance");
            gpenmpc_portable::Array<float,16> expected{};constexpr unsigned map[6]={4,0,3,5,1,2};
            for(unsigned j=0;j<6;++j)expected[map[j]]=static_cast<float>(f.expected[j+4]/32.145727009134916);
            equal16=std::memcmp(expected.data(),p.execution.rfly_controls16.data(),64)==0;
            check(equal16,"official16 direct float32 bits retained");if(equal16)++exact16;
            now=1001500;NumericalReceipt receipt{};
            const bool commit=executor.commitNumericalReceipt(ticket,ack(p,1001400),receipt);
            check(commit&&!receipt.board_authority_proven&&!receipt.actual_output_consumption_proven,"MOCK ACK is not downstream consumption");
            if(commit)++committed_count;
            check(!executor.prepareNumericalOnly(received,p)&&executor.kernel_calls()==1,"same source not relabelled or executed twice");
        }
        actual_calls+=executor.kernel_calls();
        rows<<row+1<<','<<r.generation<<','<<wire.packets<<','<<wire.bytes<<','<<previous_bytes<<','<<std::setprecision(17)<<error<<','<<equal16<<'\n';
    }
    const auto&f=fixtures[0];const auto&r=states[0];unsigned negative_cases=0,negative_packets=0;
    const old::Fault expected_faults[]={old::Fault::None,old::Fault::Metadata,old::Fault::Metadata,old::Fault::Integrity,
        old::Fault::AbiInvalid,old::Fault::AbiInvalid,old::Fault::Expired,old::Fault::Index,old::Fault::Index,
        old::Fault::UnknownSchema,old::Fault::Length,old::Fault::Padding,old::Fault::Interleaved,old::Fault::Source,
        old::Fault::IngressFault,old::Fault::Index,old::Fault::IngressFault,old::Fault::Timestamp,old::Fault::Timestamp,
        old::Fault::IngressFault,old::Fault::UorbGap,old::Fault::SequenceGap,old::Fault::Expired,old::Fault::Replay};
    for(unsigned mutation=1;mutation<=23;++mutation){
        std::uint64_t now=1000300;const auto cfg=execution_config(f);
        gpenmpc_rfly_slim::SnapshotExecutor<> executor(source_config(),cfg,{clock_now,&now});SnapshotTicket ticket{};
        check(executor.capture(raw(r),unsigned(r.generation),now,id(),&topic,0,ticket),"negative private source capture");
        const auto baseline=slim_command(f,ticket);const auto wire=transport(baseline.arguments,metadata(f,ticket),mutation);
        negative_packets+=wire.packets;gpenmpc_rfly_slim::Command received{};NumericalPrepared p{};
        const bool stepped=wire.accepted&&t::bind_command(wire.done,baseline.reference,baseline.outer,cfg,received)&&executor.prepareNumericalOnly(received,p);
        check(!stepped&&executor.kernel_calls()==0,"wire negative cannot reach actual kernel");
        if(wire.fault!=expected_faults[mutation])std::cerr<<"mutation "<<mutation<<" actual fault "<<unsigned(wire.fault)<<" expected "<<unsigned(expected_faults[mutation])<<'\n';
        check(wire.fault==expected_faults[mutation]&&wire.sticky,"exact transport first fault remains latched");
        if(mutation==15)check(wire.crc_rejects==1,"actual MAVLink bad CRC rejection");
        if(mutation==14)check(wire.first_ingress_fault==ingress::Result::WrongTarget&&wire.first_ingress_fault_hrt==1000500,"original wrong-target fault not washed by later good packets");
        if(mutation==16)check(wire.first_ingress_fault==ingress::Result::QueueFull&&wire.first_ingress_fault_hrt==1001300,"original queue overflow fault HRT retained");
        if(mutation==19)check(wire.first_ingress_fault==ingress::Result::PublicationFailure&&wire.first_ingress_fault_hrt==1000850,"original publication failure retained on later drain");
        ++negative_cases;
    }
    // Semantically invalid source generations remain uint64 on wire and must
    // fail only against the real private snapshot, before the numerical step.
    for(unsigned mutation=0;mutation<5;++mutation){
        std::uint64_t now=1000300;const auto cfg=execution_config(f);
        gpenmpc_rfly_slim::SnapshotExecutor<> executor(source_config(),cfg,{clock_now,&now});SnapshotTicket ticket{};
        check(executor.capture(raw(r),unsigned(r.generation),now,id(),&topic,0,ticket),"semantic negative capture");
        auto baseline=slim_command(f,ticket);
        if(mutation<2){gpenmpc_rfly_slim::Writer w(baseline.arguments.data()+245+8*mutation);w.u64(r.generation+1);}
        const auto wire=transport(baseline.arguments,metadata(f,ticket));check(wire.accepted,"valid framing not mistaken for source admission");
        negative_packets+=wire.packets;
        if(mutation==2)++baseline.reference.identity.uid;
        if(mutation==3)++baseline.reference.generation;
        gpenmpc_rfly_slim::Command received{};const bool bound=t::bind_command(wire.done,baseline.reference,baseline.outer,cfg,received);
        now=mutation==4?1005001:1001100;NumericalPrepared p{};
        const bool stepped=bound&&executor.prepareNumericalOnly(received,p);
        check(!stepped&&executor.kernel_calls()==0,"generation identity context or stale snapshot rejects pre-kernel");
        ++negative_cases;
    }
    // Unchanged schema1 must reject schema2; no dual-schema reinterpretation.
    {old::Metadata meta{};meta.command_generation=meta.reference_generation=meta.outer_generation=1;
        meta.snapshot_ticket[0]=1;meta.configuration_sha256[0]=1;old::Assembler legacy({42,191,1,1,3,5000});
        check(legacy.expect(meta,1000400),"legacy expectation");
        t::SlimMessage message{};const auto abi=gpenmpc_rfly_slim::encode(f.k);t::encode_slim(meta,abi.data(),261,message);
        old::Fragment part{};t::fragment_slim(message,0,part);old::Arrival a{};
        a.fields.timestamp=1000500;a.fields.reception_sequence=a.uorb_generation=1;a.fields.source_system=42;a.fields.source_component=191;
        a.fields.target_system=a.fields.target_component=1;a.fields.receiver_instance=3;a.fields.payload_type=ingress::payload_type;
        a.fields.payload_length=part.length;a.fields.wire_payload_length=133;old::copy_bytes(part.payload,128,a.fields.payload);
        check(!legacy.receive(a,1000600)&&legacy.fault()==old::Fault::UnknownSchema,"old schema1 unchanged and rejects schema2");
    }
    check(prepared_count==76&&committed_count==76&&exact16==76&&actual_calls==76&&positive_packets==304,"full chain fixed denominator");
    std::ostringstream report;report<<std::setprecision(17)<<"{\"checks\":"<<checks<<",\"failed\":"<<failures
        <<",\"schema\":2,\"message_bytes\":381,\"numerical_abi_bytes\":261,\"fragments_per_command\":4"
        <<",\"positive_fixture_rows\":76,\"actual_kernel_calls\":"<<actual_calls<<",\"prepared61_rows\":"<<prepared_count
        <<",\"official16_bit_exact_rows\":"<<exact16<<",\"mock_committed\":"<<committed_count<<",\"max_error61\":"<<max61
        <<",\"positive_actual_pack_parse_packets\":"<<positive_packets<<",\"positive_serialized_bytes\":"<<positive_bytes
        <<",\"min_serialized_bytes_per_command\":"<<min_bytes<<",\"max_serialized_bytes_per_command\":"<<max_bytes
        <<",\"old_schema1_same_input_serialized_bytes\":"<<old_bytes<<",\"unsigned_untrimmed_bound_bytes_per_command\":485"
        <<",\"negative_scenarios\":"<<negative_cases<<",\"negative_pack_parse_packets\":"<<negative_packets
        <<",\"old_schema1_modified\":false,\"matlab_encoder_connected\":false,\"matlab_encoder_function_tested\":"<<(matlab_tested?"true":"false")
        <<",\"matlab_packets_byte_crosschecked\":"<<(matlab_tested?304:0)<<",\"actual_uorb_publications\":0"
        <<",\"ingress_topic_publication_in_test\":\"HOST captured generated struct\",\"backend_ack\":\"MOCK\""
        <<",\"target_executed\":false}\n";
    std::cout<<report.str();std::ofstream out("HOST_SLIM_TRANSPORT_RESULT.json");out<<report.str();
    if(matlab_tested&&matlab.peek()!=std::char_traits<char>::eof())throw std::runtime_error("trailing MATLAB fixture bytes");
    if(!out.good()||!rows.good()||!bindings.good())throw std::runtime_error("result write failure");return failures?1:0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
