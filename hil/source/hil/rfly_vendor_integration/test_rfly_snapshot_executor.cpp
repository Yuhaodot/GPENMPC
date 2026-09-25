#include "RflySnapshotBoundExecutor.hpp"
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
unsigned checks=0,failures=0,combined_messages=0,wire_packets=0;int topic=0,other_topic=0;
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
void rehash(NumericalCommand&c){CanonicalSha256 h;for(auto b:c.arguments)h.byte(b);c.argument_sha256=h.finish();}
void put_double(NumericalCommand&c,unsigned offset,double x){std::uint64_t b=0;std::memcpy(&b,&x,8);for(unsigned j=0;j<8;++j)c.arguments[offset+j]=std::uint8_t(b>>(56-8*j));rehash(c);}
void put_u64(NumericalCommand&c,unsigned offset,std::uint64_t x){for(unsigned j=0;j<8;++j)c.arguments[offset+j]=std::uint8_t(x>>(56-8*j));rehash(c);}
gpenmpc_rfly_execution::BackendRflyAck ack(const NumericalPrepared&p,std::uint64_t now=1000800){return {p.execution.token,true,p.execution.token.output_generation,now,p.execution.rfly_controls16,p.execution.encoding,
    p.execution.generated_c_source_sha256,p.execution.wrapper_matlab_source_sha256};}
// Real MAVLink pack/parser + real generated ingress message shape. Queue/uORB
// generations and clock are explicit HOST fixtures; no broker or socket runs.
bool through_transport(const Fixture&f,const SnapshotTicket&board_ticket,NumericalCommand&out,
                       bool wrong_ticket=false,bool missing_last=false){
    namespace t=gpenmpc_argument_transport;namespace i=gpenmpc_ingress;
    t::Metadata expected{};expected.command_generation=f.k.augmentation_state_generation;
    expected.reference_generation=expected.outer_generation=1;expected.snapshot_ticket=board_ticket;
    expected.configuration_sha256=t::bytes_of(gpenmpc_rfly_execution::kCanonicalConfigurationSha);
    t::Metadata sent=expected;if(wrong_ticket)sent.snapshot_ticket[0]^=1;
    t::Message message{};if(!t::encode(sent,f.abi.data(),f.abi.size(),message))return false;
    t::Assembler assembler({42,191,1,1,3,5000});if(!assembler.expect(expected,1000400))return false;
    i::Receiver ingress;std::uint64_t generation=0;
    for(unsigned j=0;j<(missing_last?7U:8U);++j){t::Fragment fragment{};if(!t::fragment(message,j,fragment))return false;
        mavlink_message_t frame{},buffer{},parsed{};mavlink_status_t tx{},status{},report{};tx.current_tx_seq=std::uint8_t(j);
        mavlink_msg_tunnel_pack_status(42,191,&tx,&frame,1,1,i::payload_type,fragment.length,fragment.payload);
        std::uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};const auto n=mavlink_msg_to_send_buffer(bytes,&frame);unsigned good=0,bad=0;
        for(unsigned k=0;k<n;++k){const auto state=mavlink_frame_char_buffer(&buffer,&status,bytes[k],&parsed,&report);good+=state==MAVLINK_FRAMING_OK;bad+=state==MAVLINK_FRAMING_BAD_CRC;}
        ++wire_packets;if(good!=1||bad||ingress.receive(parsed,1000500+100*j,1,1,3)!=i::Result::Accepted)return false;
    }
    bool accepted=true;const bool drained=ingress.drain(1001300,[&](const i::Fields&fields){
        gpenmpc_full_inner_ingress_s raw_topic{};i::copy_to_topic(fields,ingress.counters(),raw_topic);
        accepted=assembler.receive(t::from_topic(raw_topic,++generation),1001300)&&accepted;return true;});
    if(missing_last){assembler.tick(1005401);return false;}
    t::Completed done{};if(!drained||!accepted||!assembler.take(done,1001400)||!done.complete||done.control_authority)return false;
    if(done.metadata.snapshot_ticket!=board_ticket||std::memcmp(done.abi,f.abi.data(),829)!=0)return false;
    out=command(f,done.metadata.snapshot_ticket);
    for(unsigned j=0;j<829;++j)out.arguments[j]=done.abi[j];out.argument_sha256=t::words_of(done.abi_sha256);
    out.configuration_sha256=t::words_of(done.metadata.configuration_sha256);
    out.reference.generation=done.metadata.reference_generation;out.reference.outer_generation=done.metadata.outer_generation;
    out.outer.generation=done.metadata.outer_generation;
    out.original_board_ingress_us=done.trace.last_original_hrt; // no host clock substituted
    ++combined_messages;return true;
}
template<class Edit>void rejected_command(const Fixture&f,const Raw&r,const char*name,Edit edit,Failure reason){std::uint64_t now=1000300;
    RflySnapshotBoundExecutor<> e(source_config(),execution_config(f),{clock_now,&now});SnapshotTicket t{};check(e.capture(raw(r),unsigned(r.generation),now,id(),&topic,0,t),"negative capture");
    auto c=command(f,t);now=1000400;edit(c,now);NumericalPrepared out{};out.numerically_prepared=true;out.execution.actual61.fill(1);
    check(!e.prepareNumericalOnly(c,out)&&e.failure()==reason&&!out.numerically_prepared&&!out.board_authority_proven&&e.kernel_calls()==0,name);
    check(!e.prepareNumericalOnly(command(f,t),out),"rejection not reset by good packet");}
}
int main(int argc,char**argv){try{
    if(argc!=9)throw std::runtime_error("new ABI,state; old ABI,state; actual config,matlab,new ARM C,wrapper SHA required");
    if(std::ifstream("HOST_RFLY_KERNEL_RESULT.json").good()||std::ifstream("RFLY_RAW_RESULT.csv").good())throw std::runtime_error("refusing to overwrite prior adapter result");
    check(parse(argv[5])==gpenmpc_rfly_execution::kCanonicalConfigurationSha&&parse(argv[6])==gpenmpc_rfly_execution::kMatlabExtractionSha&&parse(argv[7])==gpenmpc_rfly_execution::kGeneratedArmSourceSha&&parse(argv[8])==gpenmpc_rfly_execution::kWrapperMatlabSourceSha,"external independent config/science/generated/wrapper identities");
    auto f=read_kernel(argv[1]);auto r=read_state(argv[2]);auto old=read_kernel(argv[3]);auto oldr=read_state(argv[4]);
    unsigned prepared=0,committed=0,oldmatched=0,oldrejected=0,official16_exact=0;double max_error=0;std::uint64_t calls=0;
    std::ofstream rows("RFLY_RAW_RESULT.csv");rows<<"row,raw_generation,prepared,committed,max_error61,old_host_arguments_accepted\n";
    for(unsigned j=0;j<76;++j){
        std::uint64_t now=1000300;RflySnapshotBoundExecutor<> e(source_config(),execution_config(f[j]),{clock_now,&now});SnapshotTicket t{};
        check(e.capture(raw(r[j]),unsigned(r[j].generation),now,id(),&topic,0,t),"capture actual float32 topic");
        const auto*s=e.snapshot(t);check(s&&s->estimator().generation==r[j].generation&&s->raw().timestamp_sample==1000000,"stored snapshot original generation/time");
        NumericalCommand c{};check(through_transport(f[j],t,c),"real captured ticket and829 via MAVLink ingress eight-fragment assembler");
        now=1001500;NumericalPrepared p{};const bool ok=e.prepareNumericalOnly(c,p);check(ok,"float32 MATLAB ABI accepted");
        double error=0;if(ok){++prepared;for(unsigned k=0;k<61;++k)error=std::max(error,std::abs(p.execution.actual61[k]-f[j].expected[k]));
            max_error=std::max(max_error,error);check(error<=1e-10,"all61 actual Simulink step agrees preserved MATLAB oracle");
            gpenmpc_portable::Array<float,16> expected16{};
            constexpr unsigned map[6]={4,0,3,5,1,2};
            for(unsigned channel=0;channel<6;++channel)expected16[map[channel]]=static_cast<float>(f[j].expected[4+channel]/32.145727009134916);
            const bool exact16=std::memcmp(expected16.data(),p.execution.rfly_controls16.data(),16*sizeof(float))==0;
            check(exact16,"official16 exact expected float bits without PWM quantization");if(exact16)++official16_exact;
            check(p.execution.token.kernel_source_sha256==gpenmpc_rfly_execution::kGeneratedArmSourceSha&&
                p.execution.wrapper_matlab_source_sha256==gpenmpc_rfly_execution::kWrapperMatlabSourceSha&&
                p.execution.matlab_extraction_source_sha256==gpenmpc_rfly_execution::kMatlabExtractionSha,"three separate source identities retained");}
        now=1001900;NumericalReceipt receipt{};bool done=false;if(ok){done=e.commitNumericalReceipt(t,ack(p,1001800),receipt);check(done&&!receipt.board_authority_proven&&!receipt.actual_output_consumption_proven,"mock ACK retains numerical receipt fields");if(done)++committed;}
        calls+=e.kernel_calls();
        now=1000300;RflySnapshotBoundExecutor<> prior(source_config(),execution_config(old[j]),{clock_now,&now});SnapshotTicket prior_t{};
        check(prior.capture(raw(oldr[j]),unsigned(oldr[j].generation),now,id(),&topic,0,prior_t),"old source cast actual message");now=1000400;NumericalPrepared prior_p{};
        const bool oldok=prior.prepareNumericalOnly(command(old[j],prior_t),prior_p);
        if(oldok)++oldmatched;else{++oldrejected;check(prior.failure()==Failure::Executor&&prior.executor_error()==gpenmpc_rfly_execution::Error::StateMapping&&prior.kernel_calls()==0,"inconsistent float-cast state rejected before kernel execution");}
        calls+=prior.kernel_calls();rows<<j+1<<','<<r[j].generation<<','<<ok<<','<<done<<','<<std::setprecision(17)<<error<<','<<oldok<<'\n';
    }
    check(prepared==76&&committed==76&&oldmatched==60&&oldrejected==16&&official16_exact==76,"snapshot inputs accepted and incompatible float-cast inputs rejected");
    const auto&fx=f[0];const auto&rx=r[0];
    for(unsigned mode=0;mode<3;++mode){std::uint64_t now=1000300;RflySnapshotBoundExecutor<>e(source_config(),execution_config(f[60]),{clock_now,&now});SnapshotTicket t{};
        e.capture(raw(r[60]),unsigned(r[60].generation),now,id(),&topic,0,t);NumericalCommand c{};
        const bool received=through_transport(f[60],t,c,mode==0,mode==1);NumericalPrepared p{};bool executed=false;
        if(received){now=1005001;executed=e.prepareNumericalOnly(c,p);}
        check(!executed&&e.kernel_calls()==0&&!p.numerically_prepared,"combined wrong ticket missing fragment or expired source never calls C");}
    rejected_command(fx,rx,"unknown ticket",[](auto&c,auto&){++c.snapshot_ticket[0];},Failure::Ticket);
    rejected_command(fx,rx,"payload SHA",[](auto&c,auto&){++c.arguments[40];},Failure::Abi);
    rejected_command(fx,rx,"x position mismatch",[](auto&c,auto&){put_double(c,4,9);},Failure::Executor);
    rejected_command(fx,rx,"x rate mismatch",[](auto&c,auto&){put_double(c,4+10*8,9);},Failure::Executor);
    rejected_command(fx,rx,"augmentation generation",[](auto&c,auto&){put_u64(c,813,2);},Failure::Lineage);
    rejected_command(fx,rx,"continuity generation",[](auto&c,auto&){put_u64(c,821,2);},Failure::Lineage);
    rejected_command(fx,rx,"source identity",[](auto&c,auto&){++c.generated_source_sha256[0];},Failure::SourceConfiguration);
    rejected_command(fx,rx,"new wrapper identity",[](auto&c,auto&){++c.wrapper_matlab_source_sha256[0];},Failure::SourceConfiguration);
    rejected_command(fx,rx,"old generated C identity cannot masquerade as new wrapper",[](auto&c,auto&){
        c.generated_source_sha256={0x2BB86D0A,0x7C32CB6C,0x505748DA,0xEA9A8B20,0x69CA172B,0x90E256A5,0x55345901,0xABB5A95A};
    },Failure::SourceConfiguration);
    rejected_command(fx,rx,"configuration identity",[](auto&c,auto&){++c.configuration_sha256[0];},Failure::SourceConfiguration);
    rejected_command(fx,rx,"host now cannot become sample",[](auto&c,auto&){c.original_board_ingress_us=2000000;},Failure::CommandTime);
    rejected_command(fx,rx,"expired stored source",[](auto&,auto&now){now=1005001;},Failure::Stale);
    rejected_command(fx,rx,"foreign reference identity",[](auto&c,auto&){++c.reference.identity.boot_generation;},Failure::Executor);
    rejected_command(fx,rx,"reference mismatch",[](auto&c,auto&){c.reference.p[0]+=1;},Failure::Executor);
    for(unsigned mode=0;mode<5;++mode){std::uint64_t now=1000300;RflySnapshotBoundExecutor<> e(source_config(),execution_config(fx),{clock_now,&now});SnapshotTicket t{};
        check(e.capture(raw(rx),1,now,id(),&topic,0,t),"source negative initial");auto raw2=raw(rx,1010000);now=1010300;auto identity=id();const void*source=&topic;unsigned generation=2;
        if(mode==0)generation=1;if(mode==1)raw2.timestamp_sample=1010001;if(mode==2)raw2.reset_counter=1;if(mode==3)source=&other_topic;if(mode==4)++identity.uid;
        check(!e.capture(raw2,generation,now,identity,source,0,t)&&e.failure()==Failure::Source&&e.kernel_calls()==0,"duplicate gap reset topic identity source negative");}
    {std::uint64_t now=1000300;RflySnapshotBoundExecutor<2>e(source_config(),execution_config(fx),{clock_now,&now});SnapshotTicket t{};
        check(e.capture(raw(rx),1,now,id(),&topic,0,t),"capacity first");now+=4000;check(e.capture(raw(rx,1004000),3,now,id(),&topic,0,t),"capacity second raw jump");now+=4000;
        check(!e.capture(raw(rx,1008000),4,now,id(),&topic,0,t)&&e.failure()==Failure::Capacity,"unconsumed snapshots cannot silently evict");}
    {std::uint64_t now=1000300;RflySnapshotBoundExecutor<> e(source_config(),execution_config(fx),{clock_now,&now});SnapshotTicket t{};
        e.capture(raw(rx),1,now,id(),&topic,0,t);const auto c=command(fx,t);now=1000400;NumericalPrepared p{};check(e.prepareNumericalOnly(c,p),"duplicate prepare first");
        check(!e.prepareNumericalOnly(c,p)&&e.failure()==Failure::Pending&&e.kernel_calls()==1,"cannot prepare twice pending");calls+=e.kernel_calls();}
    {std::uint64_t now=1000300;RflySnapshotBoundExecutor<> e(source_config(),execution_config(fx),{clock_now,&now});SnapshotTicket t{};
        e.capture(raw(rx),1,now,id(),&topic,0,t);const auto c=command(fx,t);now=1000400;NumericalPrepared p{};e.prepareNumericalOnly(c,p);now=1000900;NumericalReceipt receipt{};
        check(e.commitNumericalReceipt(t,ack(p),receipt),"consume first");check(!e.prepareNumericalOnly(c,p)&&e.failure()==Failure::AlreadyConsumed&&e.kernel_calls()==1,"old snapshot cannot count as new after consume");calls+=e.kernel_calls();}
    for(unsigned mode=0;mode<8;++mode){std::uint64_t now=1000300;RflySnapshotBoundExecutor<>e(source_config(),execution_config(fx),{clock_now,&now});SnapshotTicket t{};
        e.capture(raw(rx),1,now,id(),&topic,0,t);now=1000400;NumericalPrepared p{};e.prepareNumericalOnly(command(fx,t),p);auto a=ack(p);
        if(mode==0)++a.token.sample_generation;if(mode==1)a.published_control[0]+=.1F;if(mode==2)a.publish_succeeded=false;
        if(mode==3)a.published_control[15]=.1F;if(mode==4)a.encoding=gpenmpc_rfly_execution::RflyEncoding::Unspecified;
        if(mode==5)++a.generated_arm_source_sha256[0];if(mode==6)++a.wrapper_matlab_source_sha256[0];
        if(mode==7)++a.output_generation;
        now=1000900;NumericalReceipt receipt{};check(!e.commitNumericalReceipt(t,a,receipt)&&e.failure()==Failure::Commit&&!receipt.receipt_valid,"actual existing ACK rejection retained");calls+=e.kernel_calls();}
    {std::uint64_t now=1000300;auto c=source_config();RflySnapshotBoundExecutor<>e(c,execution_config(fx),{clock_now,&now});SnapshotTicket a{},b{};
        check(e.capture(raw(rx),1,now,id(),&topic,0,a),"digest first");const auto*first=e.snapshot(a);check(first!=nullptr,"stored immutable lookup");
        if(first){auto altered=*first;check(snapshot_digest(altered,1,1)!=a&&snapshot_digest(altered,0,2)!=a,"ticket binds instance and store ordinal");}
        c.task_origin_ned_m={1,2,3};RflySnapshotBoundExecutor<>different(c,execution_config(fx),{clock_now,&now});different.capture(raw(rx),1,now,id(),&topic,0,b);check(a!=b,"ticket binds explicit origin");}
    std::ostringstream summary;summary<<std::setprecision(17)<<"{\"status\":\""<<(failures?"FAIL":"PASS_HOST_SNAPSHOT_ACTUAL_SIMULINK_ARM_C_OFFICIAL16_BINDING")<<"\",\"checks\":"<<checks<<",\"failed\":"<<failures
        <<",\"new_float32_prepared\":"<<prepared<<",\"new_float32_mock_committed\":"<<committed<<",\"old_exact_rows\":"<<oldmatched<<",\"old_float32_mismatch_rejected\":"<<oldrejected
        <<",\"official16_bit_exact_rows\":"<<official16_exact<<",\"maximum_error61\":"<<max_error<<",\"actual_generated_c_calls\":"<<calls<<",\"combined_transport_messages\":"<<combined_messages<<",\"actual_mavlink_pack_parse_packets\":"<<wire_packets<<",\"sizeof_store4\":"<<sizeof(RflySnapshotBoundExecutor<4>)
        <<",\"sizeof_ticket\":"<<sizeof(SnapshotTicket)<<",\"sizeof_command\":"<<sizeof(NumericalCommand)<<",\"sizeof_snapshot\":"<<sizeof(gpenmpc_odometry::Snapshot)
        <<",\"board_authority\":false,\"actual_publications\":0,\"backend_ACK\":\"MOCK\"}\n";
    std::cout<<summary.str();std::ofstream report("HOST_RFLY_KERNEL_RESULT.json");report<<summary.str();
    if(!report.good())throw std::runtime_error("result output failure");
    return failures?1:0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
