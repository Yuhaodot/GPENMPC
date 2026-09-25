#include "SlimSnapshotExecutor.hpp"
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
int main(int argc,char**argv){try{
    if(argc!=7)throw std::runtime_error("ABI,raw-state,config,science,generated,wrapper SHA required");
    check(parse(argv[3])==gpenmpc_rfly_execution::kCanonicalConfigurationSha&&parse(argv[4])==gpenmpc_rfly_execution::kMatlabExtractionSha&&
          parse(argv[5])==gpenmpc_rfly_execution::kGeneratedArmSourceSha&&parse(argv[6])==gpenmpc_rfly_execution::kWrapperMatlabSourceSha,"actual external source identities");
    const auto fixtures=read_kernel(argv[1]);const auto states=read_state(argv[2]);
    unsigned exact829=0,nonquaternion_exact=0,dynamic_exact=0,valid61=0,exact16=0,committed=0;
    double max61=0,max_q_difference=0;std::ostringstream differing_rows;
    for(unsigned i=0;i<76;++i){
        const auto&f=fixtures[i];const auto&r=states[i];std::uint64_t now=1000300;
        auto cfg=execution_config(f);RflySnapshotBoundExecutor<>full_core(source_config(),cfg,{clock_now,&now});
        SnapshotTicket ticket{};check(full_core.capture(raw(r),unsigned(r.generation),now,id(),&topic,0,ticket),"original private snapshot capture");
        gpenmpc_consumption::KernelArguments reconstructed{};gpenmpc_rfly_slim::Failure error{};
        const auto slim=gpenmpc_rfly_slim::encode(f.k);
        check(gpenmpc_rfly_slim::decode(slim.data(),slim.size(),gpenmpc_rfly_slim::digest(slim),*full_core.snapshot(ticket),cfg,reconstructed,error),"decode only store-owned state and startup parameters");
        const auto expanded=gpenmpc_rfly_slim::expand_bytes(reconstructed);
        const bool same=std::memcmp(expanded.data(),f.abi.data(),829)==0;
        if(same)++exact829;else{
            for(unsigned b=0;b<829;++b)if(expanded[b]!=f.abi[b]){
                if(differing_rows.tellp()>0) differing_rows<<',';
                differing_rows<<"{\"row\":"<<i+1<<",\"first_byte_offset\":"<<b<<'}';break;
            }
        }
        // Original mapped_input_matches already allows q representation error
        // <=1e-12 (MATLAB norm versus C sum/sqrt). Report raw bit equality as a
        // separate observation; neither fixture nor normalization is changed.
        bool other_bits=true;for(unsigned b=0;b<829;++b)
            if((b<52||b>=84)&&expanded[b]!=f.abi[b])other_bits=false;
        check(other_bits,"every non-quaternion ABI byte unchanged");if(other_bits)++nonquaternion_exact;
        double q_difference=0;for(unsigned j=6;j<10;++j)q_difference=std::max(q_difference,std::abs(reconstructed.x[j]-f.k.x[j]));
        max_q_difference=std::max(max_q_difference,q_difference);check(q_difference<=1e-12,"inherited quaternion representation tolerance");
        const auto roundtrip=gpenmpc_rfly_slim::encode(reconstructed);
        const bool same_dynamic=std::memcmp(slim.data(),roundtrip.data(),261)==0;
        check(same_dynamic,"all30 dynamic doubles and metadata bit exact");if(same_dynamic)++dynamic_exact;
        gpenmpc_rfly_slim::SnapshotExecutor<>thin(source_config(),cfg,{clock_now,&now});SnapshotTicket thin_ticket{};
        check(thin.capture(raw(r),unsigned(r.generation),now,id(),&topic,0,thin_ticket),"slim wrapper private capture");
        auto c=slim_command(f,thin_ticket);now=1000400;NumericalPrepared prepared{};
        const bool ready=thin.prepareNumericalOnly(c,prepared);check(ready&&thin.kernel_calls()==1,"slim step exactly once");
        if(ready){
            double error61=0;for(unsigned j=0;j<61;++j)error61=std::max(error61,std::abs(prepared.execution.actual61[j]-f.expected[j]));
            max61=std::max(max61,error61);check(error61<=1e-10,"all61 inherited numerical gate");if(error61<=1e-10)++valid61;
            gpenmpc_portable::Array<float,16>expected{};constexpr unsigned map[6]={4,0,3,5,1,2};
            for(unsigned j=0;j<6;++j)expected[map[j]]=float(f.expected[j+4]/32.145727009134916);
            const bool same16=std::memcmp(expected.data(),prepared.execution.rfly_controls16.data(),64)==0;
            check(same16,"official16 unquantized bits retained");if(same16)++exact16;
            now=1000900;NumericalReceipt receipt{};
            const bool ok=thin.commitNumericalReceipt(thin_ticket,ack(prepared),receipt);
            check(ok&&!receipt.board_authority_proven&&!receipt.actual_output_consumption_proven,"mock ACK only");if(ok)++committed;
            check(!thin.prepareNumericalOnly(c,prepared)&&thin.kernel_calls()==1,"same ticket cannot execute twice");
        }
    }
    const auto&f=fixtures[0];const auto&r=states[0];const unsigned before_negative=checks;
    std::uint64_t now=1000300;const auto cfg=execution_config(f);
    RflySnapshotBoundExecutor<>private_store(source_config(),cfg,{clock_now,&now});SnapshotTicket ticket{};
    check(private_store.capture(raw(r),unsigned(r.generation),now,id(),&topic,0,ticket),"negative fixture private capture");
    const auto*snapshot=private_store.snapshot(ticket);const auto valid_slim=gpenmpc_rfly_slim::encode(f.k);
    gpenmpc_consumption::KernelArguments decoded{};gpenmpc_rfly_slim::Failure why{};
    // Reject every incomplete payload length, including missing suffixes/fragments.
    for(unsigned length=0;length<261;++length){
        check(!gpenmpc_rfly_slim::decode(valid_slim.data(),length,gpenmpc_rfly_slim::digest(valid_slim),*snapshot,cfg,decoded,why)&&
              why==gpenmpc_rfly_slim::Failure::Length,"every incomplete payload rejected");
    }
    check(!gpenmpc_rfly_slim::decode(nullptr,261,gpenmpc_rfly_slim::digest(valid_slim),*snapshot,cfg,decoded,why)&&
          why==gpenmpc_rfly_slim::Failure::Length,"null payload rejected");
    auto changed=valid_slim;changed[0]='A';
    check(!gpenmpc_rfly_slim::decode(changed.data(),changed.size(),gpenmpc_rfly_slim::digest(changed),*snapshot,cfg,decoded,why)&&
          why==gpenmpc_rfly_slim::Failure::Magic,"different codec magic rejected");
    changed=valid_slim;changed[20]^=1;
    check(!gpenmpc_rfly_slim::decode(changed.data(),changed.size(),gpenmpc_rfly_slim::digest(valid_slim),*snapshot,cfg,decoded,why)&&
          why==gpenmpc_rfly_slim::Failure::DigestMismatch,"corrupt payload digest rejected");
    changed=valid_slim;changed[124]=2;
    check(!gpenmpc_rfly_slim::decode(changed.data(),changed.size(),gpenmpc_rfly_slim::digest(changed),*snapshot,cfg,decoded,why)&&
          why==gpenmpc_rfly_slim::Failure::Boolean,"nonboolean continuity rejected");
    for(unsigned field=0;field<30;++field){
        changed=valid_slim;const unsigned offset=field<15?4+8*field:125+8*(field-15);
        gpenmpc_rfly_slim::Writer writer(changed.data()+offset);writer.real(std::numeric_limits<double>::quiet_NaN());
        check(!gpenmpc_rfly_slim::decode(changed.data(),changed.size(),gpenmpc_rfly_slim::digest(changed),*snapshot,cfg,decoded,why)&&
              why==gpenmpc_rfly_slim::Failure::Nonfinite,"each dynamic double rejects nonfinite");
    }
    for(unsigned generation=0;generation<2;++generation){
        changed=valid_slim;gpenmpc_rfly_slim::Writer writer(changed.data()+245+8*generation);writer.u64(r.generation+1);
        check(!gpenmpc_rfly_slim::decode(changed.data(),changed.size(),gpenmpc_rfly_slim::digest(changed),*snapshot,cfg,decoded,why)&&
              why==gpenmpc_rfly_slim::Failure::Lineage,"each source generation bound to private snapshot");
    }
    // Startup configuration is the existing trusted approved_parameters object;
    // the codec does not accept a host replacement or invent a config authority.
    for(unsigned mutation=0;mutation<3;++mutation){
        auto wrong=cfg;if(mutation==0)wrong.approved_parameters.kp[0]+=1;
        if(mutation==1)wrong.configuration_payload_sha256[0]^=1;
        if(mutation==2)wrong.identity.system^=1;
        check(!gpenmpc_rfly_slim::decode(valid_slim.data(),261,gpenmpc_rfly_slim::digest(valid_slim),*snapshot,wrong,decoded,why)&&
              why==gpenmpc_rfly_slim::Failure::Configuration,"wrong startup configuration rejected");
    }
    for(unsigned mutation=0;mutation<11;++mutation){
        now=1000300;gpenmpc_rfly_slim::SnapshotExecutor<>thin(source_config(),cfg,{clock_now,&now});SnapshotTicket t{};
        check(thin.capture(raw(r),unsigned(r.generation),now,id(),&topic,0,t),"negative wrapper capture");
        auto c=slim_command(f,t);now=1000400;
        if(mutation==0)c.snapshot_ticket[0]^=1;
        if(mutation==1)c.configuration_sha256[0]^=1;
        if(mutation==2)c.generated_source_sha256[0]^=1;
        if(mutation==3)c.wrapper_matlab_source_sha256[0]^=1;
        if(mutation==4)c.matlab_source_sha256[0]^=1;
        if(mutation==5)c.argument_sha256[0]^=1;
        if(mutation==6)now=1006000;
        if(mutation==7)c.original_board_ingress_us=now+1;
        if(mutation==8)c.reference.identity.system^=1;
        if(mutation==9){c.arguments[260]^=1;c.argument_sha256=gpenmpc_rfly_slim::digest(c.arguments);}
        if(mutation==10){gpenmpc_rfly_slim::Writer w(c.arguments.data()+4);w.real(f.k.refP[0]+1);c.argument_sha256=gpenmpc_rfly_slim::digest(c.arguments);}
        NumericalPrepared p{};check(!thin.prepareNumericalOnly(c,p)&&thin.kernel_calls()==0,"wrong identity/ticket/stale/illegal command rejected before kernel");
        c=slim_command(f,t);now=1000400;
        check(!thin.prepareNumericalOnly(c,p)&&thin.kernel_calls()==0,"failure stays latched without executing");
    }
    check(exact829==74&&nonquaternion_exact==76&&dynamic_exact==76&&valid61==76&&exact16==76&&committed==76,
          "explicit final denominators, including two retained quaternion bit differences");
    std::ostringstream report;report<<std::setprecision(17)
      <<"{\"checks\":"<<checks<<",\"failed\":"<<failures<<",\"negative_checks\":"<<checks-before_negative-1
      <<",\"fixture_rows\":76,\"slim_bytes\":261,\"old_abi_bytes\":829,\"removed_repeated_bytes\":568,\"retained_dynamic_doubles\":30"
      <<",\"exact829_rows\":"<<exact829<<",\"nonquaternion_abi_bit_exact_rows\":"<<nonquaternion_exact<<",\"slim_dynamic_bit_exact_rows\":"<<dynamic_exact
      <<",\"quaternion_difference_rows\":["<<differing_rows.str()<<"],\"max_quaternion_difference\":"<<max_q_difference
      <<",\"quaternion_tolerance_provenance\":\"inherited mapped_input_matches 1e-12\""
      <<",\"valid61_rows\":"<<valid61<<",\"official16_bit_exact_rows\":"<<exact16<<",\"mock_committed\":"<<committed
      <<",\"max_error61\":"<<max61
      <<",\"incomplete_payload_lengths_rejected\":261,\"new_fragment_assembler_tested\":false,\"wire_protocol_installed\":false,\"runtime_replaced\":false,\"ack\":\"HOST MOCK only\"}\n";
    std::ofstream result("HOST_SLIM_CODEC_RESULT.json");result<<report.str();result.close();
    if(!result)throw std::runtime_error("result write failed");std::cout<<report.str();
    return failures?1:0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
