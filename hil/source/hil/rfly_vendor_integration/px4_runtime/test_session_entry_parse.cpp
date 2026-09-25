// Section garbage collection omits the unexecuted owner/device/Module paths.
#define GPENMPC_BOARD_UID 0x1122334455667788ULL
#include "CanonicalSessionEntry.cpp"
#include <fstream>
#include <string>

namespace {
unsigned checks{},failed{};
void check(bool value,const char *name){++checks;if(!value){++failed;std::fprintf(stderr,"FAIL %s\n",name);}}
}
int main(int argc,char **argv)
{
    if(argc!=2)return 2;
    std::uint64_t nonce{};std::uint8_t number{};double coordinate{};SessionDigest sha{};
    check(hex64("0123456789aBcDeF",nonce)&&nonce==0x0123456789abcdefULL,"hex64 exact");
    check(!hex64("0123456789abcdeg",nonce),"hex64 illegal");
    const std::string nonzero="0000000000000000000000000000000000000000000000000000000000000001";
    check(digest(nonzero.c_str(),sha)&&sha[7]==1,"digest nonzero");
    check(!digest(nullptr,sha),"digest null");check(!digest("",sha),"digest empty");
    check(!digest(std::string(63,'a').c_str(),sha),"digest short");
    check(!digest(std::string(65,'a').c_str(),sha),"digest long");
    check(!digest(std::string(64,'0').c_str(),sha),"digest zero");
    check(!digest(std::string(64,'z').c_str(),sha),"digest invalid");
    for(const char *v:{"1","255"})check(byte(v,number),"byte accepted boundary");
    for(const char *v:{"","0","256","-1","1x","999999999999999999999999999999"})check(!byte(v,number),"byte rejected");
    check(!byte(nullptr,number),"byte null");
    for(const char *v:{"0","-0","1.125","-1.25e-20"})check(real(v,coordinate),"finite coordinate");
    for(const char *v:{"","nan","inf","-inf","1e9999","1.0x"})check(!real(v,coordinate),"coordinate rejected");
    check(!real(nullptr,coordinate),"coordinate null");
    gpenmpc_feedback_wire::Bytes bytes{},encoded{};CommittedFeedback decoded{};
    std::ifstream fixture(argv[1],std::ios::binary);char header[8]{};fixture.read(header,8);fixture.read(reinterpret_cast<char *>(bytes.data()),bytes.size());
    check(bool(fixture)&&std::memcmp(header,"RFC5",4)==0,"original production fixture");
    check(gpenmpc_feedback_wire::decode(bytes,decoded),"actual production fixture decoded");
    check(retained_evidence("failed",encoded)==-EBUSY,"unclosed observation refused");
    observation.state=ApplicationState::Retired;
    check(retained_evidence("failed",encoded)==-ENODATA,"empty failed refused");
    check(retained_evidence("interrupted",encoded)==-ENODATA,"empty interrupted refused");
    check(retained_evidence("arbitrary",encoded)==-EINVAL,"unknown slot refused");
    check(retained_evidence(nullptr,encoded)==-EINVAL,"null slot refused");
    observation.failed_raw_feedback=decoded;
    check(retained_evidence("failed",encoded)==0&&encoded==bytes,"retained raw exact RFC1");
    check(retained_evidence("failed",encoded)==0&&encoded==bytes,"read only repeat exact");
    check(observation.failed_raw_feedback.commit_completed_us==decoded.commit_completed_us&&
        observation.failed_raw_feedback.original_valid_until_us==decoded.original_valid_until_us&&
        observation.failed_raw_feedback.disposition==decoded.disposition,"raw fields never relabelled");
    observation.interrupted_raw_feedback=decoded;
    check(retained_evidence("interrupted",encoded)==0&&encoded==bytes,"independent interrupted exact");
    observation.failed_raw_feedback.actual61[0]=NAN;
    check(retained_evidence("failed",encoded)==-ERANGE,"invalid raw not repaired");
    check(retained_evidence("interrupted",encoded)==0&&encoded==bytes,"other real evidence preserved");
    observation.failed_raw_feedback=decoded;
    observation.failed_raw_feedback.disposition=FeedbackDisposition::Revoked;
    observation.failed_raw_feedback.original_valid_until_us=decoded.commit_completed_us-1;
    check(retained_evidence("failed",encoded)==-ERANGE,"late commit not forced into RFC1");
    check(print_retained_raw("failed",observation.failed_raw_feedback)==0,"late original raw diagnostic emitted");
    check(observation.failed_raw_feedback.original_valid_until_us+1==decoded.commit_completed_us&&
        observation.failed_raw_feedback.commit_completed_us==decoded.commit_completed_us,"late times unchanged by print");
    std::printf("{\"checks\":%u,\"failed\":%u,\"actual_entry_helpers\":true,\"owner_or_module_executed\":false}\n",checks,failed);
    return failed?1:0;
}
