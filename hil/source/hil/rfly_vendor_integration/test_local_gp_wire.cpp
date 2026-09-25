// Executes original complete generated C and exact GP256 DLL. Publication,
// source and identity metadata below are explicit HOST fixtures, not devices.
#define wmain retained_full_abi_test_main
#include "full_inner_abi/test_full_inner_abi.cpp"
#undef wmain
#include "px4_wire/CanonicalLocalGpWire.hpp"
#include <common/mavlink.h>
#include <limits>
namespace gw=gpenmpc_local_gp_wire;
static unsigned wire_gp_calls=0,packets=0;
template<std::size_t N>static bool mav_roundtrip(const gpenmpc_portable::Array<std::uint8_t,N>&b,
    gpenmpc_portable::Array<std::uint8_t,N>&joined){
    joined={};unsigned offset=0;
    for(unsigned k=0;k<gw::fragment_count;++k){gw::Fragment f{};if(!gw::fragment(b,k,f))return false;
        mavlink_message_t sent{},got{};mavlink_status_t state{};mavlink_tunnel_t t{};
        const bool request=N==gw::request_bytes;
        // Exact existing experimental TUNNEL payload type. Direction is a
        // disclosed fixture; actual registered live owner is still separate.
        mavlink_msg_tunnel_pack(request?1:255,request?1:190,&sent,
            request?255:1,request?190:1,42002,f.length,f.payload);
        uint8_t raw[MAVLINK_MAX_PACKET_LEN]{};const auto len=mavlink_msg_to_send_buffer(raw,&sent);unsigned accepted=0;
        for(unsigned j=0;j<len;++j)accepted+=mavlink_parse_char(MAVLINK_COMM_1,raw[j],&got,&state)==MAVLINK_FRAMING_OK;
        if(accepted!=1||got.msgid!=MAVLINK_MSG_ID_TUNNEL)return false;
        mavlink_msg_tunnel_decode(&got,&t);++packets;
        if(got.sysid!=(request?1:255)||got.compid!=(request?1:190)||
           t.target_system!=(request?255:1)||t.target_component!=(request?190:1)||
           t.payload_type!=42002||t.payload_length!=f.length||std::memcmp(t.payload,f.payload,f.length))return false;
        const unsigned n=f.length-9;if(offset+n>N)return false;
        std::memcpy(joined.data()+offset,t.payload+9,n);offset+=n;
    }return offset==N&&same(b.data(),joined.data(),N);
}
int wmain(int argc,wchar_t**argv){
    if(argc!=4||!load(argv[1],argv[2]))return 2;
    HMODULE dll=LoadLibraryExW(argv[3],nullptr,LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR|LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
    if(!dll)return 3;using Predict=int(*)(const double*,double*);
    const auto predict=reinterpret_cast<Predict>(GetProcAddress(dll,"gpenmpc_gp256_predict"));if(!predict)return 3;
    Owner owner;check(owner.create(),"real complete canonical owner created");
    for(unsigned i=0;i<60;++i){gpenmpc_full_inner_candidate c{};gpenmpc_full_inner_state s{};
        check(prepare(owner,i,c)==RFI_OK,"actual full C produces original per-inner query");
        check(commit(owner,receipts(c))==RFI_OK,"actual full numerical joint install, MOCK publication");
        check(gpenmpc_full_inner_copy_state(owner.handle,&s)==RFI_OK,"actual pending requirement read");
        if(s.prediction_required){
            gw::Request q{};q.identity={0x1122334455667788ULL,42,1,1};
            q.source_timestamp_ns=c.original_tags2[0];q.source_generation=c.original_tags2[1];q.output_generation=i+1;
            q.original_publication_us=q.source_timestamp_ns/1000+400;q.publication_valid_until_us=q.original_publication_us+1000;
            q.configuration_sha256=gw::canonical_configuration();q.gp_model_sha256=gw::canonical_model();std::memcpy(q.request19,c.request19,152);
            gw::RequestBytes b{},rb{};gw::Request decoded{};
            check(gw::encode(q,b)&&mav_roundtrip(b,rb)&&gw::decode(rb,decoded),"actual MAVLink query three-fragment roundtrip");
            check(same(decoded.request19,c.request19,152)&&decoded.source_timestamp_ns==q.source_timestamp_ns&&
                decoded.source_generation==q.source_generation,"binary64 GP inputs and uint64 tags retained");
            gw::Reply reply{};reply.identity=decoded.identity;reply.source_timestamp_ns=decoded.source_timestamp_ns;
            reply.source_generation=decoded.source_generation;reply.output_generation=decoded.output_generation;
            reply.gp_model_sha256=decoded.gp_model_sha256;reply.original_request_sha256=gw::digest(rb.data(),rb.size());
            check(predict(decoded.request19+1,reply.result18)==0&&same(reply.result18,original[i].gp,144),"original actual GP256 bit exact after wire");++wire_gp_calls;
            gw::ReplyBytes sb{},joined{};gw::Reply answer{};
            check(gw::encode(reply,sb)&&mav_roundtrip(sb,joined)&&gw::decode(joined,answer)&&gw::matches(q,answer),"actual MAVLink reply bound to entire original request");
            const uint64_t tags[2]={answer.source_timestamp_ns,answer.source_generation};
            check(gpenmpc_full_inner_fill_gp(owner.handle,tags,answer.result18)==RFI_OK,"actual original pending filled from decoded GP, never held at outer rate");
            for(unsigned byte=0;byte<gw::request_bytes;++byte){auto bad=b;bad[byte]^=1;gw::Request z{};check(!gw::decode(bad,z),"each query byte corruption rejects");}
            for(unsigned byte=0;byte<gw::reply_bytes;++byte){auto bad=sb;bad[byte]^=1;gw::Reply z{};check(!gw::decode(bad,z),"each reply byte corruption rejects");}
            for(unsigned kind=0;kind<9;++kind){auto bad=answer;
                if(kind==0)++bad.identity.uid;if(kind==1)++bad.identity.boot_generation;if(kind==2)++bad.identity.system;
                if(kind==3)++bad.identity.component;if(kind==4)++bad.source_timestamp_ns;if(kind==5)++bad.source_generation;
                if(kind==6)++bad.output_generation;if(kind==7)bad.original_request_sha256[0]^=1;if(kind==8)bad.gp_model_sha256[0]^=1;
                check(!gw::matches(q,bad),"well-formed but wrong session/tag/request/model rejects");
            }
            auto hard=answer;hard.result18[14]=1;
            check(gw::encode(hard,sb)&&gw::decode(sb,hard)&&hard.result18[14]==1,"numerical hard-invalid is preserved, not rewritten into transport status");
            auto high=q;high.source_timestamp_ns=UINT64_MAX-1000;high.source_generation=9007199254740993ULL;
            high.original_publication_us=high.source_timestamp_ns/1000+1;high.publication_valid_until_us=high.original_publication_us+1;
            high.request19[1]=-0.0;
            check(gw::encode(high,b)&&gw::decode(b,decoded)&&decoded.source_timestamp_ns==high.source_timestamp_ns&&
                decoded.source_generation==high.source_generation&&same(&decoded.request19[1],&high.request19[1],8),"high uint64 and signed zero lossless");
            high=q;high.request19[0]=0;check(!gw::encode(high,b),"GP query requires a nonzero request flag");
            high=q;high.request19[3]=std::numeric_limits<double>::quiet_NaN();check(!gw::encode(high,b),"nonfinite query rejected");
            high=q;high.configuration_sha256[0]^=1;check(!gw::encode(high,b),"wrong canonical configuration rejected");
        }
        check(gpenmpc_full_inner_copy_state(owner.handle,&s)==RFI_OK&&same(s.pending70,original[i].pending,560)&&
            same(s.state64,original[i].state,512),"all actual next pending/inner states unchanged after wire GP");++rows_done;
    }
    check(rows_done==60&&wire_gp_calls==59&&packets==354,"complete 60-inner/59-GP original chain through 354 real MAVLink packets");
    FreeLibrary(dll);std::printf("{\"checks\":%u,\"failed\":%u,\"actual_inner_rows\":%u,\"actual_GP_calls\":%u,\"actual_MAVLink_packets\":%u,\"COM\":0,\"board\":0,\"source_publication_and_session_mock\":true,\"live_scheduler_or_authority_proven\":false}\n",checks,failed,rows_done,wire_gp_calls,packets);
    return failed?1:0;
}
