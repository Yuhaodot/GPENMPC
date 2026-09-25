// Process retained fixtures with RflyUdpRaw as the IO owner.
// Preserve source and publication timestamps.
// mdlOutputs reads the previous queue; mdlUpdate receives and predicts.
// Both callbacks use fixed memory without allocation or file/workspace IO.
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <new>
#include <initializer_list>
#include "../rfly_vendor_integration/px4_wire/CanonicalLocalGpWire.hpp"
#include "../rfly_vendor_integration/application_integration/build_fmuv6c/mavlink/common/mavlink.h"
#include "canonical_gp_standalone_api.h"

namespace rawgp {
namespace gw=gpenmpc_local_gp_wire;
constexpr unsigned N=59, PAIR=596, MAX_UPDATES=10000, RAW_CAPACITY=100000;
constexpr unsigned MAX_TX=1+N*3, MAX_DATAGRAM=300;
struct RawRecord { std::uint32_t offset{},length{}; double sim_s{}; std::int64_t qpc{},processed_qpc{}; };
struct Frame { unsigned char bytes[MAX_DATAGRAM]{}; std::uint16_t length{};
    std::uint16_t query{}; std::int16_t fragment{-1}; double queued_s{},output_s{}; };
struct Datagram { unsigned char bytes[MAX_DATAGRAM]{}; std::uint16_t length{},frame_count{},first_frame{};
    std::uint16_t offsets[2]{},lengths[2]{}; double output_s{}; std::int64_t exposed_qpc{}; };
struct Query { gw::RequestBytes request{}; gw::ReplyBytes reply{}; double result[18]{};
    double completed_s{}; std::int64_t first_rx_qpc{},gp_begin_qpc{},gp_end_qpc{}; };
enum Error : unsigned { OK=0, CONFIG=1, TIME=2, CAPACITY=3, FRAME=4, CRC=5,
    SOURCE=6, SEQUENCE=7, SCHEMA=8, FRAGMENT=9, OVERLAP=10, FIXTURE=11,
    REQUEST=12, GP_CALL=13, REPLY=14, WALL_BOUND=15, INCOMPLETE=16 };
inline std::int64_t ticks() { LARGE_INTEGER t{};QueryPerformanceCounter(&t);return t.QuadPart; }
inline bool same_identity(const gw::Identity&a,const gw::Identity&b) { return a==b; }
inline bool pair_fits(unsigned a,unsigned b) { return a>5&&b>5&&a<=MAX_DATAGRAM&&b<=MAX_DATAGRAM-a; }
struct Core {
    unsigned char pairs[PAIR*N]{};
    gw::Request expected[N]{};
    RawRecord raw[MAX_UPDATES]{}; unsigned char raw_bytes[RAW_CAPACITY]{};
    Frame queue[3]{},transmitted[MAX_TX]{}; Query queries[N]{};
    Datagram datagrams[MAX_TX]{},pending_output{};
    gw::RequestBytes assembling{};
    unsigned updates{},raw_used{},rx_frames{},gp_calls{},completed{},tx_count{},queue_count{},queue_head{},fragment_index{};
    unsigned sentinel_count{},maximum_aggregate{},error{},error_update{};
    unsigned datagram_count{}; bool reply_pairs{},output_prepared{};
    double last_sim{-1},error_sim{-1}; std::int64_t error_qpc{},frequency{},begin_qpc{},first_rx_qpc{};
    std::uint8_t expected_sequence{}; mavlink_status_t tx_status{};
    const char *error_message{""}; bool initialized{};
    bool fail(Error e,const char *message,double t) {
        if(!error){error=e;error_message=message;error_update=updates;error_sim=t;error_qpc=ticks();}
        return false;
    }
    bool initialize(const unsigned char *input,std::size_t count,bool combine_reply_pairs=false) {
        if(count!=sizeof(pairs))return fail(CONFIG,"Require exactly 59 original request/reply pairs.",-1);
        reply_pairs=combine_reply_pairs;
        std::memcpy(pairs,input,count);LARGE_INTEGER f{};
        if(!QueryPerformanceFrequency(&f)||f.QuadPart<=0)return fail(CONFIG,"QPC unavailable.",-1);
        frequency=f.QuadPart;begin_qpc=ticks();
        for(unsigned k=0;k<N;++k){
            gw::RequestBytes q{};gw::ReplyBytes r{};gw::Reply reply{};
            std::memcpy(q.data(),pairs+PAIR*k,q.size());std::memcpy(r.data(),pairs+PAIR*k+q.size(),r.size());
            if(!gw::decode(q,expected[k])||!gw::decode(r,reply)||!gw::matches(expected[k],reply))
                return fail(CONFIG,"Fixture checksum/configuration/model/request/reply binding invalid.",-1);
            if(k&&(!same_identity(expected[k].identity,expected[0].identity)||
               expected[k].source_timestamp_ns<=expected[k-1].source_timestamp_ns||
               expected[k].source_generation<=expected[k-1].source_generation||
               expected[k].output_generation<=expected[k-1].output_generation))
                return fail(CONFIG,"Fixture identity or generation order invalid.",-1);
        }
        // Send a disarmed GCS readiness heartbeat.
        mavlink_message_t hb{};
        mavlink_msg_heartbeat_pack_status(255,190,&tx_status,&hb,MAV_TYPE_GCS,
            MAV_AUTOPILOT_INVALID,0,0,MAV_STATE_ACTIVE);
        queue[0].length=mavlink_msg_to_send_buffer(queue[0].bytes,&hb);
        queue[0].queued_s=-.001;queue_count=1;initialized=true;return true;
    }
    const Datagram *output() {
        if(error||!queue_count)return nullptr;
        if(output_prepared)return &pending_output;
        if(queue_head>=3||queue_count>3-queue_head){fail(CAPACITY,"Original reply queue bounds invalid.",last_sim);return nullptr;}
        const auto&first=queue[queue_head];unsigned count=1;
        if(first.length<=5||first.length>MAX_DATAGRAM){fail(CAPACITY,"Complete queued frame exceeds datagram bound.",last_sim);return nullptr;}
        if(reply_pairs&&queue_count>1&&first.query>0){
            const auto&next=queue[queue_head+1];
            if(next.query!=first.query||next.fragment!=first.fragment+1||next.queued_s!=first.queued_s){
                fail(FRAGMENT,"Only original consecutive same-query reply frames may share a datagram.",last_sim);return nullptr;}
            if(next.length<=5||next.length>MAX_DATAGRAM){fail(CAPACITY,"Second queued frame exceeds datagram bound.",last_sim);return nullptr;}
            if(pair_fits(first.length,next.length))count=2;
        }
        if(tx_count+count>MAX_TX||datagram_count>=MAX_TX){fail(CAPACITY,"Fixed frame/datagram output ledger exhausted.",last_sim);return nullptr;}
        pending_output={};pending_output.first_frame=std::uint16_t(tx_count);pending_output.frame_count=std::uint16_t(count);
        for(unsigned j=0;j<count;++j){const auto&f=queue[queue_head+j];
            pending_output.offsets[j]=pending_output.length;pending_output.lengths[j]=f.length;
            std::memcpy(pending_output.bytes+pending_output.length,f.bytes,f.length);pending_output.length+=f.length;}
        // Record the tick when both frames first reach mdlOutputs.
        pending_output.exposed_qpc=ticks();output_prepared=true;return &pending_output;
    }
    bool decode_frame(const unsigned char *b,unsigned n,mavlink_message_t&out,double t) {
        if(n<12||n>MAX_DATAGRAM||b[0]!=253||b[2]!=0||b[3]!=0||unsigned(b[1])+12!=n)
            return fail(FRAME,"Unsigned complete MAVLink2 frame required; no resynchronizing skips.",t);
        mavlink_message_t parser{};mavlink_status_t parse{},result{};unsigned accepted=0;
        for(unsigned j=0;j<n;++j){
            const auto state=mavlink_frame_char_buffer(&parser,&parse,b[j],&out,&result);
            if(state==MAVLINK_FRAMING_BAD_CRC||state==MAVLINK_FRAMING_BAD_SIGNATURE)
                return fail(CRC,"Generated MAVLink decoder rejected CRC/signature.",t);
            if(state==MAVLINK_FRAMING_OK){++accepted;if(j+1!=n)return fail(FRAME,"Trailing bytes inside frame.",t);}
        }
        if(accepted!=1||parse.parse_error||out.magic!=MAVLINK_STX)
            return fail(CRC,"MAVLink frame did not decode exactly once.",t);
        return true;
    }
    bool receive_frame(const unsigned char *b,unsigned n,std::int64_t received,double t) {
        mavlink_message_t message{};if(!decode_frame(b,n,message,t))return false;
        if(completed>=N)return fail(OVERLAP,"Unexpected frame after the complete retained fixture.",t);
        const auto&q=expected[completed];
        if(message.sysid!=q.identity.system||message.compid!=q.identity.component)
            return fail(SOURCE,"MAVLink source differs from exact fixture identity.",t);
        if(message.seq!=expected_sequence)return fail(SEQUENCE,"Missing/duplicate/out-of-order MAVLink sequence.",t);
        ++expected_sequence;++rx_frames;
        if(message.msgid!=MAVLINK_MSG_ID_TUNNEL)return fail(SCHEMA,"Only TUNNEL schema8 is admitted.",t);
        mavlink_tunnel_t tunnel{};mavlink_msg_tunnel_decode(&message,&tunnel);
        if(tunnel.target_system!=255||tunnel.target_component!=190||tunnel.payload_type!=42002||
           (tunnel.payload[0]>>4)!=gw::request_schema)
            return fail(SCHEMA,"Target/payload type/schema invalid.",t);
        if(queue_count)return fail(OVERLAP,"New query overlaps a not-yet-output reply queue.",t);
        const unsigned offset=119*fragment_index,body_size=gw::request_bytes-offset<119?unsigned(gw::request_bytes)-offset:119;
        std::uint64_t generation=0;for(unsigned j=1;j<9;++j)generation=(generation<<8)|tunnel.payload[j];
        if(fragment_index>=3||tunnel.payload[0]!=(0x80|fragment_index)||
           tunnel.payload_length!=body_size+9||generation!=q.output_generation)
            return fail(FRAGMENT,"Missing/duplicate/overlapping fragment or generation mismatch.",t);
        for(unsigned j=tunnel.payload_length;j<128;++j)
            if(tunnel.payload[j])return fail(FRAGMENT,"Nonzero unused fragment payload differs from exact generated fixture.",t);
        if(fragment_index==0)first_rx_qpc=received;
        std::memcpy(assembling.data()+offset,tunnel.payload+9,body_size);++fragment_index;
        if(fragment_index<3)return true;
        if(std::memcmp(assembling.data(),pairs+completed*PAIR,gw::request_bytes))
            return fail(FIXTURE,"Received request differs from next retained original C body.",t);
        gw::Request decoded{};
        if(!gw::decode(assembling,decoded))return fail(REQUEST,"Original GP request checksum or semantics invalid.",t);
        auto&entry=queries[completed];entry.request=assembling;entry.first_rx_qpc=first_rx_qpc;
        entry.completed_s=t;entry.gp_begin_qpc=ticks();++gp_calls;
        gw::Reply reply{};reply.identity=decoded.identity;reply.source_timestamp_ns=decoded.source_timestamp_ns;
        reply.source_generation=decoded.source_generation;reply.output_generation=decoded.output_generation;
        reply.original_request_sha256=gw::digest(assembling.data(),assembling.size());reply.gp_model_sha256=decoded.gp_model_sha256;
        const int result=gpenmpc_gp256_predict(decoded.request19+1,reply.result18);entry.gp_end_qpc=ticks();
        std::memcpy(entry.result,reply.result18,sizeof(entry.result));
        if(result!=GPENMPC_GP256_OK)return fail(GP_CALL,"Original atomic GP API rejected the single call.",t);
        if(!gw::encode(reply,entry.reply)||!gw::matches(decoded,reply)||
           std::memcmp(entry.reply.data(),pairs+completed*PAIR+gw::request_bytes,gw::reply_bytes))
            return fail(REPLY,"Original GP reply differs from validated C fixture; no approximate acceptance.",t);
        queue_head=0;queue_count=0;
        for(unsigned j=0;j<3;++j){
            gw::Fragment frag{};mavlink_message_t msg{};if(!gw::fragment(entry.reply,j,frag))return fail(REPLY,"Reply fragment codec rejected.",t);
            mavlink_msg_tunnel_pack_status(255,190,&tx_status,&msg,q.identity.system,q.identity.component,42002,frag.length,frag.payload);
            auto&f=queue[j];f={};f.length=mavlink_msg_to_send_buffer(f.bytes,&msg);f.query=std::uint16_t(completed+1);
            f.fragment=std::int16_t(j);f.queued_s=t;
            if(f.length<=5||f.length>MAX_DATAGRAM)return fail(REPLY,"Reply frame does not fit official Raw datagram.",t);
            ++queue_count;
        }
        ++completed;assembling={};fragment_index=0;first_rx_qpc=0;return true;
    }
    bool update(const unsigned char *bytes,unsigned length,double sim_s) {
        if(error)return false;
        if(!initialized||!std::isfinite(sim_s)||sim_s<=last_sim||updates>=MAX_UPDATES)
            return fail(TIME,"Increasing finite discrete time and fixed update bound required.",sim_s);
        const auto received=ticks();
        if(double(received-begin_qpc)/double(frequency)>=30.0)
            return fail(WALL_BOUND,"Frozen HOST retained-replay resource bound reached.",sim_s);
        last_sim=sim_s;
        if(!bytes||length<1||length>4999||raw_used+length>RAW_CAPACITY)
            return fail(CAPACITY,"Native aggregate or fixed raw ledger bound exceeded.",sim_s);
        // Account for the previous output before accepting another query.
        if(queue_count){
            const auto*sent=output();if(!sent)return false;
            auto&packet=datagrams[datagram_count++];packet=*sent;packet.output_s=sim_s;
            for(unsigned j=0;j<sent->frame_count;++j){
                auto&record=transmitted[tx_count++];record=queue[queue_head+j];record.output_s=sim_s;}
            queue_head+=sent->frame_count;queue_count-=sent->frame_count;output_prepared=false;
            if(!queue_count)queue_head=0;
        }
        auto&record=raw[updates++];record.offset=raw_used;record.length=length;record.sim_s=sim_s;record.qpc=received;
        struct EndObservation { RawRecord&r;~EndObservation(){r.processed_qpc=ticks();} } observation{record};
        std::memcpy(raw_bytes+raw_used,bytes,length);raw_used+=length;if(length>maximum_aggregate)maximum_aggregate=length;
        if(length==1&&bytes[0]==0){++sentinel_count;return true;}
        unsigned p=0;
        while(p<length){
            if(length-p<12||bytes[p]!=253)return fail(FRAME,"Partial/unknown aggregate entry.",sim_s);
            const unsigned n=unsigned(bytes[p+1])+12;
            if(n>MAX_DATAGRAM||n>length-p)return fail(FRAME,"Aggregate contains truncated/overlength frame.",sim_s);
            if(!receive_frame(bytes+p,n,received,sim_s))return false;p+=n;
        }
        return true;
    }
    bool finish() {
        if(error)return false;
        if(completed!=N||gp_calls!=N||fragment_index||queue_count||tx_count!=MAX_TX||rx_frames!=N*3||
           datagram_count!=(reply_pairs?1+N*2:MAX_TX))
            return fail(INCOMPLETE,"Retained replay ended with missing query/fragment/reply or incomplete accounting.",last_sim);
        return true;
    }
};
} // namespace rawgp

#if !defined(GPENMPC_NATIVE_RAW_GP_STANDALONE_TEST)
#define S_FUNCTION_NAME gpenmpc_native_raw_gp_sfcn
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

namespace {
const mxArray *field(SimStruct*S,const char*name) { return mxGetField(ssGetSFcnParam(S,0),0,name); }
bool text_matches(const mxArray*p,const char*expected) {
    if(!p||!mxIsChar(p))return false;char *s=mxArrayToUTF8String(p);if(!s)return false;
    const bool ok=std::strcmp(s,expected)==0;mxFree(s);return ok;
}
bool hash_text(const mxArray*p,rawgp::gw::Hash&out) {
    if(!p||!mxIsChar(p))return false;char*s=mxArrayToUTF8String(p);if(!s)return false;
    bool ok=std::strlen(s)==64;out={};
    for(unsigned j=0;j<64&&ok;++j){char c=s[j];unsigned n=16;
        if(c>='0'&&c<='9')n=unsigned(c-'0');else if(c>='a'&&c<='f')n=unsigned(c-'a'+10);else if(c>='A'&&c<='F')n=unsigned(c-'A'+10);
        if(n>15)ok=false;else out[j/8]=(out[j/8]<<4)|n;
    }mxFree(s);return ok&&!rawgp::gw::empty(out);
}
mxArray *bytes_matrix(const unsigned char *bytes,mwSize m,mwSize n) {
    mxArray*a=mxCreateNumericMatrix(m,n,mxUINT8_CLASS,mxREAL);std::memcpy(mxGetData(a),bytes,m*n);return a;
}
mxArray *u64(std::uint64_t n) {mxArray*a=mxCreateNumericMatrix(1,1,mxUINT64_CLASS,mxREAL);*static_cast<std::uint64_t*>(mxGetData(a))=n;return a;}
void put(mxArray*a,const char*n,mxArray*v){mxSetField(a,0,n,v);}
mxArray *ledger(const rawgp::Core&c,SimStruct*S) {
    const char*names[]={"scope","all_complete","error_code","error_message","error_update","error_simulation_s","error_qpc", "updates","raw_used","raw_records","raw_bytes","raw_qpc", "rx_frames","sentinel_count","maximum_aggregate", "gp_calls","queries_completed","query_requests","query_replies","query_result18","query_times","query_qpc", "tx_count","tx_frames","tx_metadata","remaining_queue_count","pending_fragment_count", "qpc_frequency","qpc_origin", "fixture_sha256","replay_binding_sha256", "COM","board","sockets","plant","controller","live_admission_proven", "transport_scope","reply_pair_datagrams","tx_datagram_count","tx_datagram_bytes","tx_datagram_metadata","tx_datagram_qpc"};
    mxArray*a=mxCreateStructMatrix(1,1,int(sizeof(names)/sizeof(names[0])),names);
    put(a,"scope",mxCreateString("COMPILED_RETAINED_C_QUERY_REPLAY_ONLY_NO_LIVE_AUTHORITY"));
    put(a,"all_complete",mxCreateLogicalScalar(c.error==0&&c.completed==rawgp::N));
    put(a,"error_code",u64(c.error));put(a,"error_message",mxCreateString(c.error_message));put(a,"error_update",u64(c.error_update));
    put(a,"error_simulation_s",mxCreateDoubleScalar(c.error_sim));put(a,"error_qpc",u64(c.error_qpc));
    put(a,"updates",u64(c.updates));put(a,"raw_used",u64(c.raw_used));put(a,"raw_bytes",bytes_matrix(c.raw_bytes,c.raw_used,1));
    mxArray*r=mxCreateDoubleMatrix(c.updates,3,mxREAL);double*d=mxGetDoubles(r);
    mxArray*rq=mxCreateNumericMatrix(c.updates,2,mxUINT64_CLASS,mxREAL);auto*rqd=static_cast<std::uint64_t*>(mxGetData(rq));
    for(unsigned k=0;k<c.updates;++k){d[k]=c.raw[k].offset;d[k+c.updates]=c.raw[k].length;d[k+2*c.updates]=c.raw[k].sim_s;rqd[k]=std::uint64_t(c.raw[k].qpc);rqd[k+c.updates]=std::uint64_t(c.raw[k].processed_qpc);}
    put(a,"raw_records",r);put(a,"raw_qpc",rq); // Exact ticks never cast through binary64.
    put(a,"rx_frames",u64(c.rx_frames));put(a,"sentinel_count",u64(c.sentinel_count));put(a,"maximum_aggregate",u64(c.maximum_aggregate));
    put(a,"gp_calls",u64(c.gp_calls));put(a,"queries_completed",u64(c.completed));
    mxArray*q=mxCreateNumericMatrix(310,c.gp_calls,mxUINT8_CLASS,mxREAL),*reply=mxCreateNumericMatrix(286,c.gp_calls,mxUINT8_CLASS,mxREAL);
    mxArray*y=mxCreateDoubleMatrix(c.gp_calls,18,mxREAL),*times=mxCreateDoubleMatrix(c.gp_calls,1,mxREAL);
    mxArray*qt=mxCreateNumericMatrix(c.gp_calls,3,mxUINT64_CLASS,mxREAL);auto*qtd=static_cast<std::uint64_t*>(mxGetData(qt));
    auto*qb=static_cast<unsigned char*>(mxGetData(q));auto*rb=static_cast<unsigned char*>(mxGetData(reply));
    auto*yd=mxGetDoubles(y);auto*td=mxGetDoubles(times);
    for(unsigned k=0;k<c.gp_calls;++k){const auto&e=c.queries[k];std::memcpy(qb+310*k,e.request.data(),310);std::memcpy(rb+286*k,e.reply.data(),286);
        for(unsigned j=0;j<18;++j)yd[k+j*c.gp_calls]=e.result[j];
        td[k]=e.completed_s;qtd[k]=std::uint64_t(e.first_rx_qpc);qtd[k+c.gp_calls]=std::uint64_t(e.gp_begin_qpc);qtd[k+2*c.gp_calls]=std::uint64_t(e.gp_end_qpc);}
    put(a,"query_requests",q);put(a,"query_replies",reply);put(a,"query_result18",y);put(a,"query_times",times);put(a,"query_qpc",qt);
    mxArray*tx=mxCreateNumericMatrix(300,c.tx_count,mxUINT8_CLASS,mxREAL),*metadata=mxCreateDoubleMatrix(c.tx_count,5,mxREAL);
    auto*tb=static_cast<unsigned char*>(mxGetData(tx));auto*md=mxGetDoubles(metadata);
    for(unsigned k=0;k<c.tx_count;++k){const auto&e=c.transmitted[k];std::memcpy(tb+300*k,e.bytes,300);md[k]=e.length;md[k+c.tx_count]=e.query;
        md[k+2*c.tx_count]=e.fragment;md[k+3*c.tx_count]=e.queued_s;md[k+4*c.tx_count]=e.output_s;}
    put(a,"tx_count",u64(c.tx_count));put(a,"tx_frames",tx);put(a,"tx_metadata",metadata);
    put(a,"transport_scope",mxCreateString("HOST_ONLY_NO_COPTERSIM_UDP_COM_PROOF"));
    put(a,"reply_pair_datagrams",mxCreateLogicalScalar(c.reply_pairs));put(a,"tx_datagram_count",u64(c.datagram_count));
    mxArray*dg=mxCreateNumericMatrix(300,c.datagram_count,mxUINT8_CLASS,mxREAL);
    mxArray*dm=mxCreateDoubleMatrix(c.datagram_count,8,mxREAL);
    mxArray*dq=mxCreateNumericMatrix(c.datagram_count,1,mxUINT64_CLASS,mxREAL);
    auto*dgb=static_cast<unsigned char*>(mxGetData(dg));auto*dmd=mxGetDoubles(dm);auto*dqd=static_cast<std::uint64_t*>(mxGetData(dq));
    for(unsigned k=0;k<c.datagram_count;++k){const auto&e=c.datagrams[k];std::memcpy(dgb+300*k,e.bytes,300);
        const double values[]={double(e.length),double(e.frame_count),double(e.first_frame),double(e.offsets[0]),
            double(e.lengths[0]),double(e.offsets[1]),double(e.lengths[1]),e.output_s};
        for(unsigned j=0;j<8;++j)dmd[k+j*c.datagram_count]=values[j];dqd[k]=std::uint64_t(e.exposed_qpc);}
    put(a,"tx_datagram_bytes",dg);put(a,"tx_datagram_metadata",dm);put(a,"tx_datagram_qpc",dq);
    put(a,"remaining_queue_count",u64(c.queue_count));put(a,"pending_fragment_count",u64(c.fragment_index));
    put(a,"qpc_frequency",u64(c.frequency));put(a,"qpc_origin",u64(c.begin_qpc));
    put(a,"fixture_sha256",mxDuplicateArray(field(S,"fixture_sha256")));put(a,"replay_binding_sha256",mxDuplicateArray(field(S,"replay_binding_sha256")));
    for(const auto*n:{"COM","board","sockets","plant","controller"})put(a,n,u64(0));put(a,"live_admission_proven",mxCreateLogicalScalar(false));return a;
}
}

#define MDL_CHECK_PARAMETERS
static void mdlCheckParameters(SimStruct*S) {
    const auto*p=ssGetSFcnParam(S,0);rawgp::gw::Hash fixture{},binding{};
    if(!mxIsStruct(p)||mxGetNumberOfElements(p)!=1||!text_matches(field(S,"scope"),"RETAINED_C_GP_FIXTURE_ONLY")||
       !hash_text(field(S,"fixture_sha256"),fixture)||!hash_text(field(S,"replay_binding_sha256"),binding)){
        ssSetErrorStatus(S,"Explicit HOST retained-fixture scope and two SHA256 bindings required.");return;}
    const auto*b=field(S,"pairs");const auto*v=field(S,"ledger_variable");
    if(!b||!mxIsUint8(b)||mxIsSparse(b)||mxIsComplex(b)||mxGetM(b)!=596||mxGetN(b)!=59||
       rawgp::gw::digest(static_cast<const unsigned char*>(mxGetData(b)),596*59)!=fixture||!v||!mxIsChar(v)){
        ssSetErrorStatus(S,"Exact 596x59 original pair bytes/hash and ledger variable required.");return;}
    const auto*pair=field(S,"reply_pair_datagrams");
    if(pair&&(!mxIsLogical(pair)||mxGetNumberOfElements(pair)!=1)){
        ssSetErrorStatus(S,"Optional reply_pair_datagrams must be an explicit logical scalar.");return;}
    char*name=mxArrayToUTF8String(v);bool ok=name&&std::strlen(name)>0&&std::strlen(name)<64;
    for(unsigned j=0;name&&name[j]&&ok;++j)ok=(name[j]>='A'&&name[j]<='Z')||(name[j]>='a'&&name[j]<='z')||(j&&((name[j]>='0'&&name[j]<='9')||name[j]=='_'));
    if(name)mxFree(name);if(!ok)ssSetErrorStatus(S,"Unique valid MATLAB ledger variable name required.");
}
static void mdlInitializeSizes(SimStruct*S) {
    ssSetNumSFcnParams(S,1);if(ssGetNumSFcnParams(S)!=ssGetSFcnParamsCount(S))return;
    ssSetSFcnParamTunable(S,0,0);mdlCheckParameters(S);if(ssGetErrorStatus(S))return;
    ssSetNumContStates(S,0);ssSetNumDiscStates(S,0);
    if(!ssSetNumInputPorts(S,2))return;
    if(!ssSetInputPortDimensionInfo(S,0,DYNAMIC_DIMENSION))return;
    ssSetInputPortDataType(S,0,SS_UINT8);ssSetInputPortDimensionsMode(S,0,INHERIT_DIMS_MODE);
    ssSetInputPortRequiredContiguous(S,0,1);ssSetInputPortDirectFeedThrough(S,0,0);
    ssSetInputPortWidth(S,1,1);ssSetInputPortDataType(S,1,SS_DOUBLE);ssSetInputPortRequiredContiguous(S,1,1);ssSetInputPortDirectFeedThrough(S,1,0);
    if(!ssSetNumOutputPorts(S,2))return;
    ssSetOutputPortWidth(S,0,300);ssSetOutputPortDataType(S,0,SS_UINT8);ssSetOutputPortWidth(S,1,1);ssSetOutputPortDataType(S,1,SS_UINT16);
    ssSetNumSampleTimes(S,1);ssSetNumPWork(S,1);ssSetOptions(S,SS_OPTION_EXCEPTION_FREE_CODE);
}
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
static void mdlSetInputPortDimensionInfo(SimStruct*S,int_T port,const DimsInfo_T*dims) {
    if(port==0&&(dims->width>5000||dims->numDims>2)){ssSetErrorStatus(S,"Native Raw upper dimension must fit 5000 elements.");return;}
    ssSetInputPortDimensionInfo(S,port,dims);
}
#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
static void mdlSetOutputPortDimensionInfo(SimStruct*S,int_T port,const DimsInfo_T*dims) {
    const int_T expected=port==0?300:1;
    if(port<0||port>1||dims->width!=expected||dims->numDims>2){
        ssSetErrorStatus(S,"Compiled output must remain 300 uint8 values and one uint16 length.");return;
    }
    ssSetOutputPortDimensionInfo(S,port,dims);
}
#define MDL_SET_DEFAULT_PORT_DIMENSION_INFO
static void mdlSetDefaultPortDimensionInfo(SimStruct*S) {
    if(ssGetInputPortWidth(S,0)==DYNAMICALLY_SIZED)ssSetInputPortWidth(S,0,5000);
}
static void mdlInitializeSampleTimes(SimStruct*S){ssSetSampleTime(S,0,.001);ssSetOffsetTime(S,0,0);}
#define MDL_START
static void mdlStart(SimStruct*S) {
    char*name=mxArrayToUTF8String(field(S,"ledger_variable"));mxArray*old=mexGetVariable("base",name);mxFree(name);
    if(old){mxDestroyArray(old);ssSetErrorStatus(S,"Do not overwrite an existing compiled GP ledger variable.");return;}
    auto*c=new(std::nothrow)rawgp::Core;if(!c){ssSetErrorStatus(S,"Fixed retained-replay memory allocation failed.");return;}
    ssSetPWorkValue(S,0,c);const auto*p=field(S,"pairs");
    const auto*pair=field(S,"reply_pair_datagrams");
    if(!c->initialize(static_cast<const unsigned char*>(mxGetData(p)),mxGetNumberOfElements(p),pair&&mxIsLogicalScalarTrue(pair)))ssSetErrorStatus(S,c->error_message);
}
static void mdlOutputs(SimStruct*S,int_T) {
    auto*out=static_cast<unsigned char*>(ssGetOutputPortSignal(S,0));auto*n=static_cast<uint16_T*>(ssGetOutputPortSignal(S,1));
    std::memset(out,0,300);*n=1;auto*c=static_cast<rawgp::Core*>(ssGetPWorkValue(S,0));
    if(c){const auto*f=c->output();if(f){std::memcpy(out,f->bytes,f->length);*n=f->length;}if(c->error)ssSetErrorStatus(S,c->error_message);}
}
#define MDL_UPDATE
static void mdlUpdate(SimStruct*S,int_T) {
    auto*c=static_cast<rawgp::Core*>(ssGetPWorkValue(S,0));if(!c){ssSetErrorStatus(S,"Retained GP core not initialized.");return;}
    const int_T n=ssGetCurrentInputPortWidth(S,0);const auto*b=static_cast<const unsigned char*>(ssGetInputPortSignal(S,0));
    const auto*t=static_cast<const real_T*>(ssGetInputPortSignal(S,1));
    if(n<1||!c->update(b,unsigned(n),*t)){if(!c->error)c->fail(rawgp::CAPACITY,"Empty native input.",*t);ssSetErrorStatus(S,c->error_message);}
}
static void mdlTerminate(SimStruct*S) {
    auto*c=static_cast<rawgp::Core*>(ssGetPWorkValue(S,0));if(!c)return;
    if(c->updates||c->error){c->finish();mxArray*a=ledger(*c,S);char*name=mxArrayToUTF8String(field(S,"ledger_variable"));
        if(mexPutVariable("base",name,a))ssSetErrorStatus(S,"Unable to export final compiled GP ledger.");mxFree(name);mxDestroyArray(a);}
    delete c;ssSetPWorkValue(S,0,nullptr);
}
#include "simulink.c"
#else
// Drive the Core directly from the standalone test.
#include "gpenmpc_native_raw_gp_sfcn_standalone_test.hpp"
#endif
