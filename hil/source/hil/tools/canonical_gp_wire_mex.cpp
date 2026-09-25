// Convert RGP1 requests through GP256 to RGR1 replies.
// The MATLAB service owns lifecycle, identity, arrival and replay validation.
#include "mex.h"
#include "../rfly_vendor_integration/px4_wire/CanonicalLocalGpWire.hpp"
#include "canonical_gp_standalone_api.h"
#include <cstring>
#include <vector>
#include "../host_runtime/native_include/mavlink/common/mavlink.h"

namespace gw = gpenmpc_local_gp_wire;

namespace {
// Assemble three RGP1 fragments at the receiver dequeue point.
// The GP service validates session and replay state before prediction.
struct Partial {
    std::uint64_t generation{}, first{}, last{};
    unsigned next{}, lengths[3]{};
    std::uint64_t received[3]{};
    unsigned char frames[3][300]{};
    gw::RequestBytes body{};
};
const mxArray* field(const mxArray* a,mwIndex i,const char* name) {
    const auto* p=mxGetField(a,i,name);
    if(!p)mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveShape","Missing original receive field %s.",name);
    return p;
}
std::uint64_t scalar64(const mxArray* a) {
    if(!mxIsUint64(a)||mxGetNumberOfElements(a)!=1)
        mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveShape","Original uint64 time required.");
    return *static_cast<const std::uint64_t*>(mxGetData(a));
}
mxArray* byte_array(const unsigned char* p,std::size_t n) {
    auto* a=mxCreateNumericMatrix(n,1,mxUINT8_CLASS,mxREAL);
    if(n)std::memcpy(mxGetData(a),p,n);return a;
}
mxArray* time_array(const std::uint64_t* p,std::size_t n) {
    auto* a=mxCreateNumericMatrix(n,1,mxUINT64_CLASS,mxREAL);
    if(n)std::memcpy(mxGetData(a),p,n*sizeof(*p));return a;
}
void collect(int nlhs,mxArray* out[],int nrhs,const mxArray* in[]) {
    if((nrhs!=5&&nrhs!=6)||nlhs!=2||!mxIsStruct(in[1])||mxGetNumberOfElements(in[1])>64
        ||!mxIsUint8(in[2])||!mxIsUint8(in[3])||mxGetNumberOfElements(in[3])!=4
        ||!mxIsUint64(in[4])||mxGetNumberOfElements(in[4])!=1)
        mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveShape","collect(batch,partial,source/target uint8[4],processing uint64).");
    Partial p{};const auto state_n=mxGetNumberOfElements(in[2]);
    if(state_n){
        if(state_n!=sizeof(p))mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveShape","Partial assembly shape changed.");
        std::memcpy(&p,mxGetData(in[2]),sizeof(p));
        if(p.next>2)mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveState","Invalid retained fragment index.");
    }
    const auto* ids=static_cast<const unsigned char*>(mxGetData(in[3]));
    const auto now=scalar64(in[4]);std::vector<Partial> complete;
    const bool predict=nrhs==6;
    const std::uint64_t* authority=nullptr;
    if(predict){
        if(!mxIsUint64(in[5])||mxGetNumberOfElements(in[5])!=5)
            mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveSource","Exact registered UID/boot and prior query generations required.");
        authority=static_cast<const std::uint64_t*>(mxGetData(in[5]));
        if(!authority[0]||!authority[1])mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveSource","Missing registered board identity.");
    }
    for(mwIndex j=0;j<mxGetNumberOfElements(in[1]);++j){
        if(!mxIsLogicalScalarTrue(field(in[1],j,"ok"))||!mxIsLogicalScalarTrue(field(in[1],j,"source_matched"))
            ||!mxIsLogicalScalarTrue(field(in[1],j,"bytes_complete")))
            mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveSource","Unvalidated original native datagram.");
        const auto* a=field(in[1],j,"bytes");const auto n=mxGetNumberOfElements(a);
        if(!mxIsUint8(a)||!n||n>300)mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveShape","Invalid original bytes.");
        const auto rx=scalar64(field(in[1],j,"dequeue_ns"));
        if(!rx||rx>now)mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveClock","Future/absent original receive time.");
        const auto* b=static_cast<const unsigned char*>(mxGetData(a));
        for(std::size_t offset=0;offset+12<=n;){
            // Accept complete unsigned MAVLink2 frames and pass other datagrams to the full parser.
            if(b[offset]!=253||b[offset+2]!=0)break;
            const auto length=std::size_t(b[offset+1])+12;if(offset+length>n)break;
            mavlink_message_t buffer{},message{};mavlink_status_t parser{},status{};unsigned valid=0;
            for(std::size_t k=0;k<length;++k)
                valid=mavlink_frame_char_buffer(&buffer,&parser,b[offset+k],&message,&status);
            if(valid!=MAVLINK_FRAMING_OK)break;
            if(message.msgid==MAVLINK_MSG_ID_TUNNEL){
                mavlink_tunnel_t tunnel{};mavlink_msg_tunnel_decode(&message,&tunnel);
                if(tunnel.payload_type==42002&&(tunnel.payload[0]>>4)==gw::request_schema){
                    if(message.sysid!=ids[0]||message.compid!=ids[1]||tunnel.target_system!=ids[2]||tunnel.target_component!=ids[3])
                        mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveSource","GP fragment address mismatch.");
                    const unsigned index=tunnel.payload[0]&15;
                    const unsigned expected=index<3?unsigned((index==2?72:119)+9):0;
                    if(!expected||tunnel.payload_length!=expected)
                        mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveFragment","Exact original three-fragment shape required.");
                    std::uint64_t generation=0;for(unsigned k=1;k<9;++k)generation=(generation<<8)|tunnel.payload[k];
                    if(!generation||index!=p.next||(p.next&&generation!=p.generation))
                        mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveOrder","Partial GP gap/reorder/interleave; no inference.");
                    if(!index){p=Partial{};p.generation=generation;p.first=rx;}
                    if(rx<p.first||rx<p.last||rx-p.first>50000000ULL)
                        mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveClock","Original 50ms fragment span exceeded.");
                    p.last=rx;p.received[index]=rx;p.lengths[index]=unsigned(length);
                    std::memcpy(p.frames[index],b+offset,length);
                    std::memcpy(p.body.data()+119*index,tunnel.payload+9,expected-9);++p.next;
                    if(p.next==3){
                        gw::Request q{};
                        if(!gw::decode(p.body,q)||q.output_generation!=p.generation)
                            mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveBody","Original GP checksum/body generation rejected.");
                        // Retire an expired completed request without inference or timestamp renewal.
                        if(now-p.first<=50000000ULL)complete.push_back(p);
                        p=Partial{};
                    }
                }
            }
            offset+=length;
        }
    }
    // Validate the candidate batch against the service's source and replay state before prediction.
    std::uint64_t last_output=predict?authority[2]:0,last_source=predict?authority[3]:0,last_sample=predict?authority[4]:0;
    if(predict)for(const auto& original:complete){
        gw::Request q{};if(!gw::decode(original.body,q)||q.identity.uid!=authority[0]||q.identity.boot_generation!=authority[1]
            ||q.identity.system!=ids[0]||q.identity.component!=ids[1]||q.output_generation<=last_output
            ||q.source_generation<=last_source||q.source_timestamp_ns<=last_sample)
            mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveReplay","Registered identity or original monotonic query rejected before prediction.");
        last_output=q.output_generation;last_source=q.source_generation;last_sample=q.source_timestamp_ns;
    }
    out[0]=mxCreateCellMatrix(complete.size(),1);
    const char* names[]={"request_bytes","original_host_receive_ns","fragment_rx_ns","raw_frames","reply_bytes","result18"};
    for(std::size_t j=0;j<complete.size();++j){const auto& q=complete[j];
        auto* r=mxCreateStructMatrix(1,1,6,names);
        mxSetField(r,0,"request_bytes",byte_array(q.body.data(),q.body.size()));
        mxSetField(r,0,"original_host_receive_ns",time_array(&q.first,1));
        mxSetField(r,0,"fragment_rx_ns",time_array(q.received,3));
        auto* frames=mxCreateCellMatrix(3,1);
        for(unsigned k=0;k<3;++k)mxSetCell(frames,k,byte_array(q.frames[k],q.lengths[k]));
        mxSetField(r,0,"raw_frames",frames);
        if(predict){
            gw::Request request{};gw::decode(q.body,request);gw::Reply reply{};
            reply.identity=request.identity;reply.source_timestamp_ns=request.source_timestamp_ns;
            reply.source_generation=request.source_generation;reply.output_generation=request.output_generation;
            reply.original_request_sha256=gw::digest(q.body.data(),q.body.size());reply.gp_model_sha256=request.gp_model_sha256;
            if(gpenmpc_gp256_predict(request.request19+1,reply.result18)!=GPENMPC_GP256_OK)
                mexErrMsgIdAndTxt("gpenmpcNative:CanonicalGpWireMexPrediction","Original GP numerical call failed.");
            gw::ReplyBytes encoded{};if(!gw::encode(reply,encoded))
                mexErrMsgIdAndTxt("gpenmpcNative:CanonicalGpWireMexReply","Original GP result rejected.");
            mxSetField(r,0,"reply_bytes",byte_array(encoded.data(),encoded.size()));
            auto* values=mxCreateDoubleMatrix(1,18,mxREAL);std::memcpy(mxGetDoubles(values),reply.result18,sizeof(reply.result18));
            mxSetField(r,0,"result18",values);
        }
        mxSetCell(out[0],j,r);
    }
    out[1]=byte_array(reinterpret_cast<const unsigned char*>(&p),sizeof(p));
}
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if(nrhs>0&&mxIsChar(prhs[0])){
        char command[16]{};mxGetString(prhs[0],command,sizeof(command));
        if(std::strcmp(command,"collect")==0){collect(nlhs,plhs,nrhs,prhs);return;}
        mexErrMsgIdAndTxt("gpenmpcNative:GpReceiveCommand","Unknown operation.");
    }
    if (nrhs != 1 || nlhs < 1 || nlhs > 2) {
        mexErrMsgIdAndTxt("gpenmpcNative:CanonicalGpWireMexArity",
            "One RGP1 input and one or two outputs are required.");
    }
    const mxArray *input = prhs[0];
    if (!mxIsUint8(input) || mxIsComplex(input) || mxIsSparse(input) ||
        mxGetNumberOfDimensions(input) != 2 ||
        (mxGetM(input) != 1 && mxGetN(input) != 1) ||
        mxGetNumberOfElements(input) != gw::request_bytes) {
        mexErrMsgIdAndTxt("gpenmpcNative:CanonicalGpWireMexShape",
            "Input must be a real full uint8 row/column vector of 310 bytes.");
    }

    gw::RequestBytes request_bytes{};
    std::memcpy(request_bytes.data(), mxGetData(input), request_bytes.size());
    gw::Request request{};
    if (!gw::decode(request_bytes, request)) {
        mexErrMsgIdAndTxt("gpenmpcNative:CanonicalGpWireMexRequest",
            "Invalid RGP1 checksum, metadata, canonical configuration/model or features.");
    }

    gw::Reply reply{};
    reply.identity = request.identity;
    reply.source_timestamp_ns = request.source_timestamp_ns;
    reply.source_generation = request.source_generation;
    reply.output_generation = request.output_generation;
    reply.original_request_sha256 = gw::digest(request_bytes.data(), request_bytes.size());
    reply.gp_model_sha256 = request.gp_model_sha256;
    // Use the request19[1..17] numerical ABI shared with the MATLAB predictor.
    // The C API enforces single-owner access.
    const int status = gpenmpc_gp256_predict(request.request19 + 1, reply.result18);
    if (status != GPENMPC_GP256_OK) {
        mexErrMsgIdAndTxt("gpenmpcNative:CanonicalGpWireMexPrediction",
            "Original GP256 API rejected the numerical call (status %d).", status);
    }
    gw::ReplyBytes reply_bytes{};
    if (!gw::encode(reply, reply_bytes)) {
        mexErrMsgIdAndTxt("gpenmpcNative:CanonicalGpWireMexReply",
            "Original GP256 result cannot be represented by the RGR1 contract.");
    }
    // Preserve the hard-invalid flag in result18[14].
    plhs[0] = mxCreateNumericMatrix(gw::reply_bytes, 1, mxUINT8_CLASS, mxREAL);
    std::memcpy(mxGetData(plhs[0]), reply_bytes.data(), reply_bytes.size());
    if (nlhs == 2) {
        plhs[1] = mxCreateDoubleMatrix(1, 18, mxREAL);
        std::memcpy(mxGetDoubles(plhs[1]), reply.result18, sizeof(reply.result18));
    }
}
