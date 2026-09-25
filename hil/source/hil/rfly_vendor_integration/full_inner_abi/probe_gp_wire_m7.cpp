#include "../px4_wire/CanonicalLocalGpWire.hpp"
extern "C" {
bool gpenmpc_probe_gp_request(const gpenmpc_local_gp_wire::Request*r,
    gpenmpc_local_gp_wire::RequestBytes*b,gpenmpc_local_gp_wire::Request*out) {
    return gpenmpc_local_gp_wire::encode(*r,*b)&&gpenmpc_local_gp_wire::decode(*b,*out);
}
bool gpenmpc_probe_gp_response(const gpenmpc_local_gp_wire::Reply*r,
    gpenmpc_local_gp_wire::ReplyBytes*b,gpenmpc_local_gp_wire::Reply*out) {
    return gpenmpc_local_gp_wire::encode(*r,*b)&&gpenmpc_local_gp_wire::decode(*b,*out);
}
bool gpenmpc_probe_gp_fragment(const gpenmpc_local_gp_wire::RequestBytes*b,unsigned index,
    gpenmpc_local_gp_wire::Fragment*f){return gpenmpc_local_gp_wire::fragment(*b,index,*f);}
}
