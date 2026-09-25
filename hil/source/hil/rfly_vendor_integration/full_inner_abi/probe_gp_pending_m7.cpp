#include "../px4_runtime/CanonicalLocalGpPending.hpp"
extern "C" {
unsigned char gpenmpc_size_gp_pending[sizeof(gpenmpc_rfly_px4::CanonicalLocalGpPending)];
gpenmpc_rfly_px4::LocalGpBegin gpenmpc_probe_gp_observe(gpenmpc_rfly_px4::CanonicalLocalGpPending*p) {
    return p->observe_actual_commit();
}
bool gpenmpc_probe_gp_reply(gpenmpc_rfly_px4::CanonicalLocalGpPending*p,
    const gpenmpc_local_gp_wire::ReplyBytes*b,std::uint64_t original,std::uint64_t now) {
    return p->accept_reply(*b,original,now);
}
void gpenmpc_probe_gp_stop(gpenmpc_rfly_px4::CanonicalLocalGpPending*p){p->stop();}
}
