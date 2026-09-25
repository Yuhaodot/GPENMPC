#include "CanonicalLocalIngress.hpp"
#include <uORB/topics/gpenmpc_full_inner_ingress.h>
namespace ri=gpenmpc_local_ingress;
// Compile-only explicit configuration fixture, never linked into an app.
ri::CanonicalLocalIngress gpenmpc_ingress_probe_owner({255,190,1,1,2,2000});
extern "C" {
char gpenmpc_local_ingress_size[sizeof(ri::CanonicalLocalIngress)];
char gpenmpc_local_ingress_completed_size[sizeof(ri::Completed)];
bool gpenmpc_ingress_probe_expect(ri::CanonicalLocalIngress*p,const ri::RequestBytes*b,std::uint64_t ready,std::uint64_t now){return p->expect_gp_reply(*b,ready,now);}
bool gpenmpc_ingress_probe_actual_topic(ri::CanonicalLocalIngress*p,const gpenmpc_full_inner_ingress_s*t,std::uint32_t actual_generation,std::uint64_t now){
    return p->receive(gpenmpc_argument_transport::from_topic(*t,actual_generation),now);
}
bool gpenmpc_ingress_probe_take(ri::CanonicalLocalIngress*p,ri::Completed*out,std::uint64_t now){return p->take_gp_reply(*out,now);}
void gpenmpc_ingress_probe_stop(ri::CanonicalLocalIngress*p,std::uint64_t now){p->stop(now);}
}
