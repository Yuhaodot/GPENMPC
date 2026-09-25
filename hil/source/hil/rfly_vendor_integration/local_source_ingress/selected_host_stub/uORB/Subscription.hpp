#pragma once
#include <uORB/uORB.h>
bool gpenmpc_test_topic_copy(const orb_metadata*,void*,std::uint32_t&,std::uint8_t)noexcept;
bool gpenmpc_test_topic_advertised(const orb_metadata*,std::uint8_t)noexcept;
bool gpenmpc_test_topic_updated(const orb_metadata*,std::uint32_t,std::uint8_t)noexcept;
// Only the real Subscription API is mocked; official generated topic layouts
// and the actual parameter header/C++ typed param_get wrapper are used.
namespace uORB {
class Subscription final {
public:
    Subscription(const orb_metadata*t=nullptr,std::uint8_t i=0)noexcept:topic_(t),instance_(i){}
    // Numerical fixtures pre-create topic identities; cold-start semantics
    // are tested separately using the extracted original PX4 methods.
    bool valid()const noexcept{return topic_!=nullptr;}
    bool advertised()noexcept{return gpenmpc_test_topic_advertised(topic_,instance_);}
    bool updated()noexcept{return gpenmpc_test_topic_updated(topic_,generation_,instance_);}
    bool update(void*out)noexcept{return gpenmpc_test_topic_update(topic_,out,generation_,instance_);}
    bool copy(void*out)noexcept{return gpenmpc_test_topic_copy(topic_,out,generation_,instance_);}
    std::uint32_t get_last_generation()const noexcept{return generation_;}
    const orb_metadata*get_topic()const noexcept{return topic_;}
    std::uint8_t get_instance()const noexcept{return instance_;}
private:const orb_metadata*topic_;std::uint32_t generation_{};std::uint8_t instance_{};
};
}
