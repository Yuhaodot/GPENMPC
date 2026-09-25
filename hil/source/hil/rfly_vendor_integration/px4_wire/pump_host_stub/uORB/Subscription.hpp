#pragma once
#include "uORB.h"
bool gpenmpc_test_topic_copy(const orb_metadata*,void*,std::uint32_t&,std::uint8_t)noexcept;
namespace uORB {
class Subscription final {
public:
    explicit Subscription(const orb_metadata*topic,std::uint8_t instance=0)noexcept:topic_(topic),instance_(instance){}
    // Existing numerical fixtures model topics as pre-created. The separate
    // ingress-startup fixture exercises actual PX4 lazy subscription code.
    bool valid()const noexcept{return topic_!=nullptr;}
    bool update(void*out)noexcept{return gpenmpc_test_topic_update(topic_,out,generation_,instance_);}
    bool copy(void*out)noexcept{return gpenmpc_test_topic_copy(topic_,out,generation_,instance_);}
    std::uint32_t get_last_generation()const noexcept{return generation_;}
    const orb_metadata*get_topic()const noexcept{return topic_;}
    std::uint8_t get_instance()const noexcept{return instance_;}
private:const orb_metadata*topic_;std::uint32_t generation_{};std::uint8_t instance_{};
};
}
