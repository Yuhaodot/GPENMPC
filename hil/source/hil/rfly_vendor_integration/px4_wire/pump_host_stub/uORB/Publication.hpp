#pragma once
#include "uORB.h"
namespace uORB {
template<class T>class Publication final {
public:
    explicit Publication(const orb_metadata*topic)noexcept:topic_(topic){}
    bool publish(const T&message)noexcept{return gpenmpc_test_topic_publish(topic_,&message);}
private:const orb_metadata*topic_;
};
}
