#pragma once
#include "mavlink_main.h"
#include "mavlink_stream.h"
#include "CommittedFeedbackRouteRegistry.hpp"
class MavlinkStreamGPENMPCRflyCommittedFeedback final:public MavlinkStream {
public:
    static MavlinkStream*new_instance(Mavlink*m){return new MavlinkStreamGPENMPCRflyCommittedFeedback(m);}
    static constexpr const char*get_name_static(){return "GPENMPC_COMMITTED_FEEDBACK";}
    static constexpr uint16_t get_id_static(){return MAVLINK_MSG_ID_TUNNEL;}
    const char*get_name()const override{return get_name_static();}
    uint16_t get_id()override{return get_id_static();}
    unsigned get_size()override{return MAVLINK_MSG_ID_TUNNEL_LEN+MAVLINK_NUM_NON_PAYLOAD_BYTES;}
    unsigned get_size_avg()override{return gpenmpc_feedback_wire::fragment_count*get_size();}
    ~MavlinkStreamGPENMPCRflyCommittedFeedback()override{if(registry_)(void)registry_->release_view(link_,this);}
private:
    explicit MavlinkStreamGPENMPCRflyCommittedFeedback(Mavlink*m):MavlinkStream(m){}
    bool send()override{
        using namespace gpenmpc_rfly_stream;using namespace gpenmpc_rfly_px4;
        if(!resolved_){auto*l=link_lifetime_registry();if(!l||l->lookup(_mavlink,link_)!=LinkAccess::Read)return false;resolved_=true;}
        if(!registry_)registry_=committed_feedback_route_registry();
        if(!registry_)return false;
        bool sent=false;
        for(unsigned i=0;i<gpenmpc_feedback_wire::fragment_count;++i){
            if(_mavlink->get_free_tx_buf()<get_size())break;
            FeedbackFragment f{};
            if(registry_->copy_next(link_,this,f,hrt_absolute_time())!=FeedbackExport::Fragment)break;
            const auto now=hrt_absolute_time();
            if(!registry_->allows_send(link_,this,f.generation,f.index,now)){
                (void)registry_->interrupt(link_,this);break;
            }
            mavlink_tunnel_t message{};message.target_system=f.target_system;message.target_component=f.target_component;
            message.payload_type=42002;message.payload_length=f.fragment.length;memcpy(message.payload,f.fragment.payload,f.fragment.length);
            mavlink_msg_tunnel_send_struct(_mavlink->get_channel(),&message);sent=true;
            // Local copy retirement after a void send, NOT a downstream ACK.
            if(!registry_->retire_copy(link_,this,f.generation,f.index))break;
        }
        return sent;
    }
    gpenmpc_rfly_stream::CommittedFeedbackRouteRegistry*registry_{};
    gpenmpc_rfly_stream::LinkToken link_{};bool resolved_{};
};
