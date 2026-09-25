#ifndef GPENMPC_RFLY_SNAPSHOT_STREAM_HPP
#define GPENMPC_RFLY_SNAPSHOT_STREAM_HPP
#include "mavlink_main.h"
#include "mavlink_stream.h"
#include "SnapshotRouteRegistry.hpp"
class MavlinkStreamGPENMPCRflySnapshot final:public MavlinkStream {
public:
    static MavlinkStream *new_instance(Mavlink *m){return new MavlinkStreamGPENMPCRflySnapshot(m);}
    static constexpr const char *get_name_static(){return "GPENMPC_PRIVATE_SNAPSHOT";}
    static constexpr uint16_t get_id_static(){return MAVLINK_MSG_ID_TUNNEL;}
    const char *get_name()const override{return get_name_static();}
    uint16_t get_id()override{return get_id_static();}
    unsigned get_size()override{return MAVLINK_MSG_ID_TUNNEL_LEN+MAVLINK_NUM_NON_PAYLOAD_BYTES;}
    unsigned get_size_avg()override{return gpenmpc_snapshot_wire::fragment_count*get_size();}
    ~MavlinkStreamGPENMPCRflySnapshot()override{
        if(registry_)(void)registry_->release_view(link_,this);
    }
    std::uint64_t send_calls()const noexcept{return send_calls_;}
    std::uint64_t post_copy_expired()const noexcept{return post_copy_expired_;}
private:
    explicit MavlinkStreamGPENMPCRflySnapshot(Mavlink *m):MavlinkStream(m){}
    bool send()override{
        using namespace gpenmpc_rfly_stream;
        using gpenmpc_rfly_px4::ExportTake;
        if(!resolved_){
            auto *l=link_lifetime_registry();
            if(!l||l->lookup(_mavlink,link_)!=LinkAccess::Read)return false;
            resolved_=true; // never bind a same-address replacement
        }
        if(!registry_)registry_=snapshot_route_registry();
        if(!registry_)return false;
        bool sent=false;
        // Original three-fragment RSP1 packet, bounded work. No consumption
        // before capacity exists. Real send is void, NOT a delivery ACK.
        for(unsigned i=0;i<gpenmpc_snapshot_wire::fragment_count;++i){
            if(_mavlink->get_free_tx_buf()<get_size())break;
            gpenmpc_rfly_px4::SnapshotFragment f{};
            const auto r=registry_->take(link_,this,f,hrt_absolute_time());
            if(r!=ExportTake::Fragment)break;
            const auto now=hrt_absolute_time();
            if(!f.original_source_sample_us||now<f.original_source_sample_us||
               !f.original_valid_until_us||now>f.original_valid_until_us){
                ++post_copy_expired_;return sent;
            }
            mavlink_tunnel_t message{};
            message.target_system=f.target_system;message.target_component=f.target_component;
            message.payload_type=42002;message.payload_length=f.fragment.length;
            memcpy(message.payload,f.fragment.payload,f.fragment.length);
            mavlink_msg_tunnel_send_struct(_mavlink->get_channel(),&message);
            ++send_calls_;sent=true;
        }
        return sent;
    }
    gpenmpc_rfly_stream::SnapshotRouteRegistry *registry_{};
    gpenmpc_rfly_stream::LinkToken link_{};bool resolved_{};
    std::uint64_t send_calls_{},post_copy_expired_{};
};
#endif
