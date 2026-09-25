// Test ingress using PX4 method bodies and production codecs.
// Provide allocation, scheduling and locking fixtures.
#include "../rfly_vendor_integration/px4_wire/CanonicalLocalIngress.hpp"
#include <uORB/topics/gpenmpc_full_inner_ingress.h>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <vector>
namespace ww=gpenmpc_local_window_wire;
namespace ing=gpenmpc_local_ingress;
static unsigned checks{},failures{},frames{};
static void check(bool ok,const char*what){++checks;if(!ok){++failures;std::fprintf(stderr,"FAIL %s\n",what);}}
// Compile extracted method bodies with the test class qualifier.
#define ATOMIC_ENTER do {} while(0)
#define ATOMIC_LEAVE do {} while(0)
struct Meta {unsigned o_size=sizeof(gpenmpc_full_inner_ingress_s),o_queue=16;};
struct Node {
    Meta meta{};Meta*_meta=&meta;
    std::vector<unsigned char> storage;
    unsigned char*_data=nullptr;
    std::atomic<unsigned> _generation{0};bool _data_valid=false;
    unsigned publishes{},overwrites{};
    explicit Node(unsigned capacity):storage(sizeof(gpenmpc_full_inner_ingress_s)*capacity){meta.o_queue=capacity;}
#include "actual_node_copy.inc"
#include "actual_node_initial.inc"
#include "actual_node_range.inc"
    bool publish(const gpenmpc_full_inner_ingress_s&v){
        _data=storage.data();unsigned g=_generation.fetch_add(1);
        if(g>=meta.o_queue)++overwrites;
        std::memcpy(_data+meta.o_size*(g%meta.o_queue),&v,meta.o_size);
        _data_valid=true;++publishes;return true;
    }
};
enum class ORB_ID {INVALID,Ingress};
static Node*current_node{};static bool node_exists{},advertise_allowed=true;
static unsigned advertise_calls{},non_null_advertise_calls{};
static void*orb_advertise(const void*,const void*data){
    ++advertise_calls;if(data)++non_null_advertise_calls;
    if(!advertise_allowed)return nullptr;
    node_exists=true;return current_node;
}
struct Publication {
    void*_handle=nullptr;
    bool advertised()const{return _handle!=nullptr;}
    const void*get_topic()const{return &current_node->meta;}
#include "actual_publication_advertise.inc"
};
namespace uORB {
struct Manager {
    static Manager*get_instance(){static Manager value;return &value;}
    static void*orb_add_internal_subscriber(ORB_ID,uint8_t,unsigned*generation){
        if(!node_exists)return nullptr;
        *generation=current_node->get_initial_generation();return current_node;
    }
};
struct Subscription {
    void*_node=nullptr;ORB_ID _orb_id=ORB_ID::Ingress;uint8_t _instance=0;unsigned _last_generation=0;
    Subscription(){subscribe();}
    bool subscribe();bool valid()const{return _node!=nullptr;}
    bool update(gpenmpc_full_inner_ingress_s&v){
        if(!subscribe())return false;
        auto&n=*static_cast<Node*>(_node);
        return n._data_valid&&n._generation.load()!=_last_generation&&n.copy(&v,_last_generation);
    }
};
#include "actual_subscription_subscribe.inc"
}
struct Setup {
    Node node;Publication publisher;gpenmpc_ingress::Receiver receiver;
    Setup(unsigned capacity):node(capacity){current_node=&node;node_exists=false;advertise_allowed=true;advertise_calls=non_null_advertise_calls=0;}
    bool send(const ww::Encoder&encoder,unsigned index,uint64_t original){
        ww::Fragment f{};if(!encoder.fragment(index,f))return false;
        mavlink_message_t sent{},got{},parser{};mavlink_status_t status{},reported{};
        mavlink_msg_tunnel_pack(255,190,&sent,1,1,42002,f.length,f.payload);
        uint8_t raw[MAVLINK_MAX_PACKET_LEN]{};const auto n=mavlink_msg_to_send_buffer(raw,&sent);unsigned parsed=0;
        for(unsigned j=0;j<n;++j)if(mavlink_frame_char_buffer(&parser,&status,raw[j],&got,&reported)==MAVLINK_FRAMING_OK)++parsed;
        if(parsed!=1||receiver.receive(got,original,1,1,2)!=gpenmpc_ingress::Result::Accepted)return false;
        ++frames;
        // Same ordering as the actual receiver: per parsed frame, immediate
        // real Receiver::drain into the topic; no extra receiving owner.
        return receiver.drain(original,[&](const gpenmpc_ingress::Fields&fields){
            if(!publisher.advertised()&&!publisher.advertise())return false;
            gpenmpc_full_inner_ingress_s t{};gpenmpc_ingress::copy_to_topic(fields,receiver.counters(),t);return node.publish(t);
        });
    }
};
static ing::Configuration transport(){return {255,190,1,1,2,5000};}
static bool poll(uORB::Subscription&sub,ing::CanonicalLocalIngress&in,uint64_t now,unsigned&received){
    if(!sub.valid())return false;
    for(unsigned i=0;i<gpenmpc_full_inner_ingress_s::ORB_QUEUE_LENGTH;++i){
        gpenmpc_full_inner_ingress_s t{};if(!sub.update(t))break;
        const auto a=gpenmpc_argument_transport::from_topic(t,sub._last_generation);++received;
        if(!in.receive(a,now))return false;
    }
    return in.tick(now);
}
static bool load(const char*path,ww::Window&w){
    FILE*f=std::fopen(path,"rb");if(!f)return false;
    uint8_t head[12]{},wire[ww::window_bytes]{};
    const bool ok=std::fread(head,1,12,f)==12&&!std::memcmp(head,"RWI1",4)&&std::fread(wire,1,sizeof wire,f)==sizeof wire;
    std::fclose(f);return ok&&ww::copy_wire_to_window(wire,sizeof wire,0,w);
}
int main(int argc,char**argv){
    if(argc!=2)return 2;
    ww::Window window{};if(!load(argv[1],window))return 3;
    ww::Configuration cfg{};cfg.max_assembly_us=1000000;cfg.expected.identity={123,7,1,1};cfg.expected.leg_index=window.leg_index;
    cfg.expected.configuration_sha256=gpenmpc_local_gp_wire::canonical_configuration();
    for(unsigned k=0;k<8;++k){cfg.expected.execution_session_sha256[k]=k+1;
        const auto*p=window.reference_asset_sha256+4*k;cfg.expected.reference_asset_sha256[k]=(uint32_t(p[0])<<24)|(uint32_t(p[1])<<16)|(uint32_t(p[2])<<8)|p[3];}
    cfg.expected.task_sha256=cfg.expected.reference_asset_sha256;
    ww::Encoder encoder(cfg.expected,window);check(encoder.valid(),"actual retained original RWI window encodes");
    check(gpenmpc_full_inner_ingress_s::ORB_QUEUE_LENGTH==16,"official generated topic queue16");
    {
        Setup s(16);uORB::Subscription early;check(!early.valid(),"absent node cannot silently become a startup-ready subscriber");
        advertise_allowed=false;check(!s.publisher.advertise()&&!early.subscribe(),"failed advertisement remains unavailable without fake data");
        advertise_allowed=true;check(s.publisher.advertise()&&early.subscribe(),"no-data advertise then existing subscription succeeds");
        gpenmpc_full_inner_ingress_s out{};check(!early.update(out)&&s.node.publishes==0&&s.node._generation.load()==0&&non_null_advertise_calls==0,
            "actual advertise body publishes no sample sequence or timestamp");
        ww::Assembler receiver(cfg);ing::CanonicalLocalIngress in(transport(),&receiver);unsigned read=0;
        for(unsigned first=0;first<ww::fragment_count;first+=16){
            const unsigned end=first+16<ww::fragment_count?first+16:ww::fragment_count;
            for(unsigned k=first;k<end;++k)check(s.send(encoder,k,1000000+k),"actual parsed RWW frame publishes unchanged receive HRT");
            check(poll(early,in,1000000+end+5,read),"16 actual Receiver publications before one bounded ingress poll");
            check(read==end&&in.diagnostics().last_reception_sequence==end&&in.diagnostics().last_uorb_generation==end&&
                in.diagnostics().last_original_arrival_us==1000000+end-1,"all original counts and arrival clocks preserved");
        }
        const auto*received=receiver.ready_window(1000300);uint8_t a[ww::window_bytes]{},b[ww::window_bytes]{};
        check(received&&ww::copy_window_to_wire(window,0,a,sizeof a)&&ww::copy_window_to_wire(*received,0,b,sizeof b)&&!std::memcmp(a,b,sizeof a),
            "244 RWW fragments preserve every original window byte");
        check(receiver.diagnostics().first_original_arrival_us==1000000&&s.receiver.counters().published==244&&s.receiver.pending()==0,
            "one original first-arrival lifetime and unchanged per-frame receiver drain");
    }
    {
        Setup s(8);check(s.publisher.advertise(),"old capacity replay starts advertised");uORB::Subscription sub;
        ww::Assembler receiver(cfg);ing::CanonicalLocalIngress in(transport(),&receiver);unsigned read=0;
        for(unsigned k=0;k<16;++k)check(s.send(encoder,k,2000000+k),"old-capacity real parsed burst");
        check(!poll(sub,in,2000020,read)&&receiver.diagnostics().first_fault==ww::Fault::Index&&s.node.overwrites==8,
            "old8 with16 before poll deterministically loses first8 and fails closed");
        check(s.receiver.counters().published==16&&s.receiver.counters().first_fault==gpenmpc_ingress::Result::Accepted,
            "uORB overwrite is not falsely reported as Receiver internal queue overflow");
    }
    {
        Setup s(16);uORB::Subscription late;check(!late.valid(),"cold missing node reproduced");
        for(unsigned k=0;k<4;++k)check(s.send(encoder,k,3000000+k),"cold first burst creates topic only on publication");
        ww::Assembler receiver(cfg);ing::CanonicalLocalIngress in(transport(),&receiver);unsigned read=0;
        check(late.subscribe()&&late._last_generation==3&&!poll(late,in,3000010,read)&&receiver.diagnostics().first_fault==ww::Fault::Index,
            "actual lazy subscribe starts at latest: queue enlargement alone is insufficient");
    }
    {
        Setup s(16);s.publisher.advertise();uORB::Subscription sub;ww::Assembler receiver(cfg);ing::CanonicalLocalIngress in(transport(),&receiver);unsigned read=0;
        check(s.send(encoder,0,4000000)&&poll(sub,in,4000001,read),"overflow test has a genuine accepted baseline");
        for(unsigned k=1;k<=17;++k)check(s.send(encoder,k,4000010+k),"capacity+1 real parsed burst");
        check(!poll(sub,in,4000040,read)&&in.diagnostics().first_fault==ing::Fault::SequenceGap,
            "queue16 overflow remains permanently fail closed");
        const auto*bad=in.first_fault_arrival();check(bad&&bad->fields.timestamp==4000012&&bad->fields.reception_sequence==3&&
            !in.tick(4000050)&&!receiver.ready_window(4000050),"first dropped-gap evidence and original time retained without reset");
    }
    {
        Setup s(16);s.publisher.advertise();uORB::Subscription sub;ww::Assembler receiver(cfg);ing::CanonicalLocalIngress in(transport(),&receiver);unsigned read=0;
        check(s.send(encoder,0,5000000)&&poll(sub,in,5000001,read),"age regression actual first fragment");
        check(!in.tick(6000001)&&receiver.diagnostics().first_fault==ww::Fault::Expired&&receiver.diagnostics().first_original_arrival_us==5000000,
            "unchanged one-second assembly expiry is not renewed by more queue space");
    }
    std::printf("{\"checks\":%u,\"failed\":%u,\"actual_crc_parsed_frames\":%u,\"original_window_fragments\":244,\"queue_old\":8,\"queue_new\":16,\"scheduler_and_allocation_mock\":true,\"board\":0,\"COM\":0}\n",checks,failures,frames);
    return failures?1:0;
}
