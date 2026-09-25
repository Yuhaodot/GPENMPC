// Include in the standalone host test build.
#include <memory>
#include <vector>
namespace {
using rawgp::Core;namespace gw=rawgp::gw;
struct TestFrame { unsigned char b[300]{};unsigned n{}; };
TestFrame frame(const Core&c,unsigned query,unsigned frag,mavlink_status_t&seq,int variant=0) {
    gw::RequestBytes body{};std::memcpy(body.data(),c.pairs+query*rawgp::PAIR,body.size());
    gw::Fragment f{};gw::fragment(body,frag,f);const auto&id=c.expected[query].identity;
    if(variant==1)f.payload[0]=0x70|frag;
    if(variant==2)f.payload[8]^=1;
    if(variant==3)f.payload[15]^=1;
    if(variant==6)f.payload[127]=1;
    mavlink_message_t msg{};TestFrame o{};
    mavlink_msg_tunnel_pack_status(variant==5?id.system+1:id.system,id.component,&seq,&msg,
        variant==4?254:255,190,42002,f.length,f.payload);
    o.n=mavlink_msg_to_send_buffer(o.b,&msg);return o;
}
void drain(Core&c,double&t){const unsigned char z=0;for(unsigned j=0;j<3&&!c.error;++j){t+=.001;c.update(&z,1,t);}}
bool query(Core&c,unsigned k,mavlink_status_t&seq,double&t){
    unsigned char aggregate[900]{};unsigned n=0;
    for(unsigned j=0;j<3;++j){auto f=frame(c,k,j,seq);std::memcpy(aggregate+n,f.b,f.n);n+=f.n;}
    t+=.001;return c.update(aggregate,n,t);
}
}
int wmain(int argc,wchar_t**argv){
    if(argc!=2)return 2;std::vector<unsigned char>pairs(rawgp::PAIR*rawgp::N+1);
    FILE*f=_wfopen(argv[1],L"rb");if(!f)return 2;const auto n=std::fread(pairs.data(),1,pairs.size(),f);
    const bool read=std::ferror(f)==0;std::fclose(f);if(!read||n!=rawgp::PAIR*rawgp::N)return 2;pairs.resize(n);
    unsigned total=0,passed=0;const auto check=[&](const char*name,bool ok){++total;if(ok)++passed;std::printf("%s %s\n",ok?"PASS":"FAIL",name);};
    const auto core=[&](){auto c=std::unique_ptr<Core>(new Core);c->initialize(pairs.data(),pairs.size());const unsigned char z=0;c->update(&z,1,0);return c;};
    auto c=core();mavlink_status_t seq{};double t=0;
    check("fixture59_and_unarmed_readiness",c->initialized&&!c->error&&c->tx_count==1&&c->transmitted[0].query==0);
    bool exact=true,causal=true;
    for(unsigned k=0;k<59&&!c->error;++k){query(*c,k,seq,t);exact=exact&&c->gp_calls==k+1&&c->completed==k+1&&
        !std::memcmp(c->queries[k].reply.data(),pairs.data()+596*k+310,286);drain(*c,t);}
    for(unsigned j=1;j<c->tx_count;++j)causal=causal&&c->transmitted[j].output_s>c->transmitted[j].queued_s;
    check("59_real_original_GP_calls_and_exact_286_byte_replies",exact&&c->gp_calls==59&&c->completed==59);
    check("177_query_frames_178_outputs_and_empty_queue",c->rx_frames==177&&c->tx_count==178&&c->queue_count==0&&c->fragment_index==0);
    check("next_tick_output_never_same_query_tick",causal);
    check("complete_final_accounting",c->finish()&&c->error==0);
    bool originals=true;for(unsigned k=0;k<59;++k)originals=originals&&!std::memcmp(c->queries[k].request.data(),pairs.data()+596*k,310);
    check("all_source_publication_times_and_bytes_preserved",originals);
    auto d=std::unique_ptr<Core>(new Core);check("wrong_fixture_size_rejected",!d->initialize(pairs.data(),pairs.size()-1)&&d->error==rawgp::CONFIG);
    auto bad=pairs;bad[300]^=1;d.reset(new Core);check("bad_fixture_checksum_rejected",!d->initialize(bad.data(),bad.size())&&d->error==rawgp::CONFIG);
    const auto one=[&](unsigned frag,int variant,unsigned first_sequence){auto x=core();mavlink_status_t s{};s.current_tx_seq=std::uint8_t(first_sequence);
        auto f0=frame(*x,0,frag,s,variant);x->update(f0.b,f0.n,.001);return x;};
    d=one(0,0,7);check("missing_or_reordered_mavlink_sequence",d->error==rawgp::SEQUENCE&&d->gp_calls==0);
    d=one(0,5,0);check("wrong_source",d->error==rawgp::SOURCE&&d->gp_calls==0);
    d=one(0,4,0);check("wrong_target",d->error==rawgp::SCHEMA&&d->gp_calls==0);
    d=one(0,1,0);check("unknown_schema",d->error==rawgp::SCHEMA&&d->gp_calls==0);
    d=one(0,2,0);check("wrong_output_generation",d->error==rawgp::FRAGMENT&&d->gp_calls==0);
    d=one(1,0,0);check("missing_first_fragment",d->error==rawgp::FRAGMENT&&d->gp_calls==0);
    d=core();seq={};auto a=frame(*d,0,0,seq);d->update(a.b,a.n,.001);a=frame(*d,0,0,seq);d->update(a.b,a.n,.002);
    check("duplicate_fragment",d->error==rawgp::FRAGMENT&&d->gp_calls==0);
    d=core();seq={};a=frame(*d,0,0,seq);a.b[a.n-1]^=1;d->update(a.b,a.n,.001);
    check("CRC_fault",d->error==rawgp::CRC&&d->gp_calls==0);
    const unsigned char zero=0;const auto failures=d->error;d->update(&zero,1,.002);
    check("first_failure_latches_and_no_restart",d->error==failures&&d->gp_calls==0);
    d=core();seq={};a=frame(*d,0,0,seq);d->update(a.b,a.n-1,.001);
    check("truncated_datagram_no_resync",d->error==rawgp::FRAME&&d->gp_calls==0);
    d=core();seq={};a=frame(*d,0,0,seq);a.b[2]=1;d->update(a.b,a.n,.001);
    check("unsupported_signing_flag_no_skip",d->error==rawgp::FRAME&&d->gp_calls==0);
    d=core();seq={};double nt=0;query(*d,0,seq,nt);a=frame(*d,1,0,seq);d->update(a.b,a.n,.002);
    check("query_overlaps_not_output_reply_queue",d->error==rawgp::OVERLAP&&d->gp_calls==1);
    d=core();seq={};unsigned char bytes[900]{};unsigned size=0;
    for(unsigned j=0;j<3;++j){a=frame(*d,0,j,seq,j==0?3:0);std::memcpy(bytes+size,a.b,a.n);size+=a.n;}
    d->update(bytes,size,.001);check("checksum_or_body_mismatch_before_GP",d->error==rawgp::FIXTURE&&d->gp_calls==0);
    d=core();d->update(&zero,1,0);check("nonincreasing_simulation_time",d->error==rawgp::TIME);
    d=core();d->update(&zero,1,std::nan(""));check("nonfinite_simulation_time",d->error==rawgp::TIME);
    d=core();d->update(&zero,0,.001);check("empty_native_aggregate",d->error==rawgp::CAPACITY);
    d=core();d->update(&zero,5000,.001);check("native_4999_byte_bound",d->error==rawgp::CAPACITY);
    d=core();d->raw_used=rawgp::RAW_CAPACITY;d->update(&zero,1,.001);check("fixed_ledger_bound_fail_closed",d->error==rawgp::CAPACITY);
    d=core();d->begin_qpc-=31*d->frequency;d->update(&zero,1,.001);check("host_resource_wall_bound_not_age_grant",d->error==rawgp::WALL_BOUND);
    d=core();seq={};a=frame(*d,0,0,seq);d->update(a.b,a.n,.001);d->finish();check("missing_remaining_fragment_at_stop",d->error==rawgp::INCOMPLETE);
    d=core();d->finish();check("no_queries_not_complete",d->error==rawgp::INCOMPLETE);
    d=core();seq={};nt=0;query(*d,0,seq,nt);d->finish();check("queued_reply_not_complete_at_stop",d->error==rawgp::INCOMPLETE);
    d=core();seq={};for(unsigned j=0;j<3;++j){a=frame(*d,0,j,seq,j==2?6:0);d->update(a.b,a.n,.001*(j+1));}
    check("nonzero_unused_payload_rejected",d->error==rawgp::FRAGMENT&&d->gp_calls==0);
    bool crc_ok=true;for(unsigned j=0;j<c->tx_count;++j){mavlink_message_t m{};crc_ok=crc_ok&&c->decode_frame(c->transmitted[j].bytes,c->transmitted[j].length,m,t)&&m.seq==std::uint8_t(j)&&m.sysid==255&&m.compid==190;}
    check("generated_MAVLink_reply_CRC_source_and_sequence",crc_ok);
    // Compare packetized and single-frame output, including CRC, sequence
    // and serializer tail-zero trimming.
    const auto paired_core=[&](){auto x=std::unique_ptr<Core>(new Core);x->initialize(pairs.data(),pairs.size(),true);x->update(&zero,1,0);return x;};
    auto paired=paired_core();seq={};nt=0;
    for(unsigned k=0;k<59&&!paired->error;++k){query(*paired,k,seq,nt);drain(*paired,nt);}
    bool paired_exact=paired->tx_count==c->tx_count;
    for(unsigned j=0;j<paired->tx_count&&j<c->tx_count;++j)paired_exact=paired_exact&&
        paired->transmitted[j].length==c->transmitted[j].length&&
        !std::memcmp(paired->transmitted[j].bytes,c->transmitted[j].bytes,c->transmitted[j].length);
    check("paired119_datagrams_preserve178_exact_original_frames_and59_GP",paired->finish()&&paired->datagram_count==119&&
        paired->rx_frames==177&&paired->gp_calls==59&&paired_exact);
    bool mapping=true,observed=true;unsigned next_frame=0,paired_packets=0;
    std::int64_t last_qpc=0;
    for(unsigned k=0;k<paired->datagram_count;++k){const auto&packet=paired->datagrams[k];
        mapping=mapping&&packet.frame_count>=1&&packet.frame_count<=2&&packet.first_frame==next_frame&&packet.length<=300;
        observed=observed&&packet.exposed_qpc>0&&packet.exposed_qpc>=last_qpc;last_qpc=packet.exposed_qpc;
        if(packet.frame_count==2)++paired_packets;unsigned offset=0;
        for(unsigned j=0;j<packet.frame_count;++j){const auto&original=paired->transmitted[next_frame++];
            mapping=mapping&&packet.offsets[j]==offset&&packet.lengths[j]==original.length&&
                !std::memcmp(packet.bytes+offset,original.bytes,original.length)&&packet.output_s==original.output_s;
            offset+=original.length;}
        mapping=mapping&&offset==packet.length;
    }
    check("paired_offsets_lengths_cover_exact_datagrams_without_padding",mapping&&paired_packets==59&&next_frame==178);
    check("paired_datagram_QPC_is_actual_shared_exposure_observation",observed);
    // Check width arithmetic.
    check("pair_width300_allowed301_not_combined",rawgp::pair_fits(145,155)&&!rawgp::pair_fits(145,156)&&
        !rawgp::pair_fits(301,1)&&!rawgp::pair_fits(5,145));
    d=paired_core();seq={};nt=0;query(*d,0,seq,nt);d->update(&zero,1,.002);d->finish();
    check("paired_missing_last_reply_frame_remains_incomplete",d->error==rawgp::INCOMPLETE&&d->queue_count==1&&d->tx_count==3);
    d=paired_core();seq={};nt=0;query(*d,0,seq,nt);d->queue[1].fragment=2;
    check("paired_missing_middle_queue_fragment_rejected",!d->output()&&d->error==rawgp::FRAGMENT&&d->tx_count==1);
    d=paired_core();seq={};nt=0;query(*d,0,seq,nt);d->datagram_count=rawgp::MAX_TX;
    check("paired_datagram_capacity_failure_before_exposure",!d->output()&&d->error==rawgp::CAPACITY&&d->tx_count==1);
    d=paired_core();seq={};nt=0;query(*d,0,seq,nt);d->tx_count=rawgp::MAX_TX-1;
    check("paired_two_frame_capacity_is_atomic_not_partial",!d->output()&&d->error==rawgp::CAPACITY&&d->queue_count==3);
    std::printf("RESULT %u/%u fixed_state_bytes=%zu gp_calls_positive=%u no_socket_no_board_no_MATLAB=1\n",passed,total,sizeof(Core),c->gp_calls);
    return passed==total?0:1;
}
