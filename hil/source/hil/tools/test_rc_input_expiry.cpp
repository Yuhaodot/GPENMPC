// Test the task receiver and ingress with retained sources
// and mock clock, session, uORB and publication.
#define GPENMPC_TASK_INPUT_TEST_MAIN retained_task_owner_main
#include "../rfly_vendor_integration/test_local_task_input_owner.cpp"
#undef GPENMPC_TASK_INPUT_TEST_MAIN

static unsigned rc_cases{};
struct RcExpiryFixture {
    CycleOwner cycle{false,false,nullptr,true,true};
    TaskOwner original;
    li::LocalTaskInputConfiguration cfg;
    std::unique_ptr<li::CanonicalLocalTaskInputOwner> owner;
    li::LocalTaskInputPort port{};
    ss::Receipt receipt{};
    tw::Message msg{};
    tw::Bytes bytes{};
    bool captured{};
    explicit RcExpiryFixture(unsigned mode=7):cfg(input_config(original.window)) {
        ++rc_cases;extra_clear();cfg.input_assembly_max_us=50000;cfg.outer_max_age_us=700000;
        cfg.runtime_state_only=(mode&1)!=0;cfg.component_initialization=(mode&2)!=0;
        cfg.operator_reference=(mode&4)!=0;
        owner.reset(new li::CanonicalLocalTaskInputOwner(cfg));port=owner->port();capture(0);
    }
    void capture(unsigned row) {
        if(captured) {
            // Advance the transport clock while the independent mock Commander publishes.
            // Preserve odometry and held input timestamps.
            bus_status.timestamp=bus_mode.timestamp=clock_us;
            ++bus_generation[1];++bus_generation[2];
            check(cycle.cycle->release_disarmed(),"release original fixture capture");
        }
        check(observe(cycle,row)==li::LocalCapture::Accepted,"capture genuine fixture odometry");captured=true;
        if(cfg.runtime_state_only)
            check(port.observe_source(port.owner,*cycle.cycle->snapshot(),receipt,true)==li::LocalTaskRead::Ready,"retain exported source anchor");
        msg=message(cycle,row,cfg);msg.outer_target4[0]=.1;msg.outer_target4[1]=.2;
        msg.outer_target4[2]=-.3;msg.outer_target4[3]=.4;
        check(tw::encode(msg,bytes),"encode actual source-bound six-fragment packet");
    }
    bool send(unsigned index,std::uint64_t stamp=0) {
        gpenmpc_argument_transport::Fragment f{};if(!tw::fragment(bytes,index,f))return false;
        gpenmpc_argument_transport::Arrival a{};a.fields.timestamp=stamp?stamp:clock_us;
        a.fields.payload_length=f.length;std::memcpy(a.fields.payload,f.payload,f.length);
        return owner->receive(a,clock_us);
    }
    bool prefix(unsigned count) { for(unsigned j=0;j<count;++j){if(!send(j))return false;clock_us+=100;}return true; }
    li::LocalTaskRead install() { return port.observe_source(port.owner,*cycle.cycle->snapshot(),receipt,false); }
    void no_control() {check(no_math_or_publication(cycle),"receiver fixture never runs control or hardware publication");}
};

static void boundary_and_scope() {
    for(const auto age:{49999ULL,50000ULL,50001ULL,75000ULL}) {
        RcExpiryFixture f;const auto first=clock_us;check(f.prefix(1),"first fragment admitted");
        clock_us=first+age;check(f.owner->tick(clock_us,false),"RC age is not a terminal parser error");
        for(unsigned j=1;j<6;++j)check(f.send(j),"same generation tail retains original order");
        check(f.owner->diagnostics().maximum_assembly_span_us==age,"assembly span remains diagnostic, not RC admission");
        check(f.install()==li::LocalTaskRead::Ready&&f.owner->diagnostics().bindings==1&&
            f.owner->diagnostics().outer_installs==1,"valid source and unexpired command install independent of old assembly timer");
        f.no_control();
    }
    for(unsigned mode=0;mode<7;++mode) {
        RcExpiryFixture f(mode);const auto first=clock_us;check(f.prefix(1),"non-RC prefix admitted");
        clock_us=first+50001;check(!f.owner->tick(clock_us,false)&&f.owner->diagnostics().first_fault==li::LocalTaskInputFault::Expired,
            "every missing RC mode flag retains original fatal expiration");f.no_control();
    }
}

static void delayed_partial_and_ready() {
    for(unsigned count=1;count<=6;++count) {
        RcExpiryFixture f;const auto first=clock_us;check(f.prefix(count),"partial or complete prefix admitted");
        clock_us=first+75000;
        check(f.owner->tick(clock_us,false)&&f.owner->tick(clock_us+50,false),"RC partial and ready slots have no separate assembly timeout");clock_us+=100;
        for(unsigned j=count;j<6;++j)check(f.send(j),"late tail fully validated");
        check(!f.owner->diagnostics().bindings&&!f.owner->diagnostics().outer_installs,
            "assembly alone grants no numerical or command installation");
        check(f.install()==li::LocalTaskRead::Ready&&f.owner->diagnostics().bindings==1,
            "complete source-valid command passes normal installation checks");
        f.capture(15);check(f.prefix(6)&&f.install()==li::LocalTaskRead::Ready,"next fresh higher generation installs normally");
        check(f.owner->diagnostics().bindings==2&&f.owner->diagnostics().outer_installs==2,"each legitimate complete packet installs exactly once");
        const auto retained=f.owner->diagnostics();f.owner->stop(clock_us);
        check(f.owner->diagnostics().maximum_assembly_span_us==retained.maximum_assembly_span_us,
            "stop preserves observed assembly-span diagnostic");f.no_control();
    }
}

static void invalid_late_packets() {
    // Whole-message integrity/identity/source/command errors remain terminal,
    // even when the six-fragment staging spans the former assembly deadline.
    for(unsigned bad=0;bad<5;++bad) {
        RcExpiryFixture f;
        if(bad==1)f.msg.session_sha[0]^=1;
        if(bad==2)f.msg.source.state_and_origin_sha256[0]^=1;
        if(bad==3){f.msg.source.sample_us=clock_us+200000;f.msg.source.publication_us=f.msg.source.sample_us;
            f.msg.source.original_receipt_us=f.msg.source.sample_us;}
        if(bad==4)f.msg.outer_target4[3]=gpenmpc_operator_reference::vertical_speed_mps+1;
        check(tw::encode(f.msg,f.bytes),"negative packet remains structurally encoded");if(bad==0)f.bytes[615]^=1;
        const auto first=clock_us;check(f.prefix(1),"negative first fragment admitted before complete validation");
        clock_us=first+75000;check(f.owner->tick(clock_us,false),"negative late packet keeps normal validation path");
        for(unsigned j=1;j<5;++j)check(f.send(j),"negative tail prefix stays ordered");
        const bool complete=f.send(5);
        if(bad==2||bad==4)check(complete&&f.install()==li::LocalTaskRead::Invalid,"late source or command error rejected at normal observe gate");
        else check(!complete,"late invalid complete packet remains fatal");
        const li::LocalTaskInputFault expected[]={li::LocalTaskInputFault::Integrity,li::LocalTaskInputFault::IdentityMismatch,
            li::LocalTaskInputFault::Source,li::LocalTaskInputFault::Clock,li::LocalTaskInputFault::OuterMutation};
        check(f.owner->diagnostics().first_fault==expected[bad]&&!f.owner->diagnostics().bindings,
            "exact invalid reason retained; no invalid installation");
        const auto first_fault=f.owner->diagnostics();f.owner->stop(clock_us+1);
        check(f.owner->diagnostics().first_fault==first_fault.first_fault&&
            f.owner->diagnostics().first_fault_us==first_fault.first_fault_us,"cleanup cannot overwrite actual first fault");f.no_control();
    }
    for(unsigned bad=0;bad<9;++bad) {
        RcExpiryFixture f;const auto first=clock_us;check(f.prefix(2),"sequence test prefix admitted");
        clock_us=first+75000;check(f.owner->tick(clock_us,false),"late sequence test remains active");
        gpenmpc_argument_transport::Fragment piece{};const unsigned index=bad==0?1:bad==1?3:bad==2?0:2;
        check(tw::fragment(f.bytes,index,piece),"prepare deliberately invalid late fragment");
        if(bad==2)gpenmpc_argument_transport::put64(piece.payload+1,f.msg.source.source_generation+1);
        if(bad==3)--piece.length;
        if(bad==7)gpenmpc_argument_transport::put64(piece.payload+1,f.msg.source.source_generation+1);
        if(bad==8)gpenmpc_argument_transport::put64(piece.payload+1,f.msg.source.source_generation-1);
        gpenmpc_argument_transport::Arrival a{};a.fields.timestamp=bad==4?clock_us+1:bad==5?first-1:bad==6?0:clock_us;
        a.fields.payload_length=piece.length;std::memcpy(a.fields.payload,piece.payload,piece.length);
        check(!f.owner->receive(a,clock_us)&&f.owner->diagnostics().first_fault==li::LocalTaskInputFault::Fragment,
            "duplicate, gap, interleaving, length, generation or invalid timestamp remains fatal");f.no_control();
    }
    { RcExpiryFixture f;const auto first=clock_us;check(f.prefix(6),"complete known packet ready");
      clock_us=first+75000;check(f.owner->tick(clock_us,false)&&f.install()==li::LocalTaskRead::Ready,"late complete packet installed only through normal source gate");
      check(!f.send(0)&&f.owner->diagnostics().first_fault==li::LocalTaskInputFault::Regression,"installed same generation cannot replay");f.no_control(); }
    { RcExpiryFixture f;f.msg.source.state_and_origin_sha256[0]^=1;check(tw::encode(f.msg,f.bytes),"encode unknown ready source");
      const auto first=clock_us;check(f.prefix(6),"unknown source structurally ready");clock_us=first+50001;
      check(f.owner->tick(clock_us,false)&&f.install()==li::LocalTaskRead::Invalid&&f.owner->diagnostics().first_fault==li::LocalTaskInputFault::Source,
          "complete ready unknown source cannot bypass normal source validation");f.no_control(); }
    { RcExpiryFixture f;const auto first=clock_us;check(f.prefix(6),"complete ready packet before busy check");clock_us=first+75000;
      check(f.owner->tick(clock_us,false)&&!f.send(0)&&f.owner->diagnostics().first_fault==li::LocalTaskInputFault::Busy,
          "unconsumed ready slot still rejects another message instead of silently overwriting it");f.no_control(); }
}

static void previous_command_not_renewed() {
    RcExpiryFixture f;check(f.prefix(6)&&f.install()==li::LocalTaskRead::Ready,"install original current command");
    li::LocalTaskView before{},after{};check(f.port.read(f.port.owner,*f.cycle.cycle->snapshot(),f.receipt,before)==li::LocalTaskRead::Missing&&
        before.inputs.rotor,"fixture intentionally lacks route window but exposes original current input");
    f.capture(1);const auto earlier_snapshot=*f.cycle.cycle->snapshot();
    f.msg.outer_target4[3]=1;check(tw::encode(f.msg,f.bytes),"encode distinct candidate command");
    const auto first=clock_us;check(f.prefix(3),"late candidate prefix admitted");clock_us=first+125000;
    check(f.owner->tick(clock_us,false),"old assembly timer does not interrupt ordered message");for(unsigned j=3;j<6;++j)check(f.send(j),"late candidate structurally complete");
    f.capture(15);check(f.owner->diagnostics().missing_inputs==1,"complete packet beyond 100-ms original source age retires before binding");
    // Inspect held fields with the captured owner key without making it eligible for control.
    check(f.port.read(f.port.owner,earlier_snapshot,f.receipt,after)==li::LocalTaskRead::Missing&&after.inputs.rotor,
        "old valid current is available independently of discarded staging");
    check(same(before.inputs.target4,after.inputs.target4,sizeof before.inputs.target4)&&
        before.inputs.outer.generation==after.inputs.outer.generation&&before.inputs.outer.board_rx_us==after.inputs.outer.board_rx_us&&
        before.inputs.outer.valid_until_us==after.inputs.outer.valid_until_us&&f.owner->diagnostics().bindings==1&&
        f.owner->diagnostics().outer_installs==1,"retirement never changes old command, generation, receive time or expiry");
    clock_us=before.inputs.outer.valid_until_us+1;
    check(f.port.read(f.port.owner,earlier_snapshot,f.receipt,after)==li::LocalTaskRead::Invalid&&
        f.owner->diagnostics().first_fault==li::LocalTaskInputFault::OuterExpired,"original held command expires without a fresh command");f.no_control();
}

static void common_ingress_stays_strict() {
    for(unsigned bad=0;bad<3;++bad) {
        RcExpiryFixture f;auto cfg=exchange_config();gpenmpc_local_ingress::CanonicalLocalIngress ingress(cfg.transport,nullptr,f.owner.get(),true);
        const auto first=clock_us;
        auto arrival=[&](unsigned j){gpenmpc_argument_transport::Fragment p{};tw::fragment(f.bytes,j,p);
            gpenmpc_argument_transport::Arrival a{};auto&v=a.fields;v.timestamp=clock_us;v.source_system=255;v.source_component=190;
            v.target_system=v.target_component=1;v.receiver_instance=2;v.payload_type=gpenmpc_ingress::payload_type;
            v.payload_length=p.length;v.wire_payload_length=p.length+5;v.reception_sequence=j+1;a.uorb_generation=j+1;
            std::memcpy(v.payload,p.payload,p.length);return a;};
        check(ingress.receive(arrival(0),clock_us,false),"common ingress admits real envelope");
        clock_us=first+75000;check(ingress.tick(clock_us,false),"late RC packet does not reset common ingress watermarks");
        auto a=arrival(1);if(bad==0)++a.fields.reception_sequence;if(bad==1)++a.uorb_generation;if(bad==2)++a.fields.source_system;
        check(!ingress.receive(a,clock_us,false),"late tail still traverses central envelope validation");
        const gpenmpc_local_ingress::Fault expected[]={gpenmpc_local_ingress::Fault::SequenceGap,gpenmpc_local_ingress::Fault::UorbGap,gpenmpc_local_ingress::Fault::Source};
        check(ingress.diagnostics().first_fault==expected[bad]&&!f.owner->diagnostics().bindings,"central counter/source error is not ignored");f.no_control();
    }
}

static void held_input_age_not_renewed() {
    RcExpiryFixture f;const auto original_sample=f.msg.source.sample_us;
    check(f.prefix(6)&&f.install()==li::LocalTaskRead::Ready,"install original held-input age baseline");
    f.capture(1);const auto first=clock_us;check(f.prefix(2),"stage independent late candidate");
    clock_us=first+75000;check(f.owner->tick(clock_us,false),"late candidate keeps original source age");
    for(unsigned j=2;j<6;++j)check(f.send(j),"late candidate tail validated");
    f.capture(15);check(f.msg.source.sample_us-original_sample>gpenmpc_local_input::maximum_held_input_age_us,
        "new genuine source is beyond unchanged 100-ms held-input margin");
    li::LocalTaskView view{};
    check(f.port.read(f.port.owner,*f.cycle.cycle->snapshot(),f.receipt,view)==li::LocalTaskRead::Missing&&
        !view.inputs.rotor&&f.owner->diagnostics().bindings==1&&f.owner->diagnostics().outer_installs==1,
        "expired held input cannot be reused after late packet retirement");
    check(f.owner->diagnostics().missing_inputs>=1&&f.owner->diagnostics().first_fault==li::LocalTaskInputFault::None,
        "missing fresh input grants no new command and is not concealed as an install");f.no_control();
}

// Vary synthetic timestamps around retained state values and execute the
// Adapter -> Cycle/Phase -> Builder -> ABI -> Consumption path with mock uORB publication.
struct ControlIntervalFixture:CycleOwner {
    explicit ControlIntervalFixture(bool manual):CycleOwner(false,false,nullptr,true,true){
        cycle.reset();io.reset();
        // Create a new host owner after the base fixture's destructor revokes its authority.
        authority=MockAuthority{};
        auto consumption=config_consumption();consumption.operator_reference=manual;
        if(manual){
            // Use manual phase rate 1 without autonomous phase acceleration or outer-force state.
            consumption.numerical.initial_phase_acceleration=0;
            for(double& value:consumption.numerical.initial_outer_i)value=0;
        }
        io.reset(new li::Px4CanonicalLocalIo(config_source(),consumption,100000,&authority));active_io=io.get();
        auto cc=cycle_config();cc.component_initialization=true;cc.runtime_state_only=true;
        cc.operator_reference=cc.context.operator_reference=manual;cc.context.allow_valid_held_inputs=true;
        cycle.reset(new li::CanonicalLocalExecutionCycle(*io,*phase,*reader,cc));
        check(load_window(false),"interval fixture loads real retained reference window");
    }
};

static void manual_control_intervals() {
    const auto raw1=raw_fixture[1],raw2=raw_fixture[2];const auto old1=original[1],old2=original[2];
    const auto row1=rows[1],row2=rows[2];
    for(const bool manual:{true,false})for(const uint64_t delta:{50000ULL,50001ULL,401000ULL,408236ULL,1000000ULL,20000000ULL}){
        if(!manual&&delta>50001)continue;
        ++rc_cases;raw_fixture[1]=raw1;raw_fixture[2]=raw2;original[1]=old1;original[2]=old2;rows[1]=row1;rows[2]=row2;
        for(unsigned i=1;i<=2;++i){
            raw_fixture[i].timestamp_sample=raw_fixture[0].timestamp_sample+delta+(i==2?10000:0);
            raw_fixture[i].timestamp=raw_fixture[i].timestamp_sample+100;
            original[i].tags[0]=raw_fixture[i].timestamp_sample*1000;
            rows[i].args[9]=original[i].input[34]=double(i==1?delta:10000)*1e-6;
        }
        extra_clear();ControlIntervalFixture f(manual);li::CanonicalLocalGpPending gp(*f.cycle);
        unsigned completed=0;
        for(unsigned i=0;i<3;++i){
            StepInputs in{};const bool captured=f.capture(i,in);
            if(!manual&&i==1&&delta>50000){
                check(!captured&&f.io->diagnostics().source_adapter_fault==od::Failure::SampleDelta&&publish_calls==1,
                    "autonomous 50001-us interval still rejects before second control");break;
            }
            check(captured,"real active adapter accepts valid fresh source and actual committed interval");
            if(!captured)break;
            if(i)check(f.cycle->snapshot()->actual_sample_delta_us()==(i==1?delta:10000),"adapter preserves unclipped committed-source delta");
            if(manual){
                in.value.target4[0]=1.;in.value.target4[1]=.2;in.value.target4[2]=-.3;in.value.target4[3]=.4;
                gpenmpc_consumption::CanonicalSha256 h;for(double value:in.value.target4)h.real(value);in.value.outer.payload_sha256=h.finish();
            }
            const auto executed=f.cycle->execute(in.value);
            if(executed!=li::LocalCycleExecute::Committed)std::fprintf(stderr,"interval manual=%u dt=%llu step=%u cycle=%u io=%u phase=%u adapter=%u\n",unsigned(manual),static_cast<unsigned long long>(delta),i,unsigned(f.cycle->diagnostics().first_fault),unsigned(f.io->diagnostics().first_fault),unsigned(f.phase->diagnostics().first_fault),unsigned(f.io->diagnostics().source_adapter_fault));
            check(executed==li::LocalCycleExecute::Committed,"real production private74 and joint install accept interval");
            if(executed!=li::LocalCycleExecute::Committed)break;
            check(gp.observe_actual_commit()==li::LocalGpBegin::NotRequired,"existing component-only unavailable GP path retained");
            const auto& actual=gp.retained_actual_feedback().actual;
            const double expected_dt=i?double(i==1?delta:10000)*1e-6:rows[0].args[9];
            li::LocalNumericalObservation observed{};
            check(f.io->copy_local_numerical_state(observed)&&same(observed.last_installed.actual_input36+34,&expected_dt,sizeof expected_dt),"actual numerical input dt is bit-exact not clamped");
            check(actual.token.lease_envelope.timestamp_sample_us==raw_fixture[i].timestamp_sample&&actual.original_valid_until_us<=raw_fixture[i].timestamp_sample+5000,
                "fresh sample identity and original output lifetime remain independent of control interval");
            for(unsigned j=0;j<6;++j)check(std::isfinite(actual.actual_control16[j])&&actual.actual_control16[j]>=0&&actual.actual_control16[j]<=1,"rotor output remains finite and within original absolute bounds");
            ++completed;
        }
        check(completed==(manual||delta<=50000?3U:1U),"manual long interval followed by next ordinary step completes; autonomous exception remains");
        check(publish_calls==completed&&f.phase->diagnostics().installs==completed,"no fabricated intermediate ticks or publications");
    }
    raw_fixture[1]=raw1;raw_fixture[2]=raw2;original[1]=old1;original[2]=old2;rows[1]=row1;rows[2]=row2;
}

int wmain(int argc,wchar_t**argv) {
    if(argc!=4||!load(argv[1],argv[2])||!load_private(argv[2])||!parse_hash(argv[3]))return 2;
    const auto canonical=gw::canonical_configuration();for(unsigned j=0;j<32;++j)config.configuration_sha256[j]=uint8_t(canonical[j/4]>>(24-8*(j%4)));
    boundary_and_scope();delayed_partial_and_ready();invalid_late_packets();previous_command_not_renewed();common_ingress_stays_strict();held_input_age_not_renewed();manual_control_intervals();
    std::printf("{\"scope\":\"RC_INPUT_EXPIRY_OFFLINE\",\"cases\":%u,\"checks\":%u,\"failed\":%u,\"rc_separate_assembly_deadline\":false,\"manual_control_interval_upper_bound\":null,\"manual_interval_fixtures_us\":[50000,50001,401000,408236,1000000,20000000],\"autonomous_control_limit_us\":50000,\"non_rc_assembly_limit_us\":50000,\"COM\":0,\"hardware_actions\":0}\n",rc_cases,checks,failed);
    return failed?1:0;
}
