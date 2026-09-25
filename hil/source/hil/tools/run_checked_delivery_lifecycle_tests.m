function result=run_checked_delivery_lifecycle_tests()
% Test the lifecycle state machine using synthetic board and plant observations.
root=string(gpenmpc_external_path('delivery_method_project'));
oldPath=path; cleanup=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(root,'matlab'),'-begin');
addpath(fileparts(mfilename('fullpath')),'-begin');
cfg=jsondecode(fileread(fullfile(root,'config','DELIVERY_LIFECYCLE_V1.json')));
assert(gpenmpcHil.sha256File(fullfile(root,'config','DELIVERY_LIFECYCLE_V1.json'))== ...
    "B1CCE6C7C11CC87B23D23B7FC683039BD442C8DB5598F45F75DDFC1295295B61");
assert(cfg.service_dwell_s==8 && cfg.native_land_confirmation_timeout_s==30);
checks=struct('name',{},'pass',{});
state=gpenmpcHil.newDeliveryState(2.21); obs=observation(); now=0.0;
last=struct();
events=strings(0,1); count=zeroCounts(); parityCases=0;
step(); require('initial_arm_request',last.request_arm);
obs.armed=true; now=0.05; step();
require('initial_takeoff_state',state.name=="INITIAL_TAKEOFF");
obs.initial_takeoff_complete=true; obs.altitude_agl_m=10;
obs.plant_contact=false; obs.px4_landed=false;
now=25; step(); require('initial_takeoff_to_transit',state.name=="TRANSIT");
payloadAfter=[1.75,0.98,0.55,0.0]; dwellObserved=zeros(4,1);
for ordinal=1:4
    obs.in_service_window=true; obs.service_phase="SERVICE_DESCENT";
    obs.service_ordinal=ordinal; obs.payload_update_ack=false;
    obs.px4_offboard=true; obs.land_ack=false; obs.px4_auto_land=false;
    now=now+0.05; step();
    obs.altitude_agl_m=4.9; now=now+0.05; step();
    require(sprintf('service_%d_one_land_request',ordinal),last.request_land);
    require(sprintf('service_%d_clock_hold_for_land',ordinal),~last.advance_reference);
    obs.land_ack=true; obs.px4_auto_land=true; obs.px4_landed=true;
    obs.plant_contact=false; now=now+0.05; step();
    require(sprintf('service_%d_px4_ground_alone_no_service',ordinal),state.name=="NATIVE_LAND" && ~last.allow_payload_update);
    obs.px4_landed=false; obs.plant_contact=true; now=now+0.05; step();
    require(sprintf('service_%d_plant_ground_alone_no_service',ordinal),state.name=="NATIVE_LAND" && ~last.allow_payload_update);
    obs.px4_landed=true; obs.altitude_agl_m=0; now=now+0.05; step();
    require(sprintf('service_%d_dual_ground_confirm',ordinal),state.name=="GROUND_CONFIRM");
    now=now+0.51; step();
    require(sprintf('service_%d_standard_disarm_request',ordinal),last.request_disarm);
    obs.armed=false; now=now+0.05; step();
    require(sprintf('service_%d_ground_dwell_start',ordinal),state.name=="SERVICE_GROUNDED_DISARMED" && last.align_ground_dwell_start);
    serviceStart=now;
    obs.service_phase="SERVICE_GROUND_DWELL";
    for k=1:159
        now=serviceStart+0.05*k; step();
        assert(~last.allow_payload_update && last.advance_reference);
    end
    require(sprintf('service_%d_no_unload_before_8s',ordinal),state.total_payload_update_request_count==ordinal-1);
    obs.service_phase="SERVICE_ASCENT"; now=serviceStart+8.0; step();
    require(sprintf('service_%d_unload_after_8s',ordinal),last.allow_payload_update && ~obs.armed && obs.px4_landed && obs.plant_contact);
    dwellObserved(ordinal)=now-serviceStart;
    % Missing/delayed acknowledgement cannot repeat the mass mutation request.
    now=now+0.05; step();
    require(sprintf('service_%d_single_payload_request',ordinal),~last.allow_payload_update);
    obs.payload_update_ack=true; obs.payload_after_service_kg=payloadAfter(ordinal);
    now=now+0.05; step();
    require(sprintf('service_%d_payload_committed',ordinal),state.name=="WAIT_OFFBOARD" && state.payload_kg==payloadAfter(ordinal));
    obs.payload_update_ack=false; obs.px4_offboard=false;
    now=now+0.05; step(); require(sprintf('service_%d_offboard_request',ordinal),last.request_offboard);
    obs.px4_offboard=true; now=now+0.05; step();
    now=now+0.05; step(); rearmStart=now;
    now=rearmStart+1.01; step(); require(sprintf('service_%d_rearm_request',ordinal),last.request_arm);
    obs.armed=true; now=now+0.05; step();
    require(sprintf('service_%d_relaunch',ordinal),state.name=="SERVICE_ASCENT");
    obs.px4_landed=false; obs.plant_contact=false; obs.altitude_agl_m=10;
    obs.in_service_window=false; obs.service_phase=""; now=now+11; step();
    require(sprintf('service_%d_transit_resumes',ordinal),state.name=="TRANSIT");
end
obs.mission_complete=true; obs.at_return_base=true;
now=now+0.05; step();
require('final_return_land_requested',last.request_land && state.name=="FINAL_NATIVE_LAND");
obs.land_ack=true; obs.px4_auto_land=true; obs.px4_landed=true;
obs.plant_contact=true; obs.altitude_agl_m=0; now=now+5; step();
require('final_standard_disarm_requested',last.request_disarm && ~last.advance_reference);
obs.armed=false; now=now+0.05; step();
require('complete_after_final_disarm',state.name=="COMPLETE" && state.task_complete);
require('five_land_four_relaunch',count.land==5 && state.completed_relaunches==4 && state.completed_ground_services==4);
require('five_arm_five_standard_disarm',count.arm==5 && count.disarm==5);
require('four_payload_no_airdrop',count.payload==4 && count.airborne_payload==0 && state.payload_kg==0);
require('four_exact_eight_second_dwells',all(abs(dwellObserved-8)<1e-12));

% Reproduce the parent's failure, then verify the local bound rejects it.
f=gpenmpcHil.newDeliveryState(0); f.name="FINAL_NATIVE_LAND";
f.state_enter_time_s=10; f.completed_ground_services=4; f.completed_relaunches=4;
o=observation(); o.armed=true; o.land_ack=true; o.px4_auto_land=true;
[p,~,~]=gpenmpcHil.deliveryLifecycleStep(f,o,41,cfg);
require('parent_dual_ground_armed_timeout_gap_reproduced',p.name=="FINAL_NATIVE_LAND");
[d,c,e,a]=delivery_lifecycle_step_checked(f,o,40,cfg);
require('final_dual_ground_armed_bound_rejected',d.name=="FAIL_CLOSED" && e=="FAIL_CLOSED" && a.precondition_override && ...
    d.failure_code=="FINAL_NATIVE_LAND_CONFIRMATION_TIMEOUT" && noAction(c));
[d,c]=delivery_lifecycle_step_checked(f,o,39.999,cfg);
require('before_deadline_parent_semantics_retained',d.name=="FINAL_NATIVE_LAND" && c.request_disarm);
o.armed=false; [d,~,~]=delivery_lifecycle_step_checked(f,o,39.999,cfg);
require('fresh_safe_completion_before_deadline',d.task_complete && d.name=="COMPLETE");
o.armed=true; o.land_ack_rejected=true;
[d,c]=delivery_lifecycle_step_checked(f,o,11,cfg);
require('final_land_ack_reject',d.name=="FAIL_CLOSED" && ~c.request_arm);
o=observation(); o.armed=true; o.px4_landed=false; o.plant_contact=false;
[d,c]=delivery_lifecycle_step_checked(f,o,40,cfg);
require('final_no_ground_deadline',d.name=="FAIL_CLOSED" && noAction(c));
o=observation(); o.armed=false; o.land_ack=true; o.px4_auto_land=true;
incomplete=f; incomplete.completed_ground_services=3;
[d,~]=delivery_lifecycle_step_checked(incomplete,o,11,cfg);
require('incomplete_services_not_task_complete',d.name=="FAIL_CLOSED" && ~d.task_complete);

g=gpenmpcHil.newDeliveryState(2.21); g.name="SERVICE_GROUNDED_DISARMED";
g.service_start_s=10; g.state_enter_time_s=10;
o=observation(); o.in_service_window=true; o.service_phase="SERVICE_ASCENT";
[p,c]=gpenmpcHil.deliveryLifecycleStep(g,o,17.95,cfg);
require('parent_early_phase_payload_gap_reproduced',p.name=="SERVICE_GROUNDED_DISARMED" && c.allow_payload_update);
[d,c,e]=delivery_lifecycle_step_checked(g,o,17.95,cfg);
require('early_phase_cannot_unload',d.name=="FAIL_CLOSED" && e=="FAIL_CLOSED" && noAction(c));
[d,c]=delivery_lifecycle_step_checked(g,o,18,cfg);
require('eight_second_boundary_allows_one_unload',d.name=="SERVICE_GROUNDED_DISARMED" && c.allow_payload_update);
o.armed=true; [d,c]=delivery_lifecycle_step_checked(g,o,18,cfg);
require('armed_ground_service_cannot_unload',d.name=="FAIL_CLOSED" && ~c.allow_payload_update);
o=observation(); o.altitude_agl_m=NaN;
[d,~]=delivery_lifecycle_step_checked(gpenmpcHil.newDeliveryState(2.21),o,0,cfg);
require('nonfinite_observation_fail_closed',d.name=="FAIL_CLOSED");
require('parent_equivalence_exercised',parityCases>600);
result=struct('status','PASS_HOST_ONLY_LIFECYCLE_STATE_MACHINE_TESTS', ...
    'cases',numel(checks),'passed',sum([checks.pass]),'checks',checks, ...
    'unchanged_parent_parity_cases',parityCases,'five_landing_mock_counts',count, ...
    'ground_dwell_s',dwellObserved,'event_ledger',events, ...
    'evidence_kind','SYNTHETIC_STATE_MACHINE__NOT_DYNAMICS_TOUCHDOWN_OR_HIL', ...
    'hardware_actions',0,'com_open',0,'board_access',0,'plant_simulations',0);
fprintf('CHECKED_DELIVERY_LIFECYCLE_PASS checks=%d parity=%d\n',result.passed,parityCases);

    function step()
        before=state;
        [state,last,event]=delivery_lifecycle_step_checked(state,obs,now,cfg);
        assert(state.name~="FAIL_CLOSED",'gpenmpcTask:LifecyclePositive','Positive state machine failed: %s',state.failure_code);
        [parent,parentCommand,parentEvent]=gpenmpcHil.deliveryLifecycleStep(before,obs,now,cfg);
        assert(isequaln(state,parent) && isequaln(last,parentCommand) && isequaln(event,parentEvent), ...
            'gpenmpcTask:UnrelatedDelta','Valid path differs from immutable parent.');
        parityCases=parityCases+1;
        count.land=count.land+double(last.request_land);
        count.arm=count.arm+double(last.request_arm);
        count.disarm=count.disarm+double(last.request_disarm);
        count.offboard=count.offboard+double(last.request_offboard);
        count.payload=count.payload+double(last.allow_payload_update);
        count.airborne_payload=count.airborne_payload+double(last.allow_payload_update && ...
            (obs.armed || ~obs.px4_landed || ~obs.plant_contact));
        if event~="NONE", events(end+1,1)=event; end %#ok<AGROW>
    end
    function require(name,condition)
        assert(isscalar(condition) && condition,'gpenmpcTask:LifecycleTest','Failed: %s',name);
        checks(end+1)=struct('name',string(name),'pass',true); %#ok<AGROW>
    end
end

function o=observation()
o=struct('prearm_ready',true,'armed',false,'px4_landed',true, ...
    'plant_contact',true,'altitude_agl_m',0,'horizontal_error_m',0, ...
    'land_ack',false,'land_ack_rejected',false,'px4_auto_land',false, ...
    'px4_offboard',true,'payload_update_ack',false,'payload_after_service_kg',2.21, ...
    'at_return_base',false,'initial_takeoff_complete',false, ...
    'in_service_window',false,'service_phase',"",'service_ordinal',0,'mission_complete',false);
end
function c=zeroCounts()
c=struct('land',0,'arm',0,'disarm',0,'offboard',0,'payload',0,'airborne_payload',0);
end
function value=noAction(c)
value=~c.request_arm && ~c.request_land && ~c.request_disarm && ...
    ~c.request_offboard && ~c.allow_payload_update && ~c.advance_reference;
end
