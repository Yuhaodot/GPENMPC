function report=test_rfly_delivery_time_in_exchange(outputRoot)
% Test delivery-time binding through in-memory exchange endpoints.
build=string(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(build,'host_runtime'));
assert(~isfolder(outputRoot));mkdir(outputRoot);
b=gpenmpcNative.loadRflyCanonicalDeliveryTask(fullfile(build,'task_packages','cambridge_canonical', ...
    'MU_CAMBRIDGE_MA_02__CANONICAL_PHYSICAL_TASK.mat'), ...
    'B876546B468265F84BD028AC549B06EF8ADDCB25CBA09CF389A08F4B21290A5F');
[tr,~]=gpenmpcNative.bindRflyCanonicalInitialTakeoffTrajectory(b.legs{1},[0;0;0]);
q=24.99;next=1;closed=0;observed=[];
c=struct('failed',false,'initialized',true,'state','FLIGHT','service', ...
    struct('leg_index',1,'phase_s',q,'payload_kg',b.legs{1}.meta.payload_kg,'outer_suspended',false));
io=struct('pollCanonical',@()struct('bound',true,'failure',[],'completed_queue_capacity',3), ...
    'takeCanonicalRotorRecords',@()struct('fixture',true),'takeCanonical',@take, ...
    'snapshot',@()struct('armed',1,'landed_state',2,'heartbeat_rx_s',.5,'extended_rx_s',.5), ...
    'now',@().5);
x=struct('status',@status,'source',@source,'poll',@(varargin)struct('fixture_poll',true),'close',@closeOwner);
rotor=struct('Failed',false,'ingest',@(varargin)[],'current',@rotorCurrent);
wind=struct('observe',@observeWind);
r=struct('origin',struct('explicit_host_fixture',true),'task_time_s',999,'request_startup',false, ...
    'maximum_sources_per_poll',3,'heartbeat_max_age_s',1,'landed_max_age_s',1, ...
    'delivery_binding',struct('bundle',b,'trajectory',tr));
checks=struct('name',{},'pass',{});
a=gpenmpcNative.advanceRflyCanonicalIoExchange(io,x,rotor,wind,r);
check('production_advance_consumes_three_sources',~a.failed&&a.source_messages==3);
check('per_source_actual_phase_used_not_caller_wall_or_999', ...
    numel(observed)==3&&observed(1)==0&&observed(2)==0&&abs(observed(3)-.01)<1e-12);
ev=a.events(cellfun(@(v)strcmp(v.kind,'ORIGINAL_RSP1_DISPATCH'),a.events));
check('every_dispatch_contains_its_causal_time_view',numel(ev)==3 ...
    &&all(cellfun(@(v)~isempty(v.task_time_view),ev)));
check('no_endpoint_control_or_solver_created',a.inner_messages_sent==0 ...
    &&a.additional_solvers==0&&a.new_endpoints==0&&~a.arm_authorized&&closed==0);
r=rmfield(r,'delivery_binding');r.task_time_s=7;next=1;observed=[];
legacy=gpenmpcNative.advanceRflyCanonicalIoExchange(io,x,rotor,wind,r);
check('legacy_non_delivery_fixture_behavior_retained',~legacy.failed&&isequal(observed,[7,7,7]));
r.delivery_binding=struct('bundle',b,'trajectory',tr);r.delivery_binding.trajectory.canonical_task_phase_offset_s=20;
next=1;observed=[];bad=gpenmpcNative.advanceRflyCanonicalIoExchange(io,x,rotor,wind,r);
check('wrong_binding_fails_before_source_is_taken',bad.failed&&next==1&&isempty(observed));
check('wrong_binding_requests_existing_io_cleanup_and_closes_worker', ...
    bad.same_io_cleanup_required&&closed==1&&strcmp(bad.failure.identifier,'gpenmpcNative:DeliveryClockInitialPrefix'));
report=struct('passed',all([checks.pass]),'total',numel(checks),'checks',checks, ...
    'production_function',which('gpenmpcNative.advanceRflyCanonicalIoExchange'), ...
    'original_function_result',a,'legacy_result',legacy,'negative_result',bad, ...
    'endpoint_provenance','EXPLICIT_IN_MEMORY_FIXTURE', ...
    'actual_production_advance_executed',true,'COM_open',0,'board_actions',0,'solver_calls',0,'plant_instances',0);
save(fullfile(outputRoot,'RAW.mat'),'report');f=fopen(fullfile(outputRoot,'RESULT.json'),'w');assert(f>=0);
d=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));clear d
disp(jsonencode(struct('passed',report.passed,'checks',report.total,'hardware_actions',0)));
    function v=status()
        c.service.phase_s=q;
        v=struct('pending_command',false,'coordinator',c,'startup',struct('requested',true,'ready',false));
    end
    function v=take(kind)
        v=[];if strcmp(kind,'feedback')||next>3,return;end
        v=struct('message',uint8(next),'original_host_receive_ns',gpenmpcNative.rflyOriginalHostMonotonicNs());
        next=next+1;
    end
    function v=source(varargin)
        q=q+.01;v=struct('packets',{{}},'fixture_only',true);
    end
    function [v,receipt]=rotorCurrent(varargin)
        v=struct('valid',true);receipt=struct('fixture_only',true);
    end
    function [v,receipt]=observeWind(taskTime,varargin)
        observed(end+1)=taskTime;v=struct('fixture_only',true);receipt=struct('task_time_s',taskTime);
    end
    function closeOwner(),closed=closed+1;end
    function check(name,ok)
        checks(end+1)=struct('name',name,'pass',logical(ok));assert(ok,'gpenmpcNative:DeliveryExchangeTest','%s',name);
    end
end
