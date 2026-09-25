function result=run_plant_environment_acceptance_tests(outputJson)
% Test the environment state machine with synthetic timing fixtures.
assert(~isfile(outputJson));root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'matlab_validation'));
p=struct('initial_payload_kg',2.21,'initial_wind_ned_xy_mps',[0;0], ...
    'initial_reference_jet_ned',zeros(12,1),'max_env_age_s',.5, ...
    'max_board_age_s',.05,'max_future_skew_s',.001,'unload_dwell_s',.2, ...
    'max_ground_sample_gap_s',.11);
checks=struct('name',{},'pass',{});
v=struct('schema','M600_PLANT_ENVIRONMENT_V1','generation',1,'source_io_time_s',0, ...
    'task_reference_time_s',0,'payload_kg',2.21,'wind_ned_xy_mps',[-3;4], ...
    'mission_phase',1,'reference_jet_ned',(1:12)','payload_generation',0,'task_clock_paused',false);
[~,f]=gpenmpcTaskIo.encodePlantEnvironment(v,1,p.initial_payload_kg);
[s,r]=call([],f,0,true,board(0,0));c('initial_full_payload_frame_accepted',r.task_may_continue&&r.accepted_new_frame);
c('wire_environment_exactly_routed',isequal(s.environment.reference_jet_ned,(1:12)')&&isequal(s.environment.wind_xy_mps,[-3;4]));
[held,hr]=call(s,[],.1,true,board(.1,0));c('no_datagram_holds_not_freshens',hr.task_may_continue&&~hr.accepted_new_frame&&held.source_io_time_s==0);
[dup,dr]=call(s,f,.1,true,board(.1,0));c('exact_duplicate_does_not_refresh',dr.task_may_continue&&dup.generation==1&&dup.source_io_time_s==0);
[waiting,wr]=call([],[],0,true,board(0,0));c('startup_wait_is_not_task_ready',~wr.task_may_continue&&~wr.task_env_failed);
[~,wr]=call(waiting,[],.6,true,board(.6,0));c('startup_timeout_latches',wr.task_env_failed);
bad=f;bad(6)=77;negative('changed_duplicate_rejected',s,bad,.1,board(.1,0),7);
bad=f;bad(1)=2;negative('wrong_version',s,bad,.1,board(.1,0),5);
bad=f;bad(28)=1;negative('reserved_nonzero',s,bad,.1,board(.1,0),5);
bad=f;bad(9)=NaN;negative('nonfinite_reference',s,bad,.1,board(.1,0),4);
negative('bad_shape',s,f(1:27),.1,board(.1,0),4);
bad=f;bad(2)=1.5;negative('fractional_sequence',s,bad,.1,board(.1,0),5);
bad=f;bad(2)=2;bad(3)=.2;negative('future_clock',s,bad,.1,board(.1,0),6);
bad=f;bad(2)=2;bad(3)=.1;bad(4)=.1;
[progress,pr]=call(s,bad,.1,true,board(.1,0));c('new_generation_preserves_source_time',pr.accepted_new_frame&&progress.generation==2);
negative('reordered_generation',progress,f,.2,board(.2,0),8);
bad2=bad;bad2(2)=3;negative('nonadvancing_source_time',progress,bad2,.2,board(.2,0),8);
bad2=bad;bad2(2)=3;bad2(3)=.2;bad2(4)=0;
negative('task_clock_regression',progress,bad2,.2,board(.2,0),8);
bad2=bad;bad2(2)=3;bad2(3)=.6;negative('late_packet_cannot_erase_timeout',s,bad2,.6,board(.6,0),3);
negative('io_clock_regression',progress,bad,.05,board(.05,0),1);
bad2=bad;bad2(2)=3;bad2(3)=.2;bad2(5)=2;bad2(21)=1;
negative('airborne_unload_denied',s,bad2,.2,board(.2,0),10,false);
negative('armed_unload_denied',s,bad2,.2,board(.2,1),10);
negative('unknown_armed_not_disarmed',s,bad2,.2,board(.2,-1),10);
negative('stale_board_denied',s,bad2,.2,board(0,0),10);
b=board(.2,0);b.source_identity_valid=false;negative('invalid_board_identity_denied',s,bad2,.2,b,10);
b=board(.2,0);b.clock_map_valid=false;negative('unproved_board_clock_denied',s,bad2,.2,b,10);
negative('missing_board_proof_denied',s,bad2,.2,struct(),10);
early=bad2;early(3)=.1;negative('insufficient_dwell_denied',s,early,.1,board(.1,0),10);
negative('observation_gap_resets_dwell',s,bad2,.2,board(.2,0),10);
[ready,~]=call(held,[],.2,true,board(.2,0));
[unloaded,ur]=call(ready,bad2,.2,true,board(.2,0));
c('fresh_ground_disarmed_dwell_allows_one_unload',ur.task_may_continue&&ur.accepted_new_frame&&unloaded.environment.payload_kg==2&&unloaded.payload_generation==1);
after=bad2;after(2)=4;after(3)=.3;after(5)=2.1;negative('mass_reincrease_denied',unloaded,after,.3,board(.3,0),9);
after(5)=2;after(21)=2;negative('generation_change_without_unload_denied',unloaded,after,.3,board(.3,0),11);
[failed,fr]=call(s,[],.6,true,board(.6,0));
[latched,lr]=call(failed,bad2,.61,true,board(.61,0));
c('fault_permanently_latched_no_good_frame_recovery',fr.task_env_failed&&lr.task_env_failed&&latched.failure_code==failed.failure_code);
c('fault_keeps_exact_last_accepted_environment',isequal(latched.environment,s.environment));
c('failure_prescribes_physics_continue_not_reset',lr.continue_existing_plant_with_last_accepted_environment&&~lr.plant_step_or_reset_performed);
c('live_evidence_transport_gap_explicit',~lr.live_board_evidence_transport_implemented);
result=struct('status','HOST_ONLY_SYNTHETIC_ENVIRONMENT_STATE_MACHINE', ...
    'passed',all([checks.pass]),'checks_total',numel(checks),'checks_passed',sum([checks.pass]), ...
    'checks',checks,'fixture_policy_not_live_thresholds',p,'hardware_actions',0,'socket_open',0,'plant_steps',0, ...
    'limitation','NO_LIVE_ENV_PORT_OR_TRUSTED_BOARD_STATE_PATH_IMPLEMENTED__NO_UNLOAD_OR_LANDING_SAFETY_PROOF');
fid=fopen(outputJson,'w','n','UTF-8');assert(fid>=0);closeFile=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));clear closeFile
disp(jsonencode(result));assert(result.passed);
    function c(name,value),checks(end+1)=struct('name',name,'pass',logical(value));end
    function [ns,nr]=call(ps,ff,t,g,b),[ns,nr]=gpenmpcTaskIo.acceptPlantEnvironmentFrame(ps,ff,t,g,b,p);end
    function b=board(t,armed)
        b=struct('source_identity_valid',true,'clock_map_valid',true,'valid',true,'armed',armed,'mapped_io_time_s',t);
    end
    function negative(name,ps,ff,t,b,code,g)
        if nargin<7,g=true;end
        [ns,nr]=call(ps,ff,t,g,b);
        c(name,nr.task_env_failed&&nr.failure_code==code&&~nr.task_may_continue&&isequal(ns.environment,ps.environment));
    end
end
