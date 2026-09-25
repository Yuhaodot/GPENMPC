function result=test_plant_environment_v2(outputDir,generateCpp)
% Test environment policy with synthetic timing and mass values.
% Use the task's 8 s service duration.
if nargin<2,generateCpp=false;end
assert(islogical(generateCpp)&&isscalar(generateCpp),'gpenmpcTaskIo:V2CodegenFlag','Logical source-generation flag required.');
root=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(root,'matlab_validation'));
assert(~isfolder(outputDir)&&~isfile(outputDir),'gpenmpcTaskIo:V2Output','Fresh output directory required.');
mkdir(outputDir);checks=struct('name',{},'pass',{});calls=0;firstError=struct('identifier','','message','');
codegenEntries=struct('name',{},'generated',{});
p=struct('initial_payload_kg',2.21,'initial_wind_ned_xy_mps',zeros(2,1), ...
    'initial_reference_jet_ned',zeros(12,1),'expected_session_token',1234567, ...
    'max_env_age_s',.5,'max_board_age_s',.25,'max_future_skew_s',.001, ...
    'max_ground_sample_gap_s',.025,'unload_dwell_s',8.0,'max_commit_delay_s',.02, ...
    'base_mass_kg',2.27,'mass_bias_kg',.03,'mass_tolerance_kg',1e-12, ...
    'service_payload_targets_kg',[1.7;1.2;.5;0]);
try
    v=value(1,0,2.21,0);[wire,f]=gpenmpcTaskIo.encodePlantEnvironmentV2(v,1,2.21);
    check('wire_232_bytes_uint8',isa(wire,'uint8')&&isequal(size(wire),[1,232]));
    check('official_magic_and_id_LE',isequal(wire(1:8),uint8([217,2,150,73,1,0,0,0])));
    check('wire_28d_exact_roundtrip',isequal(unpack(wire(9:end)),f));
    check('new_six_fields_complete',isequal(f(23:28),[1234567;0;15;1;1;0]));
    legacy=struct('schema','M600_PLANT_ENVIRONMENT_V1','generation',v.generation, ...
        'source_io_time_s',v.source_io_time_s,'task_reference_time_s',v.task_reference_time_s, ...
        'payload_kg',v.payload_kg,'wind_ned_xy_mps',v.wind_ned_xy_mps,'mission_phase',v.mission_phase, ...
        'reference_jet_ned',v.reference_jet_ned,'payload_generation',v.payload_generation,'task_clock_paused',v.task_clock_paused);
    [oldBytes,oldFields]=gpenmpcTaskIo.encodePlantEnvironment(legacy,1,2.21);
    check('V1_fields_2_to_22_exact',isequal(f(2:22),oldFields(2:22).'));
    check('V1_envelope_and_first22_except_version_byteexact',isequal(wire(1:8),oldBytes(1:8))&&isequal(wire(17:184),oldBytes(17:184)));
    for channel=1:28
        bad=f;bad(channel)=NaN;negative(sprintf('nonfinite_channel_%02d',channel),initial(),bad,0,true,4);
    end
    bad=f;bad(6)=Inf;negative('positive_inf_wind',initial(),bad,0,true,4);
    bad=f;bad(7)=-Inf;negative('negative_inf_wind',initial(),bad,0,true,4);
    negative('short_shape',initial(),f(1:27),0,true,4);
    negative('row_shape_not_column',initial(),f.',0,true,4);
    negative('single_not_double',initial(),single(f),0,true,4);
    bad=f;bad(1)=1;negative('legacy_not_silently_accepted',initial(),bad,0,true,5);
    bad=f;bad(2)=1.5;negative('fractional_generation',initial(),bad,0,true,5);
    bad=f;bad(25)=32;negative('undefined_flag_bits',initial(),bad,0,true,5);
    bad=f;bad(26)=4;negative('undefined_landed_state',initial(),bad,0,true,5);
    bad=f;bad(23)=1234568;negative('wrong_initial_session',initial(),bad,0,true,12);
    for bit=0:3
        bad=f;bad(25)=15-2^bit;negative(sprintf('independent_valid_flag_%d_missing',bit),initial(),bad,0,true,13);
    end
    bad=f;bad(3)=.01;negative('future_environment_source',initial(),bad,0,true,6);
    bad=f;bad(24)=.01;negative('future_board_source',initial(),bad,0,true,6);
    rejected=false;try,badV=v;badV.task_clock_paused=1;gpenmpcTaskIo.encodePlantEnvironmentV2(badV,1,2.21);catch,rejected=true;end
    check('encoder_numeric_bool_rejected',rejected);
    rejected=false;try,badP=p;badP.unload_dwell_s=10;gpenmpcTaskIo.initPlantEnvironmentV2(badP,0);catch,rejected=true;end
    check('invalid_ten_second_dwell_rejected',rejected);
    s0=initial();
    coreMass=(p.base_mass_kg+p.initial_payload_kg)+p.mass_bias_kg;
    reassociatedMass=(p.base_mass_kg+p.mass_bias_kg)+p.initial_payload_kg;
    check('nonzero_bias_noninteger_operation_order_is_discriminating',typecast(coreMass,'uint64')~=typecast(reassociatedMass,'uint64'));
    check('initial_mass_rawbits_match_actual_core_operation_order',typecast(s0.applied_total_mass_kg,'uint64')==typecast(coreMass,'uint64'));
    [pending,r]=accept(s0,f,0,true);
    check('stage_is_pending_not_applied',r.pending&&r.staged_this_call&&~r.applied_ack_this_call&&~pending.has_applied_frame);
    check('staging_keeps_previous_environment',isequal(pending.environment,s0.environment));
    check('candidate_exact_for_single_core_step',isequal(r.environment_for_step.reference_jet_ned,f(9:20))&&isequal(r.environment_for_step.wind_xy_mps,f(6:7)));
    [base,ack]=commit(pending,coreFor(pending,.001,.01));
    check('actual_step_and_mass_issue_first_ACK',ack.new_credit&&ack.valid&&base.has_applied_frame&&base.applied_frame_generation==1);
    check('committed_mass_rawbits_match_actual_core_operation_order',typecast(ack.actual_total_mass_kg,'uint64')==typecast(coreMass,'uint64'));
    exactPolicy=p;exactPolicy.mass_tolerance_kg=0;
    exactCore=coreFor(pending,.001,.01);
    [~,exactAck]=gpenmpcTaskIo.commitPlantEnvironmentV2(pending,exactCore,exactPolicy);calls=calls+1;
    check('zero_tolerance_exact_core_operation_order_commits',exactAck.valid&&exactAck.new_credit);
    exactCore.total_mass_kg=reassociatedMass;
    [~,wrongOrderAck]=gpenmpcTaskIo.commitPlantEnvironmentV2(pending,exactCore,exactPolicy);calls=calls+1;
    check('zero_tolerance_wrong_association_one_ULP_rejected',~wrongOrderAck.valid&&wrongOrderAck.failure_code==15);
    [same,again]=commit(base,coreFor(pending,.001,.01));
    check('duplicate_commit_one_credit',~again.new_credit&&same.applied_count==1&&same.unload_count==0);
    [held,hr]=accept(base,f,.01,true);
    check('exact_duplicate_no_new_pending_or_freshness_credit',~hr.pending&&~hr.staged_this_call&&held.applied_source_io_time_s==0&&held.applied_count==1);
    bad=f;bad(6)=90;negative('changed_duplicate_latches',base,bad,.01,true,7);
    [waiting,wr]=noFrame(initial(),0,true);check('no_frame_initial_not_task_ready',~wr.task_may_continue&&~wr.task_env_failed);
    [~,late]=noFrame(waiting,.51,true);check('initial_no_input_timeout',late.task_env_failed&&late.failure_code==3);
    bad=frame(2,.51,2.21,0);negative('late_new_input_cannot_wash_env_expiry',base,bad,.51,true,3);
    bad=frame(2,.26,2.21,0);bad(24)=0;negative('board_stale_distinct_from_env_age',base,bad,.26,true,6);
    bad=frame(2,.01,2.21,0);bad(23)=1234568;negative('session_change_latches_not_reinitializes',base,bad,.01,true,12);
    bad=frame(2,.01,2.21,0);[progress,~]=accepted(base,bad,.01,true);
    negative('old_generation_rejected',progress,f,.02,true,8);
    bad=frame(3,.01,2.21,0);negative('source_not_advancing',progress,bad,.02,true,8);
    bad=frame(3,.02,2.21,0);bad(4)=0;negative('task_clock_reversal',progress,bad,.02,true,8);
    negative('I_O_clock_reversal',progress,f,.005,true,1);
    [badState,badReceipt]=accept(base,frame(2,.51,2.21,0),.51,true);
    [latched,latchedReceipt]=accept(badState,frame(3,.52,2.21,0),.52,true);
    check('failure_keeps_first_cause_and_does_not_wash',latchedReceipt.failure_code==badReceipt.failure_code&&latchedReceipt.task_env_failed);
    check('failure_keeps_mass_and_core_clock_not_reset',isequal(latched.environment,base.environment)&&latched.last_commit_core_time_s==base.last_commit_core_time_s);
    check('failure_continues_same_plant_not_reset',latchedReceipt.continue_same_plant&&~latchedReceipt.plant_reset_requested&&~latchedReceipt.plant_step_performed);
    [~,numericHas]=gpenmpcTaskIo.acceptPlantEnvironmentV2(initial(),f,1,0,true,p);calls=calls+1;
    check('has_frame_numeric_bool_rejected',numericHas.task_env_failed&&numericHas.failure_code==2);
    negative('ground_numeric_bool_rejected',initial(),f,0,1,2);
    changedPending=f;changedPending(2)=2;changedPending(3)=.01;
    negative('pending_cannot_be_overwritten',pending,changedPending,.01,true,14);
    [~,pr]=noFrame(pending,.021,true);check('uncommitted_pending_timeout',pr.task_env_failed&&pr.failure_code==14);
    badCore=coreFor(pending,.001,.01);badCore.total_mass_kg=badCore.total_mass_kg+.01;commitNegative('wrong_actual_mass_no_ACK',pending,badCore);
    badCore=coreFor(pending,.021,.01);commitNegative('late_ACK_rejected',pending,badCore);
    badCore=coreFor(pending,.001,.01);badCore.step_accepted=false;commitNegative('rejected_core_step_no_ACK',pending,badCore);
    badCore=coreFor(pending,.001,.01);badCore.step_accepted=1;commitNegative('numeric_one_not_step_boolean',pending,badCore);
    badCore=coreFor(pending,.001,.01);badCore.model_failed=true;commitNegative('failed_core_no_ACK',pending,badCore);
    badCore=coreFor(pending,.001,.01);badCore.reset_applied=true;commitNegative('core_reset_cannot_apply_payload',pending,badCore);
    badCore=coreFor(pending,.001,.01);badCore.applied_input_generation=2;commitNegative('wrong_core_input_generation',pending,badCore);
    [futurePending,~]=accept(base,frame(2,.01,2.21,0),.01,true);
    badCore=coreFor(futurePending,.011,.005);commitNegative('core_time_reversal_rejected',futurePending,badCore);
    % Actual 10ms sampled eight-second proof, no shortened fixture duration.
    ready=initial();
    for k=0:800,[ready,~]=accepted(ready,frame(k+1,k*.01,2.21,0),k*.01,true);end
    check('eight_seconds_from_samples_not_task_flag',ready.ground_disarmed_since_s==0&&ready.last_eval_io_time_s==8&&ready.unload_count==0);
    release=frame(802,8.01,1.7,1);release(28)=1;
    [unloadPending,ur]=accept(ready,release,8.01,true);
    check('release_only_pending_old_mass_retained',ur.pending&&unloadPending.environment.payload_kg==2.21&&unloadPending.unload_count==0&&ur.environment_for_step.payload_kg==1.7);
    [unloaded,ua]=commit(unloadPending,coreFor(unloadPending,8.011,8.02));
    check('eight_second_release_commits_actual_mass_once',ua.new_credit&&ua.valid&&ua.actual_payload_kg==1.7&&abs(ua.actual_total_mass_kg-4.0)<1e-12&&ua.unload_count==1);
    [unloadedAgain,du]=accept(unloaded,release,8.02,true);
    check('repeated_release_packet_does_not_unload_again',~du.pending&&unloadedAgain.unload_count==1&&unloadedAgain.environment.payload_kg==1.7);
    noMass=frame(803,8.02,1.7,1);noMass(28)=1;
    [sameMass,sa]=accepted(unloaded,noMass,8.02,true);
    check('held_release_with_new_wind_frame_no_second_unload',sa.new_credit&&sameMass.unload_count==1);
    bad=frame(803,8.02,1.8,1);negative('payload_increase_rejected',unloaded,bad,8.02,true,9);
    bad=frame(803,8.02,1.7,2);negative('payload_generation_without_mass_change_rejected',unloaded,bad,8.02,true,11);
    bad=release;bad(28)=0;negative('release_token_missing',ready,bad,8.01,true,10);
    bad=release;bad(28)=2;negative('release_token_wrong_service',ready,bad,8.01,true,10);
    bad=release;bad(5)=1.6;negative('payload_target_not_current_task',ready,bad,8.01,true,10);
    bad=release;bad(22)=0;negative('task_clock_must_pause_for_unload',ready,bad,8.01,true,10);
    bad=release;bad(25)=31;negative('armed_release_denied',ready,bad,8.01,true,10);
    bad=release;bad(26)=2;negative('airborne_PX4_cannot_be_replaced_by_plant_ground',ready,bad,8.01,true,10);
    bad=release;bad(26)=0;negative('unknown_PX4_landed_denied',ready,bad,8.01,true,10);
    negative('PX4_ground_cannot_replace_actual_plant_contact',ready,release,8.01,false,10);
    bad=release;bad(27)=2;negative('hidden_bad_transition_epoch_resets_dwell',ready,bad,8.01,true,10);
    bad=release;bad(3)=8.1;bad(24)=8.1;bad(4)=8.1;negative('observation_gap_resets_dwell',ready,bad,8.1,true,10);
    early=frame(2,.01,1.7,1);early(28)=1;negative('task_service_flag_cannot_forge_8s',base,early,.01,true,10);
    resetFrame=frame(802,8.01,2.21,0);resetFrame(27)=2;[resetReady,~]=accepted(ready,resetFrame,8.01,true);
    bad=frame(803,8.02,1.7,1);bad(27)=2;bad(28)=1;negative('later_good_poll_not_restore_old_continuity_credit',resetReady,bad,8.02,true,10);
    regress=frame(803,8.02,2.21,0);negative('continuity_epoch_regression_rejected',resetReady,regress,8.02,true,8);
    coreLostGround=coreFor(unloadPending,8.011,8.02);coreLostGround.ground_confirmed=false;commitNegative('ground_loss_at_actual_core_commit_rejected',unloadPending,coreLostGround);
    % Test explicit BEGIN activation and retained begun state after environment failure.
    wrapper=struct('begun',false,'state',initial(),'begin_count',0);
    for preTime=0:5:45
        [wrapper,pre]=beginWrapper(wrapper,false,zeros(28,1),preTime);
        assert(~pre.task_may_continue&&pre.continue_same_plant&&~pre.plant_reset_requested,'gpenmpcTaskIo:V2Fixture','Pre-BEGIN gained authority.');
    end
    check('pre_BEGIN_45s_does_not_start_environment_liveness',~wrapper.begun&&wrapper.begin_count==0&&~wrapper.state.task_env_failed);
    [wrapper,br]=beginWrapper(wrapper,true,frame(1,50,2.21,0),50);
    check('late_explicit_BEGIN_uses_io50_not_model_t0',br.pending&&wrapper.begun&&wrapper.begin_count==1&&wrapper.state.initialized_io_time_s==50);
    [wrapper.state,ba]=commit(wrapper.state,coreFor(wrapper.state,50.001,50.01));
    check('late_BEGIN_first_ACK_preserves_nonzero_core_clock',ba.new_credit&&ba.applied_core_time_s==50.01&&~ba.plant_reset_requested);
    [wrapper,~]=beginWrapper(wrapper,false,zeros(28,1),50.51);
    firstFailure=wrapper.state.failure_code;
    [wrapper,rebegin]=beginWrapper(wrapper,true,frame(1,51,2.21,0),51);
    check('retained_wrapper_reBEGIN_does_not_wash_failed_state',wrapper.begin_count==1&&wrapper.state.initialized_io_time_s==50&&rebegin.task_env_failed&&wrapper.state.failure_code==firstFailure&&firstFailure==3);
    stateNoCells=~any(structfun(@iscell,unloaded));check('state_no_dynamic_cells',stateNoCells);
    check('state_no_string_or_char_fields',~any(structfun(@(x)ischar(x)||isstring(x),unloaded)));
    if generateCpp
        % Generate each entry point into a separate directory.
        cfg=coder.config('lib');cfg.TargetLang='C++';cfg.GenerateReport=false;
        sourceDir=fullfile(root,'matlab_validation','+gpenmpcTaskIo');
        names={'initPlantEnvironmentV2','encodePlantEnvironmentV2','acceptPlantEnvironmentV2','commitPlantEnvironmentV2'};
        argumentsForEntry={{p,0.0},{v,1.0,2.21},{initial(),f,true,0.0,true,p},{pending,coreFor(pending,.001,.01),p}};
        for entryIndex=1:numel(names)
            name=names{entryIndex};entryDir=fullfile(outputDir,'cpp',name);
            codegen('-config',cfg,fullfile(sourceDir,[name '.m']),'-args',argumentsForEntry{entryIndex},'-c','-d',entryDir);
            codegenEntries(end+1)=struct('name',name,'generated',true); %#ok<AGROW>
            % MATLAB Coder prefixes package-qualified entry names in files.
            check(['actual_cpp_source_generation_' name],isfile(fullfile(entryDir,['gpenmpcTaskIo_' name '.cpp'])));
        end
    end
catch err
    firstError=struct('identifier',err.identifier,'message',getReport(err,'extended','hyperlinks','off'));
end
result=struct('status','HOST_ONLY_PURE_ENVIRONMENT_V2_TESTS','pass',isempty(firstError.identifier)&&all([checks.pass]), ...
    'checks_total',numel(checks),'checks_passed',sum([checks.pass]),'checks',checks,'pure_function_calls',calls, ...
    'first_error',firstError,'fixture_policy_not_live_parameters',p, ...
    'canonical_service_duration_s',8,'code_generation_requested',generateCpp, ...
    'code_generation_executed',~isempty(codegenEntries),'code_generation_entries',codegenEntries, ...
    'scope','State-machine tests with supplied observation fixtures.');
fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0,'gpenmpcTaskIo:V2Output','Cannot write result.');closer=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));clear closer;
disp(jsonencode(struct('pass',result.pass,'checks_total',result.checks_total,'checks_passed',result.checks_passed,'pure_function_calls',calls,'first_error',firstError)));
assert(result.pass,'gpenmpcTaskIo:V2Tests','See persisted RESULT.json for first failure.');
    function check(name,ok)
        checks(end+1)=struct('name',name,'pass',islogical(ok)&&isscalar(ok)&&ok);
        assert(checks(end).pass,'gpenmpcTaskIo:V2Check','Failed: %s',name);
    end
    function s=initial(),s=gpenmpcTaskIo.initPlantEnvironmentV2(p,0);calls=calls+1;end
    function v=value(g,t,m,pg)
        v=struct('schema_version',2,'generation',g,'source_io_time_s',t,'task_reference_time_s',t, ...
            'payload_kg',m,'wind_ned_xy_mps',[-3;4],'mission_phase',2,'reference_jet_ned',(1:12)', ...
            'payload_generation',pg,'task_clock_paused',true,'session_token',1234567, ...
            'board_min_rx_io_time_s',t,'board_valid_flags',15,'landed_state',1,'continuity_epoch',1,'service_release_generation',0);
    end
    function f=frame(g,t,m,pg),[~,f]=gpenmpcTaskIo.encodePlantEnvironmentV2(value(g,t,m,pg),1,p.initial_payload_kg);calls=calls+1;end
    function [s,r]=accept(old,f,t,g),[s,r]=gpenmpcTaskIo.acceptPlantEnvironmentV2(old,f,true,t,g,p);calls=calls+1;end
    function [s,r]=noFrame(old,t,g),[s,r]=gpenmpcTaskIo.acceptPlantEnvironmentV2(old,zeros(28,1),false,t,g,p);calls=calls+1;end
    function [s,r]=commit(old,c),[s,r]=gpenmpcTaskIo.commitPlantEnvironmentV2(old,c,p);calls=calls+1;end
    function c=coreFor(s,t,ct)
        c=struct('step_accepted',true,'model_failed',false,'reset_applied',false,'ground_confirmed',true, ...
            'io_time_s',t,'core_time_s',ct,'applied_input_generation',s.pending_frame(2), ...
            'total_mass_kg',p.base_mass_kg+s.pending_frame(5)+p.mass_bias_kg);
    end
    function [s,ack]=accepted(old,f,t,g)
        [staged,rr]=accept(old,f,t,g);assert(rr.pending,'gpenmpcTaskIo:V2Fixture','Fixture did not stage.');
        [s,ack]=commit(staged,coreFor(staged,t+.001,t+.01));assert(ack.new_credit,'gpenmpcTaskIo:V2Fixture','Fixture did not commit.');
    end
    function negative(name,old,f,t,g,code)
        [s,r]=accept(old,f,t,g);
        check(name,r.task_env_failed&&r.failure_code==code&&~r.task_may_continue&&~r.pending&&isequal(s.environment,old.environment)&&s.last_commit_core_time_s==old.last_commit_core_time_s);
    end
    function commitNegative(name,old,c)
        [s,a]=commit(old,c);
        check(name,a.task_env_failed&&a.failure_code==15&&~a.new_credit&&~a.valid&&isequal(s.environment,old.environment)&&s.applied_count==old.applied_count);
    end
    function [w,r]=beginWrapper(w,begin,f,t)
        r=struct('task_may_continue',false,'continue_same_plant',true,'plant_reset_requested',false,'pending',false,'task_env_failed',false);
        if ~w.begun
            if ~begin,return;end
            assert(f(1)==2&&f(2)==1&&f(5)==p.initial_payload_kg&&f(21)==0&&f(23)==p.expected_session_token&&f(25)==15&&f(26)==1,'gpenmpcTaskIo:V2Fixture','Fixture requires an explicit legal initial BEGIN.');
            w.state=gpenmpcTaskIo.initPlantEnvironmentV2(p,t);calls=calls+1;
            w.begun=true;w.begin_count=w.begin_count+1;
        end
        if begin,[w.state,a]=accept(w.state,f,t,true);else,[w.state,a]=noFrame(w.state,t,true);end
        r.task_may_continue=a.task_may_continue;r.pending=a.pending;r.task_env_failed=a.task_env_failed;
    end
end
function f=unpack(bytes)
f=zeros(28,1);
for k=1:28
    bits=uint64(0);for j=1:8,bits=bitor(bits,bitshift(uint64(bytes((k-1)*8+j)),8*(j-1)));end
    f(k)=typecast(bits,'double');
end
end
