function report=test_canonical_hil16ctrlsnorm_adapter(outputRoot)
% Test HIL16CtrlsNorm conversion with fixed data.
arguments
    outputRoot (1,1) string
end
build=string(fileparts(fileparts(mfilename('fullpath'))));
outputRoot=string(char(java.io.File(char(outputRoot)).getCanonicalPath()));
evidence=string(char(java.io.File(char(fullfile(build,'evidence'))).getCanonicalPath()));
assert(startsWith(lower(outputRoot),lower(evidence+filesep)));
assert(~isfolder(outputRoot)&&~isfile(outputRoot),'Never overwrite prior evidence.');
mkdir(outputRoot);
old=path;guard=onCleanup(@()path(old)); %#ok<NASGU>
parent=string(gpenmpc_external_path('native_visual_host_runtime'));
addpath(fullfile(parent,'src'),'-end');addpath(fullfile(build,'host_runtime'),'-begin');
a=gpenmpcNative.loadCanonicalAssets(gpenmpcNative.canonicalAssetRoot());
allocation=gpenmpcM600Allocation(a.calibration);upper=allocation.per_rotor_upper_n;
interface=struct('schema','CANONICAL_M600_HIL16CTRLSNORM_INTERFACE_V1', ...
    'input_units','NEWTON','input_order','CANONICAL_M600_ORDER', ...
    'allocation_matrix',allocation.matrix,'per_rotor_upper_n',upper, ...
    'software_from_hil_one_based',[5;1;4;6;2;3], ...
    'hil_signal_units','LINEAR_NORMALIZED_THRUST', ...
    'official_block_name','HIL16CtrlsNorm','official_output_topic','actuator_outputs_rfly', ...
    'inactive_channel_value',0);
% Pinned accepted delivery-model decode, distinct from a stock RPM/PWM model.
sources={ ...
    'matlab_validation/+m600check/stepPx4Rk4.m','3CB98C99DCAE7A3731CAEDBD220B3F50F826E3705BDDCD191FB0E8FCF3DB426A'; ...
    'matlab_validation/+m600check/derivativePx4.m','FBC6B2C766C39D94A9D811BE65BAFBC65D81401C950BB711AE5BB72945F61C87'; ...
    '','877328A3F0D003EFCBCD3B9EDCFE65620D091569880A130DEF20F73F9884ABFB'; ...
    '','B2A08CB5A4CD3076B488D9308CE08A402F19F3F2DF9B7D87220C5B789597FA37'; ...
    '','6C48D754C8F07C0BE9497A2072F59A17EC58E6DE8437189EF405BB23EC54DAFE'};
for k=1:2,sources{k,1}=fullfile(build,sources{k,1});end
modelBuild=gpenmpc_external_path('canonical_delivery_dll_build');
sources{3,1}=fullfile(modelBuild,'GPENMPC_M600_Canonical_ert_rtw','GPENMPC_M600_Canonical.cpp');
sources{4,1}=fullfile(modelBuild,'compiled','modeldllgen.cpp');
sources{5,1}=fullfile(modelBuild,'compiled','GPENMPC_M600_Canonical.dll');
checks=struct('name',{},'pass',{});raw=struct();
try
    for k=1:size(sources,1)
        check("consumer_source_identity_"+k,strcmpi( ...
            gpenmpcNative.fileSha256(sources{k,1}),sources{k,2}));
    end
    source=fileread(sources{3,1});
    check('generated_rotor_permutation_not_assumed', ...
        contains(source,'static const int8_T h[6] = { 4, 0, 3, 5, 1, 2 };')&& ...
        contains(source,'rotor[i] = controls[h[i]] * 32.145727009134916;'));
    wrapper=fileread(sources{4,1});
    check('dll_input_copy_without_pwm_conversion',contains(wrapper, ...
        'memcpy(mmc.GPENMPC_M600_Canonical_U.inPWMs, inPWMs, sizeof(double)*16);'));
    fixture=gpenmpc_external_path('canonical_actuator_encoding_fixture');
    check('canonical_fixture_identity',strcmpi(gpenmpcNative.fileSha256(fixture), ...
        '3896CDC2B329E751F8E524EDD8FF7FADCCAF63B764A17CC79261FABDBC89A9DD'));
    loaded=load(fixture,'raw');commands=loaded.raw.oracle61(:,5:10);
    predicted=zeros(size(commands));encoded=zeros(size(commands,1),16,'single');
    maxError=0;nonzeroRows=0;
    for k=1:size(commands,1)
        [u,en,r,why]=gpenmpcNative.canonicalRotorToHIL16CtrlsNorm( ...
            commands(k,:).',true,a.calibration,interface);
        assert(en&&r.valid,'gpenmpc:HIL16Encoding','%s',why);
        % Independent source-derived DLL decode, not the helper's receipt.
        decoded=double(u([5;1;4;6;2;3]))*32.145727009134916;
        assert(isequal(decoded,r.expected_linear_host_decoded_rotor_newton));
        assert(isa(u,'single')&&isequal(size(u),[16,1])&&all(u>=0&u<=1));
        assert(all(u(7:16)==0)&&~r.publication_allowed);
        assert(~r.helper_clipping_performed&&~r.controller_or_allocator_recomputed);
        assert(~r.pwm_quantization_performed&&~r.exact_arbitrary_real_output_equivalence);
        encoded(k,:)=u.';predicted(k,:)=decoded.';
        maxError=max(maxError,max(abs(decoded-commands(k,:).')));
        nonzeroRows=nonzeroRows+double(any(decoded~=commands(k,:).'));
    end
    bound=r.float32_half_step_bound_newton+r.float64_arithmetic_guard_newton;
    check('all_2123_current_kernel_outputs_converted',size(commands,1)==2123);
    check('source_derived_float32_bound',maxError<=bound);
    check('rounding_not_misrepresented_as_exact_real_equivalence',nonzeroRows>0);
    check('no_pwm_quantization_path',~r.pwm_quantization_performed);
    for rotor=1:6
        n=zeros(6,1);n(rotor)=upper;
        [u,en,q]=gpenmpcNative.canonicalRotorToHIL16CtrlsNorm(n,true,a.calibration,interface);
        expected=zeros(16,1,'single');expected(interface.software_from_hil_one_based(rotor))=1;
        check("single_rotor_order_"+rotor,en&&isequal(u,expected)&& ...
            isequal(q.expected_linear_host_decoded_rotor_newton,n));
    end
    fractions=[0,.125,.5,1];
    for k=1:numel(fractions)
        n=ones(6,1)*upper*fractions(k);
        [u,en,q]=gpenmpcNative.canonicalRotorToHIL16CtrlsNorm(n,true,a.calibration,interface);
        check("dyadic_endpoint_"+k,en&&all(u(1:6)==single(fractions(k)))&& ...
            q.maximum_absolute_error_newton<=bound);
    end
    % Test values at and on both sides of a float32 midpoint.
    midpoint=.5+2^-25;fractions=midpoint+[-2^-28,0,2^-28];
    expected=single([.5,.5,.5+2^-24]);
    for k=1:3
        [u,en,q]=gpenmpcNative.canonicalRotorToHIL16CtrlsNorm( ...
            repmat(upper*fractions(k),6,1),true,a.calibration,interface);
        check("float32_midpoint_"+k,en&&all(u(1:6)==expected(k))&& ...
            q.maximum_absolute_error_newton<=bound);
    end
    % Disabled controls are zero, but they cannot certify a downstream stop.
    [u,en,q,why]=gpenmpcNative.canonicalRotorToHIL16CtrlsNorm(NaN,false,struct(),struct());
    check('disabled_is_fail_closed_even_with_bad_inputs',~en&&all(u==0)&& ...
        ~q.valid&&why=="DISABLED_ZERO_OUTPUT"&&~q.disabled_output_is_disarm_or_last_value_clear);
    n=ones(6,1)*upper/2;
    badEnable={0,1,NaN,Inf,[],[true;true],"true"};
    for k=1:numel(badEnable)
        rejected("enable_type_"+k,n,badEnable{k},a.calibration,interface);
    end
    changes={ ...
        'schema','schema','CANONICAL_M600_ROTOR_EXECUTION_INTERFACE_V1'; ...
        'units','input_units','NORMALIZED'; ...
        'order','input_order','PX4_MOTOR_ORDER'; ...
        'signal_units','hil_signal_units','PWM_US'; ...
        'block','official_block_name','HIL16Ctrls'; ...
        'topic','official_output_topic','actuator_outputs_sim'; ...
        'inactive_nan','inactive_channel_value',NaN; ...
        'inactive_nonzero','inactive_channel_value',-1; ...
        'wrong_upper','per_rotor_upper_n',upper+1; ...
        'identity_map','software_from_hil_one_based',(1:6)'; ...
        'duplicate_map','software_from_hil_one_based',[5;1;4;6;2;2]};
    for k=1:size(changes,1)
        c=interface;c.(changes{k,2})=changes{k,3};
        rejected(changes{k,1},n,true,a.calibration,c);
    end
    c=interface;c.allocation_matrix(2,1)=1;rejected('allocation_matrix',n,true,a.calibration,c);
    c=rmfield(interface,'software_from_hil_one_based');rejected('missing_map',n,true,a.calibration,c);
    c=a.calibration;c.rotor_allocation.spin_sign(1)=-1;rejected('spin_geometry',n,true,c,interface);
    rejected('nan_input',[NaN;n(2:end)],true,a.calibration,interface);
    rejected('inf_input',[Inf;n(2:end)],true,a.calibration,interface);
    rejected('negative_input_not_clipped',[-eps;n(2:end)],true,a.calibration,interface);
    rejected('above_upper_before_single_round',[upper+eps(upper);n(2:end)],true,a.calibration,interface);
    rejected('five_channels',n(1:5),true,a.calibration,interface);
    rejected('matrix_input',reshape(n,2,3),true,a.calibration,interface);
    rejected('complex_input',complex(n,1),true,a.calibration,interface);
    rejected('integer_input',uint16(n),true,a.calibration,interface);
    rejected('missing_calibration',n,true,struct(),interface);
    [u,en]=gpenmpcNative.canonicalRotorToHIL16CtrlsNorm(n.',true,a.calibration,interface);
    check('row_vector_canonicalized',en&&isequal(size(u),[16,1]));
    [u,en]=gpenmpcNative.canonicalRotorToHIL16CtrlsNorm(single(n),true,a.calibration,interface);
    check('single_newton_input',en&&all(isfinite(u)));
    % Compare with the preceding conversion path.
    pwmInterface=struct('schema','CANONICAL_M600_ROTOR_EXECUTION_INTERFACE_V1', ...
        'input_units','NEWTON','input_order','CANONICAL_M600_ORDER', ...
        'allocation_matrix',allocation.matrix,'per_rotor_upper_n',upper, ...
        'software_from_px4_one_based',[5;1;4;6;2;3], ...
        'hil_output_functions',[(101:106)';zeros(10,1)],'thr_mdl_fac',0, ...
        'reversible_flags',0,'output_reverse_mask',0, ...
        'pwm_sim_min',1000,'pwm_sim_max',2000,'output_limit_state','ON');
    pwmMaxError=0;
    for k=1:size(commands,1)
        [q,ok]=gpenmpcNative.canonicalRotorToActuatorMotors(commands(k,:).',a.calibration,pwmInterface);
        assert(ok);pwmMaxError=max(pwmMaxError,q.maximum_absolute_error_newton);
    end
    check('smaller_fixture_error_than_pwm_preview',maxError<pwmMaxError);
    raw.input_rotor_newton=commands;raw.controls16_single=encoded;
    raw.predicted_rotor_newton=predicted;
    report=struct('status','PASS_HOST_ONLY_HIL16CTRLSNORM_NUMERIC_ADAPTER', ...
        'case_count',size(commands,1),'checks',checks,'check_count',numel(checks), ...
        'maximum_float32_error_per_rotor_newton',maxError, ...
        'float32_half_step_bound_newton',r.float32_half_step_bound_newton, ...
        'float64_arithmetic_guard_newton',r.float64_arithmetic_guard_newton, ...
        'nonzero_quantization_rows',nonzeroRows,'prior_pwm_preview_maximum_error_newton',pwmMaxError, ...
        'interface',interface,'consumer_source_paths_and_sha256',{sources}, ...
        'canonical_passport_path',a.binding_path,'fixture_path',fixture, ...
        'hardware_actions',0,'COM_open',0,'UDP_open',0,'matlab_model_runs',0,'dll_loads',0, ...
        'publication_authority',false,'firmware_route_or_single_publisher_verified',false, ...
        'disabled_output_is_disarm_or_last_value_clear',false, ...
        'exact_arbitrary_real_output_equivalence',false, ...
        'helper_sha256',gpenmpcNative.fileSha256(which('gpenmpcNative.canonicalRotorToHIL16CtrlsNorm')), ...
        'test_sha256',gpenmpcNative.fileSha256(mfilename('fullpath')+".m"));
    save(fullfile(outputRoot,'RAW.mat'),'report','raw','-v7');
    fid=fopen(fullfile(outputRoot,'RESULT.json'),'w','n','UTF-8');assert(fid>=0);
    outGuard=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear outGuard
    disp(jsonencode(report));
catch err
    raw.failure=getReport(err,'extended','hyperlinks','off');
    save(fullfile(outputRoot,'FAILURE_RAW.mat'),'raw','checks','-v7');rethrow(err)
end
    function check(name,ok)
        checks(end+1)=struct('name',string(name),'pass',logical(ok)); %#ok<AGROW>
        assert(ok,'gpenmpc:HIL16AdapterTest','%s',name);
    end
    function rejected(name,n,e,c,contract)
        [v,en,q,~]=gpenmpcNative.canonicalRotorToHIL16CtrlsNorm(n,e,c,contract);
        check("reject_"+name,~en&&~q.valid&&isa(v,'single')&& ...
            isequal(size(v),[16,1])&&all(v==0)&&~q.publication_allowed&&q.hardware_actions==0);
    end
end
