function r=run_utc_pair_host_tests(outputPath)
% Test clock pairing with synthetic and system-clock cases.
assert(~isfile(outputPath));root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'m600_coptersim','matlab_validation'));
names={};passed=[];mc=[];wc=[];mi=0;wi=0;
mc=[0,.15,.16,.1602,.17,.1701];wc=[1000.075,1000.1601,1000.17005];
r0=m600check.sampleUtcMonotonicPair(@mockMono,@mockUtc,3,.05);
check('all_slow_first_call_evidence_retained',numel(r0.samples)==3&&r0.samples(1).bracket_s==.15);
check('tightest_measured_bracket_selected',r0.selected_sample==3);
check('uncertainty_measured_not_set_to_zero',r0.uncertainty_s>=.00005&&r0.uncertainty_s<.000051);
check('expected_clock_offset',abs(r0.utc_zero_s-1000)<1e-10);
check('old_single_pair_would_fail_unchanged_gate',r0.samples(1).bracket_s/2>.051);
check('new_measured_pair_can_pass_unchanged_gate',.0122206/2+r0.uncertainty_s+.0005<.051);
check('no_sample_discard',r0.sample_count==3&&r0.interval_separation_s<1e-9);
negative([0,.1,.05,.2],[1000.05,1000.125],2,.05,'m600check:ClockSamplerMonotonicReversal');
negative([0,.1,.2,.3],[1000.05,1001.25],2,.05,'m600check:ClockSamplerUtcDiscontinuity');
negative([0,NaN],[1000],1,.05,'m600check:ClockSamplerNonfinite');
negative([0,.1],[Inf],1,.05,'m600check:ClockSamplerNonfinite');
negative([0,.1],[1000],0,.05,'m600check:ClockSamplerCount');
negative([0,.1],[1000],33,.05,'m600check:ClockSamplerCount');
negative([0,.1],[1000],1,0,'m600check:ClockSamplerDriftBound');
mc=[0,.02,.03,.05];wc=[1000.01,1000.04];mi=0;wi=0;
same=m600check.sampleUtcMonotonicPair(@mockMono,@mockUtc,2,.05);
check('equal_brackets_deterministic_first',same.selected_sample==1);
timer=tic;realPair=m600check.sampleUtcMonotonicPair(@()toc(timer), ...
    @()posixtime(datetime('now','TimeZone','UTC')),10,.05);
check('real_clock_ten_samples',numel(realPair.samples)==10);
check('real_uncertainty_measured',realPair.uncertainty_s>0&&isfinite(realPair.uncertainty_s));
check('real_clock_all_brackets_nonnegative',all([realPair.samples.bracket_s]>=0));
check('real_clock_existing_alignment_gate',realPair.uncertainty_s+.0005<.051);
q=struct('utc_zero_s',1000,'uncertainty_s',.020);
drift=m600check.evaluateUtcDrift(10,1010.041,10.002,q,.05);
check('both_endpoint_uncertainties_reject_061_second_drift', ...
    ~drift.passed&&drift.absolute_drift_upper_bound_s>.0609&&drift.absolute_drift_upper_bound_s<.0611);
q.uncertainty_s=.00001;
drift=m600check.evaluateUtcDrift(10,1010.001,10.002,q,.05);
check('finite_consistent_drift_admitted',drift.passed&&drift.combined_uncertainty_s>.001);
try,m600check.evaluateUtcDrift(10,NaN,10.002,q,.05);bad=false;catch e,bad=strcmp(e.identifier,'m600check:UtcDriftInput');end
check('nonfinite_drift_rejected',bad);
try,m600check.evaluateUtcDrift(10,1010,9,q,.05);bad=false;catch e,bad=strcmp(e.identifier,'m600check:UtcDriftInput');end
check('reversed_drift_bracket_rejected',bad);
r=struct('passed',all(passed),'checks_total',numel(passed),'checks_passed',sum(passed), ...
    'checks',struct('name',names,'passed',num2cell(passed)),'real_clock_pair',realPair, ...
    'timing_gates_changed',false,'hardware_actions',0,'COM_open',0,'UDP_open',0);
fid=fopen(outputPath,'w','n','UTF-8');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(r,PrettyPrint=true));clear cleanup;disp(jsonencode(r));assert(r.passed);
    function v=mockMono(),mi=mi+1;v=mc(mi);end
    function v=mockUtc(),wi=wi+1;v=wc(wi);end
    function check(name,value),names{end+1}=name;passed(end+1)=logical(value);end
    function negative(m,w,n,d,expected)
        mc=m;wc=w;mi=0;wi=0;ok=false;
        try,m600check.sampleUtcMonotonicPair(@mockMono,@mockUtc,n,d);catch e,ok=strcmp(e.identifier,expected);end
        check(['negative_' expected '_' num2str(numel(names))],ok);
    end
end
