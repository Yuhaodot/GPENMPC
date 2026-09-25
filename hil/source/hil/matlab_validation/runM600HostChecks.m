function receipt = runM600HostChecks(outputFile)
% One MATLAB-only arithmetic/interface regression receipt; no live actions.
arguments
    outputFile (1,1) string
end
assert(~isfile(outputFile),'m600check:ExistingReceipt','Do not overwrite previous results.');
here=string(fileparts(mfilename('fullpath')));
buildRoot=fileparts(here);
interfaces=fullfile(buildRoot,'m600_coptersim','matlab_validation');
addpath(here,interfaces);
a=run_m600_truth_decoder_tests();
b=run_m600_three_stream_tests();
c=m600check.runPointwiseEquivalence();
assert(a.passed==a.cases && b.passed==b.cases && c.passed_count==c.test_count);
receipt=struct('status','PASS_MATLAB_ONLY_INTERFACE_AND_POINTWISE_REGRESSION',...
    'cases',a.cases+b.cases+c.test_count,...
    'passed',a.passed+b.passed+c.passed_count,...
    'truth_decoder',a,'three_stream_evaluation',b,'numeric_adapter',c,...
    'com_open_count',0,'board_actions',0,'simulator_actions',0,...
    'compiled_current_model_equivalence_established',false);
f=fopen(outputFile,'w','n','UTF-8'); assert(f>=0);
cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',jsonencode(receipt,PrettyPrint=true));
fprintf('MATLAB_HOST_CHECKS %d/%d\n',receipt.passed,receipt.cases);
end
