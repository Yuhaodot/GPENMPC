function run_m600_canonical_exchange_localhost(outputRoot)
try
    test_m600_canonical_exchange_localhost(outputRoot);
catch problem
    if ~isfolder(outputRoot),mkdir(outputRoot);end
    r=struct('passed',false,'scope','ACTUAL_M600_IO_LOOPBACK_TEST_ONLY', ...
        'identifier',problem.identifier,'message',problem.message,'stack',problem.stack, ...
        'board_actions',0,'simulator_actions',0,'com_opens',0);
    f=fopen(fullfile(outputRoot,'FAILED_ATTEMPT.json'),'w');assert(f>0);c=onCleanup(@()fclose(f));
    fprintf(f,'%s\n',jsonencode(r,PrettyPrint=true));clear c
    rethrow(problem)
end
end
