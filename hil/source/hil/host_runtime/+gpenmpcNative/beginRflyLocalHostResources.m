function [guard,receipt]=beginRflyLocalHostResources()
% Prepare process-local numerical resources before IO and worker creation.
% Limit BLAS concurrency for small GP operations.
names={'OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS'};
values=cellfun(@getenv,names,'UniformOutput',false);previous=maxNumCompThreads;
guard=onCleanup(@restore);
for k=1:numel(names),setenv(names{k},'1');end
maxNumCompThreads(1);
receipt=struct('scope','CALLING_MATLAB_PROCESS_ONLY_BEFORE_IO_AND_WORKER', ...
    'prior_numerical_threads',previous,'numerical_threads',maxNumCompThreads, ...
    'environment_names',{names},'previous_environment_values',{values}, ...
    'environment_values',{{'1','1','1'}},'restoration_owner','RETURNED_ONCLEANUP_GUARD', ...
    'B_resource_hold',false,'other_processes_changed',false,'hardware_actions',0);
    function restore()
        for j=1:numel(names),setenv(names{j},values{j});end
        maxNumCompThreads(previous);
    end
end
