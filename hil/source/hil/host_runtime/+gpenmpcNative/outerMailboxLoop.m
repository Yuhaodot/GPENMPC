function summary=outerMailboxLoop(inputQueue,outputQueue,fixedConstant)
% Process requests serially on the persistent solver worker.
processed=0;stopped=false;
while ~stopped
    message=poll(inputQueue,0.05);
    if isempty(message),continue;end
    if isfield(message,'kind') && strcmp(message.kind,'STOP')
        stopped=true;continue
    end
    response=struct('ok',false,'generation',message.generation, ...
        'result',struct(),'error','');
    try
        response.result=gpenmpcNative.solveCurrentOuterConstant( ...
            message.snapshot,fixedConstant);
        response.ok=true;
    catch ex
        response.error=getReport(ex,'extended','hyperlinks','off');
    end
    send(outputQueue,response);processed=processed+1;
end
summary=struct('processed',processed,'clean_stop',true);
end
