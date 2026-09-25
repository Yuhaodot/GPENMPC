function r=advanceRflyCanonicalIoExchange(io,exchange,rotorProvider,windProvider,request)
% Advance the IO -> solver -> C/RFC1 exchange.
% The outer owner services heartbeat/environment and handles stop, LAND and cleanup.
% request_startup schedules preparation only.
r=struct('failed',false,'failure',[],'events',{{}},'source_messages',0, ...
    'feedback_messages',0,'inner_messages_sent',0,'startup_ready',false, ...
    'arm_authorized',false,'new_endpoints',0,'additional_solvers',0, ...
    'same_io_cleanup_required',false,'exchange_status',[],'partial_send_evidence',[]);
sendAuditStart=[];sendEvent=[];
try
    required={'origin','task_time_s','request_startup','maximum_sources_per_poll', ...
        'heartbeat_max_age_s','landed_max_age_s'};
    assert(isstruct(request)&&isscalar(request)&&all(isfield(request,required)) ...
        &&islogical(request.request_startup)&&isscalar(request.request_startup), ...
        'gpenmpcNative:IoExchangeRequest');
    n=request.maximum_sources_per_poll;
    assert(isnumeric(n)&&isscalar(n)&&isfinite(n)&&n>0&&n==fix(n), ...
        'gpenmpcNative:IoExchangeMemoryBound');
    for key={'heartbeat_max_age_s','landed_max_age_s'}
        v=request.(key{1});assert(isnumeric(v)&&isscalar(v)&&isfinite(v)&&v>0, ...
            'gpenmpcNative:IoExchangeHealthBound');
    end
    incoming=io.pollCanonical();
    assert(incoming.bound&&isempty(incoming.failure)&&incoming.completed_queue_capacity>0 ...
        &&n<=incoming.completed_queue_capacity,'gpenmpcNative:IoExchangeIngress', ...
        'The already bound same IO must provide the explicit completed FIFO.');
    rotorIngress=io.takeCanonicalRotorRecords();rotorProvider.ingest(rotorIngress);
    % Retire the prior inner command on confirmed commit feedback.
    for k=1:n
        item=io.takeCanonical('feedback');if isempty(item),break;end
        result=exchange.feedback(item.message,item.original_host_receive_ns, ...
            gpenmpcNative.rflyOriginalHostMonotonicNs(),request.origin);
        r.feedback_messages=r.feedback_messages+1;
        r.events{end+1}=struct('kind','ORIGINAL_RFC1_COMMIT','ingress',item,'result',result); %#ok<AGROW>
    end
    for k=1:n
        state=exchange.status();
        if state.pending_command,break;end % Keep the next source queued.
        board=io.snapshot();nowWall=io.now();
        assert(all(isfield(board,{'armed','landed_state','heartbeat_rx_s','extended_rx_s'})) ...
            &&all(isfinite([board.armed,board.landed_state,board.heartbeat_rx_s,board.extended_rx_s])) ...
            &&ismember(board.armed,[0 1])&&nowWall>=board.heartbeat_rx_s&&nowWall>=board.extended_rx_s ...
            &&nowWall-board.heartbeat_rx_s<=request.heartbeat_max_age_s ...
            &&nowWall-board.extended_rx_s<=request.landed_max_age_s, ...
            'gpenmpcNative:IoExchangeBoardObservation','Fresh observed board state is required.');
        nowNs=gpenmpcNative.rflyOriginalHostMonotonicNs();
        [rotor,rotorReceipt]=rotorProvider.current(nowNs,nowWall);
        assert(~rotorProvider.Failed,'gpenmpcNative:IoExchangeRotorFault');
        c=state.coordinator;wind=[];windReceipt=[];taskTimeView=[];
        % Full delivery uses the SAME causal service phase as its reference
        % and outer predictor. Do this per original source, not once per
        % host poll: one poll can consume several ordered source events.
        taskTime=request.task_time_s;
        if isfield(request,'delivery_binding')
            binding=request.delivery_binding;
            assert(isstruct(binding)&&isscalar(binding) ...
                &&all(isfield(binding,{'bundle','trajectory'})), ...
                'gpenmpcNative:IoExchangeDeliveryBinding','The actual task and shared leg trajectory are required.');
            taskTimeView=gpenmpcNative.rflyCanonicalDeliveryTimeView(binding.bundle,c,binding.trajectory);
            taskTime=taskTimeView.saved_task_time_s;
        end
        usesInputs=string(c.state)=="UNPREPARED"||board.armed==1 ...
            ||(string(c.state)=="PREPARED_PAUSED"&&request.request_startup&&~state.startup.requested);
        if usesInputs&&~rotor.valid
            assert(board.armed==0,'gpenmpcNative:IoExchangeArmedWithoutRotor');
            % Retain the FIFO until model clock and rotor observations are valid.
            r.events{end+1}=struct('kind','WAIT_ORIGINAL_ROTOR_OBSERVATION','result',rotorReceipt); %#ok<AGROW>
            break
        end
        item=io.takeCanonical('snapshot');if isempty(item),break;end
        nowNs=gpenmpcNative.rflyOriginalHostMonotonicNs();
        if usesInputs
            [wind,windReceipt]=windProvider.observe(taskTime,item.message, ...
                item.original_host_receive_ns,nowNs);
        end
        if board.armed==1
            assert(c.initialized&&state.startup.requested ...
                &&any(string(c.state)==["PREPARED_PAUSED","FLIGHT"]), ...
                'gpenmpcNative:IoExchangePrematureArm');
            result=exchange.source(item.message,item.original_host_receive_ns,nowNs, ...
                rotor,wind,request.origin);
        elseif string(c.state)=="UNPREPARED"
            result=exchange.beginPrepare(item.message,item.original_host_receive_ns,nowNs, ...
                rotor,wind,request.origin);
        elseif string(c.state)=="PREPARED_PAUSED"&&request.request_startup&&~state.startup.requested
            result=exchange.disarmedStartupSnapshot(item.message,item.original_host_receive_ns,nowNs, ...
                rotor,wind,request.origin,false);
        else
            assert(any(string(c.state)==["PREPARING","PREPARED_PAUSED"]), ...
                'gpenmpcNative:IoExchangeUnexpectedDisarm');
            result=exchange.observeDisarmedSnapshot(item.message,item.original_host_receive_ns, ...
                nowNs,request.origin,false);
        end
        r.source_messages=r.source_messages+1;
        entry=struct('kind','ORIGINAL_RSP1_DISPATCH','ingress',item,'board_observation',board, ...
            'rotor_receipt',rotorReceipt,'wind_receipt',windReceipt,'task_time_view',taskTimeView, ...
            'result',result,'send',[],'submission',[]);
        r.events{end+1}=entry;index=numel(r.events); %#ok<AGROW>
        if isfield(result,'packets')&&~isempty(result.packets)
            assert(board.armed==1&&numel(result.packets)==7, ...
                'gpenmpcNative:IoExchangeCommandEnvelope');
            % Record send attempts before socket calls and retain the first submission time.
            % Check the original host-context expiry before transmitting the group.
            preSendNs=gpenmpcNative.rflyOriginalHostMonotonicNs();
            r.events{index}.send_admission=struct('original_host_check_ns',preSendNs, ...
                'original_source_validity',result.send_validity, ...
                'original_reference_expiry_ns',result.context.reference_expiry_ns, ...
                'original_outer_expiry_ns',result.context.outer_expiry_ns);
            assert(preSendNs>=result.context.reference_creation_ns ...
                &&preSendNs>=result.context.outer_creation_ns ...
                &&preSendNs<=result.context.reference_expiry_ns ...
                &&preSendNs<=result.context.outer_expiry_ns, ...
                'gpenmpcNative:IoExchangeOriginalContextExpiredBeforeSend', ...
                'The original context lifetime was crossed before submission.');
            assert(preSendNs>=result.send_validity.source_host_receive_ns ...
                &&preSendNs<=result.send_validity.source_valid_until_ns, ...
                'gpenmpcNative:IoExchangeOriginalSourceExpiredBeforeSend', ...
                'The original source lifetime was crossed before submission.');
            audit=io.evidence();sendAuditStart=numel(audit.raw_transmit_messages);sendEvent=index;
            sent=io.sendCanonicalPackets(result.packets,result.context,result.send_validity);r.events{index}.send=sent;
            r.inner_messages_sent=r.inner_messages_sent+sent.messages_submitted;
            r.events{index}.submission=exchange.submitted(result.binding.command_generation, ...
                sent.original_host_submit_ns(1));
            sendAuditStart=[];sendEvent=[];
            break % Wait for RFC1 before the next source.
        end
    end
    state=exchange.status();
    if string(state.coordinator.state)=="PREPARING"
        result=exchange.pollPreparation(gpenmpcNative.rflyOriginalHostMonotonicNs());
        r.events{end+1}=struct('kind','SAME_WORKER_PREPARATION_POLL','result',result);
    elseif state.coordinator.initialized
        result=exchange.poll(gpenmpcNative.rflyOriginalHostMonotonicNs());
        r.events{end+1}=struct('kind','SAME_WORKER_OUTER_POLL','result',result);
    end
    r.exchange_status=exchange.status();r.startup_ready=r.exchange_status.startup.ready;
catch problem
    r.failed=true;r.same_io_cleanup_required=true;
    r.failure=struct('identifier',problem.identifier,'message',problem.message,'stack',problem.stack, ...
        'original_failure_observed_host_ns',gpenmpcNative.rflyOriginalHostMonotonicNs());
    if ~isempty(sendAuditStart)
        try
            audit=io.evidence();rows=audit.raw_transmit_messages(sendAuditStart+1:end);
            returned=cellfun(@(v)isfield(v,'send_returned')&&v.send_returned,rows);
            attempted=cellfun(@(v)~isfield(v,'send_attempted')||v.send_attempted,rows);
            r.partial_send_evidence=struct('original_transmit_rows',{rows}, ...
                'evidence_rows',numel(rows),'messages_attempted',sum(attempted),'messages_send_returned',sum(returned), ...
                'complete_submission_recorded',false,'board_commit_proven',false);
            r.events{sendEvent}.send_failure_evidence=r.partial_send_evidence;
            if isempty(r.events{sendEvent}.send)
                r.inner_messages_sent=r.inner_messages_sent+sum(returned);
            end
        catch evidenceProblem
            r.failure.send_evidence_read_error=evidenceProblem.message;
        end
    end
    try,r.exchange_status=exchange.status();catch,end
    try,exchange.close();catch cleanup,r.failure.host_close_error=cleanup.message;end
    % Leave IO open for the outer safety cleanup; closing the worker does not stop the board.
end
end
