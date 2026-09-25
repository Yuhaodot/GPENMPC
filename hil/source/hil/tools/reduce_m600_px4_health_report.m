function report=reduce_m600_px4_health_report(rawMav,request,metadataPath,expectedSha256,options)
% REDUCE_M600_PX4_HEALTH_REPORT Decode PX4 EVENT reports with fail-closed validation.
% rawMav contains {topic,rx_s,message}; request contains sent_s, source_system,
% source_component and minimum_event_boot_ms. An ACK alone does not confirm arming health.
% Keep detail text as the metadata template.
arguments
    rawMav
    request (1,1) struct
    metadataPath (1,1) string
    expectedSha256 (1,1) string
    options.max_event_rows (1,1) double = 256
    options.overflow (1,1) logical = false
end
report=struct('schema','M600_PX4_HEALTH_EVENT_REPORT_V1','status','UNKNOWN', ...
    'complete',false,'can_arm_offboard',false,'can_arm_offboard_known',false, ...
    'failure','','request',request,'source_event_count',0,'duplicate_count',0, ...
    'ignored_pre_request_count',0,'ignored_other_source_count',0,'ignored_old_boot_count',0, ...
    'details',struct([]),'arming_summary',struct(),'health_summary',struct(), ...
    'metadata_path',char(metadataPath),'metadata_sha256','','offboard_mode_mask',16384, ...
    'first_sequence',[],'last_sequence',[],'received_at_s',[], ...
    'health_pass_is_not_actual_arm_guarantee',true,'hardware_actions',0);
try
    require(all(isfield(request,{'sent_s','source_system','source_component','minimum_event_boot_ms'})), ...
        'REQUEST_SCHEMA');
    require(all(isfinite([request.sent_s,request.source_system,request.source_component, ...
        request.minimum_event_boot_ms]))&&request.sent_s>=0&&request.minimum_event_boot_ms>=0, ...
        'REQUEST_VALUES');
    require(options.max_event_rows>=2&&options.max_event_rows<=65535&& ...
        options.max_event_rows==fix(options.max_event_rows),'EVENT_BOUND');
    require(~options.overflow,'BUFFER_OVERFLOW');
    report.metadata_sha256=m600check.fileSha256(metadataPath);
    require(strcmpi(report.metadata_sha256,expectedSha256),'METADATA_SHA_MISMATCH');
    metadata=jsondecode(fileread(metadataPath));
    component=metadata.components.x1;
    require(strcmp(component.namespace,'px4')&& ...
        strcmp(component.enums.navigation_mode_group_t.type,'uint32_t')&& ...
        strcmp(component.enums.health_component_t.type,'uint32_t')&& ...
        strcmp(component.enums.navigation_mode_group_t.entries.x16384.name,'offboard'), ...
        'METADATA_PROTOCOL_MISMATCH');
    if isstruct(rawMav),rawMav=num2cell(rawMav);end
    require(iscell(rawMav),'RAW_SCHEMA');
    rows=struct('sequence',{},'rx_s',{},'payload',{},'group',{},'definition',{});
    for k=1:numel(rawMav)
        r=rawMav{k};
        require(isstruct(r)&&all(isfield(r,{'topic','rx_s','message'})),'RAW_ROW_SCHEMA');
        if ~any(strcmp(r.topic,{'EVENT','CURRENT_EVENT_SEQUENCE','RESPONSE_EVENT_ERROR'})),continue;end
        require(isscalar(r.rx_s)&&isfinite(r.rx_s),'RX_TIME_INVALID');
        if r.rx_s<request.sent_s,report.ignored_pre_request_count=report.ignored_pre_request_count+1;continue;end
        m=r.message;require(all(isfield(m,{'SystemID','ComponentID','Payload'})),'MESSAGE_SCHEMA');
        if double(m.SystemID)~=request.source_system||double(m.ComponentID)~=request.source_component
            report.ignored_other_source_count=report.ignored_other_source_count+1;continue
        end
        p=m.Payload;
        if strcmp(r.topic,'CURRENT_EVENT_SEQUENCE')
            require(isfield(p,'flags')&&isfinite(double(p.flags)),'SEQUENCE_SCHEMA');
            require(bitand(uint8(p.flags),uint8(1))==0,'SEQUENCE_RESET_AFTER_REQUEST');continue
        elseif strcmp(r.topic,'RESPONSE_EVENT_ERROR')
            error('m600health:Invalid','EVENT_RETRIEVAL_ERROR');
        end
        fields={'id','event_time_boot_ms','sequence','destination_system', ...
            'destination_component','log_levels','arguments'};
        require(all(isfield(p,fields)),'EVENT_SCHEMA');
        nums=[double(p.id),double(p.event_time_boot_ms),double(p.sequence), ...
            double(p.destination_system),double(p.destination_component),double(p.log_levels)];
        require(numel(nums)==6&&all(isfinite(nums))&&all(nums>=0)&&all(nums==fix(nums))&& ...
            nums(1)<=4294967295&&nums(2)<=4294967295&&nums(3)<=65535&&all(nums(4:6)<=255), ...
            'EVENT_TYPED_RANGE');
        if double(p.event_time_boot_ms)<request.minimum_event_boot_ms
            report.ignored_old_boot_count=report.ignored_old_boot_count+1;continue
        end
        require(numel(p.arguments)==40&&all(isfinite(double(p.arguments(:))))&& ...
            all(double(p.arguments(:))>=0)&all(double(p.arguments(:))<=255)&& ...
            all(double(p.arguments(:))==fix(double(p.arguments(:)))),'EVENT_ARGUMENT_BYTES');
        p.arguments=uint8(p.arguments(:).');
        require(any(double(p.destination_system)==[0 255])&& ...
            any(double(p.destination_component)==[0 190]),'EVENT_DESTINATION_MISMATCH');
        [group,definition]=lookup(metadata,double(p.id));
        duplicate=find([rows.sequence]==double(p.sequence),1);
        if ~isempty(duplicate)
            require(isequaln(rows(duplicate).payload,p),'SEQUENCE_PAYLOAD_CONFLICT');
            report.duplicate_count=report.duplicate_count+1;continue
        end
        require(numel(rows)<options.max_event_rows,'BUFFER_OVERFLOW');
        rows(end+1)=struct('sequence',double(p.sequence),'rx_s',r.rx_s,'payload',p, ...
            'group',group,'definition',definition); %#ok<AGROW>
    end
    report.source_event_count=numel(rows);
    require(~isempty(rows),'NO_CURRENT_EVENT_REPORT');
    names=arrayfun(@(x)string(x.definition.name),rows);
    starts=find(names=="commander_arming_check_summary");
    require(~isempty(starts),'ARMING_SUMMARY_MISSING');
    % Use the latest complete health summary.
    [~,n]=max([rows(starts).rx_s]);start=starts(n);first=rows(start).sequence;
    distance=mod([rows.sequence]-first,65536);
    ends=find(names=="commander_health_summary"&distance>0&distance<options.max_event_rows);
    require(~isempty(ends),'HEALTH_SUMMARY_MISSING');
    [lastDistance,n]=min(distance(ends));last=ends(n);
    selected=find(distance<=lastDistance);[~,order]=sort(distance(selected));selected=selected(order);
    require(isequal(distance(selected),0:lastDistance),'EVENT_SEQUENCE_GAP');
    require(nnz(names(selected)=="commander_arming_check_summary")==1,'NESTED_ARMING_SUMMARY');
    a=decodeSummary(rows(start),'arming');h=decodeSummary(rows(last),'health');
    details=struct('sequence',{},'rx_s',{},'event_time_boot_ms',{},'event_id',{}, ...
        'group',{},'name',{},'message_template',{},'description_template',{}, ...
        'raw_arguments_uint8',{},'external_log_level',{},'modes',{},'applies_to_offboard',{});
    for idx=selected(2:end-1)
        row=rows(idx);d=row.definition;
        if any(strcmp(row.group,{'arming_check','health'}))
            require(isfield(d,'arguments')&&numel(d.arguments)>=2&& ...
                strcmp(d.arguments(1).name,'modes')&&strcmp(d.arguments(2).name,'health_component_index'), ...
                'DETAIL_METADATA_ARGUMENT_SCHEMA');
            modes=u32(row.payload.arguments,1);description='';
            if isfield(d,'description'),description=d.description;end
            details(end+1)=struct('sequence',row.sequence,'rx_s',row.rx_s, ...
                'event_time_boot_ms',double(row.payload.event_time_boot_ms), ...
                'event_id',double(row.payload.id),'group',row.group,'name',d.name, ...
                'message_template',d.message,'description_template',description, ...
                'raw_arguments_uint8',row.payload.arguments, ...
                'external_log_level',double(bitand(uint8(row.payload.log_levels),uint8(15))), ...
                'modes',double(modes),'applies_to_offboard',bitand(modes,uint32(16384))~=0); %#ok<AGROW>
        end
    end
    report.complete=true;report.status='COMPLETE_CURRENT_HEALTH_REPORT';
    report.arming_summary=a;report.health_summary=h;report.details=details;
    report.can_arm_offboard=bitand(uint32(a.can_arm),uint32(16384))~=0;
    report.can_arm_offboard_known=true;report.first_sequence=first;
    report.last_sequence=rows(last).sequence;report.received_at_s=rows(last).rx_s;
catch problem
    report.status='UNKNOWN';report.complete=false;report.can_arm_offboard=false;
    report.can_arm_offboard_known=false;report.failure=problem.message;
end
end

function [group,definition]=lookup(metadata,id)
componentKey=sprintf('x%u',bitshift(uint32(id),-24));local=bitand(uint32(id),uint32(16777215));
require(isfield(metadata.components,componentKey),'UNKNOWN_EVENT_COMPONENT_NAMESPACE');
groups=metadata.components.(componentKey).event_groups;key=sprintf('x%u',local);
group='';definition=struct();
for name=string(fieldnames(groups)).'
    events=groups.(name).events;
    if isfield(events,key),require(isempty(group),'AMBIGUOUS_EVENT_METADATA');group=char(name);definition=events.(key);end
end
require(~isempty(group)&&isfield(definition,'name'),'UNKNOWN_EVENT_ID');
end

function value=decodeSummary(row,kind)
d=row.definition;a=row.payload.arguments;
if strcmp(kind,'arming')
    expected={'chunk_idx','error','warning','can_arm','can_run'};
    require(strcmp(row.group,'arming_check'),'SUMMARY_GROUP');
else
    expected={'chunk_idx','is_present','error','warning'};
    require(strcmp(row.group,'health'),'SUMMARY_GROUP');
end
require(isfield(d,'arguments')&&isequal({d.arguments.name},expected)&& ...
    strcmp(d.arguments(1).type,'uint8_t'),'SUMMARY_METADATA_ARGUMENT_SCHEMA');
require(a(1)==0,'UNSUPPORTED_SUMMARY_CHUNK');
value=struct('chunk_idx',0,'raw_arguments_uint8',a,'event_id',double(row.payload.id));
for k=2:numel(expected),value.(expected{k})=double(u32(a,2+(k-2)*4));end
end

function x=u32(a,offset)
b=double(a(offset:offset+3));x=uint32(sum(b.*[1 256 65536 16777216]));
end
function require(condition,message)
assert(isscalar(condition)&&condition,'m600health:Invalid','%s',message);
end
