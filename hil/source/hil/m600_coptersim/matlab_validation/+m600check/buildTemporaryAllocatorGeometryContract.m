function [contract,validation]=buildTemporaryAllocatorGeometryContract(auditPath)
% Read-only assembly from the exact actual math report; creates no files.
contract=struct();validation=struct('passed',false,'failure','','errors',{{}});
try
    math=jsondecode(fileread(auditPath));a=math.audit;delta=a.proposed_parameter_delta;
    e=repmat(struct('name','','mav_type',9,'original_raw_bits_hex','','target_raw_bits_hex',''),numel(delta),1);
    for k=1:numel(delta)
        e(k)=struct('name',delta(k).name,'mav_type',delta(k).mav_type, ...
            'original_raw_bits_hex',delta(k).original_raw_bits_hex,'target_raw_bits_hex',delta(k).candidate_raw_bits_hex);
    end
    rows=a.unchanged_diagnostic_parameters;g=repmat(struct('name','','mav_type',0,'raw_bits_hex',''),numel(rows),1);
    for k=1:numel(rows),p=rows(k).typed_value;g(k)=struct('name',p.name,'mav_type',p.mav_type,'raw_bits_hex',p.raw_bits_hex);end
    contract=struct('schema','TEMPORARY_M600_NATIVE_ALLOCATOR_GEOMETRY_V1','entries',e, ...
        'bindings',struct('geometry_audit',identity(auditPath),'native79',identity(a.receipt_path), ...
            'canonical_source',identity(a.source_path)), ...
        'maximum_apply_passes',1,'restore_policy','IDEMPOTENT_PER_OWNER_AFTER_DISARM_ZERO_OUTPUTS', ...
        'gains_changes',0,'failure_detector_changes',0,'physical_mapping_changes',0,'model_changes',0, ...
        'unchanged_guard_entries',g);
    validation=m600check.validateTemporaryAllocatorGeometry(contract);
catch p
    validation.failure=[p.identifier ': ' p.message];validation.errors={struct('identifier',p.identifier,'message',p.message)};
end
end
function r=identity(path)
fid=fopen(path,'rb');assert(fid>=0,'m600check:GeometryFileRead','Cannot read %s',path);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
raw=fread(fid,Inf,'*uint8');md=java.security.MessageDigest.getInstance('SHA-256');md.update(raw);
h=upper(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));r=struct('path',char(path),'bytes',numel(raw),'sha256',h);
end
