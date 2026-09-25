function receipt=verifyNativeHoverParameterUnion(readParameter,tuning,geometry,phase)
% Read-only typed 138-field comparison at an explicit mutation phase.
% ORIGINAL: before either of the separately logged 12 + 3 changes.
% APPLIED: disjoint geometry12 and tuning3 targets; remaining123 unchanged.
% RESTORED: all138 original. No exemption for any overlapping/missing field.
receipt=struct('schema','M600_NATIVE_HOVER_TYPED_UNION_V1','phase','', ...
    'passed',false,'failure','','rows',[],'read_attempts',0,'expected_count',138, ...
    'parameter_writes',0,'geometry_delta_count',12,'tuning_delta_count',3,'unchanged_count',123);
try
    assert(isa(readParameter,'function_handle'),'m600check:UnionReader');
    assert((ischar(phase)&&isrow(phase))||(isstring(phase)&&isscalar(phase)),'m600check:UnionPhase');
    phase=char(phase);receipt.phase=phase;
    assert(any(strcmp(phase,{'ORIGINAL','APPLIED','RESTORED'})),'m600check:UnionPhase');
    tv=m600check.validateNativeHoverTuningContract(tuning);
    gv=m600check.validateTemporaryAllocatorGeometry(geometry);
    assert(tv.passed&&gv.passed,'m600check:UnionContracts','Both independent contracts must pass.');
    expected=tuning.unchanged_guard_entries(:);
    for k=1:3
        e=tuning.entries(k);expected(end+1,1)=struct('name',e.name,'mav_type',e.mav_type, ...
            'raw_bits_hex',e.original_raw_bits_hex); %#ok<AGROW>
    end
    assert(numel(expected)==138&&numel(unique({expected.name}))==138,'m600check:UnionCoverage');
    assert(isempty(intersect({geometry.entries.name},{tuning.entries.name})),'m600check:UnionOverlap');
    % Every geometry original must already exist in the independent 138
    % snapshot, rather than trusting a target as an original identity.
    for k=1:12
        e=geometry.entries(k);i=find(strcmp({expected.name},e.name));
        assert(isscalar(i)&&expected(i).mav_type==9&& ...
            strcmpi(expected(i).raw_bits_hex,e.original_raw_bits_hex),'m600check:UnionOriginalConflict');
    end
    % The old 67-field geometry guard remains untouched. Its three tuning
    % originals are compared here before phase-specific targets are applied.
    for k=1:numel(geometry.unchanged_guard_entries)
        e=geometry.unchanged_guard_entries(k);i=find(strcmp({expected.name},e.name));
        assert(isscalar(i)&&expected(i).mav_type==e.mav_type&& ...
            strcmpi(expected(i).raw_bits_hex,e.raw_bits_hex),'m600check:UnionGuardConflict');
    end
    if strcmp(phase,'APPLIED')
        deltas=[geometry.entries(:);tuning.entries(:)];
        assert(numel(deltas)==15&&numel(unique({deltas.name}))==15,'m600check:UnionDeltaCount');
        for k=1:numel(deltas)
            i=find(strcmp({expected.name},deltas(k).name));expected(i).raw_bits_hex=deltas(k).target_raw_bits_hex;
        end
    end
    rows=repmat(struct('name','','expected_type',0,'expected_bits','','attempted',false, ...
        'passed',false,'observed',[],'error',''),138,1);
    for k=1:138
        e=expected(k);rows(k).name=e.name;rows(k).expected_type=e.mav_type;rows(k).expected_bits=e.raw_bits_hex;
        rows(k).attempted=true;receipt.read_attempts=receipt.read_attempts+1;
        try
            p=readParameter(e.name);rows(k).observed=p;
            assert(isstruct(p)&&isscalar(p)&&all(isfield(p,{'name','mav_type','raw_bits_hex','decoded'}))&& ...
                strcmp(p.name,e.name)&&p.mav_type==e.mav_type&&strcmpi(p.raw_bits_hex,e.raw_bits_hex), ...
                'm600check:UnionTypedMismatch');
            bits=uint32(hex2dec(e.raw_bits_hex));
            if e.mav_type==9,v=double(typecast(bits,'single'));
            elseif e.mav_type==6,v=double(typecast(bits,'int32'));
            else,error('m600check:UnionUnsupportedType','Unexpected parameter type.');end
            assert(isnumeric(p.decoded)&&isreal(p.decoded)&&isscalar(p.decoded)&&isfinite(p.decoded)&& ...
                isfinite(v)&&double(p.decoded)==v,'m600check:UnionDecodedMismatch');
            rows(k).passed=true;
        catch problem,rows(k).error=[problem.identifier ': ' problem.message];end
    end
    receipt.rows=rows;receipt.passed=all([rows.passed]);
    if ~receipt.passed,receipt.failure=['Typed mismatch: ' strjoin({rows(~[rows.passed]).name},', ')];end
catch problem,receipt.failure=[problem.identifier ': ' problem.message];end
end
