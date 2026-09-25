function receipt=verify_m600_delivery_parameter_union(readParameter,tuning,geometry,phase)
% Typed 138-row union for delivery tuning3 + geometry12.
receipt=struct('schema','M600_CANONICAL_DELIVERY_TYPED_UNION_V1','phase',char(phase), ...
    'passed',false,'failure','','rows',[],'read_attempts',0,'expected_count',138, ...
    'parameter_writes',0,'geometry_delta_count',12,'tuning_delta_count',3,'unchanged_count',123);
try
    tv=validate_m600_delivery_native_tuning_contract(tuning);
    gv=m600check.validateTemporaryAllocatorGeometry(geometry);
    assert(tv.passed&&gv.passed,'m600delivery:UnionContracts');
    assert(any(strcmp(char(phase),{'ORIGINAL','APPLIED','RESTORED'})),'m600delivery:UnionPhase');
    expected=tuning.unchanged_guard_entries(:);
    for k=1:3
        e=tuning.entries(k);expected(end+1)=struct('name',e.name,'mav_type',e.mav_type, ...
            'raw_bits_hex',e.original_raw_bits_hex); %#ok<AGROW>
    end
    assert(numel(expected)==138&&numel(unique({expected.name}))==138);
    for k=1:12
        e=geometry.entries(k);i=find(strcmp({expected.name},e.name));
        assert(isscalar(i)&&expected(i).mav_type==e.mav_type&& ...
            strcmpi(expected(i).raw_bits_hex,e.original_raw_bits_hex));
    end
    if strcmp(char(phase),'APPLIED')
        deltas=[geometry.entries(:);tuning.entries(:)];
        for k=1:numel(deltas)
            i=find(strcmp({expected.name},deltas(k).name));
            expected(i).raw_bits_hex=deltas(k).target_raw_bits_hex;
        end
    end
    template=struct('name','','expected_type',0,'expected_bits','','attempted',false, ...
        'passed',false,'observed',[],'error','');rows=repmat(template,138,1);
    for k=1:138
        e=expected(k);rows(k).name=e.name;rows(k).expected_type=e.mav_type;
        rows(k).expected_bits=e.raw_bits_hex;rows(k).attempted=true;receipt.read_attempts=receipt.read_attempts+1;
        try
            p=readParameter(e.name);rows(k).observed=p;
            assert(strcmp(p.name,e.name)&&p.mav_type==e.mav_type&&strcmpi(p.raw_bits_hex,e.raw_bits_hex));
            rows(k).passed=true;
        catch problem,rows(k).error=[problem.identifier ': ' problem.message];end
    end
    receipt.rows=rows;receipt.passed=all([rows.passed]);
    if ~receipt.passed,receipt.failure=['Typed mismatch: ' strjoin({rows(~[rows.passed]).name},', ')];end
catch problem
    receipt.failure=[problem.identifier ': ' problem.message];
end
end
