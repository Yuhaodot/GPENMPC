function value=normalizeTerrainFixtureMetadata(value)
% Read the cause-identification field from external terrain fixture formats.
aliases={'live004_unique_cause_proven','unique_live004_cause_proven'};
target='terrain_source_cause_identified';
if isstruct(value)
    for k=1:numel(aliases)
        name=aliases{k};
        if isfield(value,name)
            assert(~isfield(value,target),'m600check:TerrainFixtureAlias', ...
                'Terrain fixture contains duplicate cause-identification fields.');
            for j=1:numel(value)
                flag=value(j).(name);
                assert(islogical(flag)&&isscalar(flag),'m600check:TerrainFixtureType', ...
                    'Terrain cause-identification field must be a scalar logical.');
                value(j).(target)=flag;
            end
            value=rmfield(value,name);
        end
    end
    names=fieldnames(value);
    for j=1:numel(value)
        for k=1:numel(names)
            value(j).(names{k})=m600check.normalizeTerrainFixtureMetadata(value(j).(names{k}));
        end
    end
elseif iscell(value)
    for j=1:numel(value),value{j}=m600check.normalizeTerrainFixtureMetadata(value{j});end
end
end
