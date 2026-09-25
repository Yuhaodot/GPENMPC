function receipt=save_m600_short_raw(path,data)
% Preserve every original variable, type, timestamp and byte; change container only.
% v7 avoids HDF5's per-small-object overhead. Large variables retain v7.3.
assert(startsWith(string(path),gpenmpc_external_path('raw_recording_output_root'),'IgnoreCase',true));
assert(isstruct(data)&&isscalar(data)&&~isfile(path),'gpenmpcShort:RawTarget','Fresh E-drive raw target required.');
names=fieldnames(data);sizes=zeros(numel(names),1);
for k=1:numel(names),value=data.(names{k});s=whos('value');sizes(k)=s.bytes;end
clear value
version='-v7';if any(sizes>=1.8e9),version='-v7.3';end % Headroom below v7's 2^31-byte/variable limit.
partial=[char(path) '.saving.mat'];assert(~isfile(partial),'gpenmpcShort:PartialRawExists','Preserve previous partial file.');
started=tic;save(partial,'-struct','data',version);
stored=whos('-file',partial);
assert(isequal(sort({stored.name}),sort(names.')),'gpenmpcShort:RawVariables','All raw variable names must survive storage.');
for k=1:numel(names)
 at=find(strcmp({stored.name},names{k}));value=data.(names{k});
 assert(strcmp(stored(at).class,class(value))&&isequal(stored(at).size,size(value)),'gpenmpcShort:RawShape','Raw class/shape mismatch.');
end
[ok,msg]=movefile(partial,path);assert(ok,'%s',msg);
f=dir(path);receipt=struct('saved',true,'format',version,'elapsed_s',toc(started),'bytes',f.bytes, ...
 'variables',{names},'variable_memory_bytes',sizes,'content_dropped',false,'path',char(path));
end
