function receipt=export_retained_local_input_codec()
% Export records from the completed host peer test.
build=string(fileparts(fileparts(mfilename('fullpath'))));
root=fullfile(gpenmpc_external_path('stream_startup_method_mex'));
source=fullfile(root,'RAW_EXECUTION_STREAM_MEX.mat');
out=fullfile(root,'RETAINED_RLI_CODEC_INPUTS');if ~isfolder(out),mkdir(out);end
s=load(source,'flightFixture','registered');f=s.flightFixture;
assert(numel(f.actual_input)==647&&numel(f.source)==382&&numel(f.peer_cases)==1);
item=struct('input647',uint8(f.actual_input(:)),'source382',uint8(f.source(:)), ...
 'registered',s.registered,'original_source_receive_ns',f.responses{1}.input_event.source.original_host_receive_ns, ...
 'scope','RETAINED_EXPLICIT_HOST_PEER_INPUT_CODEC_ONLY_NO_NEW_GETTER_OR_BOARD_BINDING');
matpath=fullfile(out,'RETAINED_RLI_CODEC_INPUT.mat');
if isfile(matpath)
 prior=load(matpath,'item');assert(isequaln(prior.item,item),'Existing retained input differs');
else
 save(matpath,'item');
end
writebin(fullfile(out,'RLI647.bin'),item.input647);writebin(fullfile(out,'RLS382.bin'),item.source382);
receipt=struct('source_mat',source,'source_sha256',sha(source),'scope',item.scope, ...
 'rows',1,'input_sha256',sha(fullfile(out,'RLI647.bin')),'rls_sha256',sha(fullfile(out,'RLS382.bin')), ...
 'COM',0,'board',0,'model',0,'new_numerical_execution',false,'new_original_getter_binding',false);
receipt.packaging_note='MAT/BIN exports are available; verify them before writing the JSON receipt.';
jsonpath=fullfile(out,'EXPORT.json');assert(~isfile(jsonpath));
fid=fopen(jsonpath,'w');assert(fid>=0);guard=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(receipt,PrettyPrint=true));disp(jsonencode(receipt));
end
function writebin(p,b)
if isfile(p)
 f=fopen(p,'rb');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
 assert(isequal(fread(f,Inf,'*uint8'),b(:)),'Existing retained bytes differ');return
end
f=fopen(p,'wb');assert(f>=0);g=onCleanup(@()fclose(f)); %#ok<NASGU>
assert(fwrite(f,b,'uint8')==numel(b));
end
function h=sha(p)
f=fopen(p,'rb');assert(f>=0);g=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8'); %#ok<NASGU>
m=java.security.MessageDigest.getInstance('SHA-256');m.update(typecast(b,'int8'));
h=upper(reshape(dec2hex(typecast(m.digest(),'uint8'),2).',1,[]));
end
