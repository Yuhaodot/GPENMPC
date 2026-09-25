function test_rc_console_storage()
root=gpenmpc_external_path('host_recording_root');
out=fullfile(root,'console_storage');if ~isfolder(out),mkdir(out);end
source=fullfile(gpenmpc_external_path('compact_storage_recording'),'SHORT_HIL','RAW_BOARD_LOCAL_SHORT_HIL.mat');
tic;data=load(source);fprintf('OLD_LOAD_S %.3f\n',toc);
% Check storage using recorded input data.
dest=fullfile(out,'RAW_LOSSLESS_V7.mat');receipt=save_m600_short_raw(dest,data);
tic;back=load(dest);readSeconds=toc;
assert(isequaln(data,back),'Full raw structure round-trip must be exactly equal, including NaNs and integer stamps.');
old=dir(source);fprintf('LOSSLESS true OLD_BYTES %.0f NEW_BYTES %.0f SAVE_S %.3f LOAD_S %.3f\n',old.bytes,receipt.bytes,receipt.elapsed_s,readSeconds);
large=whos('-file',fullfile(gpenmpc_external_path('large_storage_recording'),'SHORT_HIL','RAW_BOARD_LOCAL_SHORT_HIL.mat'));
for k=1:numel(large),fprintf('RECORDED_VARIABLE %s %.0f\n',large(k).name,large(k).bytes);end
end
