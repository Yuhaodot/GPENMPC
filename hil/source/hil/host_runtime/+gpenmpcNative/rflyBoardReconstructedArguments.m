function [bytes,receipt]=rflyBoardReconstructedArguments(hostBytes,command,source)
% Reconstruct RKS1-to-RAK1 state and retain both host and board hashes.
% MATLAB norm and C sum/sqrt can differ by one ulp.
raw=double(source.raw13_float32(:));p=raw(1:3)-source.task_origin_ned_m(:);
n2=0.0;for k=7:10,term=raw(k)*raw(k);n2=n2+term;end
n=sqrt(n2);assert(isfinite(n)&&n>0);
% Preserve the C++ diagonal-product zero additions, including signed zero.
mapped=zeros(6,1);vectors=[p,raw(4:6)];C=diag([1,1,-1]);
for side=1:2
    for row=1:3
        value=0.0;for col=1:3,term=C(row,col)*vectors(col,side);value=value+term;end
        mapped((side-1)*3+row)=value;
    end
end
x=[mapped;(raw(7:10)/n).*[1;-1;-1;1];raw(11:13).*[-1;-1;1];zeros(6,1)];
original=command.state_up(:);
assert(isequal(original([1:6,11:19]),x([1:6,11:19])) ...
    &&max(abs(original(7:10)-x(7:10)))<=1e-12, ...
    'gpenmpcNative:BoardPrivateStateMapping','Existing C++ state mapping allowance only; no origin/state replacement.');
assert(numel(hostBytes)==829&&isa(hostBytes,'uint8'));
bytes=hostBytes(:);[~,~,endian]=computer;encoded=x;if endian=='L',encoded=swapbytes(encoded);end
bytes(5:156)=reshape(typecast(encoded,'uint8'),[],1);
receipt=struct('scope','EXPECTED_CPP_STATE_RECONSTRUCTION', ...
    'quaternion_representation_max_delta',max(abs(original(7:10)-x(7:10))), ...
    'original_HOST829_preserved',true,'board_reconstructed_state19',x,'controller_recomputed',false);
end
