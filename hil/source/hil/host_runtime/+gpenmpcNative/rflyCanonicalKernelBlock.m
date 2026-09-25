function [controls16, result61, valid] = rflyCanonicalKernelBlock(u,continuityEnabled,enable)
%#codegen
% Map the 101-double kernel input to normalized RflySim controls.
% The caller binds a fresh coherent generation.
controls16=zeros(16,1,'single');result61=zeros(61,1);valid=false;
if ~enable,return;end
p=struct('kp',u(50:52),'kd',u(53:55),'kr',u(56:58),'kw',u(59:61), ...
    'drag',u(62:64),'inertia',reshape(u(65:73),3,3), ...
    'pseudoinverse',reshape(u(74:97),6,4),'baseMass',u(98), ...
    'totalThrust',u(99),'rotorUpper',u(100),'maxTilt',u(101));
[w,r,d,kernelValid]=gpenmpcNative.se3WrenchKernel(u(1:19),u(20:22),u(23:25), ...
    u(26:28),u(29),u(30:31),u(32:34),continuityEnabled,reshape(u(35:43),3,3), ...
    u(44:46),u(47:49),p);
result61=[w;r;d];
% Validate the six-motor input before narrowing the wider vendor interface.
upper=32.145727009134916;
if ~kernelValid || p.rotorUpper~=upper || any(~isfinite(r)) || any(r<0) || any(r>upper)
    return;
end
map=[5;1;4;6;2;3];
for k=1:6,controls16(map(k))=single(r(k)/upper);end
valid=true;
end
