function [dxNed,diagnosticUp,contact,rotorCommandN] = derivativeNed( ...
    xNed,controls,referenceJetNed,payloadKg,windXyMps,timeS,p)
%#codegen
%DERIVATIVENED Exact current LiveHilPlantService frame/sign adaptation.
% xNed=[world NED p;world NED v;q_wxyz body-to-NED;FRD omega;thrust N].
% This is NOT the generic RflySim body-velocity/RPM state layout.
% Contact and diagnostics retain explicitly named z-up/world quantities.
assert(isequal(size(xNed),[19,1]));
assert(isequal(size(referenceJetNed),[12,1]));
worldSign = [1;1;-1];
quaternionSign = [1;-1;-1;1];
rateSign = [-1;-1;1];
xUp = xNed;
xUp(1:3) = xNed(1:3).*worldSign;
xUp(4:6) = xNed(4:6).*worldSign;
xUp(7:10) = xNed(7:10).*quaternionSign;
xUp(11:13) = xNed(11:13).*rateSign;
jetUp = referenceJetNed;
for k=0:3
    jetUp(3*k+(1:3)) = referenceJetNed(3*k+(1:3)).*worldSign;
end
[dxUp,diagnosticUp,contact,rotorCommandN] = m600check.derivativePx4( ...
    xUp,controls,jetUp,payloadKg,windXyMps,timeS,p);
dxNed = dxUp;
dxNed(1:3) = dxUp(1:3).*worldSign;
dxNed(4:6) = dxUp(4:6).*worldSign;
dxNed(7:10) = dxUp(7:10).*quaternionSign;
dxNed(11:13) = dxUp(11:13).*rateSign;
end
