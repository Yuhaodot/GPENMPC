function [stateNextNed,diagnosticUp,contact,memoryNext,status] = stepPx4Rk4Ned( ...
    stateNed,controls,referenceNed,payloadKg,windXyMps,timeS,p,memory)
%#codegen
%STEPPX4RK4NED Current world-NED/FRD sign adapter, not Rfly RPM/body velocity.
world=[1;1;-1];qs=[1;-1;-1;1];rates=[-1;-1;1];
x=stateNed;
x(1:3)=x(1:3).*world;x(4:6)=x(4:6).*world;
x(7:10)=x(7:10).*qs;x(11:13)=x(11:13).*rates;
jet=referenceNed;
for k=0:3
    jet(3*k+(1:3))=jet(3*k+(1:3)).*world;
end
[x,diagnosticUp,contact,memoryNext,status]=m600check.stepPx4Rk4( ...
    x,controls,jet,payloadKg,windXyMps,timeS,p,memory);
stateNextNed=x;
stateNextNed(1:3)=x(1:3).*world;stateNextNed(4:6)=x(4:6).*world;
stateNextNed(7:10)=x(7:10).*qs;stateNextNed(11:13)=x(11:13).*rates;
% Keep diagnostic/candidate state explicitly z-up; do not relabel it NED.
end
