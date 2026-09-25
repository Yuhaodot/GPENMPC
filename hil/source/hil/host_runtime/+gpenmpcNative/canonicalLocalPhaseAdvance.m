function next=canonicalLocalPhaseAdvance(input7)
%#codegen
% Compute phase and rate after the reference transition.
% input = [phase;rate;acceptedAcceleration;dt;duration;rateMin;rateMax].
% Install the candidate with the associated numerical/reference commit.
arguments
    input7 (7,1) double
end
phase=input7(1);rate=input7(2);acceleration=input7(3);dt=input7(4);
duration=input7(5);rateMin=input7(6);rateMax=input7(7);
next=zeros(2,1);
next(1)=min(duration,max(0.0,phase+dt*rate+0.5*dt^2*acceleration));
next(2)=min(max(rate+dt*acceleration,rateMin),rateMax);
end
