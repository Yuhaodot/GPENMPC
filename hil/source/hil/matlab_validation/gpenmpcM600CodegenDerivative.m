function [dx,diagnostic,contact,rotorCommandN] = gpenmpcM600CodegenDerivative(x,u,jet,payload,wind,time,p)
%#codegen
% Fixed-size Coder entry to the existing MATLAB M600 equations and adapters.
[dx,diagnostic,contact,rotorCommandN]=m600check.derivativeNed(x,u,jet,payload,wind,time,p);
end
