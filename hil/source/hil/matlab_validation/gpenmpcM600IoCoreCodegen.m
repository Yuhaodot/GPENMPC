function [y,diagnostic] = gpenmpcM600IoCoreCodegen(controls,reset,position,euler,environment,p)
%#codegen
%GPENMPCM600IOCORECODEGEN Pure fixed-size complete-core Coder entry point.
[y,diagnostic]=m600check.copterSimIoCore(controls,reset,position,euler,environment,p);
end
