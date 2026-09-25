function gpenmpcNativeReferenceSfun(block)
%GPENMPCNATIVEREFERENCESFUN Analytic C3 reference and causal environment.
setup(block);
end

function setup(block)
block.NumDialogPrms = 0;
block.NumInputPorts = 0;
block.NumOutputPorts = 2;
block.OutputPort(1).Dimensions = 12;
block.OutputPort(2).Dimensions = 5;
for index = 1:2
    block.OutputPort(index).DatatypeID = 0;
    block.OutputPort(index).Complexity = "Real";
    block.OutputPort(index).SamplingMode = "Sample";
end
block.SampleTimes = [0.01, 0];
block.SimStateCompliance = "DefaultSimState";
block.RegBlockMethod("Outputs", @outputs);
end

function outputs(block)
t = block.CurrentTime;
omegaX = 0.22;
omegaY = 0.18;
omegaZ = 0.30;
x = 0.70 .* t + 0.55 .* sin(omegaX .* t);
y = 1.50 .* sin(omegaY .* t);
z = 1.50 + 0.25 .* sin(omegaZ .* t);
vx = 0.70 + 0.55 .* omegaX .* cos(omegaX .* t);
vy = 1.50 .* omegaY .* cos(omegaY .* t);
vz = 0.25 .* omegaZ .* cos(omegaZ .* t);
ax = -0.55 .* omegaX.^2 .* sin(omegaX .* t);
ay = -1.50 .* omegaY.^2 .* sin(omegaY .* t);
az = -0.25 .* omegaZ.^2 .* sin(omegaZ .* t);
jx = -0.55 .* omegaX.^3 .* cos(omegaX .* t);
jy = -1.50 .* omegaY.^3 .* cos(omegaY .* t);
jz = -0.25 .* omegaZ.^3 .* cos(omegaZ .* t);
block.OutputPort(1).Data = [x; y; z; vx; vy; vz; ax; ay; az; jx; jy; jz];

windEstimate = [2.50 + 0.10 .* sin(0.07 .* t); 1.50];
gust = 0.35 .* exp(-((t - 2.5) ./ 0.65).^2);
actualWind = windEstimate + [0.15 .* sin(0.31 .* t); -gust];
payload = 1.20 - 0.45 .* (t >= 3.0);
block.OutputPort(2).Data = [windEstimate; actualWind; payload];
end

