function [firstIndexZeroBased, uniforms, audit] = gpenmpcPcg64AeroKmeansStream(uniformCount)
%GPENMPCPCG64AEROKMEANSSTREAM NumPy-compatible PCG64 stream for the F17 seed.
%
% NumPy default_rng(1323034700) defines the inducing-point RNG sequence.
% The 128-bit post-SeedSequence state and increment below are the exact PCG64
% state produced by that seed. MATLAB performs every state transition,
% bounded first-index draw and double conversion directly.

arguments
    uniformCount (1,1) double {mustBeInteger,mustBeNonnegative}
end
base = 65536.0;
state = double([hex2dec('b8ba'), hex2dec('e4c1'), hex2dec('a04e'), ...
    hex2dec('f6f5'), hex2dec('8d7b'), hex2dec('689e'), ...
    hex2dec('a893'), hex2dec('1837')]);
increment = double([hex2dec('7cf3'), hex2dec('4845'), hex2dec('f45c'), ...
    hex2dec('52da'), hex2dec('15fe'), hex2dec('e1cd'), ...
    hex2dec('e6a1'), hex2dec('f475')]);
multiplier = double([hex2dec('f645'), hex2dec('9fcc'), hex2dec('df64'), ...
    hex2dec('4385'), hex2dec('5da4'), hex2dec('1fc6'), ...
    hex2dec('ed05'), hex2dec('2360')]);

% NumPy Generator.integers(0, 4680) consumes the low uint32 from one raw
% PCG64 word using Lemire's multiply-high bounded conversion.
[state, raw] = nextRaw(state, multiplier, increment, base);
low32 = bitand(raw, uint64(4294967295));
firstIndexZeroBased = floor(double(low32) * 4680.0 / 4294967296.0);
if firstIndexZeroBased ~= 4648
    error("gpenmpcPcg64AeroKmeansStream:Identity", ...
        "Frozen NumPy PCG64 first index drifted: %d", firstIndexZeroBased);
end

uniforms = zeros(uniformCount, 1);
rawPrefix = zeros(1, min(uniformCount, 4), "uint64");
for index = 1:uniformCount
    [state, raw] = nextRaw(state, multiplier, increment, base);
    if index <= numel(rawPrefix)
        rawPrefix(index) = raw;
    end
    uniforms(index) = double(bitshift(raw, -11)) * 2.0^-53;
end
expectedPrefix = uint64([ ...
    hexToUint64('94d69b4d75dcd6d3'), ...
    hexToUint64('e723e83a83f8dcd3'), ...
    hexToUint64('90974a4d8df618b2'), ...
    hexToUint64('5aa3966a0aba1265')]);
if numel(rawPrefix) >= 4 && ~isequal(rawPrefix(1:4), expectedPrefix)
    error("gpenmpcPcg64AeroKmeansStream:Parity", ...
        "MATLAB PCG64 raw prefix does not match the frozen NumPy stream.");
end

audit = struct;
audit.schema = "GPENMPC_MATLAB_NATIVE_NUMPY_PCG64_STREAM_V1";
audit.seed = uint32(1323034700);
audit.seed_sequence_post_state_hex = "1837a893689e8d7bf6f5a04ee4c1b8ba";
audit.increment_hex = "f475e6a1e1cd15fe52daf45c48457cf3";
audit.first_index_zero_based = firstIndexZeroBased;
audit.uniform_count = uniformCount;
audit.numpy_default_rng_semantics = true;
audit.python_runtime_used = false;
end


function [nextState, raw] = nextRaw(state, multiplier, increment, base)
product = zeros(1, 8);
carry = 0.0;
for outputLimb = 1:8
    total = carry;
    for leftLimb = 1:outputLimb
        rightLimb = outputLimb - leftLimb + 1;
        total = total + state(leftLimb) * multiplier(rightLimb);
    end
    product(outputLimb) = mod(total, base);
    carry = floor(total / base);
end
carry = 0.0;
for limb = 1:8
    total = product(limb) + increment(limb) + carry;
    product(limb) = mod(total, base);
    carry = floor(total / base);
end
nextState = product;
low = limbsToUint64(nextState(1:4));
high = limbsToUint64(nextState(5:8));
xorFolded = bitxor(high, low);
rotation = double(bitshift(high, -58));
if rotation == 0
    raw = xorFolded;
else
    raw = bitor(bitshift(xorFolded, -rotation), ...
        bitshift(xorFolded, 64 - rotation));
end
end


function value = limbsToUint64(limbs)
value = uint64(limbs(1));
value = bitor(value, bitshift(uint64(limbs(2)), 16));
value = bitor(value, bitshift(uint64(limbs(3)), 32));
value = bitor(value, bitshift(uint64(limbs(4)), 48));
end


function value = hexToUint64(text)
high = uint64(hex2dec(text(1:8)));
low = uint64(hex2dec(text(9:16)));
value = bitor(bitshift(high, 32), low);
end
