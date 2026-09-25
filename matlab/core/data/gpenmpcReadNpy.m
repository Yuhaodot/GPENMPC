function [value, metadata] = gpenmpcReadNpy(path)
%GPENMPCREADNPY Read a numeric NumPy NPY array in MATLAB.
%
% The GPENMPC raw traces use little endian, C ordered, numeric NPY arrays.
% This reader supports the numeric dtypes used by the GPENMPC input files and
% rejects object arrays or unsupported encodings.

arguments
    path (1,1) string
end

fileId = fopen(path, "r", "ieee-le");
if fileId < 0
    error("gpenmpcReadNpy:Open", "Cannot open NPY file: %s", path);
end
cleanup = onCleanup(@() fclose(fileId));

magic = fread(fileId, 6, "*uint8").';
expected = uint8([147, double('NUMPY')]);
if ~isequal(magic, expected)
    error("gpenmpcReadNpy:Magic", "Invalid NPY magic in %s", path);
end
major = fread(fileId, 1, "*uint8");
minor = fread(fileId, 1, "*uint8");
if major == 1
    headerLength = double(fread(fileId, 1, "*uint16"));
elseif major == 2 || major == 3
    headerLength = double(fread(fileId, 1, "*uint32"));
else
    error("gpenmpcReadNpy:Version", "Unsupported NPY version %d.%d", major, minor);
end
header = char(fread(fileId, headerLength, "*uint8").');

descriptionToken = regexp(header, "'descr'\s*:\s*'([^']+)'", "tokens", "once");
fortranToken = regexp(header, "'fortran_order'\s*:\s*(True|False)", "tokens", "once");
shapeToken = regexp(header, "'shape'\s*:\s*\(([^)]*)\)", "tokens", "once");
if isempty(descriptionToken) || isempty(fortranToken) || isempty(shapeToken)
    error("gpenmpcReadNpy:Header", "Unsupported NPY header: %s", header);
end
description = string(descriptionToken{1});
fortranOrder = strcmp(fortranToken{1}, "True");
shapeNumbers = regexp(shapeToken{1}, "\d+", "match");
shape = cellfun(@str2double, shapeNumbers);
if isempty(shape)
    shape = [1, 1];
end

[precision, machineFormat, logicalOutput] = dtypeContract(description);
raw = fread(fileId, inf, "*" + precision, 0, machineFormat);
expectedCount = prod(shape);
if numel(raw) ~= expectedCount
    error("gpenmpcReadNpy:Count", ...
        "NPY payload count mismatch for %s: expected %d, read %d", ...
        path, expectedCount, numel(raw));
end

if numel(shape) == 1
    value = reshape(raw, shape(1), 1);
elseif fortranOrder
    value = reshape(raw, shape);
else
    reversed = fliplr(shape);
    value = permute(reshape(raw, reversed), numel(shape):-1:1);
end
if logicalOutput
    value = logical(value);
else
    value = double(value);
end

metadata = struct;
metadata.schema = "GPENMPC_NUMPY_ARRAY_READER_V1";
metadata.path = path;
metadata.version = sprintf("%d.%d", major, minor);
metadata.descr = description;
metadata.fortran_order = fortranOrder;
metadata.shape = shape;
metadata.value_count = numel(value);
end


function [precision, machineFormat, logicalOutput] = dtypeContract(description)
text = char(description);
if numel(text) < 2
    error("gpenmpcReadNpy:Dtype", "Malformed NumPy dtype: %s", description);
end
byteOrder = text(1);
if ismember(byteOrder, ['<', '>', '|', '='])
    payload = text(2:end);
else
    byteOrder = '=';
    payload = text;
end
if byteOrder == '>'
    machineFormat = "ieee-be";
else
    machineFormat = "ieee-le";
end
logicalOutput = false;
switch payload
    case 'f8'
        precision = "double";
    case 'f4'
        precision = "single";
    case 'i8'
        precision = "int64";
    case 'i4'
        precision = "int32";
    case 'i2'
        precision = "int16";
    case 'i1'
        precision = "int8";
    case 'u8'
        precision = "uint64";
    case 'u4'
        precision = "uint32";
    case 'u2'
        precision = "uint16";
    case 'u1'
        precision = "uint8";
    case {'b1', '?'}
        precision = "uint8";
        logicalOutput = true;
    otherwise
        error("gpenmpcReadNpy:Dtype", "Unsupported NumPy dtype: %s", description);
end
end
