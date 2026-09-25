function yes=sameSoleMavlinkOwner(record)
% Check that IO records share the same MAVLink and transport owner.
yes=isstruct(record)&&isscalar(record)&&isfield(record,'same_existing_mavlinkio') ...
    &&islogical(record.same_existing_mavlinkio)&&isscalar(record.same_existing_mavlinkio);
if ~yes,return;end
if ~isfield(record,'transport_kind')
    yes=record.same_existing_mavlinkio;return
end
yes=isfield(record,'same_existing_transport_owner') ...
    &&isequal(record.same_existing_transport_owner,true);
if ~yes,return;end
switch string(record.transport_kind)
    case "ORIGINAL_MAVLINKIO_UDP"
        yes=record.same_existing_mavlinkio;
    case "OFFICIAL_CODEC_SOLE_RAW_UDP"
        yes=~record.same_existing_mavlinkio;
    otherwise
        yes=false;
end
end
