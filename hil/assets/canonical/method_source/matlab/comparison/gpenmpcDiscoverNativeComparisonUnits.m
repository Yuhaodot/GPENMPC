function units = gpenmpcDiscoverNativeComparisonUnits(multiCityRoot, ordinaryB2Root)
%GPENMPCDISCOVERNATIVECOMPARISONUNITS Bind the twelve shared task-route units.
%
% Load the fixed-reference R0 trace and compare mission, visit order, route
% candidates and initial remaining-plan identities with the eNMPC ledger.

arguments
    multiCityRoot (1,1) string
    ordinaryB2Root (1,1) string
end

oldCases = multiCityRoot;
newCases = ordinaryB2Root;
if ~isfolder(oldCases) || ~isfolder(newCases)
    error("gpenmpcDiscoverNativeComparisonUnits:MissingRoot", ...
        "The two read-only parent case roots must both exist.");
end

missions = ["MU_CAMBRIDGE_MA_01", "MU_CAMBRIDGE_MA_02", ...
    "MU_MANHATTAN_NY_01", "MU_MANHATTAN_NY_02", ...
    "MU_SEATTLE_WA_01", "MU_SEATTLE_WA_02"];
planners = ["P_DIST_FIXED", "P_ENERGY_WIND_PAYLOAD"];
template = struct( ...
    mission_id="", planner_id="", city="", ...
    r0_case_root="", r0_ledger_path="", r0_result_path="", ...
    r0_trace_path="", ordinary_b1_case_root="", ...
    ordinary_b1_ledger_path="", ordinary_b1_result_path="", ...
    mission_payload_sha256="", plan_payload_sha256="", ...
    reference_trace_sha256="", ...
    route_candidate_ids=strings(0,1), visit_order=zeros(0,1), ...
    route_identity_match=false, visit_order_match=false, ...
    mission_identity_match=false, plan_identity_match=false);
units = repmat(template, numel(missions) * numel(planners), 1);

cursor = 0;
for mission = missions
    for planner = planners
        cursor = cursor + 1;
        oldCase = fullfile(oldCases, mission, planner, "R0_ROBUST_SE3");
        newCase = fullfile(newCases, mission, planner, "OUTER_030", ...
            "ENMPC_ROBUST_NO_GP");
        oldLedgerPath = uniqueFile(oldCase, "METHOD_CASE_LEDGER_*.json");
        oldResultPath = uniqueFile(oldCase, "METHOD_CASE_RESULT_*.json");
        oldTracePath = uniqueFile(oldCase, "METHOD_TRACE_*.npz");
        newLedgerPath = uniqueFile(newCase, "METHOD_CASE_LEDGER_*.json");
        newResultPath = uniqueFile(newCase, "METHOD_CASE_RESULT_*.json");

        oldLedger = jsondecode(fileread(oldLedgerPath));
        newLedger = jsondecode(fileread(newLedgerPath));
        oldResult = jsondecode(fileread(oldResultPath));
        newResult = jsondecode(fileread(newResultPath));
        assertText(oldLedger.method, "R0_ROBUST_SE3", "R0 method");
        assertText(newLedger.runtime_method_id, ...
            "B1_ENMPC_TOTAL_ROBUST_TUBE_NO_GP", "ordinary-B1 method");
        if ~logical(oldResult.task_complete_under_final_definition) || ...
                ~logical(newResult.task_complete_under_final_definition)
            error("gpenmpcDiscoverNativeComparisonUnits:IncompleteParent", ...
                "Parent task-route unit %s/%s is not complete.", mission, planner);
        end

        missionMatch = string(oldLedger.mission_config.mission_payload_sha256) ...
            == string(newLedger.mission_config.mission_payload_sha256);
        planMatch = string(oldLedger.plans(1).plan_payload_sha256) ...
            == string(newLedger.plans(1).plan_payload_sha256);
        routeMatch = isequal(string(oldResult.actual_route_candidate_ids(:)), ...
            string(newResult.actual_route_candidate_ids(:)));
        visitMatch = isequal(double(oldResult.actual_visit_order(:)), ...
            double(newResult.actual_visit_order(:)));
        if ~(missionMatch && planMatch && routeMatch && visitMatch)
            error("gpenmpcDiscoverNativeComparisonUnits:IdentityDrift", ...
                "Task-route identity drift at %s/%s.", mission, planner);
        end

        units(cursor).mission_id = mission;
        units(cursor).planner_id = planner;
        units(cursor).city = cityFromMission(mission);
        units(cursor).r0_case_root = oldCase;
        units(cursor).r0_ledger_path = oldLedgerPath;
        units(cursor).r0_result_path = oldResultPath;
        units(cursor).r0_trace_path = oldTracePath;
        units(cursor).ordinary_b1_case_root = newCase;
        units(cursor).ordinary_b1_ledger_path = newLedgerPath;
        units(cursor).ordinary_b1_result_path = newResultPath;
        units(cursor).mission_payload_sha256 = ...
            string(oldLedger.mission_config.mission_payload_sha256);
        units(cursor).plan_payload_sha256 = ...
            string(oldLedger.plans(1).plan_payload_sha256);
        units(cursor).reference_trace_sha256 = gpenmpcSha256File(oldTracePath);
        units(cursor).route_candidate_ids = ...
            string(oldResult.actual_route_candidate_ids(:));
        units(cursor).visit_order = double(oldResult.actual_visit_order(:));
        units(cursor).route_identity_match = routeMatch;
        units(cursor).visit_order_match = visitMatch;
        units(cursor).mission_identity_match = missionMatch;
        units(cursor).plan_identity_match = planMatch;
    end
end
end


function path = uniqueFile(root, pattern)
listing = dir(fullfile(root, pattern));
if numel(listing) ~= 1
    error("gpenmpcDiscoverNativeComparisonUnits:FileCount", ...
        "Expected one %s below %s, found %d.", pattern, root, numel(listing));
end
path = string(fullfile(listing(1).folder, listing(1).name));
end


function assertText(actual, expected, label)
if string(actual) ~= string(expected)
    error("gpenmpcDiscoverNativeComparisonUnits:Identity", ...
        "%s differs from the required identity.", label);
end
end


function city = cityFromMission(mission)
if contains(mission, "CAMBRIDGE")
    city = "Cambridge";
elseif contains(mission, "MANHATTAN")
    city = "Manhattan";
elseif contains(mission, "SEATTLE")
    city = "Seattle";
else
    error("gpenmpcDiscoverNativeComparisonUnits:City", ...
        "Unknown mission family %s.", mission);
end
end
