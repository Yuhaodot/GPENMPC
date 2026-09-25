function [next64,kernel61,scaffold70,request19,closed5]=canonicalLocalInnerWithEvidenceFirst(numerics,input36,inputTags2)
%#codegen
% Expose first-entry numerical evidence with the candidate result.
[next64,kernel61,scaffold70,request19,closed5]=gpenmpcNative.canonicalLocalInnerFixedAbi( ...
    numerics,zeros(64,1),zeros(2,1,'uint64'),input36,inputTags2, ...
    zeros(70,1),zeros(2,1,'uint64'),true);
end
