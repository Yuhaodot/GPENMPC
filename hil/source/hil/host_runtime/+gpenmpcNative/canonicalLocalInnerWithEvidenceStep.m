function [next64,kernel61,scaffold70,request19,closed5]=canonicalLocalInnerWithEvidenceStep( ...
        numerics,state64,stateTags2,input36,inputTags2,pending70,pendingTags2)
%#codegen
% Copy evidence produced by this step.
[next64,kernel61,scaffold70,request19,closed5]=gpenmpcNative.canonicalLocalInnerFixedAbi( ...
    numerics,state64,stateTags2,input36,inputTags2,pending70,pendingTags2,false);
end
