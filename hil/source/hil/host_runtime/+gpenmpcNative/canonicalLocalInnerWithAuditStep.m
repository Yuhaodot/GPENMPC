function [next64,kernel61,scaffold70,request19,closed5,learning12]=canonicalLocalInnerWithAuditStep( ...
    numerics,state64,stateTags2,input36,inputTags2,pending70,pendingTags2)
%#codegen
% Expose audit data from the step kernel.
[next64,kernel61,scaffold70,request19,closed5,learning12]=gpenmpcNative.canonicalLocalInnerFixedAbi( ...
    numerics,state64,stateTags2,input36,inputTags2,pending70,pendingTags2,false);
end
