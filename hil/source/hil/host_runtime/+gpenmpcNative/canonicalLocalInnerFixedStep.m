function [next64,kernel61,scaffold70,request19]=canonicalLocalInnerFixedStep( ...
        numerics,state64,stateTags2,input36,inputTags2,pending70,pendingTags2)
%#codegen
% Ordinary-source specialization; all prior numerical state is supplied.
[next64,kernel61,scaffold70,request19]=gpenmpcNative.canonicalLocalInnerFixedAbi( ...
    numerics,state64,stateTags2,input36,inputTags2,pending70,pendingTags2,false);
end
