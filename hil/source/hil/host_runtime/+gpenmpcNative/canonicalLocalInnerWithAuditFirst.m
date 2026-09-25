function [next64,kernel61,scaffold70,request19,closed5,learning12]=canonicalLocalInnerWithAuditFirst(numerics,input36,inputTags2)
%#codegen
% Same first kernel, optional read-only CURRENT physical/closed diagnostics.
[next64,kernel61,scaffold70,request19,closed5,learning12]=gpenmpcNative.canonicalLocalInnerFixedAbi( ...
    numerics,zeros(64,1),zeros(2,1,'uint64'),input36,inputTags2,zeros(70,1),zeros(2,1,'uint64'),true);
end
