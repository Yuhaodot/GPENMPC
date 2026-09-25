function [next64,kernel61,scaffold70,request19]=canonicalLocalInnerFixedFirst(numerics,input36,inputTags2)
%#codegen
% Initialize a fresh owner's first leg without prior state or pending prediction.
[next64,kernel61,scaffold70,request19]=gpenmpcNative.canonicalLocalInnerFixedAbi( ...
    numerics,zeros(64,1),zeros(2,1,'uint64'),input36,inputTags2, ...
    zeros(70,1),zeros(2,1,'uint64'),true);
end
