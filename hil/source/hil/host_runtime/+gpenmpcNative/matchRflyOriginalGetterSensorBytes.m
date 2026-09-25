function m=matchRflyOriginalGetterSensorBytes(records,sensor52)
% Match getter double[1..13] to little-endian float13 bytes.
% Count matches across the supplied finite window.
assert(isa(records,'uint8')&&size(records,1)==336&&size(records,2)>0&&size(records,2)<=8192);
assert(isa(sensor52,'uint8')&&size(sensor52,1)==52&&size(sensor52,2)>0&&size(sensor52,2)<=8192);
ng=size(records,2);nw=size(sensor52,2);converted=zeros(52,ng,'uint8');
% Convert the supplied record window once in column order while preserving
% byte order, rounding, eligibility and whole-window uniqueness.
lengths=reshape(littleValue(reshape(records(321:324,:),[],1),'int32'),1,ng);
d=reshape(littleValue(reshape(records(33:136,:),[],1),'double'),13,ng);f=single(d);
eligible=(lengths>=14&lengths<=30&all(isfinite(d),1)&all(isfinite(f),1)).';
if any(eligible)
    valid=f(:,eligible);converted(:,eligible)=reshape(typecast(valid(:),'uint8'),52,[]);
end
[~,~,group]=unique([converted(:,eligible),sensor52].','rows');
gids=group(1:sum(eligible));wids=group(sum(eligible)+1:end);
gc=accumarray(gids,1,[max(group),1]);wc=accumarray(wids,1,[max(group),1]);
wireCounts=gc(wids);getterCounts=zeros(ng,1);getterCounts(eligible)=wc(gids);
originalIndices=find(eligible);first=zeros(max(group),1);
for k=1:numel(gids),first(gids(k))=originalIndices(k);end
matched=zeros(nw,1);uniqueBoth=wireCounts==1&wc(wids)==1;matched(uniqueBoth)=first(wids(uniqueBoth));
m=struct('schema','RFLY_EXACT_SENSOR52_WINDOW_MATCH_V1','getter_index',matched, ...
    'wire_match_counts',wireCounts,'getter_match_counts',getterCounts,'eligible_getters',eligible, ...
    'converted_sensor52',converted,'mutually_unique_count',sum(uniqueBoth), ...
    'window_getters',ng,'window_sources',nw,'time_or_ordinal_used',false, ...
    'uniqueness_scope','ENTIRE_SUPPLIED_WINDOW_ONLY','unseen_history_or_future_unique',false);
end
function v=littleValue(b,kind)
v=typecast(b(:),kind);[~,~,e]=computer;if e=='B',v=swapbytes(v);end
end
