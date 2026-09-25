function report=test_original_getter_vectorization(outputRoot)
% Compare vectorized matching with the scalar implementation.
build=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(build,'host_runtime'));
out=fullfile(outputRoot,'GETTER_VECTORIZATION.json');assert(~isfile(out));
raw=load(fullfile(gpenmpc_external_path('matlab_noui_original_reader'),'MATLAB_ORIGINAL_GETTERS.mat'),'records');
r=raw.records;cases={};names={};
add('actual_retained_3544',r,converted(r(:,11:17)));
add('one',r(:,11),converted(r(:,11)));
add('duplicate_getter',r(:,[11 11 12]),converted(r(:,[11 12])));
add('duplicate_source',r(:,11:14),converted(r(:,[11 11 13])));
s=converted(r(:,11:14));s(1,1)=bitxor(s(1,1),uint8(1));add('unmatched_bit',r(:,11:14),s);
x=r(:,11:18);x(321:324,1)=bytes(int32(13));x(321:324,2)=bytes(int32(31));
x(33:40,3)=bytes(NaN);x(33:40,4)=bytes(Inf);x(33:40,5)=bytes(realmax);
x(33:40,6)=bytes(-0.0);x(33:40,7)=bytes(realmin);add('eligibility_and_rounding',x,converted(r(:,11:18)));
x=r(:,11:18);x(321:324,:)=repmat(bytes(int32(0)),1,8);add('no_eligible',x,zeros(52,2,'uint8'));
x=repmat(r,1,3);x=x(:,1:8192);add('capacity8192',x,converted(r(:,11:25)));
checks=struct('name',{},'pass',{});times=zeros(numel(cases),2);
for k=1:numel(cases)
    c=cases{k};a=scalarOriginal(c{1},c{2});b=gpenmpcNative.matchRflyOriginalGetterSensorBytes(c{1},c{2});
    ok=isequaln(a,b);assert(ok,'test:MatchingChanged','%s',names{k});
    checks(end+1)=struct('name',names{k},'pass',ok); %#ok<AGROW>
    t=tic;for j=1:3,scalarOriginal(c{1},c{2});end;times(k,1)=toc(t)/3;
    t=tic;for j=1:3,gpenmpcNative.matchRflyOriginalGetterSensorBytes(c{1},c{2});end;times(k,2)=toc(t)/3;
end
report=struct('passed',all([checks.pass]),'test_count',numel(checks),'checks',checks, ...
    'mean_seconds_scalar_then_vector',times,'scope','EXACT_FULL_STRUCT_SCALAR_ORACLE_AND_CHANGED_VECTOR_MATCHER_ONLY', ...
    'board_actions',0,'COM',0,'plant_runs',0);
f=fopen(out,'w');assert(f>=0);guard=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));
disp(jsonencode(report));
    function add(n,rr,ss),names{end+1}=n;cases{end+1}={rr,ss};end
end
function s=converted(r)
d=reshape(readLittle(reshape(r(33:136,:),[],1),'double'),13,[]);v=single(d);s=reshape(typecast(v(:),'uint8'),52,[]);
end
function b=bytes(v),b=reshape(typecast(v,'uint8'),[],1);end
function m=scalarOriginal(records,sensor52)
% Scalar sensor matching oracle for the vectorized implementation.
ng=size(records,2);nw=size(sensor52,2);converted=zeros(52,ng,'uint8');eligible=false(ng,1);
for k=1:ng
    length=readLittle(records(321:324,k),'int32');d=readLittle(records(33:136,k),'double');
    f=single(d);eligible(k)=length>=14&&length<=30&&all(isfinite(d))&&all(isfinite(f));
    if eligible(k),converted(:,k)=reshape(typecast(f,'uint8'),[],1);end
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
function v=readLittle(b,t),v=typecast(b(:),t);[~,~,e]=computer;if e=='B',v=swapbytes(v);end;end
