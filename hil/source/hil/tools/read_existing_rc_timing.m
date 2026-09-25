function read_existing_rc_timing(path)
% Read a bounded tail of an existing v7.3 recording, without scanning its tree.
f=H5F.open(path,'H5F_ACC_RDONLY','H5P_DEFAULT');c=onCleanup(@()H5F.close(f)); %#ok<NASGU>
d=H5D.open(f,'methodRaw');dc=onCleanup(@()H5D.close(d)); %#ok<NASGU>
s=H5D.get_space(d);sc=onCleanup(@()H5S.close(s)); %#ok<NASGU>
[~,dims]=H5S.get_simple_extent_dims(s);n=min(4096,dims(1));
H5S.select_hyperslab(s,'H5S_SELECT_SET',[dims(1)-n 0],[],[n 1],[]);
ms=H5S.create_simple(2,[n 1],[]);mc=onCleanup(@()H5S.close(ms)); %#ok<NASGU>
r=reshape(H5D.read(d,'H5ML_DEFAULT',ms,s,'H5P_DEFAULT'),8,[]);
found=0;
for k=size(r,2):-1:1
 g=H5R.dereference(d,'H5R_OBJECT',r(:,k));gc=onCleanup(@()H5G.close(g));
 if ~H5L.exists(g,'now_ns','H5P_DEFAULT')
  clear gc;continue
 end
 out=struct('index',dims(1)-n+k);
 for field={'now_ns','status','work_timing_ns'}
  out.(field{1})=fieldValue(g,field{1});
 end
 src=fieldValue(g,'source');
 if isstruct(src)&&isfield(src,'decoded')
  out.source_sample_us=src.decoded.original_sample_us;
  out.source_generation=src.decoded.source_generation;
  out.source_receive_ns=src.original_host_receive_ns;
  if isfield(src.decoded,'sample_delta_us'),out.source_sample_delta_us=src.decoded.sample_delta_us;end
 end
 if H5L.exists(g,'input_send','H5P_DEFAULT')
  sent=fieldValue(g,'input_send');
  if isstruct(sent)&&isfield(sent,'submitted_ns')
   out.first_send_ns=sent.submitted_ns(1);out.last_send_ns=sent.returned_ns(end);
  end
 end
 fprintf('%s\n',jsonencode(out));clear gc
 found=found+1;if found==12,break,end
end
end
function v=fieldValue(g,name)
d=H5O.open(g,name,'H5P_DEFAULT');c=onCleanup(@()H5O.close(d)); %#ok<NASGU>
v=readObject(d);
end
function v=readObject(o)
t=H5I.get_type(o);
if t==H5ML.get_constant_value('H5I_GROUP')
 info=H5G.get_info(o);v=struct();
 for j=0:info.nlinks-1
  name=H5L.get_name_by_idx(o,'.','H5_INDEX_NAME','H5_ITER_INC',j,'H5P_DEFAULT');
  child=H5O.open(o,name,'H5P_DEFAULT');v.(name)=readObject(child);H5O.close(child);
 end
else
 v=H5D.read(o);dt=H5D.get_type(o);kind=H5T.get_class(dt);H5T.close(dt);
 if kind==H5ML.get_constant_value('H5T_REFERENCE')
  r=reshape(v,8,[]);items=cell(1,size(r,2));
  for j=1:size(r,2)
   child=H5R.dereference(o,'H5R_OBJECT',r(:,j));items{j}=readObject(child);H5O.close(child);
  end
  if isscalar(items),v=items{1};else,v=items;end
 else
  a=H5A.open(o,'MATLAB_class');cls=H5A.read(a);H5A.close(a);
  if strcmp(char(cls(:).'),'char'),v=char(v(:).');end
 end
end
end
