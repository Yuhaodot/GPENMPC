"""Compare state hashes with SJC output and RWI reference fixture bytes."""
from pathlib import Path
import hashlib,json,os,struct,sys
root=Path(__file__).resolve().parent;out=root/sys.argv[1];result=out/'HASH_VERIFY.json';assert not result.exists()
base=Path(os.environ['GPENMPC_REFERENCE_FIXTURES'])
class Reader:
 def __init__(self,p):self.data=Path(p).read_bytes();self.at=0
 def take(self,n):
  b=self.data[self.at:self.at+n];assert len(b)==n;self.at+=n;return b
 def u32(self):return int.from_bytes(self.take(4),'little')
 def u64(self):return int.from_bytes(self.take(8),'little')
 def ds(self,n):return [self.take(8) for _ in range(n)]
def u32(x):return x.to_bytes(4,'big')
def u64(x):return x.to_bytes(8,'big')
def ds(values):return b''.join(v[::-1] for v in values)
def domain(s):return s.encode()+b'\0'
def sha(b):return hashlib.sha256(b).digest()
z=struct.pack('<d',0.0)
r=Reader(base/'CONTINUOUS_REFERENCE_FIXTURE.bin');assert r.take(4)==b'RWI1';nw,nr=r.u32(),r.u32();assert nr==3765
windows=[]
for _ in range(nw):
 m=[r.u32() for _ in range(7)];asset=r.take(32);gen=r.u64();s=r.ds(5)
 windows.append(dict(m=m,asset=asset,gen=gen,s=s,time=r.ds(256),jet=r.ds(3072),prefix=r.ds(192),ground=r.ds(12),rest=r.ds(12),offset=r.ds(3)))
rows=[]
for _ in range(60):rows.append(dict(ids=[r.u32() for _ in range(3)],seq=r.u64(),phase=r.take(8),args=r.ds(11),jet=r.ds(12),expected=r.ds(41)))
r=Reader(Path(os.environ['GPENMPC_GP_CHAIN_FIXTURE']));assert r.take(4)==b'SJC1' and r.u32()==60;topic=r.u32();original=[]
for _ in range(60):
 raw=r.take(topic);tags=[r.u64(),r.u64()];x=r.ds(36);transition=r.ds(41);state=r.ds(64);kernel=r.ds(61);controls=r.take(64)
 request=r.ds(19);gp=r.ds(18);r.ds(64);r.ds(61);r.take(64);r.ds(18);pending=r.ds(70);r.ds(70)
 original.append(dict(tags=tags,x=x,transition=transition,state=state,request=request,gp=gp,pending=pending))
assert r.at==len(r.data)
w=windows[rows[0]['ids'][0]-1];m=w['m'];initial=rows[0]['args'];cfg=bytes(range(33,65));task=bytes(range(1,33))
window_data=domain('GPENMPC_FULL_INNER_REFERENCE_WINDOW_V1')+u32(m[0])+u32(m[1])+w['asset']+u32(m[2])+u64(w['gen'])+u32(m[3])+u32(m[4])+u32(m[5])
window_data+=ds(w['time'])+ds(w['jet'])+ds(w['s'][0:2])+u32(m[6])+ds(w['s'][2:3])+ds(w['prefix'])+ds(w['ground'])+ds(w['rest'])+ds(w['s'][3:4])+ds(w['offset'])+ds(w['s'][4:5])
window_sha=sha(window_data)
identity=json.loads((out/'RESULT.json').read_text())['identity'];observed=(out/'HASHES.bin').read_bytes();assert len(observed)==60*64
expected=[];mismatches=[];metamorphic=[]
for i,(row,o) in enumerate(zip(rows,original)):
 if i:
  p=original[i-1];pr=rows[i-1];pw=windows[pr['ids'][0]-1]
  numeric=b'\1'+ds(p['state'])+ds(p['pending'])+b''.join(u64(v) for v in p['tags'])
  required=struct.unpack('<d',p['request'][0])[0]!=0
  numeric+=bytes([int(required),1,1])
  ref=pw['asset']+u32(pw['m'][2])+u64(pw['gen'])+u64(pr['seq'])+ds([pr['phase'],pr['args'][0],p['transition'][12]])+ds(p['transition'][14:17])
  ref+=b''.join(u64(v) for v in [*p['tags'],1,1,p['tags'][1],p['tags'][0]//1000+400])
 else:
  numeric=b'\0\0\0\0'
  ref=w['asset']+u32(m[2])+u64(0)+u64(0)+ds([z,z,initial[1]])+ds(initial[3:6])+u64(0)*6
 prior_data=domain('GPENMPC_FULL_INNER_COMMITTED_STATE_V1')+numeric+ref;prior_sha=sha(prior_data)
 rw=windows[row['ids'][0]-1]
 full_data=domain('GPENMPC_FULL_INNER_INPUT36_STATE64_PENDING70_REFERENCE_V1')+bytes.fromhex(identity['generated_source_set_sha256'])+bytes.fromhex(identity['facade_source_set_sha256'])
 full_data+=task+cfg+w['asset']+u32(m[2])+ds(initial[1:2])+ds(initial[3:6])+ds(initial[10:11])+window_sha+prior_sha+ds(o['x'])+b''.join(u64(v) for v in o['tags'])
 full_data+=rw['asset']+u32(rw['m'][2])+u64(rw['gen'])+u64(row['seq'])+b''.join(u64(v) for v in [*o['tags'],1,1])
 full_data+=ds([row['phase'],row['args'][0],row['args'][2],*row['args'][6:9],row['args'][9]])
 full_sha=sha(full_data);expected.append(prior_sha+full_sha)
 if observed[64*i:64*i+64]!=expected[-1]:mismatches.append(i)
 if i==2:
  # Flip one bit in each state component and compare its digest.
  for name,offset in [('state64',len(domain('GPENMPC_FULL_INNER_COMMITTED_STATE_V1'))+1),
                      ('pending70',len(domain('GPENMPC_FULL_INNER_COMMITTED_STATE_V1'))+1+512),('reference',len(prior_data)-1)]:
   changed=bytearray(prior_data);changed[offset]^=1;metamorphic.append(dict(field=name,different=sha(changed)!=prior_sha))
report=dict(scope='INDEPENDENT_BYTE_SHA_ORACLE_FROM_RETAINED_RAW_INPUTS_AND_PRIOR_C_OUTPUTS',rows=60,
 prior_and_full_digests_checked=120,exact_rows=60-len(mismatches),mismatches=mismatches,metamorphic=metamorphic,
 full_input_domain='GPENMPC_FULL_INNER_INPUT36_STATE64_PENDING70_REFERENCE_V1',reference_window_sha256=window_sha.hex().upper(),
 C_padding_hashed=False,float_bits_modified=False)
result.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));sys.exit(bool(mismatches) or not all(m['different'] for m in metamorphic))
