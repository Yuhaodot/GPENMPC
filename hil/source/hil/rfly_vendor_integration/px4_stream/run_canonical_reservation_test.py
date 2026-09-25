"""Test registry reservations with pthread interleaving."""
from pathlib import Path
import hashlib,json,subprocess,sys
root=Path(__file__).resolve().parent;label=sys.argv[1];assert (label and all(c.isalnum() or c in '_-' for c in label))
dest=root/('canonical_reservation_host_'+label);dest.mkdir(exist_ok=False)
inputs=[root/n for n in ['test_canonical_reservation.cpp','LinkLifetimeRegistry.cpp','LinkLifetimeRegistry.hpp','NativeOffboardBorrow.hpp','RegisteredHostHeartbeat.hpp','LinkCanonicalReservation.cpp']]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
args=['g++','-std=c++14','-O2','-Wall','-Wextra','-Werror','-pthread',str(inputs[0]),str(inputs[1]),str(inputs[5]),'-o',str(dest/'test')]
c=subprocess.run(args,capture_output=True,text=True);r=None
if c.returncode==0:r=subprocess.run([str(dest/'test')],capture_output=True,text=True,timeout=30)
report=dict(argv=args,compile_exit=c.returncode,compile_output=c.stdout+c.stderr,run_exit=r.returncode if r else None,
 stdout=r.stdout if r else '',stderr=r.stderr if r else '',source_hashes=before,
 sources_unchanged=before=={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs})
report['passed']=report['compile_exit']==0 and report['run_exit']==0 and report['sources_unchanged']
(dest/'RESULT.json').write_text(json.dumps(report,indent=2)+'\n')
print(c.stdout+c.stderr);print(report['stdout']+report['stderr']);sys.exit(0 if report['passed'] else 1)
