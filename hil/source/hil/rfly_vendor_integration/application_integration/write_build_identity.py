"""Write the controller identity from its source files and compiled archive."""
import argparse
import hashlib
import json
from pathlib import Path

def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest().upper()
def main():
    p=argparse.ArgumentParser(); p.add_argument('--hil-source',type=Path,required=True)
    p.add_argument('--archive',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(); hil=args.hil_source.resolve(); rfly=hil/'rfly_vendor_integration'
    generated=hil/'evidence/tracking_controller/generated'
    groups={'generated':sorted([*generated.glob('*.c'),*generated.glob('*.h')]),
      'facade':[rfly/'full_inner_abi'/s for s in ['CanonicalFullInnerAbi.cpp','CanonicalFullInnerAbi.h']] +
        [rfly/s for s in ['CanonicalCombinedSymbolNamespace.h','CanonicalLocalInnerStateStore.hpp',
                         'CanonicalReferenceStateStore.hpp','CanonicalJointStateInstaller.hpp',
                         'CanonicalOperatorReference.hpp']] +
        [hil/'px4_full_inner'/s for s in ['consumption/CanonicalSha256.hpp','portable/CanonicalPortable.hpp']]}
    observer=hil/'live/vertical_observer/gpenmpcObserveCausalVerticalDisturbance.c'
    groups['generated']=[observer if x.name==observer.name else x for x in groups['generated']]
    manifests={}; identity={}
    for group,files in groups.items():
        rows=sorted(({'path':'source/hil/'+x.relative_to(hil).as_posix(),'sha256':digest(x)} for x in files),key=lambda x:x['path'])
        manifests[group]=rows
        encoded=''.join(x['path']+'\0'+x['sha256']+'\n' for x in rows).encode()
        identity[group+'_source_set_sha256']=hashlib.sha256(encoded).hexdigest().upper()
    identity['private_archive_sha256']=digest(args.archive)
    identity['operator_reference_sha256']=digest(rfly/'CanonicalOperatorReference.hpp')
    args.output.mkdir(parents=True,exist_ok=True)
    header='#pragma once\n'
    for symbol,key in [('rfi_generated_source_sha','generated_source_set_sha256'),
                       ('rfi_facade_source_sha','facade_source_set_sha256'),
                       ('rfi_private_archive_sha','private_archive_sha256')]:
        header+='static const unsigned char '+symbol+'[32]={'+','.join('0x'+keybyte for keybyte in
            [identity[key][i:i+2] for i in range(0,64,2)])+'};\n'
    (args.output/'FullInnerBuildIdentity.h').write_text(header)
    (args.output/'source_identity.json').write_text(json.dumps({'identity':identity,'source_files':manifests,
        'set_hash_format':'UTF-8 sorted relative path + NUL + uppercase SHA256 + LF'},indent=2)+'\n')

if __name__=='__main__': main()
