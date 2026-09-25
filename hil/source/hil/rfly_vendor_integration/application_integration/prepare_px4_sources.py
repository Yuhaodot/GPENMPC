"""Prepare local PX4 inputs for the GPENMPC application build."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
import re
from pathlib import Path
import shutil
import subprocess
from private_git_metadata import prepare as prepare_git

SUBMODULES = ['src/drivers/gps/devices','src/lib/events/libevents',
              'src/modules/mavlink/mavlink','src/lib/heatshrink/heatshrink',
              'src/drivers/actuators/vertiq_io/iq-module-communication-cpp',
              'src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client']
TEXT = {'.c','.cpp','.h','.hpp','.cmake','.msg','.py','.sh','.txt','.xml','.yaml','.json','.px4board','.em'}

def extension_prefix(source):
    """Identify the paired controller and trajectory extensions in PX4."""
    modules = Path(source) / 'src/modules'
    prefixes = []
    for path in modules.glob('*_se3_control/CMakeLists.txt'):
        prefix = path.parent.name.removesuffix('_se3_control')
        if (modules / (prefix + '_trajectory_exec') / 'CMakeLists.txt').is_file():
            prefixes.append(prefix)
    if len(prefixes) != 1:
        raise ValueError('Expected one paired SE(3) controller and trajectory extension')
    prefix = prefixes[0]
    if not re.fullmatch(r'[a-z][a-z0-9]*(?:_[a-z0-9]+)*', prefix):
        raise ValueError('Unsupported PX4 extension prefix: ' + prefix)
    return prefix

def extension_name_map(prefix):
    if prefix == 'gpenmpc':
        return []
    parts = prefix.split('_')
    return [(prefix.upper(), 'GPENMPC'), (prefix.title(), 'GPENMPC'),
            (prefix, 'gpenmpc'), (prefix.replace('_', '-'), 'gpenmpc'),
            (''.join(parts).upper(), 'GPENMPC'),
            (''.join(part.title() for part in parts), 'GPENMPC'),
            (''.join(parts), 'gpenmpc'),
            (' '.join(part.title() for part in parts), 'GPENMPC')]

def renamed(value, mapping):
    for old,new in mapping: value=value.replace(old,new)
    return value

def extension_descriptions(value, relative):
    """Use controller-purpose descriptions in the paired extension modules."""
    if not any('/'+name+'/' in '/'+relative for name in
               ['gpenmpc_se3_control', 'gpenmpc_trajectory_exec']):
        return value
    replacements = [('GPENMPC N0 nominal', 'GPENMPC nominal'),
                    ('GPENMPC R0 fixed-timescale', 'GPENMPC fixed-timescale'),
                    ('GPENMPC HIL experiment', 'GPENMPC HIL application'),
                    ('selects N0/R0', 'selects the nominal or robust controller'),
                    ('GPENMPC N0/R0 owns', 'GPENMPC nominal/robust controller owns')]
    for old, new in replacements:
        value = value.replace(old, new)
    return value

def prepare(source, target, env):
    source,target=Path(source),Path(target)
    if target==source or target.is_relative_to(source) or source.is_relative_to(target):
        raise ValueError('PX4 input and output trees overlap')
    mapping = extension_name_map(extension_prefix(source))
    rows=[]
    def repository(src,dst,migrate):
        index=subprocess.check_output(['git','-C',str(src),'ls-files','--stage','-z'],env=env)
        records=[]; children=[]
        for entry in index.split(b'\0'):
            if not entry: continue
            metadata,relative=entry.decode().split('\t',1)
            mode,blob=metadata.split()[:2]
            if mode=='160000':
                children.append(relative); continue
            records.append((relative,mode,blob))
        # Local PX4 extensions contain message and module source additions.
        if migrate:
            others=subprocess.check_output(['git','-C',str(src),'ls-files','--others',
                                             '--exclude-standard','-z','--','msg','src'],env=env)
            records.extend((p.decode(),'100644',None) for p in others.split(b'\0') if p)
        destinations=[renamed(r[0], mapping) if migrate else r[0] for r in records]
        if len(set(destinations))!=len(destinations): raise ValueError('PX4 renamed path collision')
        def one(record):
            relative,mode,blob=record
            source_file=src/relative; relative_out=renamed(relative, mapping) if migrate else relative
            output=dst/relative_out
            if source_file.is_symlink(): data=os.readlink(source_file).encode()
            elif source_file.is_file(): data=source_file.read_bytes()
            elif blob: data=subprocess.check_output(['git','-C',str(src),'cat-file','blob',blob],env=env)
            else: raise FileNotFoundError(source_file)
            old=hashlib.sha256(data).hexdigest().upper()
            if migrate and (source_file.suffix in TEXT or source_file.name in {'CMakeLists.txt','Kconfig','Makefile','defconfig'}):
                try:
                    text=renamed(data.decode('utf-8'), mapping)
                    text=extension_descriptions(text, relative_out)
                    if relative.startswith('Tools/msg/'):
                        text=re.sub(r"(re\.sub\(r'\(\?<!\^\)\(\?=\[A-Z\]\)', '_', \w+\)\.lower\(\))",
                                    r"\1.replace('g_p_e_n_m_p_c', 'gpenmpc')",text)
                    data=text.encode('utf-8')
                except UnicodeDecodeError: pass
            output.parent.mkdir(parents=True,exist_ok=True)
            if mode=='120000':
                if not output.is_symlink(): output.symlink_to(data.decode())
            elif not output.is_file() or output.read_bytes()!=data:
                output.write_bytes(data); output.chmod(0o755 if mode=='100755' else 0o644)
            return {'source':str(source_file),'relative_path':str(output.relative_to(target)),
                    'before_sha256':old,'sha256':hashlib.sha256(data).hexdigest().upper()}
        with ThreadPoolExecutor(max_workers=16) as pool: rows.extend(pool.map(one,records))
        prepare_git(src,dst,env)
        return children
    repository(source,target,True)
    def dependency(relative):
        src=source/relative
        if not (src/'.git').exists():
            if relative!='src/modules/mavlink/mavlink/pymavlink' or not (src/'generator/mavgen.py').is_file():
                raise FileNotFoundError('Local PX4 submodule: '+str(src))
            for path in src.rglob('*'):
                rel=path.relative_to(src)
                if '__pycache__' in rel.parts or path.suffix=='.pyc': continue
                if path.is_file():
                    output=target/relative/rel; output.parent.mkdir(parents=True,exist_ok=True)
                    shutil.copy2(path,output)
                    value=hashlib.sha256(path.read_bytes()).hexdigest().upper()
                    rows.append({'source':str(path),'relative_path':str(output.relative_to(target)),
                                 'before_sha256':value,'sha256':value})
            return
        for child in repository(src,target/relative,False): dependency(relative+'/'+child)
    for relative in SUBMODULES: dependency(relative)
    (target/'gpenmpc_source_inputs.json').write_text(json.dumps(rows,indent=2)+'\n')
    return target
