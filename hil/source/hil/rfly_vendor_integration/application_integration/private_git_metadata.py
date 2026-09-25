"""Create independent Git metadata backed by read-only local input objects."""
from pathlib import Path
import subprocess

def prepare(source, target, env):
    source,target=Path(source).resolve(),Path(target).resolve()
    if source==target or target.is_relative_to(source) or source.is_relative_to(target):
        raise ValueError('Private Git tree must be separate from its input')
    def read(*args):
        return subprocess.check_output(['git','-C',str(source),*args],env=env,text=True).strip()
    def run(*args): subprocess.run(['git','-C',str(target),*args],env=env,check=True,capture_output=True)
    head=read('rev-parse','HEAD')
    objects=read('rev-parse','--path-format=absolute','--git-path','objects')
    metadata=target/'.git'
    if metadata.is_file(): metadata.unlink()
    target.mkdir(parents=True,exist_ok=True)
    run('init','--quiet')
    (metadata/'objects/info/alternates').write_text(objects+'\n')
    shallow=Path(read('rev-parse','--path-format=absolute','--git-path','shallow'))
    if shallow.is_file(): (metadata/'shallow').write_bytes(shallow.read_bytes())
    run('config','core.autocrlf','false')
    run('config','core.filemode','false')
    run('symbolic-ref','HEAD','refs/heads/gpenmpc-build')
    run('update-ref','refs/heads/gpenmpc-build',head)
    for line in read('for-each-ref','--format=%(objectname) %(refname)','refs/tags').splitlines():
        sha,ref=line.split(' ',1); run('update-ref',ref,sha)
    run('read-tree',head)
    return head
