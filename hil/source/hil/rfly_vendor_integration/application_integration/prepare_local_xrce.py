#!/usr/bin/env python3
"""Prepare local XRCE/CDR sources for the application build."""
from pathlib import Path
import json
import subprocess
import sys
sys.dont_write_bytecode = True
import configure_private_nuttx as isolation

root = Path(__file__).resolve().parent
client_source = isolation.PX4 / 'src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client'
cdr_source = isolation.OLD_BUILD / ('src/modules/uxrce_dds_client/src/'
    'libmicroxrceddsclient_project-build/microcdr/src/microcdr')
private = root / 'private_dependencies'
client = private / 'Micro-XRCE-DDS-Client'
cdr = private / 'Micro-CDR'
private.mkdir(exist_ok=True)
assert 'set(_microcdr_tag v2.0.1)' in (client_source / 'CMakeLists.txt').read_text()
for source, destination in ((client_source, client), (cdr_source, cdr)):
    assert (source / 'CMakeLists.txt').is_file()
    if not destination.exists():
        subprocess.run(['cp', '-a', str(source), str(destination)], check=True)
    destination.resolve().relative_to(private.resolve())
    assert isolation.sha(source / 'CMakeLists.txt') == isolation.sha(destination / 'CMakeLists.txt')

parent = client_source / 'cmake/SuperBuild.cmake'
text = parent.read_text()
old = ('        GIT_REPOSITORY\n'
       '            https://github.com/eProsima/Micro-CDR.git\n'
       '        GIT_TAG\n'
       '            ${_microcdr_tag}\n')
assert text.count(old) == 1
new = ('        SOURCE_DIR\n            "' + str(cdr) + '"\n'
       '        DOWNLOAD_COMMAND ""\n'
       '        UPDATE_COMMAND ""\n')
derived = client / 'cmake/SuperBuild.cmake'
derived.write_text(text.replace(old, new))
receipt = {
    'scope': 'LOCAL_SOURCE_DEPENDENCY_ROUTING_ONLY',
    'xrce_source': str(client_source), 'cdr_source': str(cdr_source),
    'xrce_private': str(client), 'cdr_private': str(cdr),
    'source_xrce_cmake_sha256': isolation.sha(client_source / 'CMakeLists.txt'),
    'source_cdr_cmake_sha256': isolation.sha(cdr_source / 'CMakeLists.txt'),
    'superbuild_original_sha256': isolation.sha(parent),
    'superbuild_derived_sha256': isolation.sha(derived),
    'exact_changes': {'git_download_to_existing_local_source': 1},
    'microcdr_required_version': '2.0.1',
    'prebuilt_binary_reused': False, 'download_performed': False,
    'original_source_written': False, 'configure_executed': False}
(root / 'LOCAL_DEPENDENCY_ROUTING.json').write_text(json.dumps(receipt, indent=2) + '\n')
print(json.dumps(receipt))
