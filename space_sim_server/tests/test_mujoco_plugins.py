import hashlib,json
import pytest
from simulation.mujoco_plugins import validate_plugin_manifest

def test_missing_plugin_rejected(tmp_path):
 with pytest.raises(FileNotFoundError): validate_plugin_manifest(tmp_path/'missing.dll',tmp_path/'manifest.json',3007000)

def test_plugin_version_and_hash_checked(tmp_path):
 dll=tmp_path/'thread.dll'; dll.write_bytes(b'test binary')
 manifest=tmp_path/'manifest.json'
 payload={'runtime_version':3007000,'plugin_sha256':hashlib.sha256(dll.read_bytes()).hexdigest()}
 manifest.write_text(json.dumps(payload))
 assert validate_plugin_manifest(dll,manifest,3007000)==payload
 with pytest.raises(ValueError,match='version'):validate_plugin_manifest(dll,manifest,3008000)
 dll.write_bytes(b'changed')
 with pytest.raises(ValueError,match='hash'):validate_plugin_manifest(dll,manifest,3007000)

def test_platform_thread_source_changes_only_tooth_radius():
 # The platform plugin must stay the delivered source except for the faster
 # tooth_radius(); tools/assets/build_sarm_plugin.py proves that function numerically.
 import re
 from pathlib import Path
 from simulation.mujoco_plugins import DELIVERED_SOURCE, PLATFORM_SOURCE
 root=Path(__file__).resolve().parents[1]
 def body(name):
  text=(root/name).read_text(encoding='utf-8').replace('\r\n','\n')
  text=text[text.index('// Two-start SDF thread pair'):]
  return re.sub(r'double tooth_radius\(.*?\n}\n','',text,count=1,flags=re.S)
 assert body(PLATFORM_SOURCE)==body(DELIVERED_SOURCE)
 assert 'std::atan2(' not in re.search(r'double tooth_radius\(.*?\n}\n',(root/PLATFORM_SOURCE).read_text(encoding='utf-8'),re.S).group(0)
