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
