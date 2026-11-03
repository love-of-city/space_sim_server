"""Preserve an existing master material while enabling primitive/Nanite use."""
import json
import os
from pathlib import Path

import unreal

asset_path='/Game/BSK/M_BskPbrOpaque'
material=unreal.load_asset(asset_path)
if not material:
    raise RuntimeError('Missing runtime PBR material: '+asset_path)
changed=not material.get_editor_property('used_with_nanite')
if changed:
    material.set_editor_property('used_with_nanite',True)
    unreal.MaterialEditingLibrary.recompile_material(material)
    if not unreal.EditorAssetLibrary.save_loaded_asset(material,only_if_is_dirty=False):
        raise RuntimeError('Could not save PBR geometry usage')
if not material.get_editor_property('used_with_nanite'):
    raise RuntimeError('PBR Nanite usage did not persist')
report=dict(asset=asset_path,used_with_nanite=True,changed=changed)
output=os.environ.get('BSK_PBR_USAGE_RESULT')
if output:
    Path(output).write_text(json.dumps(report,indent=2),encoding='utf-8')
unreal.log('BSK PBR geometry usage: '+json.dumps(report))
