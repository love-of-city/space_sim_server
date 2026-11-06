"""MuJoCo orders all parent geometry before child bodies, regardless of XML order."""
from bsk_render_adapter.mjcf_assets import parse_mjcf_geometry_metadata


def test_parent_geom_after_child_and_nested_frames(tmp_path):
    path=tmp_path/'scene.xml'
    path.write_text('''<mujoco><worldbody><body name="parent">
      <geom type="sphere" size=".01" pos="1 0 0"/>
      <body name="first"><geom type="sphere" size=".01" pos="4 0 0"/></body>
      <frame><body name="second"><geom type="sphere" size=".01" pos="5 0 0"/></body>
        <frame><geom type="sphere" size=".01" pos="2 0 0"/></frame>
      </frame>
      <geom type="sphere" size=".01" pos="3 0 0"/>
    </body></worldbody></mujoco>''')
    metadata=parse_mjcf_geometry_metadata(path,compile_mesh_poses=False)
    assert [m.body_name for m in metadata]==['parent','parent','parent','first','second']
    assert [m.position_body_m[0] for m in metadata]==[1.,2.,3.,4.,5.]


def test_matches_native_compiler_order(tmp_path):
    import pytest
    mujoco=pytest.importorskip('mujoco')
    path=tmp_path/'scene.xml'
    path.write_text('''<mujoco><worldbody><body name="parent">
      <body name="child"><geom type="sphere" size=".01" pos="2 0 0"/></body>
      <geom type="sphere" size=".01" pos="1 0 0"/>
    </body></worldbody></mujoco>''')
    model=mujoco.MjModel.from_xml_path(str(path))
    metadata=parse_mjcf_geometry_metadata(path)
    assert [m.body_name for m in metadata]==[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,int(i)) for i in model.geom_bodyid]
    assert [m.position_body_m[0] for m in metadata]==model.geom_pos[:,0].tolist()
