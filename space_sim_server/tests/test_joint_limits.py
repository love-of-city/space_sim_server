from pathlib import Path
import math
import pytest
from space_arm_platform.joint_limits import load_joint_limits


def test_default_classes_units_childclass_and_continuous(tmp_path):
    p=tmp_path/"model.xml"
    p.write_text('''<mujoco><default><joint limited="true" range="-90 90"/>
      <default class="wrist"><joint range="-180 180"/></default></default>
      <worldbody><body childclass="wrist"><joint name="a"/>
      <body><joint name="b" limited="false"/></body>
      <body><joint name="c" class=""/></body>
      <body><joint name="d" type="slide" range="0 0.04"/></body>
      </body></worldbody></mujoco>''',encoding="utf-8")
    r=load_joint_limits(p,("a","b","c","d"))
    assert r.upper == (math.pi,math.inf,math.pi/2,.04)
    assert r.limited == (True,False,True,True)


def test_include_and_invalid_limits_fail_closed(tmp_path):
    p=tmp_path/"model.xml"
    (tmp_path/"arm.xml").write_text('<mujocoinclude><body><joint name="a" range="-1 2"/></body></mujocoinclude>')
    p.write_text('<mujoco><compiler angle="radian"/><worldbody><include file="arm.xml"/></worldbody></mujoco>')
    assert load_joint_limits(p,("a",)).upper == (2.,)
    with pytest.raises(ValueError,match="missing joints"): load_joint_limits(p,("missing",))
    (tmp_path/"arm.xml").write_text('<mujocoinclude><body><joint name="a" range="2 -1"/></body></mujocoinclude>')
    with pytest.raises(ValueError,match="range"): load_joint_limits(p,("a",))
