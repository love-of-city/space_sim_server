"""Collision decomposition must preserve volume and leave every hole empty."""
import pytest
import numpy as np

mf = pytest.importorskip('manifold3d', reason='Requires the optional geometry build environment')

from tools.build_independent_task_box_layout import carve_convex_hulls, mesh_of, slab_convex_cells, solid


def test_cells_preserve_rotated_holes_without_convex_bridges():
    slab = mf.Manifold.cube((.45, .55, .025)).translate((0, 0, -.025))
    holes = [mf.Manifold.cylinder(.025, .024, circular_segments=64).translate((.1, .2, -.025)),
             mf.Manifold.cube((.05, .08, .025)).rotate((0, 0, 37)).translate((.3, .3, -.025))]
    cut = slab
    for hole in holes:
        cut -= hole
    cells = slab_convex_cells(cut)
    assert len(cells) < 150
    assert np.isclose(sum(mesh.volume for mesh in cells), cut.volume(), atol=1e-10, rtol=0)
    for mesh in cells:
        assert np.isclose(mesh.convex_hull.volume, mesh.volume, atol=1e-12, rtol=0)
        for hole in holes:
            assert abs((solid(mesh) ^ hole).volume()) < 1e-12


def test_convex_carving_preserves_a_through_hole():
    mesh = mesh_of(mf.Manifold.cube((.1, .1, .025)))
    polygon = np.array([[.03, .03], [.07, .03], [.07, .07], [.03, .07]])
    cells, _ = carve_convex_hulls([(mesh.vertices, mesh.faces)], [polygon])
    assert np.isclose(sum(p.volume for p in cells), (.1**2-.04**2)*.025, atol=1e-12, rtol=0)
    hole = mf.CrossSection([polygon]).extrude(.025)
    for cell in cells:
        assert abs((solid(cell) ^ hole).volume()) < 1e-12
