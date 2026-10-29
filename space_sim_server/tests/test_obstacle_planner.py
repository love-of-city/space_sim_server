"""A blocked straight route must yield a smooth detour or an explicit rejection."""
import numpy as np
import pytest

from simulation.scripted_tasks.obstacle_planner import JointRouteSearch, PlanningFailure, SearchOptions
from simulation.scripted_tasks.joint_path import JointPath


def test_search_goes_around_wall_and_smoothing_does_not_cut_through_it():
    def collision(q):
        if -.18 < q[0] < .18 and q[1] < .65:
            return {"body1":"arm", "body2":"neighbor"}
    search=JointRouteSearch([-1.,-1.],[1.,1.],collision,seed=12)
    start,goal=np.array([-.8,0.]),np.array([.8,0.])
    route=search.search(start,goal)
    knots=search.smooth(route)
    curve=JointPath.from_waypoints(knots)
    np.testing.assert_allclose(curve.at(0),start,atol=1e-12)
    np.testing.assert_allclose(curve.at(1),goal,atol=1e-12)
    assert all(collision(curve.at(u)) is None for u in np.linspace(0,1,4001))
    assert search.report['first_collision']['body2']=='neighbor'
    assert search.report['iterations']>0


def test_sealed_layout_is_rejected_without_returning_an_unchecked_path():
    def collision(q):
        return {"blocked":True} if abs(q[0])<.18 else None
    search=JointRouteSearch([-1.,-1.],[1.,1.],collision,seed=12,
                            options=SearchOptions(iterations=80))
    with pytest.raises(PlanningFailure,match='search_budget_exhausted'):
        search.search(np.array([-.8,0.]),np.array([.8,0.]))


def test_colliding_goal_is_rejected_before_search():
    search=JointRouteSearch([-1.,-1.],[1.,1.],lambda q: {'blocked':True} if q[0]>.5 else None)
    with pytest.raises(PlanningFailure,match='goal_in_collision'):
        search.search(np.array([0.,0.]),np.array([.8,0.]))


@pytest.mark.parametrize('opening',[True,False])
def test_detour_aperture_is_monotone_and_executed_fingers_are_checked(opening):
    checked = []
    def collision(q):
        checked.append(q.copy())
        if -.18 < q[0] < .18 and q[1] < .65:
            return {'body1':'finger','body2':'neighbor'}
    search=JointRouteSearch([-1.,-1.,0.],[1.,1.,1.],collision,seed=12)
    first,last=(.1,.9) if opening else (.9,.1)
    start,goal=np.array([-.8,0.,first]),np.array([.8,0.,last])
    route=search.search(start,goal,monotone_axis=2)
    direction=1 if opening else -1
    assert np.min(direction*np.diff(route[:,2])) >= -1e-10
    knots=search.smooth(route)
    curve=JointPath.from_waypoints(knots)
    samples=np.array([curve.at(u) for u in np.linspace(0.,1.,4001)])
    assert np.min(direction*np.diff(samples[:,2])) >= -1e-10
    assert samples[:,2].min() >= .1-1e-12
    assert samples[:,2].max() <= .9+1e-12
    assert all(collision(q) is None for q in samples)
    np.testing.assert_allclose(curve.at(0),start,atol=1e-12)
    np.testing.assert_allclose(curve.at(1),goal,atol=1e-12)
    # Smoothing performs new collision queries after replacing the aperture.
    assert len(checked) > len(samples)


def test_smoothing_rechecks_coordinated_opening_instead_of_search_aperture():
    # The search route opens early. The natural opening schedule would touch a
    # narrow finger obstruction at x=0, so it must be rejected, not executed.
    def collision(q):
        return {'body1':'finger','body2':'lip'} if abs(q[0])<.1 and q[1]<.7 else None
    search=JointRouteSearch([-1.,0.],[1.,1.],collision,seed=12)
    search.monotone_axis,search.monotone_direction=1,1.
    route=np.array([[-.8,.1],[-.5,.9],[.8,.9]])
    assert all(collision(q) is None for q in route)
    with pytest.raises(PlanningFailure,match='smoothed_route_in_collision'):
        search.smooth(route)
