import numpy as np
from simulation.evaluation.terminal_observation import TerminalObservation


def test_observation_waits_for_simulation_time():
    gate=TerminalObservation(2.5,40.,4.,.15)
    assert not gate.update(10.,np.zeros(6))
    assert not gate.update(12.49,np.zeros(6))
    assert gate.update(12.5,np.zeros(6))


def test_observation_keeps_the_same_continuous_overload_stop():
    gate=TerminalObservation(2.5,40.,4.,.15)
    gate.update(0.,np.array([0,0,0,4.1,0,0]))
    gate.update(.15,np.array([0,0,0,4.1,0,0]))
    assert gate.failed
