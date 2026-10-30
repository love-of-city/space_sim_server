"""Explicit demo-only torque schedule in simulation seconds."""


def demo_command(seconds: float) -> tuple[str, float]:
    phase = max(0., float(seconds)) % 16.
    for end, start, label, torque in (
        (2., 0., 'settle', 0.), (6., 2., 'tighten', .06),
        (8., 6., 'hold_tight', 0.), (14., 8., 'loosen', -.15),
        (16., 14., 'hold_loose', 0.),
    ):
        if phase < end:
            return label, torque * min(1., (phase-start)/.2)
    raise AssertionError('unreachable')
