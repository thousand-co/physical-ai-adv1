"""Rest-to-rest quintic joint profiles with analytic peak limits (SI units)."""
import math


def plan_joint_motion(start, goal, max_velocity=.8, max_acceleration=1.4,
                      minimum_duration=.8, sample_period=.05):
    if len(start) != len(goal) or not start:
        raise ValueError('Joint vectors must have matching nonzero dimensions')
    values = (*start, *goal, max_velocity, max_acceleration, minimum_duration, sample_period)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Motion values must be finite')
    if min(max_velocity, max_acceleration, minimum_duration, sample_period) <= 0:
        raise ValueError('Motion limits and periods must be positive')
    delta = [b - a for a, b in zip(start, goal)]
    distance = max(abs(value) for value in delta)
    # Peaks of s(u)=10u^3-15u^4+6u^5: s'=1.875, |s''|=10/sqrt(3).
    duration = max(minimum_duration, 1.875 * distance / max_velocity,
                   math.sqrt((10 / math.sqrt(3)) * distance / max_acceleration))
    count = max(2, math.ceil(duration / sample_period))
    points = []
    for i in range(count + 1):
        u = i / count
        blend = 10*u**3 - 15*u**4 + 6*u**5
        velocity = (30*u**2 - 60*u**3 + 30*u**4) / duration
        acceleration = (60*u - 180*u**2 + 120*u**3) / duration**2
        points.append((duration*u,
                       [a + d*blend for a, d in zip(start, delta)],
                       [d*velocity for d in delta],
                       [d*acceleration for d in delta]))
    return duration, points
