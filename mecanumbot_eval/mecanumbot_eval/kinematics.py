"""Wheel speeds and rolling energy for the simulated Mecanumbot."""

import math

# Wheel centres in model.sdf, metres. lx/ly are the distances from the origin.
LX = 0.064
LY = 0.1522
RADIUS = 0.033
MASS = 3.0
# Hard-floor rolling resistance. Same value for every planner.
C_RR = 0.02
G = 9.81

RUN_FIELDS = ['t', 'x', 'y', 'yaw', 'vx', 'vy', 'wz', 'cmd_vx', 'cmd_vy', 'cmd_wz']


def wheel_speeds(vx, vy, wz):
    """Rad/s for front-left, front-right, rear-left, rear-right.

    The other roller layout swaps which wheel gets which speed. The sum of
    the absolute speeds stays the same, so the energy does too.
    """
    arm = LX + LY
    scale = 1.0 / RADIUS
    return (
        scale * (vx - vy - arm * wz),
        scale * (vx + vy + arm * wz),
        scale * (vx + vy - arm * wz),
        scale * (vx - vy + arm * wz),
    )


def rolling_energy(samples):
    """Integrate odometry twists. samples are dicts with t, vx, vy, wz.

    Returns (wheel_m, energy_j). wheel_m is the sum of the four arc lengths.
    Fewer than two samples has no interval, so both values are None.
    """
    if len(samples) < 2:
        return None, None
    wheel_rad = 0.0
    for prev, cur in zip(samples, samples[1:]):
        dt = cur['t'] - prev['t']
        if dt <= 0.0:
            continue
        vx = 0.5 * (prev['vx'] + cur['vx'])
        vy = 0.5 * (prev['vy'] + cur['vy'])
        wz = 0.5 * (prev['wz'] + cur['wz'])
        wheel_rad += dt * sum(abs(speed) for speed in wheel_speeds(vx, vy, wz))
    wheel_m = RADIUS * wheel_rad
    energy_j = C_RR * MASS * G * wheel_m / 4.0
    return wheel_m, energy_j


def duration_of(samples):
    if len(samples) < 2:
        return None
    span = samples[-1]['t'] - samples[0]['t']
    if span < 0.0 or math.isnan(span):
        return None
    return span
