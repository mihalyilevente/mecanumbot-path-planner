import math

from mecanumbot_eval.kinematics import C_RR, G, LX, LY, MASS, RADIUS, rolling_energy, wheel_speeds


def sample(t, vx=0.0, vy=0.0, wz=0.0):
    return {
        't': t, 'x': 0.0, 'y': 0.0, 'yaw': 0.0,
        'vx': vx, 'vy': vy, 'wz': wz,
        'cmd_vx': vx, 'cmd_vy': vy, 'cmd_wz': wz,
    }


def expected_wheel_m(vx, vy, wz, dt):
    return RADIUS * dt * sum(abs(speed) for speed in wheel_speeds(vx, vy, wz))


def test_straight_drive_matches_body_distance_times_four():
    wheel_m, energy = rolling_energy([sample(0.0, vx=0.24), sample(1.0, vx=0.24)])
    assert math.isclose(wheel_m, 0.96)
    assert math.isclose(energy, C_RR * MASS * G * wheel_m / 4.0)


def test_strafe_costs_the_same_as_forward():
    forward = rolling_energy([sample(0.0, vx=0.24), sample(1.0, vx=0.24)])
    strafe = rolling_energy([sample(0.0, vy=0.24), sample(1.0, vy=0.24)])
    assert math.isclose(forward[0], strafe[0])
    assert math.isclose(forward[1], strafe[1])


def test_spin_has_energy_with_no_translation():
    wz = 1.0
    wheel_m, energy = rolling_energy([sample(0.0, wz=wz), sample(1.0, wz=wz)])
    assert math.isclose(wheel_m, 4.0 * (LX + LY) * wz)
    assert energy > 0.0


def test_trapezoid_between_two_speeds():
    wheel_m, _ = rolling_energy([sample(0.0, vx=0.0), sample(2.0, vx=0.24)])
    assert math.isclose(wheel_m, expected_wheel_m(0.12, 0.0, 0.0, 2.0))


def test_zero_or_negative_dt_is_skipped():
    rows = [sample(1.0, vx=0.2), sample(1.0, vx=0.2), sample(0.5, vx=0.2)]
    assert rolling_energy(rows) == (0.0, 0.0)


def test_one_sample_is_unknown():
    assert rolling_energy([sample(0.0, vx=0.2)]) == (None, None)
