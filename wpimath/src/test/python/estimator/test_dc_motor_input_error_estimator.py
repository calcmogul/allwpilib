import pytest
import math
import numpy as np

from wpimath import DCMotorInputErrorEstimator


def test_dc_motor_input_error_estimator_constant_bias():
    kv = 1
    ka = 0.1
    dt = 0.005  # s
    duration = 2  # s

    # Continuous DC motor model
    A_c = -kv / ka
    B_c = 1 / ka

    # Discrete DC motor model
    A_d = math.exp(A_c * dt)
    B_d = 1 / A_c * (A_d - 1) * B_c

    estimator = DCMotorInputErrorEstimator(kv, ka, dt)
    x = 0.0
    u = 5.0
    u_bias_estimate = 0.0

    estimator.reset(x)

    # Positive bias
    u_bias = 2.0
    for _ in np.arange(0, duration, dt):
        u_bias_estimate = estimator.calculate(x, u)
        x = A_d * x + B_d * (u + u_bias)
    assert x == pytest.approx((u + u_bias) / kv, 1e-6)
    assert u_bias_estimate == pytest.approx(u_bias, 1e-6)

    # Negative bias
    u_bias = -2.0
    for _ in np.arange(0, duration, dt):
        u_bias_estimate = estimator.calculate(x, u)
        x = A_d * x + B_d * (u + u_bias)
    assert x == pytest.approx((u + u_bias) / kv, 1e-6)
    assert u_bias_estimate == pytest.approx(u_bias, 1e-6)

    # Zero bias
    u_bias = 0.0
    for _ in np.arange(0, duration, dt):
        u_bias_estimate = estimator.calculate(x, u)
        x = A_d * x + B_d * (u + u_bias)
    assert x == pytest.approx((u + u_bias) / kv, 1e-6)
    assert u_bias_estimate == pytest.approx(u_bias, 1e-6)
