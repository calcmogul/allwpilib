# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

from typing import Callable

from sleipnir.autodiff import Variable, VariableMatrix, cos, sin
from sleipnir.optimization import Problem, bounds

from wpimath import (
    Pose2d,
    Translation2d,
    ChassisAccelerations,
    ChassisVelocities,
    HolonomicSample,
    HolonomicTrajectory,
)


def _lerp(a: float, b: float, t: float) -> float:
    """
    Computes the linear interpolation between a and b, if the parameter t is
    inside [0, 1), or the linear extrapolation otherwise.

    Args:
        a: The first value.
        b: The second value.
        t: The interpolant.
    """
    return a + (b - a) * t


def _rk4(
    f: Callable[[VariableMatrix, VariableMatrix], VariableMatrix],
    x: VariableMatrix,
    u: VariableMatrix,
    dt: Variable,
) -> VariableMatrix:
    """Performs 4th order Runge-Kutta integration of dx/dt = f(x, u) for dt.

    Args:
        f: The function to integrate. It must take two arguments x and u.
        x: The initial value of x.
        u: The value u held constant over the integration period.
        dt: The time over which to integrate.
    """
    h = dt

    k1 = f(x, u)
    k2 = f(x + h * 0.5 * k1, u)
    k3 = f(x + h * 0.5 * k2, u)
    k4 = f(x + h * k3, u)

    return x + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def generate_from_translations(
    initial_waypoint: Pose2d,
    interior_waypoints: list[Translation2d],
    final_waypoint: Pose2d,
    max_linear_velocity: float,
    max_angular_velocity: float,
    max_linear_acceleration: float,
    max_angular_acceleration: float,
) -> HolonomicTrajectory:
    """Generates a trajectory through the given waypoints satisfying the given
    constraints.

    Args:
        initial_waypoint: The initial waypoint.
        interior_waypoints: The interior waypoints.
        final_waypoint: The final waypoint.
        max_linear_velocity: The maximum linear velocity.
        max_angular_velocity: The maximum angular velocity.
        max_linear_acceleration: The maximum linear acceleration.
        max_angular_acceleration: The maximum angular acceleration.
    """

    def f(x: VariableMatrix, u: VariableMatrix) -> VariableMatrix:
        #  x = [x, y, θ, v, ω]
        #  u = [a, α]
        #
        #  ẋ = v cosθ
        #  ẏ = v sinθ
        #  θ̇ = ω
        #  v̇ = a
        #  ω̇ = α
        θ = x[2, 0]
        v = x[3, 0]
        ω = x[4, 0]
        a = u[0, 0]
        α = u[1, 0]
        return VariableMatrix([[v * cos(θ)], [v * sin(θ)], [ω], [a], [α]])

    N_sgmt: int = 40
    num_segments: int = len(interior_waypoints) + 1
    T_max: float = 5.0

    problem = Problem()

    X = problem.decision_variable(5, num_segments * N_sgmt + 1)
    U = problem.decision_variable(2, num_segments * N_sgmt)
    dts = problem.decision_variable(num_segments * N_sgmt)

    waypoints: list[Translation2d] = []
    waypoints.append(initial_waypoint.translation())
    for waypoint in interior_waypoints:
        waypoints.append(waypoint)
    waypoints.append(final_waypoint.translation())

    # Pose initial guess
    for sgmt in range(num_segments):
        for k in range(N_sgmt * sgmt, N_sgmt * (sgmt + 1)):
            scale = (k - N_sgmt * sgmt) / N_sgmt
            X[0, k].set_value(_lerp(waypoints[sgmt].x, waypoints[sgmt + 1].x, scale))
            X[1, k].set_value(_lerp(waypoints[sgmt].y, waypoints[sgmt + 1].y, scale))
            angle = (waypoints[sgmt + 1] - waypoints[sgmt]).angle()
            X[2, k].set_value(angle.radians() if angle else 0.0)
    for sgmt in range(num_segments):
        X[0, N_sgmt * sgmt].set_value(waypoints[sgmt].x)
        X[1, N_sgmt * sgmt].set_value(waypoints[sgmt].y)
        if sgmt == 0:
            X[2, N_sgmt * sgmt].set_value(initial_waypoint.rotation().radians())
        elif sgmt == num_segments - 1:
            X[2, N_sgmt * sgmt].set_value(final_waypoint.rotation().radians())
    X[0, -1].set_value(final_waypoint.x)
    X[1, -1].set_value(final_waypoint.y)
    X[2, -1].set_value(final_waypoint.rotation().radians())

    # dt constraints
    problem.subject_to(bounds(0, dts, T_max / (num_segments * N_sgmt)))
    for sgmt in range(num_segments):
        for k in range(N_sgmt * sgmt, N_sgmt * (sgmt + 1) - 1):
            problem.subject_to(dts[k] == dts[k + 1])
    for k in range(X.cols() - 1):
        dts[k].set_value(T_max / N_sgmt)

    # Waypoint constraints
    for sgmt, waypoint in enumerate(waypoints):
        x = X[0, sgmt * N_sgmt]
        y = X[1, sgmt * N_sgmt]
        θ = X[2, sgmt * N_sgmt]
        problem.subject_to(x == waypoint.x)
        problem.subject_to(y == waypoint.y)
        if sgmt == 0:
            problem.subject_to(cos(θ) == initial_waypoint.Rotation().cos())
            problem.subject_to(sin(θ) == initial_waypoint.Rotation().sin())
        elif sgmt == len(waypoints) - 1:
            problem.subject_to(cos(θ) == final_waypoint.Rotation().cos())
            problem.subject_to(sin(θ) == final_waypoint.Rotation().sin())

    # Dynamics constraints
    for k in range(X.cols() - 1):
        x_k = X[:, k]
        x_k1 = X[:, k + 1]
        u_k = U[:, k]
        dt_k = dts[k]
        problem.subject_to(x_k1 == _rk4(f, x_k, u_k, dt_k))

    # Initial and final velocities
    problem.subject_to(X[3:5, 0] == 0)
    problem.subject_to(X[3:5, -1] == 0)

    # Velocity limits
    problem.subject_to(bounds(-max_linear_velocity, X[3, :], max_linear_velocity))
    problem.subject_to(bounds(-max_angular_velocity, X[4, :], max_angular_velocity))

    # Acceleration limits
    problem.subject_to(
        bounds(-max_linear_acceleration, U[0, :], max_linear_acceleration)
    )
    problem.subject_to(
        bounds(-max_angular_acceleration, U[1, :], max_angular_acceleration)
    )

    problem.minimize(sum(dts))

    problem.solve()

    samples: list[HolonomicSample] = []
    t = 0.0
    for k in range(X.cols()):
        x = X[0, k].value()
        y = X[1, k].value()
        θ = X[2, k].value()
        v_x = X[3, k].value()
        ω = X[4, k].value()

        if k < X.cols() - 1:
            a_x = U[0, k].value()
            α = U[1, k].value()
            dt = dts[k].value()
            samples.append(
                HolonomicSample(
                    t,
                    Pose2d(x, y, θ),
                    ChassisVelocities(v_x, 0.0, ω),
                    ChassisAccelerations(a_x, 0.0, α),
                )
            )
            t += dt
        else:
            samples.append(
                HolonomicSample(
                    t,
                    Pose2d(x, y, θ),
                    ChassisVelocities(v_x, 0.0, ω),
                    ChassisAccelerations(0.0, 0.0, 0.0),
                )
            )

    return HolonomicTrajectory(samples)


def generate_from_poses(
    waypoints: list[Pose2d],
    max_linear_velocity: float,
    max_angular_velocity: float,
    max_linear_acceleration: float,
    max_angular_acceleration: float,
) -> HolonomicTrajectory:
    """Generates a trajectory through the given waypoints satisfying the given
    constraints.

    Args:
        waypoints: The list of waypoints.
        max_linear_velocity: The maximum linear velocity.
        max_angular_velocity: The maximum angular velocity.
        max_linear_acceleration: The maximum linear acceleration.
        max_angular_acceleration: The maximum angular acceleration.
    """

    def f(x: VariableMatrix, u: VariableMatrix) -> VariableMatrix:
        #  x = [x, y, θ, v, ω]
        #  u = [a, α]
        #
        #  ẋ = v cosθ
        #  ẏ = v sinθ
        #  θ̇ = ω
        #  v̇ = a
        #  ω̇ = α
        θ = x[2, 0]
        v = x[3, 0]
        ω = x[4, 0]
        a = u[0, 0]
        α = u[1, 0]
        return VariableMatrix([[v * cos(θ)], [v * sin(θ)], [ω], [a], [α]])

    N_sgmt: int = 40
    num_segments: int = len(waypoints) - 1
    T_max: float = 5.0

    problem = Problem()

    X = problem.decision_variable(5, num_segments * N_sgmt + 1)
    U = problem.decision_variable(2, num_segments * N_sgmt)
    dts = problem.decision_variable(num_segments * N_sgmt)

    # Pose initial guess
    for sgmt in range(num_segments):
        for k in range(N_sgmt * sgmt, N_sgmt * (sgmt + 1)):
            scale = (k - N_sgmt * sgmt) / N_sgmt
            X[0, k].set_value(_lerp(waypoints[sgmt].x, waypoints[sgmt + 1].x, scale))
            X[1, k].set_value(_lerp(waypoints[sgmt].y, waypoints[sgmt + 1].y, scale))
            angle = (
                waypoints[sgmt + 1].translation() - waypoints[sgmt].translation()
            ).angle()
            X[2, k].set_value(angle.radians() if angle else 0.0)
    for sgmt in range(num_segments):
        X[0, N_sgmt * sgmt].set_value(waypoints[sgmt].x)
        X[1, N_sgmt * sgmt].set_value(waypoints[sgmt].y)
        X[2, N_sgmt * sgmt].set_value(waypoints[sgmt].rotation().radians())
    X[0, -1].set_value(waypoints[-1].x)
    X[1, -1].set_value(waypoints[-1].y)
    X[2, -1].set_value(waypoints[-1].rotation().radians())

    # dt constraints
    problem.subject_to(bounds(0, dts, T_max / (num_segments * N_sgmt)))
    for sgmt in range(num_segments):
        for k in range(N_sgmt * sgmt, N_sgmt * (sgmt + 1) - 1):
            problem.subject_to(dts[k] == dts[k + 1])
    for k in range(X.cols() - 1):
        dts[k].set_value(T_max / N_sgmt)

    # Waypoint constraints
    for sgmt, waypoint in enumerate(waypoints):
        x = X[0, sgmt * N_sgmt]
        y = X[1, sgmt * N_sgmt]
        θ = X[2, sgmt * N_sgmt]
        problem.subject_to(x == waypoint.x)
        problem.subject_to(y == waypoint.y)
        problem.subject_to(cos(θ) == waypoint.rotation().cos())
        problem.subject_to(sin(θ) == waypoint.rotation().sin())

    # Dynamics constraints
    for k in range(X.cols() - 1):
        x_k = X[:, k]
        x_k1 = X[:, k + 1]
        u_k = U[:, k]
        dt_k = dts[k]
        problem.subject_to(x_k1 == _rk4(f, x_k, u_k, dt_k))

    # Initial and final velocities
    problem.subject_to(X[3:5, 0] == 0)
    problem.subject_to(X[3:5, -1] == 0)

    # Velocity limits
    problem.subject_to(bounds(-max_linear_velocity, X[3, :], max_linear_velocity))
    problem.subject_to(bounds(-max_angular_velocity, X[4, :], max_angular_velocity))

    # Acceleration limits
    problem.subject_to(
        bounds(-max_linear_acceleration, U[0, :], max_linear_acceleration)
    )
    problem.subject_to(
        bounds(-max_angular_acceleration, U[1, :], max_angular_acceleration)
    )

    problem.minimize(sum(dts))

    problem.solve()

    samples: list[HolonomicSample] = []
    t = 0.0
    for k in range(X.cols()):
        x = X[0, k].value()
        y = X[1, k].value()
        θ = X[2, k].value()
        v_x = X[3, k].value()
        ω = X[4, k].value()
        a_x = U[0, k].value()
        α = U[1, k].value()
        dt = dts[k].value()

        if k < X.cols() - 1:
            samples.append(
                HolonomicSample(
                    t,
                    Pose2d(x, y, θ),
                    ChassisVelocities(v_x, 0.0, ω),
                    ChassisAccelerations(a_x, 0.0, α),
                )
            )
            t += dt
        else:
            samples.append(
                HolonomicSample(
                    t,
                    Pose2d(x, y, θ),
                    ChassisVelocities(v_x, 0.0, ω),
                    ChassisAccelerations(0.0, 0.0, 0.0),
                )
            )

    return HolonomicTrajectory(samples)
