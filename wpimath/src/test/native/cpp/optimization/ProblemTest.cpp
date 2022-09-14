// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <chrono>
#include <cmath>

#include <fmt/core.h>
#include <wpi/SmallVector.h>

#include "frc/EigenCore.h"
#include "frc/optimization/Problem.h"
#include "frc/system/Discretization.h"
#include "frc/system/plant/LinearSystemId.h"
#include "gtest/gtest.h"
#include "units/angle.h"
#include "units/angular_acceleration.h"
#include "units/angular_velocity.h"
#include "units/time.h"
#include "units/voltage.h"

wpi::SmallVector<double> Range(double start, double end, double step) {
  wpi::SmallVector<double> ret;

  for (double i = start; i < end; i += step) {
    ret.emplace_back(i);
  }

  return ret;
}

TEST(ProblemTest, EmptyProblem) {
  frc::Problem problem;
  EXPECT_EQ(frc::Problem::SolverStatus::kOk, problem.Solve());
}

TEST(ProblemTest, NoCostUnconstrained) {
  {
    frc::Problem problem;

    auto X = problem.DecisionVariable(2, 3);

    EXPECT_EQ(frc::Problem::SolverStatus::kOk, problem.Solve());

    for (int row = 0; row < X.Rows(); ++row) {
      for (int col = 0; col < X.Cols(); ++col) {
        EXPECT_EQ(0.0, X.Value(row, col));
      }
    }
  }

  {
    frc::Problem problem;

    auto X = problem.DecisionVariable(2, 3);
    X = frc::Matrixd<2, 3>{{1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}};

    EXPECT_EQ(frc::Problem::SolverStatus::kOk, problem.Solve());

    for (int row = 0; row < X.Rows(); ++row) {
      for (int col = 0; col < X.Cols(); ++col) {
        EXPECT_EQ(1.0, X.Value(row, col));
      }
    }
  }
}

TEST(ProblemTest, QuadraticUnconstrained1) {
  frc::Problem problem;

  auto x = problem.DecisionVariable();
  x = 2.0;

  problem.Minimize(x * x - 6.0 * x);

  EXPECT_EQ(frc::Problem::SolverStatus::kOk, problem.Solve());

  EXPECT_NEAR(3.0, x.Value(0), frc::Problem::kTolerance);
}

TEST(ProblemTest, QuadraticUnconstrained2) {
  {
    frc::Problem problem;

    auto x = problem.DecisionVariable();
    x = 1.0;
    auto y = problem.DecisionVariable();
    y = 2.0;

    problem.Minimize(x * x + y * y);

    EXPECT_EQ(frc::Problem::SolverStatus::kOk, problem.Solve());

    EXPECT_NEAR(0.0, x.Value(0), frc::Problem::kTolerance);
    EXPECT_NEAR(0.0, y.Value(0), frc::Problem::kTolerance);
  }

  {
    frc::Problem problem;

    auto x = problem.DecisionVariable(2);
    x(0) = 1.0;
    x(1) = 2.0;

    problem.Minimize(x.Transpose() * x);

    EXPECT_EQ(frc::Problem::SolverStatus::kOk, problem.Solve());

    EXPECT_NEAR(0.0, x.Value(0), frc::Problem::kTolerance);
    EXPECT_NEAR(0.0, x.Value(1), frc::Problem::kTolerance);
  }
}

TEST(ProblemTest, QuadraticEqualityConstrained) {
  // Maximize xy subject to x + 3y = 36.
  //
  // Maximize f(x,y) = xy
  // subject to g(x,y) = x + 3y - 36 = 0
  //
  //         value func  constraint
  //              |          |
  //              v          v
  // L(x,y,λ) = f(x,y) - λg(x,y)
  // L(x,y,λ) = xy - λ(x + 3y - 36)
  // L(x,y,λ) = xy - xλ - 3yλ + 36λ
  //
  // ∇_x,y,λ L(x,y,λ) = 0
  //
  // ∂L/∂x = y - λ
  // ∂L/∂y = x - 3λ
  // ∂L/∂λ = -x - 3y + 36
  //
  //  0x + 1y - 1λ = 0
  //  1x + 0y - 3λ = 0
  // -1x - 3y + 0λ + 36 = 0
  //
  // [ 0  1 -1][x]   [  0]
  // [ 1  0 -3][y] = [  0]
  // [-1 -3  0][λ]   [-36]
  //
  // Solve with:
  // ```python
  //   np.linalg.solve(
  //     np.array([[0,1,-1],
  //               [1,0,-3],
  //               [-1,-3,0]]),
  //     np.array([[0], [0], [-36]]))
  // ```
  //
  // [x]   [18]
  // [y] = [ 6]
  // [λ]   [ 6]
  {
    frc::Problem problem;

    auto x = problem.DecisionVariable();
    auto y = problem.DecisionVariable();

    // Maximize xy
    problem.Minimize(-x * y);

    problem.SubjectTo(x + 3 * y == 36);

    EXPECT_EQ(frc::Problem::SolverStatus::kOk, problem.Solve());

    EXPECT_NEAR(18.0, x.Value(0), frc::Problem::kTolerance);
    EXPECT_NEAR(6.0, y.Value(0), frc::Problem::kTolerance);
  }

  {
    frc::Problem problem;

    auto x = problem.DecisionVariable(2);
    x(0) = 1.0;
    x(1) = 2.0;

    problem.Minimize(x.Transpose() * x);

    problem.SubjectTo(x == frc::Matrixd<2, 1>{{3.0, 3.0}});

    EXPECT_EQ(frc::Problem::SolverStatus::kOk, problem.Solve());

    EXPECT_NEAR(3.0, x.Value(0), frc::Problem::kTolerance);
    EXPECT_NEAR(3.0, x.Value(1), frc::Problem::kTolerance);
  }
}

TEST(ProblemTest, DISABLED_RosenbrockConstrainedWithCubicAndLine) {
  // https://en.wikipedia.org/wiki/Test_functions_for_optimization#Test_functions_for_constrained_optimization
  for (auto x0 : Range(-1.5, 1.5, 0.1)) {
    for (auto y0 : Range(-0.5, 2.5, 0.1)) {
      frc::Problem problem;

      auto x = problem.DecisionVariable();
      x = x0;
      auto y = problem.DecisionVariable();
      y = y0;

      problem.Minimize(frc::pow(1 - x, 2) +
                       100 * frc::pow(y - frc::pow(x, 2), 2));

      problem.SubjectTo(frc::pow(x - 1, 3) - y + 1 <= 0);
      problem.SubjectTo(x + y - 2 <= 0);

      EXPECT_EQ(frc::Problem::SolverStatus::kOk, problem.Solve());

      EXPECT_NEAR(1.0, x.Value(0), frc::Problem::kTolerance);
      EXPECT_NEAR(1.0, y.Value(0), frc::Problem::kTolerance);
    }
  }
}

TEST(ProblemTest, DISABLED_RosenbrockConstrainedToDisk) {
  // https://en.wikipedia.org/wiki/Test_functions_for_optimization#Test_functions_for_constrained_optimization
  for (auto x0 : Range(-1.5, 1.5, 0.1)) {
    for (auto y0 : Range(-1.5, 1.5, 0.1)) {
      frc::Problem problem;

      auto x = problem.DecisionVariable();
      x = x0;
      auto y = problem.DecisionVariable();
      y = y0;

      problem.Minimize(frc::pow(1 - x, 2) +
                       100 * frc::pow(y - frc::pow(x, 2), 2));

      problem.SubjectTo(frc::pow(x, 2) + frc::pow(y, 2) <= 2);

      EXPECT_EQ(frc::Problem::SolverStatus::kOk, problem.Solve());

      EXPECT_NEAR(1.0, x.Value(0), frc::Problem::kTolerance);
      EXPECT_NEAR(1.0, y.Value(0), frc::Problem::kTolerance);
    }
  }
}

TEST(ProblemTest, DoubleIntegratorMinimumTime) {
  auto start = std::chrono::system_clock::now();

  constexpr auto T = 5_s;
  constexpr auto dt = 5_ms;
  constexpr int N = T / dt;

  frc::Problem problem;

  // 2x1 state vector with N + 1 timesteps (includes last state)
  auto X = problem.DecisionVariable(2, N + 1);

  // 1x1 input vector with N timesteps (input at last state doesn't matter)
  auto U = problem.DecisionVariable(1, N);

  // Kinematics constraint assuming constant acceleration between timesteps
  for (int k = 0; k < N; ++k) {
    constexpr double t = dt.value();
    auto p_k1 = X(0, k + 1);
    auto v_k = X(1, k);
    auto a_k = U(0, k);

    // pₖ₊₁ = 1/2aₖt² + vₖt
    problem.SubjectTo(p_k1 == 0.5 * a_k * frc::pow(t, 2) + v_k * t);
  }

  // Start and end at rest
  problem.SubjectTo(X.Col(0) == frc::Matrixd<2, 1>{{0.0}, {0.0}});
  problem.SubjectTo(X.Col(N) == frc::Matrixd<2, 1>{{10.0}, {0.0}});

  // Limit velocity
  problem.SubjectTo(-1 <= X.Row(1));
  problem.SubjectTo(X.Row(1) <= 1);

  // Limit acceleration
  problem.SubjectTo(-1 <= U);
  problem.SubjectTo(U <= 1);

  // Cost function - minimize position error
  frc::VariableMatrix J = 0.0;
  for (int k = 0; k < N + 1; ++k) {
    J += frc::pow(10.0 - X(0, k), 2);
  }
  problem.Minimize(J);

  auto end1 = std::chrono::system_clock::now();
  EXPECT_EQ(frc::Problem::SolverStatus::kOk, problem.Solve());
  auto end2 = std::chrono::system_clock::now();

  using std::chrono::duration_cast;
  using std::chrono::microseconds;
  fmt::print("Setup time={} ms\n",
             duration_cast<microseconds>(end1 - start).count() / 1000.0);
  fmt::print("Solve time={} ms\n",
             duration_cast<microseconds>(end2 - end1).count() / 1000.0);

  // TODO: Verify solution
}

TEST(ProblemTest, FlywheelDirectTranscription) {
  auto start = std::chrono::system_clock::now();

  constexpr auto T = 5_s;
  constexpr auto dt = 5_ms;
  constexpr int N = T / dt;

  // Flywheel model:
  // States: [velocity]
  // Inputs: [voltage]
  auto system = frc::LinearSystemId::IdentifyVelocitySystem<units::radians>(
      1_V / 1_rad_per_s, 1_V / 1_rad_per_s_sq);
  frc::Matrixd<1, 1> A;
  frc::Matrixd<1, 1> B;
  frc::DiscretizeAB<1, 1>(system.A(), system.B(), dt, &A, &B);

  frc::Problem problem;
  auto X = problem.DecisionVariable(1, N + 1);
  auto U = problem.DecisionVariable(1, N);

  // Dynamics constraint
  for (int k = 0; k < N; ++k) {
    problem.SubjectTo(X.Col(k + 1) == A * X.Col(k) + B * U.Col(k));
  }

  // State and input constraints
  problem.SubjectTo(X.Col(0) == 0.0);
  problem.SubjectTo(-12 <= U);
  problem.SubjectTo(U <= 12);

  // Cost function - minimize error
  frc::Matrixd<1, 1> r{10.0};
  frc::VariableMatrix J = 0.0;
  for (int k = 0; k < N + 1; ++k) {
    J += ((r - X.Col(k)).Transpose() * (r - X.Col(k)));
  }
  problem.Minimize(J);

  auto end1 = std::chrono::system_clock::now();
  EXPECT_EQ(frc::Problem::SolverStatus::kOk, problem.Solve());
  auto end2 = std::chrono::system_clock::now();

  using std::chrono::duration_cast;
  using std::chrono::microseconds;
  fmt::print("Setup time={} ms\n",
             duration_cast<microseconds>(end1 - start).count() / 1000.0);
  fmt::print("Solve time={} ms\n",
             duration_cast<microseconds>(end2 - end1).count() / 1000.0);

  // TODO: Verify solution
}
