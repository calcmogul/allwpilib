// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <chrono>
#include <cmath>
#include <fstream>

#include <fmt/core.h>
#include <wpi/SmallVector.h>

#include "Eigen/QR"
#include "frc/EigenCore.h"
#include "frc/optimization/Problem.h"
#include "frc/optimization/SolverExitCondition.h"
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

  frc::SolverConfig config;
  config.diagnostics = true;

  auto status = problem.Solve(config);
  EXPECT_EQ(frc::autodiff::ExpressionType::kNone, status.costFunctionType);
  EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
            status.equalityConstraintType);
  EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
            status.inequalityConstraintType);
  EXPECT_EQ(frc::SolverExitCondition::kOk, status.exitCondition);
}

TEST(ProblemTest, SolverStatusInfeasible) {
  // Equality constraints
  {
    frc::Problem problem;

    auto x = problem.DecisionVariable();
    auto y = problem.DecisionVariable();
    auto z = problem.DecisionVariable();

    problem.SubjectTo(x == y + 1);
    problem.SubjectTo(y == z + 1);
    problem.SubjectTo(z == x + 1);

    frc::SolverConfig config;
    config.diagnostics = true;

    auto status = problem.Solve(config);
    EXPECT_EQ(frc::autodiff::ExpressionType::kNone, status.costFunctionType);
    EXPECT_EQ(frc::autodiff::ExpressionType::kLinear,
              status.equalityConstraintType);
    EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
              status.inequalityConstraintType);
    EXPECT_EQ(frc::SolverExitCondition::kInfeasible, status.exitCondition);
  }

  // Inequality constraints
  {
    frc::Problem problem;

    auto x = problem.DecisionVariable();
    auto y = problem.DecisionVariable();
    auto z = problem.DecisionVariable();

    problem.SubjectTo(x >= y + 1);
    problem.SubjectTo(y >= z + 1);
    problem.SubjectTo(z >= x + 1);

    frc::SolverConfig config;
    config.diagnostics = true;

    auto status = problem.Solve(config);
    EXPECT_EQ(frc::autodiff::ExpressionType::kNone, status.costFunctionType);
    EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
              status.equalityConstraintType);
    EXPECT_EQ(frc::autodiff::ExpressionType::kLinear,
              status.inequalityConstraintType);
    EXPECT_EQ(frc::SolverExitCondition::kInfeasible, status.exitCondition);
  }
}

TEST(ProblemTest, SolverStatusMaxIterations) {
  frc::Problem problem;

  auto x = problem.DecisionVariable();
  x = 0.0;
  problem.Minimize(x);

  frc::SolverConfig config;
  config.diagnostics = true;
  config.maxIterations = 0;

  auto status = problem.Solve(config);
  EXPECT_EQ(frc::autodiff::ExpressionType::kLinear, status.costFunctionType);
  EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
            status.equalityConstraintType);
  EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
            status.inequalityConstraintType);
  EXPECT_EQ(frc::SolverExitCondition::kMaxIterations, status.exitCondition);
}

TEST(ProblemTest, SolverStatusTimeout) {
  frc::Problem problem;

  auto x = problem.DecisionVariable();
  x = 0.0;
  problem.Minimize(x);

  frc::SolverConfig config;
  config.diagnostics = true;
  config.timeout = 0_s;

  auto status = problem.Solve(config);
  EXPECT_EQ(frc::autodiff::ExpressionType::kLinear, status.costFunctionType);
  EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
            status.equalityConstraintType);
  EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
            status.inequalityConstraintType);
  EXPECT_EQ(frc::SolverExitCondition::kTimeout, status.exitCondition);
}

TEST(ProblemTest, NoCostUnconstrained) {
  {
    frc::Problem problem;

    auto X = problem.DecisionVariable(2, 3);

    frc::SolverConfig config;
    config.diagnostics = true;

    auto status = problem.Solve(config);
    EXPECT_EQ(frc::autodiff::ExpressionType::kNone, status.costFunctionType);
    EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
              status.equalityConstraintType);
    EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
              status.inequalityConstraintType);
    EXPECT_EQ(frc::SolverExitCondition::kOk, status.exitCondition);

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

    frc::SolverConfig config;
    config.diagnostics = true;

    auto status = problem.Solve(config);
    EXPECT_EQ(frc::autodiff::ExpressionType::kNone, status.costFunctionType);
    EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
              status.equalityConstraintType);
    EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
              status.inequalityConstraintType);
    EXPECT_EQ(frc::SolverExitCondition::kOk, status.exitCondition);

    for (int row = 0; row < X.Rows(); ++row) {
      for (int col = 0; col < X.Cols(); ++col) {
        EXPECT_EQ(1.0, X.Value(row, col));
      }
    }
  }
}

TEST(ProblemTest, DISABLED_Linear) {
  frc::Problem problem;

  auto x = problem.DecisionVariable();
  x = 1.0;

  auto y = problem.DecisionVariable();
  y = 1.0;

  problem.Maximize(50 * x + 40 * y);

  problem.SubjectTo(x + 1.5 * y <= 750);
  problem.SubjectTo(2 * x + 3 * y <= 1500);
  problem.SubjectTo(2 * x + y <= 1000);
  problem.SubjectTo(x >= 0);
  problem.SubjectTo(y >= 0);

  frc::SolverConfig config;
  config.diagnostics = true;

  auto status = problem.Solve(config);
  EXPECT_EQ(frc::autodiff::ExpressionType::kLinear, status.costFunctionType);
  EXPECT_EQ(frc::autodiff::ExpressionType::kLinear,
            status.equalityConstraintType);
  EXPECT_EQ(frc::autodiff::ExpressionType::kLinear,
            status.inequalityConstraintType);
  EXPECT_EQ(frc::SolverExitCondition::kOk, status.exitCondition);

  EXPECT_NEAR(375.0, x.Value(0), 1e-6);
  EXPECT_NEAR(250.0, y.Value(0), 1e-6);
}

TEST(ProblemTest, QuadraticUnconstrained1) {
  frc::Problem problem;

  auto x = problem.DecisionVariable();
  x = 2.0;

  problem.Minimize(x * x - 6.0 * x);

  frc::SolverConfig config;
  config.diagnostics = true;

  auto status = problem.Solve(config);
  EXPECT_EQ(frc::autodiff::ExpressionType::kQuadratic, status.costFunctionType);
  EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
            status.equalityConstraintType);
  EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
            status.inequalityConstraintType);
  EXPECT_EQ(frc::SolverExitCondition::kOk, status.exitCondition);

  EXPECT_NEAR(3.0, x.Value(0), 1e-6);
}

TEST(ProblemTest, QuadraticUnconstrained2) {
  {
    frc::Problem problem;

    auto x = problem.DecisionVariable();
    x = 1.0;
    auto y = problem.DecisionVariable();
    y = 2.0;

    problem.Minimize(x * x + y * y);

    frc::SolverConfig config;
    config.diagnostics = true;

    auto status = problem.Solve(config);
    EXPECT_EQ(frc::autodiff::ExpressionType::kQuadratic,
              status.costFunctionType);
    EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
              status.equalityConstraintType);
    EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
              status.inequalityConstraintType);
    EXPECT_EQ(frc::SolverExitCondition::kOk, status.exitCondition);

    EXPECT_NEAR(0.0, x.Value(0), 1e-6);
    EXPECT_NEAR(0.0, y.Value(0), 1e-6);
  }

  {
    frc::Problem problem;

    auto x = problem.DecisionVariable(2);
    x(0) = 1.0;
    x(1) = 2.0;

    problem.Minimize(x.Transpose() * x);

    frc::SolverConfig config;
    config.diagnostics = true;

    auto status = problem.Solve(config);
    EXPECT_EQ(frc::autodiff::ExpressionType::kQuadratic,
              status.costFunctionType);
    EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
              status.equalityConstraintType);
    EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
              status.inequalityConstraintType);
    EXPECT_EQ(frc::SolverExitCondition::kOk, status.exitCondition);

    EXPECT_NEAR(0.0, x.Value(0), 1e-6);
    EXPECT_NEAR(0.0, x.Value(1), 1e-6);
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

    problem.Maximize(x * y);

    problem.SubjectTo(x + 3 * y == 36);

    frc::SolverConfig config;
    config.diagnostics = true;

    auto status = problem.Solve(config);
    EXPECT_EQ(frc::autodiff::ExpressionType::kQuadratic,
              status.costFunctionType);
    EXPECT_EQ(frc::autodiff::ExpressionType::kLinear,
              status.equalityConstraintType);
    EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
              status.inequalityConstraintType);
    EXPECT_EQ(frc::SolverExitCondition::kOk, status.exitCondition);

    EXPECT_NEAR(18.0, x.Value(0), 1e-5);
    EXPECT_NEAR(6.0, y.Value(0), 1e-5);
  }

  {
    frc::Problem problem;

    auto x = problem.DecisionVariable(2);
    x(0) = 1.0;
    x(1) = 2.0;

    problem.Minimize(x.Transpose() * x);

    problem.SubjectTo(x == frc::Matrixd<2, 1>{{3.0, 3.0}});

    frc::SolverConfig config;
    config.diagnostics = true;

    auto status = problem.Solve(config);
    EXPECT_EQ(frc::autodiff::ExpressionType::kQuadratic,
              status.costFunctionType);
    EXPECT_EQ(frc::autodiff::ExpressionType::kLinear,
              status.equalityConstraintType);
    EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
              status.inequalityConstraintType);
    EXPECT_EQ(frc::SolverExitCondition::kOk, status.exitCondition);

    EXPECT_NEAR(3.0, x.Value(0), 1e-5);
    EXPECT_NEAR(3.0, x.Value(1), 1e-5);
  }
}

TEST(ProblemTest, Nonlinear) {
  frc::Problem problem;

  auto x = problem.DecisionVariable();
  x = 20.0;

  problem.Minimize(frc::pow(x, 4));

  problem.SubjectTo(x >= 1);

  frc::SolverConfig config;
  config.diagnostics = true;

  auto status = problem.Solve(config);
  EXPECT_EQ(frc::autodiff::ExpressionType::kNonlinear, status.costFunctionType);
  EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
            status.equalityConstraintType);
  EXPECT_EQ(frc::autodiff::ExpressionType::kLinear,
            status.inequalityConstraintType);
  EXPECT_EQ(frc::SolverExitCondition::kOk, status.exitCondition);

  EXPECT_NEAR(1.0, x.Value(0), 1e-6);
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

      frc::SolverConfig config;
      config.diagnostics = true;

      auto status = problem.Solve(config);
      EXPECT_EQ(frc::autodiff::ExpressionType::kNonlinear,
                status.costFunctionType);
      EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
                status.equalityConstraintType);
      EXPECT_EQ(frc::autodiff::ExpressionType::kNonlinear,
                status.inequalityConstraintType);
      EXPECT_EQ(frc::SolverExitCondition::kOk, status.exitCondition);

      EXPECT_NEAR(1.0, x.Value(0), 1e-6);
      EXPECT_NEAR(1.0, y.Value(0), 1e-6);
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

      frc::SolverConfig config;
      config.diagnostics = true;

      auto status = problem.Solve(config);
      EXPECT_EQ(frc::autodiff::ExpressionType::kNonlinear,
                status.costFunctionType);
      EXPECT_EQ(frc::autodiff::ExpressionType::kNone,
                status.equalityConstraintType);
      EXPECT_EQ(frc::autodiff::ExpressionType::kQuadratic,
                status.inequalityConstraintType);
      EXPECT_EQ(frc::SolverExitCondition::kOk, status.exitCondition);

      EXPECT_NEAR(1.0, x.Value(0), 1e-6);
      EXPECT_NEAR(1.0, y.Value(0), 1e-6);
    }
  }
}

TEST(ProblemTest, DoubleIntegratorMinimumTime) {
  auto start = std::chrono::system_clock::now();

  constexpr auto T = 3.5_s;
  constexpr auto dt = 5_ms;
  constexpr int N = T / dt;

  constexpr double r = 2.0;

  frc::Problem problem;

  // 2x1 state vector with N + 1 timesteps (includes last state)
  auto X = problem.DecisionVariable(2, N + 1);

  // 1x1 input vector with N timesteps (input at last state doesn't matter)
  auto U = problem.DecisionVariable(1, N);

  // Kinematics constraint assuming constant acceleration between timesteps
  for (int k = 0; k < N; ++k) {
    constexpr double t = dt.value();
    auto p_k1 = X(0, k + 1);
    auto v_k1 = X(1, k + 1);
    auto p_k = X(0, k);
    auto v_k = X(1, k);
    auto a_k = U(0, k);

    // pₖ₊₁ = pₖ + vₖt
    problem.SubjectTo(p_k1 == p_k + v_k * t);

    // vₖ₊₁ = vₖ + aₖt
    problem.SubjectTo(v_k1 == v_k + a_k * t);
  }

  // Start and end at rest
  problem.SubjectTo(X.Col(0) == frc::Matrixd<2, 1>{{0.0}, {0.0}});
  problem.SubjectTo(X.Col(N) == frc::Matrixd<2, 1>{{r}, {0.0}});

  // Limit velocity
  problem.SubjectTo(-1 <= X.Row(1));
  problem.SubjectTo(X.Row(1) <= 1);

  // Limit acceleration
  problem.SubjectTo(-1 <= U);
  problem.SubjectTo(U <= 1);

  // Cost function - minimize position error
  frc::VariableMatrix J = 0.0;
  for (int k = 0; k < N + 1; ++k) {
    J += frc::pow(r - X(0, k), 2);
  }
  problem.Minimize(J);

  auto end1 = std::chrono::system_clock::now();
  using std::chrono::duration_cast;
  using std::chrono::microseconds;
  fmt::print("Setup time: {} ms\n\n",
             duration_cast<microseconds>(end1 - start).count() / 1000.0);

  frc::SolverConfig config;
  config.diagnostics = true;

  auto status = problem.Solve(config);
  EXPECT_EQ(frc::autodiff::ExpressionType::kQuadratic, status.costFunctionType);
  EXPECT_EQ(frc::autodiff::ExpressionType::kLinear,
            status.equalityConstraintType);
  EXPECT_EQ(frc::autodiff::ExpressionType::kLinear,
            status.inequalityConstraintType);
  EXPECT_EQ(frc::SolverExitCondition::kOk, status.exitCondition);

  // TODO: Verify solution
#if 0
  std::ofstream states{"Double integrator states.csv"};
  if (states.is_open()) {
    states << "Time (s),Position (m),Velocity (m/s)\n";

    for (int k = 0; k < N + 1; ++k) {
      states << fmt::format("{},{},{}\n", k * dt.value(), X.Value(0, k),
                            X.Value(1, k));
    }
  }

  std::ofstream inputs{"Double integrator inputs.csv"};
  if (inputs.is_open()) {
    inputs << "Time (s),Acceleration (m/s²)\n";

    for (int k = 0; k < N + 1; ++k) {
      if (k < N) {
        inputs << fmt::format("{},{}\n", k * dt.value(), U.Value(0, k));
      } else {
        inputs << fmt::format("{},{}\n", k * dt.value(), 0.0);
      }
    }
  }
#endif
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
  using std::chrono::duration_cast;
  using std::chrono::microseconds;
  fmt::print("Setup time: {} ms\n\n",
             duration_cast<microseconds>(end1 - start).count() / 1000.0);

  frc::SolverConfig config;
  config.diagnostics = true;

  auto status = problem.Solve(config);
  EXPECT_EQ(frc::autodiff::ExpressionType::kQuadratic, status.costFunctionType);
  EXPECT_EQ(frc::autodiff::ExpressionType::kLinear,
            status.equalityConstraintType);
  EXPECT_EQ(frc::autodiff::ExpressionType::kLinear,
            status.inequalityConstraintType);
  EXPECT_EQ(frc::SolverExitCondition::kOk, status.exitCondition);

  // Voltage for steady-state velocity:
  //
  // rₖ₊₁ = Arₖ + Buₖ
  // uₖ = B⁺(rₖ₊₁ − Arₖ)
  // uₖ = B⁺(rₖ − Arₖ)
  // uₖ = B⁺(I − A)rₖ
  frc::Matrixd<1, 1> u_ss =
      B.householderQr().solve(decltype(A)::Identity() - A) * r;

  frc::Matrixd<1, 1> x{{0.0}};
  for (int k = 0; k < N; ++k) {
    // Verify state
    EXPECT_NEAR(x(0), X.Value(0, k), 1e-2);

    double error = r(0) - x(0);
    if (error > 1e-2) {
      // Max control input until the reference is reached
      EXPECT_NEAR(12.0, U.Value(0, k), 1e-2);

      // Project state forward
      x = A * x + B * 12.0;
    } else {
      // If control input isn't at transition value
      if (std::abs(U.Value(0, k) - 10.7065) > 1e-2) {
        // Control input that maintains steady-state velocity
        EXPECT_NEAR(u_ss(0), U.Value(0, k), 1e-2);
      }

      // Project state forward
      x = A * x + B * u_ss;
    }
  }
}
