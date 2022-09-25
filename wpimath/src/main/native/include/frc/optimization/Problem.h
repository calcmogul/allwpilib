// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>
#include <utility>
#include <vector>

#include <wpi/SymbolExports.h>

#include "Eigen/Core"
#include "frc/autodiff/Variable.h"
#include "frc/optimization/Constraints.h"
#include "frc/optimization/SolverConfig.h"
#include "frc/optimization/SolverStatus.h"
#include "frc/optimization/VariableMatrix.h"

namespace frc {

/**
 * Allows the user to pose a constrained nonlinear optimization problem in
 * natural mathematical notation and solve it.
 *
 * To motivate this class, we'll use the hypothetical problem of making a double
 * integrator (a system with position and velocity states and an acceleration
 * input) move from x=0 to x=10 in the minimum time with some velocity and
 * acceleration limits.
 *
 * This class supports problems of the
 * form:
 * @verbatim
 *       minₓ f(x)
 * subject to cᵢ(x) ≥ 0
 *            cₑ(x) = 0
 * @endverbatim
 *
 * where f(x) is the scalar cost function, x is the vector of decision variables
 * (variables the solver can tweak to minimize the cost function), cᵢ(x) are the
 * inequality constraints, and cₑ(x) are the equality constraints. Constraints
 * are equations or inequalities of the decision variables that constrain what
 * values the solver is allowed to use when searching for an optimal solution.
 *
 * The nice thing about this class is users don't have to put their system in
 * the form shown above manually; they can write it in natural mathematical form
 * and it'll be converted for them.
 *
 * The model for our double integrator is ẍ=u where x is the vector [position;
 * velocity] and u is the scalar acceleration. We want to go from 0 m at rest to
 * 10 m at rest while obeying the velocity limit -1 ≤ x(1) ≤ 1 and the
 * acceleration limit -1 ≤ u ≤ 1.
 *
 * First, we need to make decision variables for our state and input.
 * @code{.cpp}
 * #include <frc/EigenCore.h>
 * #include <frc/optimization/Problem.h>
 * #include <units/time.h>
 *
 * constexpr auto T = 5_s;
 * constexpr auto dt = 5_ms;
 * constexpr int N = T / dt;
 *
 * frc::Problem problem;
 *
 * // 2x1 state vector with N + 1 timesteps (includes last state)
 * auto X = problem.DecisionVariable(2, N + 1);
 *
 * // 1x1 input vector with N timesteps (input at last state doesn't matter)
 * auto U = problem.DecisionVariable(1, N);
 * @endcode
 * By convention, we use capital letters for the variables to designate
 * matrices.
 *
 * Now, we need to apply dynamics constraints between timesteps.
 * @code{.cpp}
 * // Kinematics constraint assuming constant acceleration between timesteps
 * for (int k = 0; k < N; ++k) {
 *   constexpr double t = dt.value();
 *   auto p_k1 = X(0, k + 1);
 *   auto v_k1 = X(1, k + 1);
 *   auto p_k = X(0, k);
 *   auto v_k = X(1, k);
 *   auto a_k = U(0, k);
 *
 *   // pₖ₊₁ = pₖ + vₖt
 *   problem.SubjectTo(p_k1 == p_k + v_k * t);
 *
 *   // vₖ₊₁ = vₖ + aₖt
 *   problem.SubjectTo(v_k1 == v_k + a_k * t);
 * }
 * @endcode
 *
 * Next, we'll apply the state and input constraints.
 * @code{.cpp}
 * // Start and end at rest
 * problem.SubjectTo(X.Col(0) == frc::Matrixd<2, 1>{{0.0}, {0.0}});
 * problem.SubjectTo(X.Col(N + 1) == frc::Matrixd<2, 1>{{10.0}, {0.0}});
 *
 * // Limit velocity
 * problem.SubjectTo(-1 <= X.Row(1));
 * problem.SubjectTo(X.Row(1) <= 1);
 *
 * // Limit acceleration
 * problem.SubjectTo(-1 <= U);
 * problem.SubjectTo(U <= 1);
 * @endcode
 *
 * Next, we'll create a cost function for minimizing position error.
 * @code{.cpp}
 * // Cost function - minimize position error
 * frc::VariableMatrix J = 0.0;
 * for (int k = 0; k < N + 1; ++k) {
 *   J += frc::pow(10.0 - X(0, k), 2);
 * }
 * problem.Minimize(J);
 * @endcode
 * The cost function passed to Minimize() should produce a scalar output.
 *
 * Now we can solve the problem.
 * @code{.cpp}
 * problem.Solve();
 * @endcode
 *
 * The solver will find the decision variable values that minimize the cost
 * function while obeying the constraints. You can obtain the solution by
 * querying the values of the variables like so.
 * @code{.cpp}
 * double input = U.Value();
 * @endcode
 *
 * In retrospect, the solution here seems obvious: if you want to reach the
 * desired position in minimal time, you just apply max input to move toward it,
 * then stop applying input once you get there. Problems can get more complex
 * than this though. In fact, we can use this same framework to design optimal
 * trajectories for a drivetrain while obeying dynamics constraints, avoiding
 * obstacles, and driving through points of interest.
 */
class WPILIB_DLLEXPORT Problem {
 public:
  /**
   * Construct the optimization problem.
   */
  Problem() noexcept;

  /**
   * Create a matrix of decision variables in the optimization problem.
   *
   * @param rows Number of matrix rows.
   * @param cols Number of matrix columns.
   */
  VariableMatrix DecisionVariable(int rows = 1, int cols = 1);

  /**
   * Tells the solver to minimize the output of the given cost function.
   *
   * Note that this is optional. If only constraints are specified, the solver
   * will find the closest solution to the initial conditions that's in the
   * feasible set.
   *
   * @param cost The cost function to minimize. It must return a 1x1 matrix.
   */
  void Minimize(const VariableMatrix& cost);

  /**
   * Tells the solver to minimize the output of the given cost function.
   *
   * Note that this is optional. If only constraints are specified, the solver
   * will find the closest solution to the initial conditions that's in the
   * feasible set.
   *
   * @param cost The cost function to minimize. It must return a 1x1 matrix.
   */
  void Minimize(VariableMatrix&& cost);

  /**
   * Tells the solver to maximize the output of the given objective function.
   *
   * Note that this is optional. If only constraints are specified, the solver
   * will find the closest solution to the initial conditions that's in the
   * feasible set.
   *
   * @param objective The objective function to minimize. It must return a 1x1
   *                  matrix.
   */
  void Maximize(const VariableMatrix& objective);

  /**
   * Tells the solver to maximize the output of the given objective function.
   *
   * Note that this is optional. If only constraints are specified, the solver
   * will find the closest solution to the initial conditions that's in the
   * feasible set.
   *
   * @param objective The objective function to maximize. It must return a 1x1
   *                  matrix.
   */
  void Maximize(VariableMatrix&& objective);

  /**
   * Tells the solver to solve the problem while obeying the given equality
   * constraint.
   *
   * @param constraint The constraint to satisfy.
   */
  void SubjectTo(EqualityConstraints&& constraint);

  /**
   * Tells the solver to solve the problem while obeying the given inequality
   * constraint.
   *
   * @param constraint The constraint to satisfy.
   */
  void SubjectTo(InequalityConstraints&& constraint);

  /**
   * Solve the optimization problem. The solution will be stored in the original
   * variables used to construct the problem.
   *
   * @param config Configuration options for the solver.
   */
  SolverStatus Solve(const SolverConfig& config = SolverConfig{});

 private:
  // Decision variables, which are the root of the problem's expression tree
  std::vector<autodiff::Variable> m_decisionVariables;

  // Cost function: f(x)
  std::optional<autodiff::Variable> m_f;

  // Equality constraints: cₑ(x) = 0
  std::vector<autodiff::Variable> m_equalityConstraints;

  // Inequality constraints: cᵢ(x) ≥ 0
  std::vector<autodiff::Variable> m_inequalityConstraints;

  SolverConfig m_config;

  /**
   * Assigns the contents of a double vector to an autodiff vector.
   *
   * @param dest The autodiff vector.
   * @param src The double vector.
   */
  static void SetAD(std::vector<autodiff::Variable>& dest,
                    const Eigen::Ref<const Eigen::VectorXd>& src);

  /**
   * Assigns the contents of a double vector to an autodiff vector.
   *
   * @param dest The autodiff vector.
   * @param src The double vector.
   */
  static void SetAD(Eigen::Ref<autodiff::VectorXvar> dest,
                    const Eigen::Ref<const Eigen::VectorXd>& src);

  /**
   * Applies fraction-to-the-boundary rule to a variable and its iterate, then
   * returns a fraction of the iterate step size within (0, 1].
   *
   * @param x The variable.
   * @param p The iterate on the variable.
   * @param tau Fraction-to-the-boundary rule scaling factor.
   * @return Fraction of the iterate step size within (0, 1].
   */
  static double FractionToTheBoundaryRule(
      const Eigen::Ref<const Eigen::VectorXd>& x,
      const Eigen::Ref<const Eigen::VectorXd>& p, double tau);

  /**
  Find the optimal solution to the nonlinear program using an interior-point
  solver.

  A nonlinear program has the form:

  @verbatim
       min_x f(x)
  subject to cₑ(x) = 0
             cᵢ(x) ≥ 0
  @endverbatim

  where f(x) is the cost function, cₑ(x) are the equality constraints, and cᵢ(x)
  are the inequality constraints.

  @param[in] initialGuess The initial guess.
  @param[out] status The solver status.
  */
  Eigen::VectorXd InteriorPoint(
      const Eigen::Ref<const Eigen::VectorXd>& initialGuess,
      SolverStatus* status);
};

}  // namespace frc
