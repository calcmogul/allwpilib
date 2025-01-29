// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <gtest/gtest.h>
#include <sleipnir/optimization/optimization_problem.hpp>

TEST(SleipnirTest, Quartic) {
  sleipnir::OptimizationProblem problem;

  auto x = problem.decision_variable();
  x.set_value(20.0);

  problem.minimize(sleipnir::pow(x, 4));

  problem.subject_to(x >= 1);

  auto status = problem.solve({.diagnostics = true});

  EXPECT_EQ(status.cost_function_type, sleipnir::ExpressionType::NONLINEAR);
  EXPECT_EQ(status.equality_constraint_type, sleipnir::ExpressionType::NONE);
  EXPECT_EQ(status.inequality_constraint_type,
            sleipnir::ExpressionType::LINEAR);
  EXPECT_EQ(status.exit_condition, sleipnir::SolverExitCondition::SUCCESS);

  EXPECT_NEAR(x.value(), 1.0, 1e-6);
}
