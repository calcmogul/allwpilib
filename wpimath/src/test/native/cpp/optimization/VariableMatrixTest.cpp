// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/EigenCore.h"
#include "frc/autodiff/Hessian.h"
#include "frc/optimization/Problem.h"
#include "frc/optimization/VariableMatrix.h"
#include "gtest/gtest.h"

TEST(VariableMatrixTest, ScalarInitAssign) {
  frc::Problem problem;

  // Scalar zero init
  auto x = problem.DecisionVariable();
  EXPECT_DOUBLE_EQ(0.0, x.Value(0));

  // Scalar assignment
  x = 1.0;
  EXPECT_DOUBLE_EQ(1.0, x.Value(0));
  x = 2.0;
  EXPECT_DOUBLE_EQ(2.0, x.Value(0));
}

TEST(VariableMatrixTest, VectorInitAssign) {
  frc::Problem problem;

  // Vector zero init
  auto y = problem.DecisionVariable(2);
  EXPECT_DOUBLE_EQ(0.0, y.Value(0));
  EXPECT_DOUBLE_EQ(0.0, y.Value(1));

  // Vector assignment
  y(0) = 1.0;
  y(1) = 2.0;
  EXPECT_DOUBLE_EQ(1.0, y.Value(0));
  EXPECT_DOUBLE_EQ(2.0, y.Value(1));
  y(0) = 3.0;
  y(1) = 4.0;
  EXPECT_DOUBLE_EQ(3.0, y.Value(0));
  EXPECT_DOUBLE_EQ(4.0, y.Value(1));
}

TEST(VariableMatrixTest, MatrixInitAssign) {
  frc::Problem problem;

  // Matrix zero init
  auto z = problem.DecisionVariable(3, 2);
  EXPECT_DOUBLE_EQ(0.0, z.Value(0, 0));
  EXPECT_DOUBLE_EQ(0.0, z.Value(0, 1));
  EXPECT_DOUBLE_EQ(0.0, z.Value(1, 0));
  EXPECT_DOUBLE_EQ(0.0, z.Value(1, 1));
  EXPECT_DOUBLE_EQ(0.0, z.Value(2, 0));
  EXPECT_DOUBLE_EQ(0.0, z.Value(2, 1));

  // Matrix assignment; element comparison
  z = frc::Matrixd<3, 2>{{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}};
  EXPECT_DOUBLE_EQ(1.0, z.Value(0, 0));
  EXPECT_DOUBLE_EQ(2.0, z.Value(0, 1));
  EXPECT_DOUBLE_EQ(3.0, z.Value(1, 0));
  EXPECT_DOUBLE_EQ(4.0, z.Value(1, 1));
  EXPECT_DOUBLE_EQ(5.0, z.Value(2, 0));
  EXPECT_DOUBLE_EQ(6.0, z.Value(2, 1));

  // Matrix assignment; matrix comparison
  {
    frc::Matrixd<3, 2> expected{{7.0, 8.0}, {9.0, 10.0}, {11.0, 12.0}};
    z = expected;
    EXPECT_EQ(expected, z.Value());
  }

  // Block assignment
  {
    frc::Problem problem;

    frc::Matrixd<3, 2> expected{{7.0, 8.0}, {9.0, 10.0}, {11.0, 12.0}};

    frc::Matrixd<2, 1> expectedBlock{{1.0}, {1.0}};
    z.Block(0, 0, 2, 1) = expectedBlock;
    expected.block<2, 1>(0, 0) = expectedBlock;

    EXPECT_EQ(expected, z.Value());

    frc::Matrixd<3, 2> expectedResult{{1.0, 8.0}, {1.0, 10.0}, {11.0, 12.0}};
    EXPECT_EQ(expectedResult, z.Value());
  }
}

TEST(VariableMatrixTest, HessianSumOfSquares) {
  frc::autodiff::VectorXvar r{{25.0, 10.0, 5.0, 0.0}};
  frc::autodiff::VectorXvar x{{0.0, 0.0, 0.0, 0.0}};

  frc::VariableMatrix J = 0.0;
  for (int i = 0; i < 4; ++i) {
    J += (r(i) - x(i)) * (r(i) - x(i));
  }

  Eigen::MatrixXd H = frc::autodiff::Hessian{J.Autodiff(0, 0), x}.Calculate();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      if (row == col) {
        EXPECT_EQ(2.0, H(row, col));
      } else {
        EXPECT_EQ(0.0, H(row, col));
      }
    }
  }
}
