// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <wpi/numbers>

#include "frc/autodiff/Variable.h"
#include "gtest/gtest.h"

TEST(VariableTest, Gradient) {
  frc::autodiff::Variable a = 10;
  frc::autodiff::Variable b = 20;
  frc::autodiff::Variable c = a;
  frc::autodiff::Variable x;
  frc::autodiff::Variable y;
  frc::autodiff::Variable r;

  //------------------------------------------------------------------------------
  // TEST TRIVIAL DERIVATIVE CALCULATIONS
  //------------------------------------------------------------------------------
  EXPECT_DOUBLE_EQ(1, Gradient(a, a));
  EXPECT_DOUBLE_EQ(0, Gradient(a, b));
  EXPECT_DOUBLE_EQ(1, Gradient(c, c));
  EXPECT_DOUBLE_EQ(1, Gradient(c, a));
  EXPECT_DOUBLE_EQ(0, Gradient(c, b));

  //------------------------------------------------------------------------------
  // TEST POSITIVE OPERATOR
  //------------------------------------------------------------------------------
  c = +a;

  EXPECT_DOUBLE_EQ(a.Value(), c.Value());
  EXPECT_DOUBLE_EQ(1.0, Gradient(c, a));

  //------------------------------------------------------------------------------
  // TEST NEGATIVE OPERATOR
  //------------------------------------------------------------------------------
  c = -a;

  EXPECT_DOUBLE_EQ(-a.Value(), c.Value());
  EXPECT_DOUBLE_EQ(-1.0, Gradient(c, a));

  //------------------------------------------------------------------------------
  // TEST WHEN IDENTICAL/EQUIVALENT VARIABLES ARE PRESENT
  //------------------------------------------------------------------------------
  x = a;
  c = a * a + x;

  EXPECT_DOUBLE_EQ(a.Value() * a.Value() + x.Value(), c.Value());
  EXPECT_DOUBLE_EQ(2 * a.Value() + Gradient(x, a), Gradient(c, a));
  EXPECT_DOUBLE_EQ(2 * a.Value() * Gradient(a, x) + 1, Gradient(c, x));

  //------------------------------------------------------------------------------
  // TEST DERIVATIVES COMPUTATION AFTER CHANGING VAR VALUE
  //------------------------------------------------------------------------------
  a = frc::autodiff::Variable{20.0};  // a is now a new independent variable

  EXPECT_DOUBLE_EQ(0.0, Gradient(c, a));
  EXPECT_DOUBLE_EQ(2 * x.Value() + 1, Gradient(c, x));

  //------------------------------------------------------------------------------
  // TEST MULTIPLICATION OPERATOR (USING CONSTANT FACTOR)
  //------------------------------------------------------------------------------
  c = -2 * a;

  EXPECT_DOUBLE_EQ(-2, Gradient(c, a));

  //------------------------------------------------------------------------------
  // TEST DIVISION OPERATOR (USING CONSTANT FACTOR)
  //------------------------------------------------------------------------------
  c = a / 3.0;

  EXPECT_DOUBLE_EQ(1.0 / 3.0, Gradient(c, a));

  //------------------------------------------------------------------------------
  // TEST DERIVATIVES WITH RESPECT TO DEPENDENT VARIABLES USING += -= *= /=
  //------------------------------------------------------------------------------

  a += 2.0;
  c = a * b;

  EXPECT_EQ(b, Gradient(c, a));

  a -= 3.0;
  c = a * b;

  EXPECT_EQ(b, Gradient(c, a));

  a *= 2.0;
  c = a * b;

  EXPECT_EQ(b, Gradient(c, a));

  a /= 3.0;
  c = a * b;

  EXPECT_EQ(b, Gradient(c, a));

  a += 2 * b;
  c = a * b;

  EXPECT_EQ(b + a * Gradient(b, a), Gradient(c, a));

  a -= 3 * b;
  c = a * b;

  EXPECT_EQ(b, Gradient(c, a));

  a *= b;
  c = a * b;

  EXPECT_EQ(b, Gradient(c, a));

  a /= b;
  c = a * b;

  EXPECT_EQ(b, Gradient(c, a));

  //------------------------------------------------------------------------------
  // TEST BINARY ARITHMETIC OPERATORS
  //------------------------------------------------------------------------------
  a = frc::autodiff::Variable{100.0};
  b = frc::autodiff::Variable{200.0};

  c = a + b;

  EXPECT_DOUBLE_EQ(1.0, Gradient(c, a));
  EXPECT_DOUBLE_EQ(1.0, Gradient(c, b));

  c = a - b;

  EXPECT_DOUBLE_EQ(1.0, Gradient(c, a));
  EXPECT_DOUBLE_EQ(-1.0, Gradient(c, b));

  c = -a + b;

  EXPECT_DOUBLE_EQ(-1.0, Gradient(c, a));
  EXPECT_DOUBLE_EQ(1.0, Gradient(c, b));

  c = a + 1;

  EXPECT_DOUBLE_EQ(1.0, Gradient(c, a));

  //------------------------------------------------------------------------------
  // TEST DERIVATIVES WITH RESPECT TO SUB-EXPRESSIONS
  //------------------------------------------------------------------------------
  x = 2 * a + b;
  r = x * x - a + b;

  EXPECT_EQ((2 * x).Value(), Gradient(r, x));
  EXPECT_EQ((2 * x * Gradient(x, a) - 1.0).Value(), Gradient(r, a));
  EXPECT_EQ((2 * x * Gradient(x, b) + 1.0).Value(), Gradient(r, b));

  //------------------------------------------------------------------------------
  // TEST COMPARISON OPERATORS
  //------------------------------------------------------------------------------
  a = 10;
  x = 10;

  EXPECT_EQ(a, a);
  EXPECT_EQ(a, x);
  EXPECT_EQ(a, 10);
  EXPECT_EQ(10, a);

  EXPECT_NE(a, b);
  EXPECT_NE(a, 20);
  EXPECT_NE(20, a);

  EXPECT_LT(a, b);
  EXPECT_LT(a, 20);

  EXPECT_GT(b, a);
  EXPECT_GT(20, a);

  EXPECT_LE(a, a);
  EXPECT_LE(a, x);
  EXPECT_LE(a, b);
  EXPECT_LE(a, 10);
  EXPECT_LE(a, 20);

  EXPECT_GE(a, a);
  EXPECT_GE(x, a);
  EXPECT_GE(b, a);
  EXPECT_GE(10, a);
  EXPECT_GE(20, a);

  //------------------------------------------------------------------------------
  // TEST COMPARISON OPERATORS BETWEEN VARIABLE AND EXPRPTR
  //------------------------------------------------------------------------------
  EXPECT_EQ(a, a / a * a);
  EXPECT_EQ(a / a * a, a);

  EXPECT_NE(a, a - a);
  EXPECT_NE(a - a, a);

  EXPECT_LT(a - a, a);
  EXPECT_LT(a, a + a);

  EXPECT_GT(a + a, a);
  EXPECT_GT(a, a - a);

  EXPECT_LE(a, a - a + a);
  EXPECT_LE(a - a + a, a);

  EXPECT_LE(a, a + a);
  EXPECT_LE(a - a, a);

  EXPECT_GE(a, a - a + a);
  EXPECT_GE(a - a + a, a);

  EXPECT_GE(a + a, a);
  EXPECT_GE(a, a - a);

  //--------------------------------------------------------------------------
  // TEST TRIGONOMETRIC FUNCTIONS
  //--------------------------------------------------------------------------
  x = 0.5;

  EXPECT_DOUBLE_EQ(std::sin(x.Value()), frc::autodiff::sin(x).Value());
  EXPECT_DOUBLE_EQ(std::cos(x.Value()), Gradient(frc::autodiff::sin(x), x));

  EXPECT_DOUBLE_EQ(std::cos(x.Value()), frc::autodiff::cos(x).Value());
  EXPECT_DOUBLE_EQ(-std::sin(x.Value()), Gradient(frc::autodiff::cos(x), x));

  EXPECT_DOUBLE_EQ(std::tan(x.Value()), frc::autodiff::tan(x).Value());
  EXPECT_DOUBLE_EQ(1.0 / (std::cos(x.Value()) * std::cos(x.Value())),
                   Gradient(frc::autodiff::tan(x), x));

  EXPECT_DOUBLE_EQ(std::asin(x.Value()), frc::autodiff::asin(x).Value());
  EXPECT_DOUBLE_EQ(1.0 / std::sqrt(1 - x.Value() * x.Value()),
                   Gradient(frc::autodiff::asin(x), x));

  EXPECT_DOUBLE_EQ(std::acos(x.Value()), frc::autodiff::acos(x).Value());
  EXPECT_DOUBLE_EQ(-1.0 / std::sqrt(1 - x.Value() * x.Value()),
                   Gradient(frc::autodiff::acos(x), x));

  EXPECT_DOUBLE_EQ(std::atan(x.Value()), frc::autodiff::atan(x).Value());
  EXPECT_DOUBLE_EQ(1.0 / (1 + x.Value() * x.Value()),
                   Gradient(frc::autodiff::atan(x), x));

  //--------------------------------------------------------------------------
  // TEST HYPERBOLIC FUNCTIONS
  //--------------------------------------------------------------------------
  EXPECT_DOUBLE_EQ(std::sinh(x.Value()), frc::autodiff::sinh(x).Value());
  EXPECT_DOUBLE_EQ(std::cosh(x.Value()), Gradient(frc::autodiff::sinh(x), x));

  EXPECT_DOUBLE_EQ(std::cosh(x.Value()), frc::autodiff::cosh(x).Value());
  EXPECT_DOUBLE_EQ(std::sinh(x.Value()), Gradient(frc::autodiff::cosh(x), x));

  EXPECT_DOUBLE_EQ(std::tanh(x.Value()), frc::autodiff::tanh(x).Value());
  EXPECT_DOUBLE_EQ(1.0 / (std::cosh(x.Value()) * std::cosh(x.Value())),
                   Gradient(frc::autodiff::tanh(x), x));

  //--------------------------------------------------------------------------
  // TEST EXPONENTIAL AND LOGARITHMIC FUNCTIONS
  //--------------------------------------------------------------------------
  EXPECT_DOUBLE_EQ(std::log(x.Value()), frc::autodiff::log(x).Value());
  EXPECT_DOUBLE_EQ(1.0 / x.Value(), Gradient(frc::autodiff::log(x), x));

  EXPECT_DOUBLE_EQ(std::log10(x.Value()), frc::autodiff::log10(x).Value());
  EXPECT_DOUBLE_EQ(1.0 / (std::log(10) * x.Value()),
                   Gradient(frc::autodiff::log10(x), x));

  EXPECT_DOUBLE_EQ(std::exp(x.Value()), frc::autodiff::exp(x).Value());
  EXPECT_DOUBLE_EQ(std::exp(x.Value()), Gradient(frc::autodiff::exp(x), x));

  //--------------------------------------------------------------------------
  // TEST POWER FUNCTIONS
  //--------------------------------------------------------------------------
  EXPECT_DOUBLE_EQ(std::sqrt(x.Value()), frc::autodiff::sqrt(x).Value());
  EXPECT_DOUBLE_EQ(0.5 / std::sqrt(x.Value()),
                   Gradient(frc::autodiff::sqrt(x), x));

  EXPECT_DOUBLE_EQ(std::pow(x.Value(), 2.0),
                   frc::autodiff::pow(x, 2.0).Value());
  EXPECT_DOUBLE_EQ(2.0 * x.Value(), Gradient(frc::autodiff::pow(x, 2.0), x));

  EXPECT_DOUBLE_EQ(std::pow(2.0, x.Value()),
                   frc::autodiff::pow(2.0, x).Value());
  EXPECT_DOUBLE_EQ(std::log(2.0) * std::pow(2.0, x.Value()),
                   Gradient(frc::autodiff::pow(2.0, x), x));

  EXPECT_DOUBLE_EQ(std::pow(x.Value(), x.Value()),
                   frc::autodiff::pow(x, x).Value());
  EXPECT_DOUBLE_EQ(
      ((frc::autodiff::log(x) + 1) * frc::autodiff::pow(x, x)).Value(),
      Gradient(frc::autodiff::pow(x, x), x));

  y = 2 * a;

  EXPECT_EQ(2 * a.Value(), y);
  EXPECT_DOUBLE_EQ(2.0, Gradient(y, a));

  EXPECT_DOUBLE_EQ(std::pow(x.Value(), y.Value()),
                   frc::autodiff::pow(x, y).Value());
  EXPECT_DOUBLE_EQ(y.Value() / x.Value() * std::pow(x.Value(), y.Value()),
                   Gradient(frc::autodiff::pow(x, y), x));
  EXPECT_DOUBLE_EQ(
      std::pow(x.Value(), y.Value()) * (y.Value() / x.Value() * Gradient(x, a) +
                                        std::log(x.Value()) * Gradient(y, a)),
      Gradient(frc::autodiff::pow(x, y), a));
  EXPECT_DOUBLE_EQ(std::log(x.Value()) * std::pow(x.Value(), y.Value()),
                   Gradient(frc::autodiff::pow(x, y), y));

  //--------------------------------------------------------------------------
  // TEST ABS FUNCTION
  //--------------------------------------------------------------------------

  x = 1.0;
  EXPECT_DOUBLE_EQ(std::abs(x.Value()), frc::autodiff::abs(x).Value());
  EXPECT_DOUBLE_EQ(1.0, Gradient(frc::autodiff::abs(x), x));
  x = -1.0;
  EXPECT_DOUBLE_EQ(std::abs(x.Value()), frc::autodiff::abs(x).Value());
  EXPECT_DOUBLE_EQ(-1.0, Gradient(frc::autodiff::abs(x), x));
  x = 0.0;
  EXPECT_DOUBLE_EQ(std::abs(x.Value()), frc::autodiff::abs(x).Value());
  EXPECT_DOUBLE_EQ(0.0, Gradient(frc::autodiff::abs(x), x));

  //--------------------------------------------------------------------------
  // TEST ATAN2 FUNCTION
  //--------------------------------------------------------------------------

  // Testing atan2 function on (double, var)
  x = 1.0;
  EXPECT_EQ(frc::autodiff::atan2(2.0, x).Value(), std::atan2(2.0, x.Value()));
  EXPECT_EQ(Gradient(frc::autodiff::atan2(2.0, x), x),
            (-2.0 / (2 * 2 + x * x)).Value());

  // Testing atan2 function on (var, double)
  x = 1.0;
  EXPECT_EQ(frc::autodiff::atan2(x, 2.0), std::atan2(x.Value(), 2.0));
  EXPECT_EQ(Gradient(frc::autodiff::atan2(x, 2.0), x),
            (2.0 / (2 * 2 + x * x)).Value());

  // Testing atan2 function on (var, var)
  x = 1.1;
  y = 0.9;
  EXPECT_EQ(frc::autodiff::atan2(y, x), std::atan2(y.Value(), x.Value()));
  EXPECT_EQ(Gradient(frc::autodiff::atan2(y, x), y), x / (x * x + y * y));
  EXPECT_EQ(Gradient(frc::autodiff::atan2(y, x), x), -y / (x * x + y * y));

  // Testing atan2 function on (expr, expr)
  EXPECT_EQ(3 * frc::autodiff::atan2(frc::autodiff::sin(y), 2 * x + 1),
            3 * std::atan2(frc::autodiff::sin(y).Value(), 2 * x.Value() + 1));
  EXPECT_EQ(
      Gradient(3 * frc::autodiff::atan2(frc::autodiff::sin(y), 2 * x + 1), y),
      3 * (2 * x + 1) * frc::autodiff::cos(y) /
          ((2 * x + 1) * (2 * x + 1) +
           frc::autodiff::sin(y) * frc::autodiff::sin(y)));
  EXPECT_EQ(
      Gradient(3 * frc::autodiff::atan2(frc::autodiff::sin(y), 2 * x + 1), x),
      3 * -2 * frc::autodiff::sin(y) /
          ((2 * x + 1) * (2 * x + 1) +
           frc::autodiff::sin(y) * frc::autodiff::sin(y)));

  //--------------------------------------------------------------------------
  // TEST HYPOT2 FUNCTIONS
  //--------------------------------------------------------------------------

  // Testing hypot function on (var, double)
  x = 1.8;
  EXPECT_EQ(std::hypot(x.Value(), 2.0), frc::autodiff::hypot(x, 2.0));
  EXPECT_EQ(x / std::hypot(x.Value(), 2.0),
            Gradient(frc::autodiff::hypot(x, 2.0), x));

  // Testing hypot function on (double, var)
  y = 1.5;
  EXPECT_EQ(std::hypot(2.0, y.Value()), frc::autodiff::hypot(2.0, y));
  EXPECT_EQ(y / std::hypot(2.0, y.Value()),
            Gradient(frc::autodiff::hypot(2.0, y), y));

  // Testing hypot function on (var, var)
  x = 1.3;
  y = 2.3;
  EXPECT_EQ(std::hypot(x.Value(), y.Value()), frc::autodiff::hypot(x, y));
  EXPECT_EQ(x / std::hypot(x.Value(), y.Value()),
            Gradient(frc::autodiff::hypot(x, y), x));
  EXPECT_EQ(y / std::hypot(x.Value(), y.Value()),
            Gradient(frc::autodiff::hypot(x, y), y));

  // Testing hypot function on (expr, expr)
  x = 1.3;
  y = 2.3;
  EXPECT_EQ(std::hypot(2.0 * x.Value(), 3.0 * y.Value()),
            frc::autodiff::hypot(2.0 * x, 3.0 * y).Value());
  EXPECT_EQ(4.0 * x / std::hypot(2.0 * x.Value(), 3.0 * y.Value()),
            Gradient(frc::autodiff::hypot(2.0 * x, 3.0 * y), x));
  EXPECT_EQ(9.0 * y / std::hypot(2.0 * x.Value(), 3.0 * y.Value()),
            Gradient(frc::autodiff::hypot(2.0 * x, 3.0 * y), y));

  //--------------------------------------------------------------------------
  // TEST OTHER FUNCTIONS
  //--------------------------------------------------------------------------
  x = 3.0;
  y = x;

  EXPECT_DOUBLE_EQ(std::abs(x.Value()), frc::autodiff::abs(x).Value());
  EXPECT_DOUBLE_EQ(1.0, Gradient(x, x));

  x = 0.5;
  EXPECT_DOUBLE_EQ(std::erf(x.Value()), frc::autodiff::erf(x).Value());
  EXPECT_EQ(2 / frc::autodiff::sqrt(wpi::numbers::pi) *
                std::exp(-x.Value() * x.Value()),
            Gradient(frc::autodiff::erf(x), x));
}

TEST(VariableTest, GradientVarsAndConstants) {
  frc::autodiff::Variable x1 = 3;
  frc::autodiff::Variable y1 = 2 * x1;

  EXPECT_DOUBLE_EQ(1.0, x1.Gradient(0).Value());
  EXPECT_DOUBLE_EQ(0.0, x1.Gradient(1).Value());
  EXPECT_DOUBLE_EQ(3.0, y1.Gradient(0).Value());
  EXPECT_DOUBLE_EQ(2.0, y1.Gradient(1).Value());

  frc::autodiff::Variable x2 = 3;
  frc::autodiff::Variable y2 = 2 * x2;

  EXPECT_DOUBLE_EQ(1.0, x2.Gradient(0).Value());
  EXPECT_DOUBLE_EQ(0.0, x2.Gradient(1).Value());
  EXPECT_DOUBLE_EQ(3.0, y2.Gradient(0).Value());
  EXPECT_DOUBLE_EQ(2.0, y2.Gradient(1).Value());
}

TEST(VariableTest, Jacobian) {
  frc::autodiff::VectorXvar y{3, 1};
  Eigen::MatrixXd J;
  frc::autodiff::VectorXvar x{3};
  x << 1, 2, 3;

  // y = x
  //
  //         [1  0  0]
  // dy/dx = [0  1  0]
  //         [0  0  1]
  y = x;
  J = Jacobian(y, x);

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      if (row == col) {
        EXPECT_DOUBLE_EQ(1.0, J(row, col));
      } else {
        EXPECT_DOUBLE_EQ(0.0, J(row, col));
      }
    }
  }

  // y = 3x
  //
  //         [3  0  0]
  // dy/dx = [0  3  0]
  //         [0  0  3]
  y = 3 * x;
  J = Jacobian(y, x);

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      if (row == col) {
        EXPECT_DOUBLE_EQ(3.0, J(row, col));
      } else {
        EXPECT_DOUBLE_EQ(0.0, J(row, col));
      }
    }
  }

  //     [x₁x₂]
  // y = [x₂x₃]
  //     [x₁x₃]
  //
  //         [x₂  x₁  0 ]
  // dy/dx = [0   x₃  x₂]
  //         [x₃  0   x₁]
  //
  //         [2  1  0]
  // dy/dx = [0  3  2]
  //         [3  0  1]
  y(0) = x(0) * x(1);
  y(1) = x(1) * x(2);
  y(2) = x(0) * x(2);
  J = Jacobian(y, x);

  EXPECT_DOUBLE_EQ(2.0, J(0, 0));
  EXPECT_DOUBLE_EQ(1.0, J(0, 1));
  EXPECT_DOUBLE_EQ(0.0, J(0, 2));
  EXPECT_DOUBLE_EQ(0.0, J(1, 0));
  EXPECT_DOUBLE_EQ(3.0, J(1, 1));
  EXPECT_DOUBLE_EQ(2.0, J(1, 2));
  EXPECT_DOUBLE_EQ(3.0, J(2, 0));
  EXPECT_DOUBLE_EQ(0.0, J(2, 1));
  EXPECT_DOUBLE_EQ(1.0, J(2, 2));
}

TEST(VariableTest, HessianLinear) {
  // y = x
  frc::autodiff::VectorXvar x{1};
  x << 3;
  frc::autodiff::Variable y = x(0);

  // dy/dx = 1
  EXPECT_DOUBLE_EQ(1.0, y.Gradient(0).Value());
  EXPECT_DOUBLE_EQ(0.0, y.Gradient(1).Value());
  EXPECT_DOUBLE_EQ(1.0, Gradient(y, x(0)));

  // d²y/dx² = d/dx(x (rhs) + x (lhs))
  //         = 1 + 1
  //         = 2
  EXPECT_DOUBLE_EQ(0.0, y.Gradient(0).Gradient(0).Value());
  EXPECT_DOUBLE_EQ(0.0, y.Gradient(1).Gradient(0).Value());
  EXPECT_DOUBLE_EQ(0.0, y.Gradient(0).Gradient(1).Value());
  EXPECT_DOUBLE_EQ(0.0, y.Gradient(1).Gradient(1).Value());
  EXPECT_DOUBLE_EQ(0.0, y.Gradient(0).Gradient(0).Gradient(0).Value());
  EXPECT_DOUBLE_EQ(0.0, y.Gradient(1).Gradient(0).Gradient(0).Value());
  Eigen::MatrixXd H = Hessian(y, x);
  EXPECT_DOUBLE_EQ(0.0, H(0, 0));
}

TEST(VariableTest, HessianQuadratic) {
  // y = x²
  // y = x * x
  frc::autodiff::VectorXvar x{1};
  x << 3;
  frc::autodiff::Variable y = x(0) * x(0);

  // dy/dx = x (rhs) + x (lhs)
  //       = (3) + (3)
  //       = 6
  EXPECT_DOUBLE_EQ(3.0, y.Gradient(0).Value());
  EXPECT_DOUBLE_EQ(3.0, y.Gradient(1).Value());
  EXPECT_DOUBLE_EQ(6.0, Gradient(y, x(0)));

  // d²y/dx² = d/dx(x (rhs) + x (lhs))
  //         = 1 + 1
  //         = 2
  EXPECT_DOUBLE_EQ(1.0, y.Gradient(0).Gradient(0).Value());
  EXPECT_DOUBLE_EQ(1.0, y.Gradient(1).Gradient(0).Value());
  EXPECT_DOUBLE_EQ(0.0, y.Gradient(0).Gradient(1).Value());
  EXPECT_DOUBLE_EQ(0.0, y.Gradient(1).Gradient(1).Value());
  EXPECT_DOUBLE_EQ(0.0, y.Gradient(0).Gradient(0).Gradient(0).Value());
  EXPECT_DOUBLE_EQ(0.0, y.Gradient(1).Gradient(0).Gradient(0).Value());
  Eigen::MatrixXd H = Hessian(y, x);
  EXPECT_DOUBLE_EQ(2.0, H(0, 0));
}

TEST(VariableTest, Hessian) {
  frc::autodiff::Variable y;
  Eigen::VectorXd g;
  Eigen::MatrixXd H;
  frc::autodiff::VectorXvar x{5};
  x << 1, 2, 3, 4, 5;

  //--------------------------------------------------------------------------
  // TESTING GRADIENT AND HESSIAN WHEN y = sum(x)
  //--------------------------------------------------------------------------
  y = x.sum();
  g = Gradient(y, x);

  EXPECT_DOUBLE_EQ(15.0, y.Value());
  for (int i = 0; i < x.rows(); ++i) {
    EXPECT_DOUBLE_EQ(1.0, g(i));
  }

  H = Hessian(y, x);
  for (int i = 0; i < x.rows(); ++i) {
    for (int j = 0; j < x.rows(); ++j) {
      EXPECT_DOUBLE_EQ(0.0, H(i, j));
    }
  }

  //--------------------------------------------------------------------------
  // TESTING GRADIENT AND HESSIAN WHEN y = ||x||^2
  //--------------------------------------------------------------------------
  x << 1, 2, 3, 4, 5;
  y = x.cwiseProduct(x).sum();
  g = Gradient(y, x);

  EXPECT_DOUBLE_EQ(1 + 2 * 2 + 3 * 3 + 4 * 4 + 5 * 5, y.Value());
  for (int i = 0; i < x.rows(); ++i) {
    EXPECT_EQ(2 * x(i), g(i));
  }

  H = Hessian(y, x);
  for (int i = 0; i < x.rows(); ++i) {
    for (int j = 0; j < x.size(); ++j) {
      if (i == j) {
        EXPECT_DOUBLE_EQ(2.0, H(i, j));
      } else {
        EXPECT_DOUBLE_EQ(0.0, H(i, j));
      }
    }
  }

  //--------------------------------------------------------------------------
  // TESTING GRADIENT AND HESSIAN WHEN y = prod(sin(x))
  //--------------------------------------------------------------------------
  y = x.array().sin().prod();
  g = Gradient(y, x);

  EXPECT_EQ(frc::autodiff::sin(1) * frc::autodiff::sin(2) *
                frc::autodiff::sin(3) * frc::autodiff::sin(4) *
                frc::autodiff::sin(5),
            y);
  for (int i = 0; i < x.rows(); ++i) {
    EXPECT_EQ(y / frc::autodiff::tan(x(i)), g(i));
  }

  H = Hessian(y, x);
  for (int i = 0; i < x.rows(); ++i) {
    for (int j = 0; j < x.rows(); ++j) {
      if (i == j) {
        EXPECT_NEAR((g(i) / frc::autodiff::tan(x(i))).Value() *
                        (1.0 - 1.0 / (frc::autodiff::cos(x(i)) *
                                      frc::autodiff::cos(x(i))))
                            .Value(),
                    H(i, j), 1e-14);
      } else {
        EXPECT_NEAR((g(j) / frc::autodiff::tan(x(i))).Value(), H(i, j), 1e-14);
      }
    }
  }

  //--------------------------------------------------------------------------
  // TESTING GRADIENT AND HESSIAN WHEN y = sum(diff(x).^2)
  //--------------------------------------------------------------------------
  x << 1, 1, 1, 1, 1;
  y = (x.head(4) - x.tail(4)).array().pow(2).sum();
  g = Gradient(y, x);

  EXPECT_EQ(0.0, y);
  EXPECT_EQ(2 * x[0] - 2 * x[1], g(0));
  EXPECT_EQ(-2 * x[0] + 4 * x[1] - 2 * x[2], g(1));
  EXPECT_EQ(-2 * x[1] + 4 * x[2] - 2 * x[3], g(2));
  EXPECT_EQ(-2 * x[2] + 4 * x[3] - 2 * x[4], g(3));
  EXPECT_EQ(-2 * x[3] + 2 * x[4], g(4));

  H = Hessian(y, x);
  EXPECT_EQ(2.0, H(0, 0));
  EXPECT_EQ(-2.0, H(0, 1));
  EXPECT_EQ(0.0, H(0, 2));
  EXPECT_EQ(0.0, H(0, 3));
  EXPECT_EQ(0.0, H(0, 4));
  EXPECT_EQ(-2.0, H(1, 0));
  EXPECT_EQ(4.0, H(1, 1));
  EXPECT_EQ(-2.0, H(1, 2));
  EXPECT_EQ(0.0, H(1, 3));
  EXPECT_EQ(0.0, H(1, 4));
  EXPECT_EQ(0.0, H(2, 0));
  EXPECT_EQ(-2.0, H(2, 1));
  EXPECT_EQ(4.0, H(2, 2));
  EXPECT_EQ(-2.0, H(2, 3));
  EXPECT_EQ(0.0, H(2, 4));
  EXPECT_EQ(0.0, H(3, 0));
  EXPECT_EQ(0.0, H(3, 1));
  EXPECT_EQ(-2.0, H(3, 2));
  EXPECT_EQ(4.0, H(3, 3));
  EXPECT_EQ(-2.0, H(3, 4));
  EXPECT_EQ(0.0, H(4, 0));
  EXPECT_EQ(0.0, H(4, 1));
  EXPECT_EQ(0.0, H(4, 2));
  EXPECT_EQ(-2.0, H(4, 3));
  EXPECT_EQ(2.0, H(4, 4));
}
