// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <wpi/numbers>

#include "frc/autodiff/Gradient.h"
#include "gtest/gtest.h"

TEST(GradientTest, Gradient) {
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
