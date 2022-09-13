// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/autodiff/Expression.h"

#include <cmath>
#include <utility>

namespace frc::autodiff {

Expression::Expression(double value) : value{value} {}

Expression::Expression(BinaryFuncDouble valueFunc,
                       BinaryFuncDouble lhsGradientValueFunc,
                       BinaryFuncExpr lhsGradientFunc,
                       SharedPtr<Expression> lhs)
    : value{valueFunc(lhs->value, 0.0)},
      valueFunc{valueFunc},
      gradientValueFuncs{std::array{lhsGradientValueFunc, BinaryFuncDouble{}}},
      gradientFuncs{std::array{lhsGradientFunc, BinaryFuncExpr{}}},
      args{std::array<SharedPtr<Expression>, 2>{lhs, nullptr}} {}

Expression::Expression(BinaryFuncDouble valueFunc,
                       BinaryFuncDouble lhsGradientValueFunc,
                       BinaryFuncDouble rhsGradientValueFunc,
                       BinaryFuncExpr lhsGradientFunc,
                       BinaryFuncExpr rhsGradientFunc,
                       SharedPtr<Expression> lhs, SharedPtr<Expression> rhs)
    : value{valueFunc(lhs->value, rhs->value)},
      valueFunc{valueFunc},
      gradientValueFuncs{
          std::array{lhsGradientValueFunc, rhsGradientValueFunc}},
      gradientFuncs{std::array{lhsGradientFunc, rhsGradientFunc}},
      args{std::array<SharedPtr<Expression>, 2>{lhs, rhs}} {}

WPILIB_DLLEXPORT SharedPtr<Expression> operator*(
    double lhs, const SharedPtr<Expression>& rhs) {
  return MakeShared<Expression>(lhs) * rhs;
}

WPILIB_DLLEXPORT SharedPtr<Expression> operator*(
    const SharedPtr<Expression>& lhs, double rhs) {
  return lhs * MakeShared<Expression>(rhs);
}

WPILIB_DLLEXPORT SharedPtr<Expression> operator*(
    const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs) {
  return MakeShared<Expression>(
      [](double lhs, double rhs) { return lhs * rhs; },
      [](double lhs, double rhs) { return rhs; },
      [](double lhs, double rhs) { return lhs; },
      [](const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs) {
        return rhs;
      },
      [](const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs) {
        return lhs;
      },
      lhs, rhs);
}

WPILIB_DLLEXPORT SharedPtr<Expression> operator/(
    double lhs, const SharedPtr<Expression>& rhs) {
  return MakeShared<Expression>(lhs) / rhs;
}

WPILIB_DLLEXPORT SharedPtr<Expression> operator/(
    const SharedPtr<Expression>& lhs, double rhs) {
  return lhs / MakeShared<Expression>(rhs);
}

WPILIB_DLLEXPORT SharedPtr<Expression> operator/(
    const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs) {
  return MakeShared<Expression>(
      [](double lhs, double rhs) { return lhs / rhs; },
      [](double lhs, double rhs) { return 1.0 / rhs; },
      [](double lhs, double rhs) { return -lhs / (rhs * rhs); },
      [](const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs) {
        return 1.0 / rhs;
      },
      [](const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs) {
        return -lhs / (rhs * rhs);
      },
      lhs, rhs);
}

WPILIB_DLLEXPORT SharedPtr<Expression> operator+(
    double lhs, const SharedPtr<Expression>& rhs) {
  return MakeShared<Expression>(lhs) + rhs;
}

WPILIB_DLLEXPORT SharedPtr<Expression> operator+(
    const SharedPtr<Expression>& lhs, double rhs) {
  return lhs + MakeShared<Expression>(rhs);
}

WPILIB_DLLEXPORT SharedPtr<Expression> operator+(
    const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs) {
  return MakeShared<Expression>(
      [](double lhs, double rhs) { return lhs + rhs; },
      [](double lhs, double rhs) { return 1.0; },
      [](double lhs, double rhs) { return 1.0; },
      [](const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs) {
        return MakeShared<Expression>(1.0);
      },
      [](const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs) {
        return MakeShared<Expression>(1.0);
      },
      lhs, rhs);
}
WPILIB_DLLEXPORT SharedPtr<Expression> operator-(
    double lhs, const SharedPtr<Expression>& rhs) {
  return MakeShared<Expression>(lhs) - rhs;
}

WPILIB_DLLEXPORT SharedPtr<Expression> operator-(
    const SharedPtr<Expression>& lhs, double rhs) {
  return lhs - MakeShared<Expression>(rhs);
}

WPILIB_DLLEXPORT SharedPtr<Expression> operator-(
    const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs) {
  return MakeShared<Expression>(
      [](double lhs, double rhs) { return lhs - rhs; },
      [](double lhs, double rhs) { return 1.0; },
      [](double lhs, double rhs) { return -1.0; },
      [](const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs) {
        return MakeShared<Expression>(1.0);
      },
      [](const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs) {
        return MakeShared<Expression>(-1.0);
      },
      lhs, rhs);
}

WPILIB_DLLEXPORT SharedPtr<Expression> operator-(
    const SharedPtr<Expression>& lhs) {
  return MakeShared<Expression>(
      [](double lhs, double) { return -lhs; },
      [](double lhs, double rhs) { return -1.0; },
      [](const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs) {
        return MakeShared<Expression>(-1.0);
      },
      lhs);
}

WPILIB_DLLEXPORT SharedPtr<Expression> operator+(
    const SharedPtr<Expression>& lhs) {
  return MakeShared<Expression>(
      [](double lhs, double) { return lhs; },
      [](double lhs, double rhs) { return 1.0; },
      [](const SharedPtr<Expression>& lhs, const SharedPtr<Expression>& rhs) {
        return MakeShared<Expression>(1.0);
      },
      lhs);
}

void Expression::Update() {
  if (args[0] != nullptr) {
    auto& lhs = args[0];
    lhs->Update();

    if (args[1] == nullptr) {
      value = valueFunc(lhs->value, 0.0);
    } else {
      auto& rhs = args[1];
      rhs->Update();

      value = valueFunc(lhs->value, rhs->value);
    }
  }
}

SharedPtr<Expression> abs(const SharedPtr<Expression>& x) {  // NOLINT
  return MakeShared<Expression>(
      [](double x, double) { return std::abs(x); },
      [](double x, double) {
        if (x < 0.0) {
          return -1.0;
        } else if (x > 0.0) {
          return 1.0;
        } else {
          return 0.0;
        }
      },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>&) {
        if (x->value < 0.0) {
          return MakeShared<Expression>(-1.0);
        } else if (x->value > 0.0) {
          return MakeShared<Expression>(1.0);
        } else {
          return MakeShared<Expression>(0.0);
        }
      },
      x);
}

SharedPtr<Expression> acos(const SharedPtr<Expression>& x) {  // NOLINT
  return MakeShared<Expression>(
      [](double x, double) { return std::acos(x); },
      [](double x, double) { return -1.0 / std::sqrt(1.0 - x * x); },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>&) {
        return -1.0 / autodiff::sqrt(1.0 - x * x);
      },
      x);
}

SharedPtr<Expression> asin(const SharedPtr<Expression>& x) {  // NOLINT
  return MakeShared<Expression>(
      [](double x, double) { return std::asin(x); },
      [](double x, double) { return 1.0 / std::sqrt(1.0 - x * x); },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>&) {
        return 1.0 / autodiff::sqrt(1.0 - x * x);
      },
      x);
}

SharedPtr<Expression> atan(const SharedPtr<Expression>& x) {  // NOLINT
  return MakeShared<Expression>(
      [](double x, double) { return std::atan(x); },
      [](double x, double) { return 1.0 / (1.0 + x * x); },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>&) {
        return 1.0 / (1.0 + x * x);
      },
      x);
}

SharedPtr<Expression> atan2(const SharedPtr<Expression>& y,  // NOLINT
                            const SharedPtr<Expression>& x) {
  return MakeShared<Expression>(
      [](double y, double x) { return std::atan2(y, x); },
      [](double y, double x) { return x / (y * y + x * x); },
      [](double y, double x) { return -y / (y * y + x * x); },
      [](const SharedPtr<Expression>& y, const SharedPtr<Expression>& x) {
        return x / (y * y + x * x);
      },
      [](const SharedPtr<Expression>& y, const SharedPtr<Expression>& x) {
        return -y / (y * y + x * x);
      },
      y, x);
}

SharedPtr<Expression> cos(const SharedPtr<Expression>& x) {  // NOLINT
  return MakeShared<Expression>(
      [](double x, double) { return std::cos(x); },
      [](double x, double) { return -std::sin(x); },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>&) {
        return -autodiff::sin(x);
      },
      x);
}

SharedPtr<Expression> cosh(const SharedPtr<Expression>& x) {  // NOLINT
  return MakeShared<Expression>(
      [](double x, double) { return std::cosh(x); },
      [](double x, double) { return std::sinh(x); },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>&) {
        return autodiff::sinh(x);
      },
      x);
}

SharedPtr<Expression> erf(const SharedPtr<Expression>& x) {  // NOLINT
  static constexpr double sqrt_pi =
      1.7724538509055160272981674833411451872554456638435L;

  return MakeShared<Expression>(
      [](double x, double) { return std::erf(x); },
      [](double x, double) { return 2.0 / sqrt_pi * std::exp(-x * x); },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>&) {
        return 2.0 / sqrt_pi * autodiff::exp(-x * x);
      },
      x);
}

SharedPtr<Expression> exp(const SharedPtr<Expression>& x) {  // NOLINT
  return MakeShared<Expression>(
      [](double x, double) { return std::exp(x); },
      [](double x, double) { return std::exp(x); },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>&) {
        return autodiff::exp(x);
      },
      x);
}

SharedPtr<Expression> hypot(const SharedPtr<Expression>& x,  // NOLINT
                            const SharedPtr<Expression>& y) {
  return MakeShared<Expression>(
      [](double x, double y) { return std::hypot(x, y); },
      [](double x, double y) { return x / std::hypot(x, y); },
      [](double x, double y) { return y / std::hypot(x, y); },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>& y) {
        return x / autodiff::hypot(x, y);
      },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>& y) {
        return y / autodiff::hypot(x, y);
      },
      x, y);
}

SharedPtr<Expression> log(const SharedPtr<Expression>& x) {  // NOLINT
  return MakeShared<Expression>(
      [](double x, double) { return std::log(x); },
      [](double x, double) { return 1.0 / x; },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>&) {
        return 1.0 / x;
      },
      x);
}

SharedPtr<Expression> log10(const SharedPtr<Expression>& x) {  // NOLINT
  static constexpr double ln10 = 2.3025850929940456840179914546843L;

  return MakeShared<Expression>(
      [](double x, double) { return std::log10(x); },
      [](double x, double) { return 1.0 / (ln10 * x); },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>&) {
        return 1.0 / (ln10 * x);
      },
      x);
}

SharedPtr<Expression> pow(const SharedPtr<Expression>& base,  // NOLINT
                          const SharedPtr<Expression>& power) {
  return MakeShared<Expression>(
      [](double base, double power) { return std::pow(base, power); },
      [](double base, double power) {
        return std::pow(base, power - 1) * power;
      },
      [](double base, double power) {
        // Since x * std::log(x) -> 0 as x -> 0
        if (base == 0.0) {
          return 0.0;
        } else {
          return std::pow(base, power - 1) * base * std::log(base);
        }
      },
      [](const SharedPtr<Expression>& base,
         const SharedPtr<Expression>& power) {
        return autodiff::pow(base, power - 1) * power;
      },
      [](const SharedPtr<Expression>& base,
         const SharedPtr<Expression>& power) {
        // Since x * std::log(x) -> 0 as x -> 0
        if (base->value == 0.0) {
          return MakeShared<Expression>(0.0);
        } else {
          return autodiff::pow(base, power - 1) * base * autodiff::log(base);
        }
      },
      base, power);
}

SharedPtr<Expression> sin(const SharedPtr<Expression>& x) {  // NOLINT
  return MakeShared<Expression>(
      [](double x, double) { return std::sin(x); },
      [](double x, double) { return std::cos(x); },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>&) {
        return autodiff::cos(x);
      },
      x);
}

SharedPtr<Expression> sinh(const SharedPtr<Expression>& x) {
  return MakeShared<Expression>(
      [](double x, double) { return std::sinh(x); },
      [](double x, double) { return std::cosh(x); },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>&) {
        return autodiff::cosh(x);
      },
      x);
}

SharedPtr<Expression> sqrt(const SharedPtr<Expression>& x) {  // NOLINT
  return MakeShared<Expression>(
      [](double x, double) { return std::sqrt(x); },
      [](double x, double) { return 1.0 / (2.0 * std::sqrt(x)); },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>&) {
        return 1.0 / (2.0 * autodiff::sqrt(x));
      },
      x);
}

SharedPtr<Expression> tan(const SharedPtr<Expression>& x) {  // NOLINT
  return MakeShared<Expression>(
      [](double x, double) { return std::tan(x); },
      [](double x, double) { return 1.0 / (std::cos(x) * std::cos(x)); },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>&) {
        return 1.0 / (autodiff::cos(x) * autodiff::cos(x));
      },
      x);
}

SharedPtr<Expression> tanh(const SharedPtr<Expression>& x) {
  return MakeShared<Expression>(
      [](double x, double) { return std::tanh(x); },
      [](double x, double) { return 1.0 / (std::cosh(x) * std::cosh(x)); },
      [](const SharedPtr<Expression>& x, const SharedPtr<Expression>&) {
        return 1.0 / (autodiff::cosh(x) * autodiff::cosh(x));
      },
      x);
}

}  // namespace frc::autodiff
