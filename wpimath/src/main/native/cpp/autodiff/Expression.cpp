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
                       IntrusiveSharedPtr<Expression> lhs)
    : value{valueFunc(lhs->value, 0.0)},
      valueFunc{valueFunc},
      gradientValueFuncs{std::array{lhsGradientValueFunc, BinaryFuncDouble{}}},
      gradientFuncs{std::array{lhsGradientFunc, BinaryFuncExpr{}}},
      args{std::array<IntrusiveSharedPtr<Expression>, 2>{lhs, nullptr}} {}

Expression::Expression(BinaryFuncDouble valueFunc,
                       BinaryFuncDouble lhsGradientValueFunc,
                       BinaryFuncDouble rhsGradientValueFunc,
                       BinaryFuncExpr lhsGradientFunc,
                       BinaryFuncExpr rhsGradientFunc,
                       IntrusiveSharedPtr<Expression> lhs,
                       IntrusiveSharedPtr<Expression> rhs)
    : value{valueFunc(lhs != nullptr ? lhs->value : 0.0,
                      rhs != nullptr ? rhs->value : 0.0)},
      valueFunc{valueFunc},
      gradientValueFuncs{
          std::array{lhsGradientValueFunc, rhsGradientValueFunc}},
      gradientFuncs{std::array{lhsGradientFunc, rhsGradientFunc}},
      args{std::array<IntrusiveSharedPtr<Expression>, 2>{lhs, rhs}} {}

WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator*(
    double lhs, const IntrusiveSharedPtr<Expression>& rhs) {
  return MakeIntrusiveShared<Expression>(lhs) * rhs;
}

WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator*(
    const IntrusiveSharedPtr<Expression>& lhs, double rhs) {
  return lhs * MakeIntrusiveShared<Expression>(rhs);
}

WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator*(
    const IntrusiveSharedPtr<Expression>& lhs,
    const IntrusiveSharedPtr<Expression>& rhs) {
  return MakeIntrusiveShared<Expression>(
      [](double lhs, double rhs) { return lhs * rhs; },
      [](double lhs, double rhs) { return rhs; },
      [](double lhs, double rhs) { return lhs; },
      [](const IntrusiveSharedPtr<Expression>& lhs,
         const IntrusiveSharedPtr<Expression>& rhs) { return rhs; },
      [](const IntrusiveSharedPtr<Expression>& lhs,
         const IntrusiveSharedPtr<Expression>& rhs) { return lhs; },
      lhs, rhs);
}

WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator/(
    double lhs, const IntrusiveSharedPtr<Expression>& rhs) {
  return MakeIntrusiveShared<Expression>(lhs) / rhs;
}

WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator/(
    const IntrusiveSharedPtr<Expression>& lhs, double rhs) {
  return lhs / MakeIntrusiveShared<Expression>(rhs);
}

WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator/(
    const IntrusiveSharedPtr<Expression>& lhs,
    const IntrusiveSharedPtr<Expression>& rhs) {
  return MakeIntrusiveShared<Expression>(
      [](double lhs, double rhs) { return lhs / rhs; },
      [](double lhs, double rhs) { return 1.0 / rhs; },
      [](double lhs, double rhs) { return -lhs / (rhs * rhs); },
      [](const IntrusiveSharedPtr<Expression>& lhs,
         const IntrusiveSharedPtr<Expression>& rhs) { return 1.0 / rhs; },
      [](const IntrusiveSharedPtr<Expression>& lhs,
         const IntrusiveSharedPtr<Expression>& rhs) {
        return -lhs / (rhs * rhs);
      },
      lhs, rhs);
}

WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator+(
    double lhs, const IntrusiveSharedPtr<Expression>& rhs) {
  return MakeIntrusiveShared<Expression>(lhs) + rhs;
}

WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator+(
    const IntrusiveSharedPtr<Expression>& lhs, double rhs) {
  return lhs + MakeIntrusiveShared<Expression>(rhs);
}

WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator+(
    const IntrusiveSharedPtr<Expression>& lhs,
    const IntrusiveSharedPtr<Expression>& rhs) {
  return MakeIntrusiveShared<Expression>(
      [](double lhs, double rhs) { return lhs + rhs; },
      [](double lhs, double rhs) { return 1.0; },
      [](double lhs, double rhs) { return 1.0; },
      [](const IntrusiveSharedPtr<Expression>& lhs,
         const IntrusiveSharedPtr<Expression>& rhs) {
        return MakeIntrusiveShared<Expression>(1.0);
      },
      [](const IntrusiveSharedPtr<Expression>& lhs,
         const IntrusiveSharedPtr<Expression>& rhs) {
        return MakeIntrusiveShared<Expression>(1.0);
      },
      lhs, rhs);
}
WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator-(
    double lhs, const IntrusiveSharedPtr<Expression>& rhs) {
  return MakeIntrusiveShared<Expression>(lhs) - rhs;
}

WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator-(
    const IntrusiveSharedPtr<Expression>& lhs, double rhs) {
  return lhs - MakeIntrusiveShared<Expression>(rhs);
}

WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator-(
    const IntrusiveSharedPtr<Expression>& lhs,
    const IntrusiveSharedPtr<Expression>& rhs) {
  return MakeIntrusiveShared<Expression>(
      [](double lhs, double rhs) { return lhs - rhs; },
      [](double lhs, double rhs) { return 1.0; },
      [](double lhs, double rhs) { return -1.0; },
      [](const IntrusiveSharedPtr<Expression>& lhs,
         const IntrusiveSharedPtr<Expression>& rhs) {
        return MakeIntrusiveShared<Expression>(1.0);
      },
      [](const IntrusiveSharedPtr<Expression>& lhs,
         const IntrusiveSharedPtr<Expression>& rhs) {
        return MakeIntrusiveShared<Expression>(-1.0);
      },
      lhs, rhs);
}

WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator-(
    const IntrusiveSharedPtr<Expression>& lhs) {
  return MakeIntrusiveShared<Expression>(
      [](double lhs, double) { return -lhs; },
      [](double lhs, double rhs) { return -1.0; },
      [](const IntrusiveSharedPtr<Expression>& lhs,
         const IntrusiveSharedPtr<Expression>& rhs) {
        return MakeIntrusiveShared<Expression>(-1.0);
      },
      lhs);
}

WPILIB_DLLEXPORT IntrusiveSharedPtr<Expression> operator+(
    const IntrusiveSharedPtr<Expression>& lhs) {
  return MakeIntrusiveShared<Expression>(
      [](double lhs, double) { return lhs; },
      [](double lhs, double rhs) { return 1.0; },
      [](const IntrusiveSharedPtr<Expression>& lhs,
         const IntrusiveSharedPtr<Expression>& rhs) {
        return MakeIntrusiveShared<Expression>(1.0);
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

IntrusiveSharedPtr<Expression> abs(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x) {
  return MakeIntrusiveShared<Expression>(
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
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>&) {
        if (x->value < 0.0) {
          return MakeIntrusiveShared<Expression>(-1.0);
        } else if (x->value > 0.0) {
          return MakeIntrusiveShared<Expression>(1.0);
        } else {
          return MakeIntrusiveShared<Expression>(0.0);
        }
      },
      x);
}

IntrusiveSharedPtr<Expression> acos(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x) {
  return MakeIntrusiveShared<Expression>(
      [](double x, double) { return std::acos(x); },
      [](double x, double) { return -1.0 / std::sqrt(1.0 - x * x); },
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>&) {
        return -1.0 / autodiff::sqrt(1.0 - x * x);
      },
      x);
}

IntrusiveSharedPtr<Expression> asin(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x) {
  return MakeIntrusiveShared<Expression>(
      [](double x, double) { return std::asin(x); },
      [](double x, double) { return 1.0 / std::sqrt(1.0 - x * x); },
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>&) {
        return 1.0 / autodiff::sqrt(1.0 - x * x);
      },
      x);
}

IntrusiveSharedPtr<Expression> atan(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x) {
  return MakeIntrusiveShared<Expression>(
      [](double x, double) { return std::atan(x); },
      [](double x, double) { return 1.0 / (1.0 + x * x); },
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>&) { return 1.0 / (1.0 + x * x); },
      x);
}

IntrusiveSharedPtr<Expression> atan2(  // NOLINT
    const IntrusiveSharedPtr<Expression>& y,
    const IntrusiveSharedPtr<Expression>& x) {
  return MakeIntrusiveShared<Expression>(
      [](double y, double x) { return std::atan2(y, x); },
      [](double y, double x) { return x / (y * y + x * x); },
      [](double y, double x) { return -y / (y * y + x * x); },
      [](const IntrusiveSharedPtr<Expression>& y,
         const IntrusiveSharedPtr<Expression>& x) {
        return x / (y * y + x * x);
      },
      [](const IntrusiveSharedPtr<Expression>& y,
         const IntrusiveSharedPtr<Expression>& x) {
        return -y / (y * y + x * x);
      },
      y, x);
}

IntrusiveSharedPtr<Expression> cos(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x) {
  return MakeIntrusiveShared<Expression>(
      [](double x, double) { return std::cos(x); },
      [](double x, double) { return -std::sin(x); },
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>&) { return -autodiff::sin(x); },
      x);
}

IntrusiveSharedPtr<Expression> cosh(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x) {
  return MakeIntrusiveShared<Expression>(
      [](double x, double) { return std::cosh(x); },
      [](double x, double) { return std::sinh(x); },
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>&) { return autodiff::sinh(x); },
      x);
}

IntrusiveSharedPtr<Expression> erf(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x) {
  static constexpr double sqrt_pi =
      1.7724538509055160272981674833411451872554456638435L;

  return MakeIntrusiveShared<Expression>(
      [](double x, double) { return std::erf(x); },
      [](double x, double) { return 2.0 / sqrt_pi * std::exp(-x * x); },
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>&) {
        return 2.0 / sqrt_pi * autodiff::exp(-x * x);
      },
      x);
}

IntrusiveSharedPtr<Expression> exp(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x) {
  return MakeIntrusiveShared<Expression>(
      [](double x, double) { return std::exp(x); },
      [](double x, double) { return std::exp(x); },
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>&) { return autodiff::exp(x); },
      x);
}

IntrusiveSharedPtr<Expression> hypot(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x,
    const IntrusiveSharedPtr<Expression>& y) {
  return MakeIntrusiveShared<Expression>(
      [](double x, double y) { return std::hypot(x, y); },
      [](double x, double y) { return x / std::hypot(x, y); },
      [](double x, double y) { return y / std::hypot(x, y); },
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>& y) {
        return x / autodiff::hypot(x, y);
      },
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>& y) {
        return y / autodiff::hypot(x, y);
      },
      x, y);
}

IntrusiveSharedPtr<Expression> log(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x) {
  return MakeIntrusiveShared<Expression>(
      [](double x, double) { return std::log(x); },
      [](double x, double) { return 1.0 / x; },
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>&) { return 1.0 / x; },
      x);
}

IntrusiveSharedPtr<Expression> log10(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x) {
  static constexpr double ln10 = 2.3025850929940456840179914546843L;

  return MakeIntrusiveShared<Expression>(
      [](double x, double) { return std::log10(x); },
      [](double x, double) { return 1.0 / (ln10 * x); },
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>&) { return 1.0 / (ln10 * x); },
      x);
}

IntrusiveSharedPtr<Expression> pow(  // NOLINT
    const IntrusiveSharedPtr<Expression>& base,
    const IntrusiveSharedPtr<Expression>& power) {
  return MakeIntrusiveShared<Expression>(
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
      [](const IntrusiveSharedPtr<Expression>& base,
         const IntrusiveSharedPtr<Expression>& power) {
        return autodiff::pow(base, power - 1) * power;
      },
      [](const IntrusiveSharedPtr<Expression>& base,
         const IntrusiveSharedPtr<Expression>& power) {
        // Since x * std::log(x) -> 0 as x -> 0
        if (base->value == 0.0) {
          return MakeIntrusiveShared<Expression>(0.0);
        } else {
          return autodiff::pow(base, power - 1) * base * autodiff::log(base);
        }
      },
      base, power);
}

IntrusiveSharedPtr<Expression> sin(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x) {
  return MakeIntrusiveShared<Expression>(
      [](double x, double) { return std::sin(x); },
      [](double x, double) { return std::cos(x); },
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>&) { return autodiff::cos(x); },
      x);
}

IntrusiveSharedPtr<Expression> sinh(const IntrusiveSharedPtr<Expression>& x) {
  return MakeIntrusiveShared<Expression>(
      [](double x, double) { return std::sinh(x); },
      [](double x, double) { return std::cosh(x); },
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>&) { return autodiff::cosh(x); },
      x);
}

IntrusiveSharedPtr<Expression> sqrt(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x) {
  return MakeIntrusiveShared<Expression>(
      [](double x, double) { return std::sqrt(x); },
      [](double x, double) { return 1.0 / (2.0 * std::sqrt(x)); },
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>&) {
        return 1.0 / (2.0 * autodiff::sqrt(x));
      },
      x);
}

IntrusiveSharedPtr<Expression> tan(  // NOLINT
    const IntrusiveSharedPtr<Expression>& x) {
  return MakeIntrusiveShared<Expression>(
      [](double x, double) { return std::tan(x); },
      [](double x, double) { return 1.0 / (std::cos(x) * std::cos(x)); },
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>&) {
        return 1.0 / (autodiff::cos(x) * autodiff::cos(x));
      },
      x);
}

IntrusiveSharedPtr<Expression> tanh(const IntrusiveSharedPtr<Expression>& x) {
  return MakeIntrusiveShared<Expression>(
      [](double x, double) { return std::tanh(x); },
      [](double x, double) { return 1.0 / (std::cosh(x) * std::cosh(x)); },
      [](const IntrusiveSharedPtr<Expression>& x,
         const IntrusiveSharedPtr<Expression>&) {
        return 1.0 / (autodiff::cosh(x) * autodiff::cosh(x));
      },
      x);
}

}  // namespace frc::autodiff
