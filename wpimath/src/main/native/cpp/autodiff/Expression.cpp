// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/autodiff/Expression.h"

#include <cmath>
#include <utility>

namespace frc::autodiff {

Expression::Expression(double value, ExpressionType type) : value{value} {
  if (type == ExpressionType::kConstant) {
    typeFunc = [](const wpi::IntrusiveSharedPtr<Expression>&,
                  const wpi::IntrusiveSharedPtr<Expression>&) {
      return ExpressionType::kConstant;
    };
  } else {
    typeFunc = [](const wpi::IntrusiveSharedPtr<Expression>&,
                  const wpi::IntrusiveSharedPtr<Expression>&) {
      return ExpressionType::kLinear;
    };
  }
}

Expression::Expression(BinaryFuncType typeFunc, BinaryFuncDouble valueFunc,
                       BinaryFuncDouble lhsGradientValueFunc,
                       BinaryFuncExpr lhsGradientFunc,
                       wpi::IntrusiveSharedPtr<Expression> lhs)
    : value{valueFunc(lhs->value, 0.0)},
      typeFunc{typeFunc},
      valueFunc{valueFunc},
      gradientValueFuncs{lhsGradientValueFunc, BinaryFuncDouble{}},
      gradientFuncs{lhsGradientFunc, BinaryFuncExpr{}},
      args{lhs, nullptr} {}

Expression::Expression(BinaryFuncType typeFunc, BinaryFuncDouble valueFunc,
                       BinaryFuncDouble lhsGradientValueFunc,
                       BinaryFuncDouble rhsGradientValueFunc,
                       BinaryFuncExpr lhsGradientFunc,
                       BinaryFuncExpr rhsGradientFunc,
                       wpi::IntrusiveSharedPtr<Expression> lhs,
                       wpi::IntrusiveSharedPtr<Expression> rhs)
    : value{valueFunc(lhs != nullptr ? lhs->value : 0.0,
                      rhs != nullptr ? rhs->value : 0.0)},
      typeFunc{typeFunc},
      valueFunc{valueFunc},
      gradientValueFuncs{lhsGradientValueFunc, rhsGradientValueFunc},
      gradientFuncs{lhsGradientFunc, rhsGradientFunc},
      args{lhs, rhs} {}

WPILIB_DLLEXPORT wpi::IntrusiveSharedPtr<Expression> operator*(
    double lhs, const wpi::IntrusiveSharedPtr<Expression>& rhs) {
  return MakeConstant(lhs) * rhs;
}

WPILIB_DLLEXPORT wpi::IntrusiveSharedPtr<Expression> operator*(
    const wpi::IntrusiveSharedPtr<Expression>& lhs, double rhs) {
  return lhs * MakeConstant(rhs);
}

WPILIB_DLLEXPORT wpi::IntrusiveSharedPtr<Expression> operator*(
    const wpi::IntrusiveSharedPtr<Expression>& lhs,
    const wpi::IntrusiveSharedPtr<Expression>& rhs) {
  if (lhs == nullptr || rhs == nullptr) {
    return nullptr;
  }

  if (lhs->Type() == ExpressionType::kConstant) {
    if (lhs->value == 1.0) {
      return rhs;
    } else if (lhs->value == 0.0) {
      return nullptr;
    }
  }

  if (rhs->Type() == ExpressionType::kConstant) {
    if (rhs->value == 1.0) {
      return lhs;
    } else if (rhs->value == 0.0) {
      return nullptr;
    }
  }

  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) {
        if (lhs->Type() == ExpressionType::kConstant) {
          return rhs->Type();
        }
        if (rhs->Type() == ExpressionType::kConstant) {
          return lhs->Type();
        }
        if (lhs->Type() == ExpressionType::kLinear &&
            rhs->Type() == ExpressionType::kLinear) {
          return ExpressionType::kQuadratic;
        }
        return ExpressionType::kNonlinear;
      },
      [](double lhs, double rhs) { return lhs * rhs; },
      [](double lhs, double rhs) { return rhs; },
      [](double lhs, double rhs) { return lhs; },
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) { return rhs; },
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) { return lhs; },
      lhs, rhs);
}

WPILIB_DLLEXPORT wpi::IntrusiveSharedPtr<Expression> operator/(
    double lhs, const wpi::IntrusiveSharedPtr<Expression>& rhs) {
  return MakeConstant(lhs) / rhs;
}

WPILIB_DLLEXPORT wpi::IntrusiveSharedPtr<Expression> operator/(
    const wpi::IntrusiveSharedPtr<Expression>& lhs, double rhs) {
  return lhs / MakeConstant(rhs);
}

WPILIB_DLLEXPORT wpi::IntrusiveSharedPtr<Expression> operator/(
    const wpi::IntrusiveSharedPtr<Expression>& lhs,
    const wpi::IntrusiveSharedPtr<Expression>& rhs) {
  if (lhs == nullptr) {
    return nullptr;
  }

  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) {
        if (rhs->Type() == ExpressionType::kConstant) {
          return lhs->Type();
        }
        return ExpressionType::kNonlinear;
      },
      [](double lhs, double rhs) { return lhs / rhs; },
      [](double lhs, double rhs) { return 1.0 / rhs; },
      [](double lhs, double rhs) { return -lhs / (rhs * rhs); },
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) { return 1.0 / rhs; },
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) {
        return -lhs / (rhs * rhs);
      },
      lhs, rhs);
}

WPILIB_DLLEXPORT wpi::IntrusiveSharedPtr<Expression> operator+(
    double lhs, const wpi::IntrusiveSharedPtr<Expression>& rhs) {
  return MakeConstant(lhs) + rhs;
}

WPILIB_DLLEXPORT wpi::IntrusiveSharedPtr<Expression> operator+(
    const wpi::IntrusiveSharedPtr<Expression>& lhs, double rhs) {
  return lhs + MakeConstant(rhs);
}

WPILIB_DLLEXPORT wpi::IntrusiveSharedPtr<Expression> operator+(
    const wpi::IntrusiveSharedPtr<Expression>& lhs,
    const wpi::IntrusiveSharedPtr<Expression>& rhs) {
  if (lhs == nullptr) {
    return rhs;
  } else if (rhs == nullptr) {
    return lhs;
  }

  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) {
        return ExpressionType{std::max(static_cast<int>(lhs->Type()),
                                       static_cast<int>(rhs->Type()))};
      },
      [](double lhs, double rhs) { return lhs + rhs; },
      [](double lhs, double rhs) { return 1.0; },
      [](double lhs, double rhs) { return 1.0; },
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) {
        return MakeConstant(1.0);
      },
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) {
        return MakeConstant(1.0);
      },
      lhs, rhs);
}
WPILIB_DLLEXPORT wpi::IntrusiveSharedPtr<Expression> operator-(
    double lhs, const wpi::IntrusiveSharedPtr<Expression>& rhs) {
  return MakeConstant(lhs) - rhs;
}

WPILIB_DLLEXPORT wpi::IntrusiveSharedPtr<Expression> operator-(
    const wpi::IntrusiveSharedPtr<Expression>& lhs, double rhs) {
  return lhs - MakeConstant(rhs);
}

WPILIB_DLLEXPORT wpi::IntrusiveSharedPtr<Expression> operator-(
    const wpi::IntrusiveSharedPtr<Expression>& lhs,
    const wpi::IntrusiveSharedPtr<Expression>& rhs) {
  if (lhs == nullptr) {
    if (rhs != nullptr) {
      return -rhs;
    } else {
      return nullptr;
    }
  } else if (rhs == nullptr) {
    return lhs;
  }

  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) {
        return ExpressionType{std::max(static_cast<int>(lhs->Type()),
                                       static_cast<int>(rhs->Type()))};
      },
      [](double lhs, double rhs) { return lhs - rhs; },
      [](double lhs, double rhs) { return 1.0; },
      [](double lhs, double rhs) { return -1.0; },
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) {
        return MakeConstant(1.0);
      },
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) {
        return MakeConstant(-1.0);
      },
      lhs, rhs);
}

WPILIB_DLLEXPORT wpi::IntrusiveSharedPtr<Expression> operator-(
    const wpi::IntrusiveSharedPtr<Expression>& lhs) {
  if (lhs == nullptr) {
    return nullptr;
  }

  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) { return lhs->Type(); },
      [](double lhs, double) { return -lhs; },
      [](double lhs, double rhs) { return -1.0; },
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) {
        return MakeConstant(-1.0);
      },
      lhs);
}

WPILIB_DLLEXPORT wpi::IntrusiveSharedPtr<Expression> operator+(
    const wpi::IntrusiveSharedPtr<Expression>& lhs) {
  if (lhs == nullptr) {
    return nullptr;
  }

  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) { return lhs->Type(); },
      [](double lhs, double) { return lhs; },
      [](double lhs, double rhs) { return 1.0; },
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) {
        return MakeConstant(1.0);
      },
      lhs);
}

ExpressionType Expression::Type() const {
  return typeFunc(args[0], args[1]);
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

wpi::IntrusiveSharedPtr<Expression> MakeConstant(double x) {
  return wpi::MakeIntrusiveShared<Expression>(x, ExpressionType::kConstant);
}

wpi::IntrusiveSharedPtr<Expression> abs(  // NOLINT
    const wpi::IntrusiveSharedPtr<Expression>& x) {
  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        if (lhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
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
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        if (x->value < 0.0) {
          return wpi::MakeIntrusiveShared<Expression>(-1.0);
        } else if (x->value > 0.0) {
          return wpi::MakeIntrusiveShared<Expression>(1.0);
        } else {
          return wpi::MakeIntrusiveShared<Expression>(0.0);
        }
      },
      x);
}

wpi::IntrusiveSharedPtr<Expression> acos(  // NOLINT
    const wpi::IntrusiveSharedPtr<Expression>& x) {
  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        if (lhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
      [](double x, double) { return std::acos(x); },
      [](double x, double) { return -1.0 / std::sqrt(1.0 - x * x); },
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        return -1.0 / autodiff::sqrt(1.0 - x * x);
      },
      x);
}

wpi::IntrusiveSharedPtr<Expression> asin(  // NOLINT
    const wpi::IntrusiveSharedPtr<Expression>& x) {
  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        if (lhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
      [](double x, double) { return std::asin(x); },
      [](double x, double) { return 1.0 / std::sqrt(1.0 - x * x); },
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        return 1.0 / autodiff::sqrt(1.0 - x * x);
      },
      x);
}

wpi::IntrusiveSharedPtr<Expression> atan(  // NOLINT
    const wpi::IntrusiveSharedPtr<Expression>& x) {
  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        if (lhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
      [](double x, double) { return std::atan(x); },
      [](double x, double) { return 1.0 / (1.0 + x * x); },
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        return 1.0 / (1.0 + x * x);
      },
      x);
}

wpi::IntrusiveSharedPtr<Expression> atan2(  // NOLINT
    const wpi::IntrusiveSharedPtr<Expression>& y,
    const wpi::IntrusiveSharedPtr<Expression>& x) {
  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) {
        if (lhs->Type() == ExpressionType::kConstant &&
            rhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
      [](double y, double x) { return std::atan2(y, x); },
      [](double y, double x) { return x / (y * y + x * x); },
      [](double y, double x) { return -y / (y * y + x * x); },
      [](const wpi::IntrusiveSharedPtr<Expression>& y,
         const wpi::IntrusiveSharedPtr<Expression>& x) {
        return x / (y * y + x * x);
      },
      [](const wpi::IntrusiveSharedPtr<Expression>& y,
         const wpi::IntrusiveSharedPtr<Expression>& x) {
        return -y / (y * y + x * x);
      },
      y, x);
}

wpi::IntrusiveSharedPtr<Expression> cos(  // NOLINT
    const wpi::IntrusiveSharedPtr<Expression>& x) {
  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        if (lhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
      [](double x, double) { return std::cos(x); },
      [](double x, double) { return -std::sin(x); },
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        return -autodiff::sin(x);
      },
      x);
}

wpi::IntrusiveSharedPtr<Expression> cosh(  // NOLINT
    const wpi::IntrusiveSharedPtr<Expression>& x) {
  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        if (lhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
      [](double x, double) { return std::cosh(x); },
      [](double x, double) { return std::sinh(x); },
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        return autodiff::sinh(x);
      },
      x);
}

wpi::IntrusiveSharedPtr<Expression> erf(  // NOLINT
    const wpi::IntrusiveSharedPtr<Expression>& x) {
  static constexpr double sqrt_pi =
      1.7724538509055160272981674833411451872554456638435L;

  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        if (lhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
      [](double x, double) { return std::erf(x); },
      [](double x, double) { return 2.0 / sqrt_pi * std::exp(-x * x); },
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        return 2.0 / sqrt_pi * autodiff::exp(-x * x);
      },
      x);
}

wpi::IntrusiveSharedPtr<Expression> exp(  // NOLINT
    const wpi::IntrusiveSharedPtr<Expression>& x) {
  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        if (lhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
      [](double x, double) { return std::exp(x); },
      [](double x, double) { return std::exp(x); },
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        return autodiff::exp(x);
      },
      x);
}

wpi::IntrusiveSharedPtr<Expression> hypot(  // NOLINT
    const wpi::IntrusiveSharedPtr<Expression>& x,
    const wpi::IntrusiveSharedPtr<Expression>& y) {
  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) {
        if (lhs->Type() == ExpressionType::kConstant &&
            rhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
      [](double x, double y) { return std::hypot(x, y); },
      [](double x, double y) { return x / std::hypot(x, y); },
      [](double x, double y) { return y / std::hypot(x, y); },
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>& y) {
        return x / autodiff::hypot(x, y);
      },
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>& y) {
        return y / autodiff::hypot(x, y);
      },
      x, y);
}

wpi::IntrusiveSharedPtr<Expression> log(  // NOLINT
    const wpi::IntrusiveSharedPtr<Expression>& x) {
  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        if (lhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
      [](double x, double) { return std::log(x); },
      [](double x, double) { return 1.0 / x; },
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>&) { return 1.0 / x; },
      x);
}

wpi::IntrusiveSharedPtr<Expression> log10(  // NOLINT
    const wpi::IntrusiveSharedPtr<Expression>& x) {
  static constexpr double ln10 = 2.3025850929940456840179914546843L;

  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        if (lhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
      [](double x, double) { return std::log10(x); },
      [](double x, double) { return 1.0 / (ln10 * x); },
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        return 1.0 / (ln10 * x);
      },
      x);
}

wpi::IntrusiveSharedPtr<Expression> pow(  // NOLINT
    const wpi::IntrusiveSharedPtr<Expression>& base,
    const wpi::IntrusiveSharedPtr<Expression>& power) {
  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>& rhs) {
        if (lhs->Type() == ExpressionType::kConstant &&
            rhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        if (rhs->Type() == ExpressionType::kConstant && rhs->value == 0.0) {
          return ExpressionType::kConstant;
        }
        if (lhs->Type() == ExpressionType::kLinear &&
            rhs->Type() == ExpressionType::kConstant && rhs->value == 1.0) {
          return ExpressionType::kLinear;
        }
        if (lhs->Type() == ExpressionType::kLinear &&
            rhs->Type() == ExpressionType::kConstant && rhs->value == 2.0) {
          return ExpressionType::kQuadratic;
        }
        if (lhs->Type() == ExpressionType::kQuadratic &&
            rhs->Type() == ExpressionType::kConstant && rhs->value == 1.0) {
          return ExpressionType::kQuadratic;
        }
        return ExpressionType::kNonlinear;
      },
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
      [](const wpi::IntrusiveSharedPtr<Expression>& base,
         const wpi::IntrusiveSharedPtr<Expression>& power) {
        return autodiff::pow(base, power - 1) * power;
      },
      [](const wpi::IntrusiveSharedPtr<Expression>& base,
         const wpi::IntrusiveSharedPtr<Expression>& power) {
        // Since x * std::log(x) -> 0 as x -> 0
        if (base->value == 0.0) {
          return wpi::MakeIntrusiveShared<Expression>(0.0);
        } else {
          return autodiff::pow(base, power - 1) * base * autodiff::log(base);
        }
      },
      base, power);
}

wpi::IntrusiveSharedPtr<Expression> sin(  // NOLINT
    const wpi::IntrusiveSharedPtr<Expression>& x) {
  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        if (lhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
      [](double x, double) { return std::sin(x); },
      [](double x, double) { return std::cos(x); },
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        return autodiff::cos(x);
      },
      x);
}

wpi::IntrusiveSharedPtr<Expression> sinh(
    const wpi::IntrusiveSharedPtr<Expression>& x) {
  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        if (lhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
      [](double x, double) { return std::sinh(x); },
      [](double x, double) { return std::cosh(x); },
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        return autodiff::cosh(x);
      },
      x);
}

wpi::IntrusiveSharedPtr<Expression> sqrt(  // NOLINT
    const wpi::IntrusiveSharedPtr<Expression>& x) {
  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        if (lhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
      [](double x, double) { return std::sqrt(x); },
      [](double x, double) { return 1.0 / (2.0 * std::sqrt(x)); },
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        return 1.0 / (2.0 * autodiff::sqrt(x));
      },
      x);
}

wpi::IntrusiveSharedPtr<Expression> tan(  // NOLINT
    const wpi::IntrusiveSharedPtr<Expression>& x) {
  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        if (lhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
      [](double x, double) { return std::tan(x); },
      [](double x, double) { return 1.0 / (std::cos(x) * std::cos(x)); },
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        return 1.0 / (autodiff::cos(x) * autodiff::cos(x));
      },
      x);
}

wpi::IntrusiveSharedPtr<Expression> tanh(
    const wpi::IntrusiveSharedPtr<Expression>& x) {
  return wpi::MakeIntrusiveShared<Expression>(
      [](const wpi::IntrusiveSharedPtr<Expression>& lhs,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        if (lhs->Type() == ExpressionType::kConstant) {
          return ExpressionType::kConstant;
        }
        return ExpressionType::kNonlinear;
      },
      [](double x, double) { return std::tanh(x); },
      [](double x, double) { return 1.0 / (std::cosh(x) * std::cosh(x)); },
      [](const wpi::IntrusiveSharedPtr<Expression>& x,
         const wpi::IntrusiveSharedPtr<Expression>&) {
        return 1.0 / (autodiff::cosh(x) * autodiff::cosh(x));
      },
      x);
}

}  // namespace frc::autodiff
