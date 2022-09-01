// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/autodiff/Variable.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <queue>
#include <string_view>
#include <tuple>
#include <vector>

#include <fmt/core.h>

#include "frc/autodiff/Tape.h"
#include "frc/fmt/Eigen.h"

namespace frc::autodiff {

Variable::Variable(double value) {
  *this = Tape::GetCurrentTape().PushNullary(
      value, []() -> Variable { return Constant(1.0); });
}

Variable::Variable(int value) {
  *this = Tape::GetCurrentTape().PushNullary(
      value, []() -> Variable { return Constant(1.0); });
}

Variable::Variable(Tape* tape, int index) : tape{tape}, index{index} {}

Variable& Variable::operator=(double value) {
  if (tape == nullptr) {
    *this = Tape::GetCurrentTape().PushNullary(
        value, []() -> Variable { return Constant(1.0); });
  } else {
    GetNode().value = value;
  }
  return *this;
}

Variable& Variable::operator=(int value) {
  if (tape == nullptr) {
    *this = Tape::GetCurrentTape().PushNullary(
        value, []() -> Variable { return Constant(1.0); });
  } else {
    GetNode().value = value;
  }
  return *this;
}

WPILIB_DLLEXPORT Variable operator*(double lhs, const Variable& rhs) {
  return Constant(lhs) * rhs;
}

WPILIB_DLLEXPORT Variable operator*(const Variable& lhs, double rhs) {
  return lhs * Constant(rhs);
}

WPILIB_DLLEXPORT Variable operator*(const Variable& lhs, const Variable& rhs) {
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushBinary(
      lhs, rhs, [](double lhs, double rhs) { return lhs * rhs; },
      [](const Variable& lhs, const Variable& rhs) -> Variable { return rhs; },
      [](const Variable& lhs, const Variable& rhs) -> Variable {
        return lhs;
      })};
}

Variable& Variable::operator*=(double rhs) {
  *this = *this * rhs;
  return *this;
}

Variable& Variable::operator*=(const Variable& rhs) {
  *this = *this * rhs;
  return *this;
}

WPILIB_DLLEXPORT Variable operator/(double lhs, const Variable& rhs) {
  return Constant(lhs) / rhs;
}

WPILIB_DLLEXPORT Variable operator/(const Variable& lhs, double rhs) {
  return lhs / Constant(rhs);
}

WPILIB_DLLEXPORT Variable operator/(const Variable& lhs, const Variable& rhs) {
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushBinary(
      lhs, rhs, [](double lhs, double rhs) { return lhs / rhs; },
      [](const Variable& lhs, const Variable& rhs) -> Variable {
        return 1.0 / rhs;
      },
      [](const Variable& lhs, const Variable& rhs) -> Variable {
        return -lhs / (rhs * rhs);
      })};
}

Variable& Variable::operator/=(double rhs) {
  *this = *this / rhs;
  return *this;
}

Variable& Variable::operator/=(const Variable& rhs) {
  *this = *this / rhs;
  return *this;
}

WPILIB_DLLEXPORT Variable operator+(double lhs, const Variable& rhs) {
  return Constant(lhs) + rhs;
}

WPILIB_DLLEXPORT Variable operator+(const Variable& lhs, double rhs) {
  return lhs + Constant(rhs);
}

WPILIB_DLLEXPORT Variable operator+(const Variable& lhs, const Variable& rhs) {
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushBinary(
      lhs, rhs, [](double lhs, double rhs) { return lhs + rhs; },
      [](const Variable&, const Variable&) -> Variable {
        return Constant(1.0);
      },
      [](const Variable&, const Variable&) -> Variable {
        return Constant(1.0);
      })};
}

Variable& Variable::operator+=(double rhs) {
  *this = *this + rhs;
  return *this;
}

Variable& Variable::operator+=(const Variable& rhs) {
  *this = *this + rhs;
  return *this;
}

WPILIB_DLLEXPORT Variable operator-(double lhs, const Variable& rhs) {
  return Constant(lhs) - rhs;
}

WPILIB_DLLEXPORT Variable operator-(const Variable& lhs, double rhs) {
  return lhs - Constant(rhs);
}

WPILIB_DLLEXPORT Variable operator-(const Variable& lhs, const Variable& rhs) {
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushBinary(
      lhs, rhs, [](double lhs, double rhs) { return lhs - rhs; },
      [](const Variable&, const Variable&) -> Variable {
        return Constant(1.0);
      },
      [](const Variable&, const Variable&) -> Variable {
        return Constant(-1.0);
      })};
}

Variable& Variable::operator-=(double rhs) {
  *this = *this - rhs;
  return *this;
}

Variable& Variable::operator-=(const Variable& rhs) {
  *this = *this - rhs;
  return *this;
}

WPILIB_DLLEXPORT Variable operator-(const Variable& lhs) {
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      lhs, [](double lhs) { return -lhs; },
      [](const Variable&) -> Variable { return Constant(-1.0); })};
}

WPILIB_DLLEXPORT Variable operator+(const Variable& lhs) {
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      lhs, [](double lhs) { return +lhs; },
      [](const Variable&) -> Variable { return Constant(1.0); })};
}

WPILIB_DLLEXPORT bool operator==(double lhs, const Variable& rhs) {
  return std::abs(lhs - rhs.Value()) < 1e-9;
}

WPILIB_DLLEXPORT bool operator==(const Variable& lhs, double rhs) {
  return std::abs(lhs.Value() - rhs) < 1e-9;
}

WPILIB_DLLEXPORT bool operator==(const Variable& lhs, const Variable& rhs) {
  return std::abs(lhs.Value() - rhs.Value()) < 1e-9;
}

WPILIB_DLLEXPORT bool operator!=(double lhs, const Variable& rhs) {
  return lhs != rhs.Value();
}

WPILIB_DLLEXPORT bool operator!=(const Variable& lhs, double rhs) {
  return lhs.Value() != rhs;
}

WPILIB_DLLEXPORT bool operator!=(const Variable& lhs, const Variable& rhs) {
  return lhs.Value() != rhs.Value();
}

WPILIB_DLLEXPORT bool operator<(double lhs, const Variable& rhs) {
  return lhs < rhs.Value();
}

WPILIB_DLLEXPORT bool operator<(const Variable& lhs, double rhs) {
  return lhs.Value() < rhs;
}

WPILIB_DLLEXPORT bool operator<(const Variable& lhs, const Variable& rhs) {
  return lhs.Value() < rhs.Value();
}

WPILIB_DLLEXPORT bool operator>(double lhs, const Variable& rhs) {
  return lhs > rhs.Value();
}

WPILIB_DLLEXPORT bool operator>(const Variable& lhs, double rhs) {
  return lhs.Value() > rhs;
}

WPILIB_DLLEXPORT bool operator>(const Variable& lhs, const Variable& rhs) {
  return lhs.Value() > rhs.Value();
}

WPILIB_DLLEXPORT bool operator<=(double lhs, const Variable& rhs) {
  return lhs <= rhs.Value();
}

WPILIB_DLLEXPORT bool operator<=(const Variable& lhs, double rhs) {
  return lhs.Value() <= rhs;
}

WPILIB_DLLEXPORT bool operator<=(const Variable& lhs, const Variable& rhs) {
  return lhs.Value() <= rhs.Value();
}

WPILIB_DLLEXPORT bool operator>=(double lhs, const Variable& rhs) {
  return lhs >= rhs.Value();
}

WPILIB_DLLEXPORT bool operator>=(const Variable& lhs, double rhs) {
  return lhs.Value() >= rhs;
}

WPILIB_DLLEXPORT bool operator>=(const Variable& lhs, const Variable& rhs) {
  return lhs.Value() >= rhs.Value();
}

double Variable::Value() const {
  if (tape == nullptr) {
    return 0.0;
  } else {
    return GetNode().value;
  }
}

Variable Variable::Gradient(int arg) const {
  if (tape == nullptr) {
    return Constant(0.0);
  } else {
    return GetNode().Gradient(arg);
  }
}

void Variable::Update() {
  GetNode().Update();
}

const TapeNode& Variable::GetNode() const {
  return (*tape)[index];
}

TapeNode& Variable::GetNode() {
  return (*tape)[index];
}

namespace {
/**
 * Returns a breadth-first search list of tape nodes.
 *
 * @param tape Tape of nodes to enumerate.
 * @param root Starting tape node.
 * @param earliestLeaf Ending tape node. If a multivariate gradient is being
 *                     computed, this should be the leaf node with the smallest
 *                     index.
 */
std::vector<int> GenerateBFSList(const Tape& tape, Variable& root,
                                 Variable& earliestLeaf) {
  std::vector<int> l;
  std::queue<int> q;

  l.push_back(root.index);
  q.push(root.index);
  while (!q.empty()) {
    auto& parent = tape[q.front()];
    q.pop();

    // If parent is most nested leaf of the gradient calculation, don't push any
    // children because their adjoints won't be used
    if (parent.index == earliestLeaf.index) {
      continue;
    }

    for (int child = 0; child < TapeNode::kNumArgs; ++child) {
      auto& childNode = parent.args[child];

      // Don't push leaf nodes
      if (childNode.tape == nullptr) {
        continue;
      }

      // Push child if it's not a duplicate
      if (std::find(l.begin(), l.end(), childNode.index) == l.end()) {
        l.push_back(childNode.index);
        q.push(childNode.index);
      }
    }
  }

  return l;
}
}  // namespace

WPILIB_DLLEXPORT Variable Constant(double value) {
  return Tape::GetCurrentTape().PushNullary(
      value, []() -> Variable { return Constant(0.0); });
}

double Gradient(Variable variable, Variable& wrt) {
  // Based on algorithm 2 of "A new framework for the computation of Hessians"
  // https://arxiv.org/pdf/2007.15040.pdf

  auto& tape = *variable.tape;
  int tapeSize = tape.Size();

  std::vector<int> tapeIndices = GenerateBFSList(tape, variable, wrt);
  int tapeIndicesSize = static_cast<int>(tapeIndices.size());

  // wrt might not be in tape
  wrt.GetNode().adjoint = 0.0;

  tape[tapeIndices[0]].adjoint = 1.0;
  for (int i = 1; i < tapeIndicesSize; ++i) {
    tape[tapeIndices[i]].adjoint = 0.0;
  }

  for (int parent : tapeIndices) {
    for (int child = 0; child < TapeNode::kNumArgs; ++child) {
      if (tape[parent].args[child].tape != nullptr) {
        // v̅ⱼ += v̅ᵢ ∂ϕᵢ/∂vⱼ
        tape[parent].args[child].GetNode().adjoint +=
            tape[parent].adjoint * tape[parent].Gradient(child).Value();
      }
    }
  }

  tape.Resize(tapeSize);

  return wrt.GetNode().adjoint;
}

Eigen::VectorXd Gradient(Variable variable, VectorXvar& wrt) {
  // Based on algorithm 2 of "A new framework for the computation of Hessians"
  // https://arxiv.org/pdf/2007.15040.pdf

  auto& tape = *variable.tape;
  int tapeSize = tape.Size();

  Variable& minIndexWrtVariable = *std::min_element(
      wrt.begin(), wrt.end(),
      [](const auto& i, const auto& j) { return i.index < j.index; });

  std::vector<int> tapeIndices =
      GenerateBFSList(tape, variable, minIndexWrtVariable);
  int tapeIndicesSize = static_cast<int>(tapeIndices.size());

  // wrt might not be in tape, so zero its adjoints separately
  for (int row = 0; row < wrt.rows(); ++row) {
    wrt(row).GetNode().adjoint = 0.0;
  }

  tape[tapeIndices[0]].adjoint = 1.0;
  for (int i = 1; i < tapeIndicesSize; ++i) {
    tape[tapeIndices[i]].adjoint = 0.0;
  }

  for (int parent : tapeIndices) {
    for (int child = 0; child < TapeNode::kNumArgs; ++child) {
      if (tape[parent].args[child].tape != nullptr) {
        // v̅ⱼ += v̅ᵢ ∂ϕᵢ/∂vⱼ
        tape[parent].args[child].GetNode().adjoint +=
            tape[parent].adjoint * tape[parent].Gradient(child).Value();
      }
    }
  }

  tape.Resize(tapeSize);

  // Select elements of gradient in wrt
  Eigen::VectorXd grad{wrt.rows(), 1};
  for (int row = 0; row < wrt.rows(); ++row) {
    grad(row) = wrt(row).GetNode().adjoint;
  }

  return grad;
}

Eigen::SparseMatrix<double> Jacobian(VectorXvar& variables, VectorXvar& wrt) {
  Eigen::SparseMatrix<double> J{variables.rows(), wrt.rows()};

  std::vector<Eigen::Triplet<double>> triplets;
  for (int row = 0; row < variables.rows(); ++row) {
    Eigen::RowVectorXd g = Gradient(variables(row), wrt).transpose();
    for (int col = 0; col < g.cols(); ++col) {
      if (g(col) != 0.0) {
        triplets.emplace_back(row, col, g(col));
      }
    }
  }
  J.setFromTriplets(triplets.begin(), triplets.end());

  return J;
}

Eigen::SparseMatrix<double> Hessian(Variable variable, VectorXvar& wrt) {
  // Based on algorithm 4 of "A new framework for the computation of Hessians"
  // https://arxiv.org/pdf/2007.15040.pdf

  auto& tape = *variable.tape;
  int tapeSize = tape.Size();

  Variable& minIndexWrtVariable = *std::min_element(
      wrt.begin(), wrt.end(),
      [](const auto& i, const auto& j) { return i.index < j.index; });

  std::vector<int> tapeIndices =
      GenerateBFSList(tape, variable, minIndexWrtVariable);
  int tapeIndicesSize = static_cast<int>(tapeIndices.size());

  // wrt might not be in tape, so zero its adjoints separately
  for (int row = 0; row < wrt.rows(); ++row) {
    wrt(row).GetNode().adjoint = 0.0;
  }

  tape[tapeIndices[0]].adjoint = 1.0;
  for (int i = 1; i < tapeIndicesSize; ++i) {
    tape[tapeIndices[i]].adjoint = 0.0;
  }

  Eigen::SparseMatrix<double> W{tape.Size(), tape.Size()};

  auto GetW = [&W](int row, int col) -> double {
    return W.coeff(std::max(row, col), std::min(row, col));
  };

  auto GetWRef = [&W](int row, int col) -> double& {
    return W.coeffRef(std::max(row, col), std::min(row, col));
  };

  // for i from l to 1
  for (int i : tapeIndices) {
    // Pushing
    // for each p such that p <= i and w_{pi} != 0
    for (int p : tapeIndices) {
      if (!(p <= i && GetW(p, i) != 0.0)) {
        continue;
      }

      if (p != i) {
        // for each j < i
        for (int jArg = 0; jArg < TapeNode::kNumArgs; ++jArg) {
          if (tape[i].args[jArg].tape == nullptr) {
            continue;
          }

          double grad = tape[i].Gradient(jArg).Value();
          if (grad == 0.0) {
            continue;
          }

          int j = tape[i].args[jArg].index;
          if (j == p) {
            // w_{pp} += 2∂ϕᵢ/∂vₚ w_{pi}
            GetWRef(j, p) += 2.0 * grad * GetW(p, i);
          } else {
            // w_{jp} += ∂ϕᵢ/∂vⱼ w_{pi}
            GetWRef(j, p) += grad * GetW(p, i);
          }
        }
      } else {
        for (auto [jArg, kArg] : std::initializer_list<std::tuple<int, int>>{
                 {0, 0}, {1, 0}, {1, 1}}) {
          if (tape[i].args[jArg].tape == nullptr ||
              tape[i].args[kArg].tape == nullptr) {
            continue;
          }

          double grad =
              tape[i].Gradient(kArg).Value() * tape[i].Gradient(jArg).Value();
          if (grad == 0.0) {
            continue;
          }

          // w_{jk} += ∂ϕᵢ/∂vₖ ∂ϕᵢ/∂vⱼ w_{ii}
          int j = tape[i].args[jArg].index;
          int k = tape[i].args[kArg].index;
          GetWRef(j, k) += grad * GetW(i, i);
        }
      }
    }

    // Creating
    for (auto [jArg, kArg] :
         std::initializer_list<std::tuple<int, int>>{{0, 0}, {1, 0}, {1, 1}}) {
      if (tape[i].args[kArg].tape == nullptr) {
        continue;
      }
      auto grad_i_wrt_k = tape[i].Gradient(kArg);

      if (grad_i_wrt_k.GetNode().args[jArg].tape == nullptr) {
        continue;
      }
      double grad2_i_wrt_kj = grad_i_wrt_k.Gradient(jArg).Value();
      if (grad2_i_wrt_kj == 0.0) {
        continue;
      }

      // w_{jk} += v̅ᵢ ∂²ϕᵢ/∂vₖ∂vⱼ
      int j = grad_i_wrt_k.GetNode().args[jArg].index;
      int k = tape[i].args[kArg].index;
      GetWRef(j, k) += tape[i].adjoint * grad2_i_wrt_kj;
    }

    // Adjoint
    for (int jArg = 0; jArg < TapeNode::kNumArgs; ++jArg) {
      // v̅ⱼ += v̅ᵢ ∂ϕᵢ/∂vⱼ
      if (tape[i].args[jArg].tape != nullptr) {
        tape[i].args[jArg].GetNode().adjoint +=
            tape[i].adjoint * tape[i].Gradient(jArg).Value();
      }
    }
  }

  tape.Resize(tapeSize);

  Eigen::SparseMatrix<double> P{wrt.rows(), tape.Size()};
  P.setZero();
  for (int row = 0; row < wrt.rows(); ++row) {
    P.coeffRef(row, wrt(row).index) = 1.0;
  }

  W.makeCompressed();
  Eigen::SparseMatrix<double> H = P * W * P.transpose();

  // Make H symmetric by iterating over the lower triangular elements and
  // copying them to the upper triangle
  for (int row = 1; row < H.rows(); ++row) {
    for (int col = 0; col < row; ++col) {
      H.coeffRef(col, row) = H.coeff(row, col);
    }
  }

  fmt::print("P =\n{}", P);
  fmt::print("W =\n{}", W);
  fmt::print("H =\n{}", H);

  return H;
}

Variable abs(double x) {  // NOLINT
  return abs(Constant(x));
}

Variable abs(const Variable& x) {  // NOLINT
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      x, [](double x) { return std::abs(x); },
      [](const Variable& x) -> Variable {
        if (x.Value() < 0.0) {
          return Constant(-1.0);
        } else if (x.Value() > 0.0) {
          return Constant(1.0);
        } else {
          return Constant(0.0);
        }
      })};
}

Variable acos(double x) {  // NOLINT
  return acos(Constant(x));
}

Variable acos(const Variable& x) {  // NOLINT
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      x, [](double x) { return std::acos(x); },
      [](const Variable& x) -> Variable {
        return -1.0 / frc::autodiff::sqrt(1.0 - x * x);
      })};
}

Variable asin(double x) {  // NOLINT
  return asin(Constant(x));
}

Variable asin(const Variable& x) {  // NOLINT
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      x, [](double x) { return std::asin(x); },
      [](const Variable& x) -> Variable {
        return 1.0 / frc::autodiff::sqrt(1.0 - x * x);
      })};
}

Variable atan(double x) {  // NOLINT
  return atan(Constant(x));
}

Variable atan(const Variable& x) {  // NOLINT
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      x, [](double x) { return std::atan(x); },
      [](const Variable& x) -> Variable { return 1.0 / (1.0 + x * x); })};
}

Variable atan2(double y, const Variable& x) {  // NOLINT
  return atan2(Constant(y), x);
}

Variable atan2(const Variable& y, double x) {  // NOLINT
  return atan2(y, Constant(x));
}

Variable atan2(const Variable& y, const Variable& x) {  // NOLINT
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushBinary(
      y, x, [](double y, double x) { return std::atan2(y, x); },
      [](const Variable& y, const Variable& x) -> Variable {
        return x / (y * y + x * x);
      },
      [](const Variable& y, const Variable& x) -> Variable {
        return -y / (y * y + x * x);
      })};
}

Variable cos(double x) {  // NOLINT
  return cos(Constant(x));
}

Variable cos(const Variable& x) {  // NOLINT
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      x, [](double x) { return std::cos(x); },
      [](const Variable& x) -> Variable { return -frc::autodiff::sin(x); })};
}

Variable cosh(double x) {  // NOLINT
  return cosh(Constant(x));
}

Variable cosh(const Variable& x) {  // NOLINT
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      x, [](double x) { return std::cosh(x); },
      [](const Variable& x) -> Variable { return frc::autodiff::sinh(x); })};
}

Variable erf(double x) {  // NOLINT
  return erf(Constant(x));
}

Variable erf(const Variable& x) {  // NOLINT
  static constexpr double sqrt_pi =
      1.7724538509055160272981674833411451872554456638435L;

  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      x, [](double x) { return std::erf(x); },
      [](const Variable& x) -> Variable {
        return 2.0 / sqrt_pi * frc::autodiff::exp(-x * x);
      })};
}

Variable exp(double x) {  // NOLINT
  return exp(Constant(x));
}

Variable exp(const Variable& x) {  // NOLINT
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      x, [](double x) { return std::exp(x); },
      [](const Variable& x) -> Variable { return frc::autodiff::exp(x); })};
}

Variable hypot(double x, const Variable& y) {  // NOLINT
  return hypot(Constant(x), y);
}

Variable hypot(const Variable& x, double y) {  // NOLINT
  return hypot(x, Constant(y));
}

Variable hypot(const Variable& x, const Variable& y) {  // NOLINT
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushBinary(
      x, y, [](double x, double y) { return std::hypot(x, y); },
      [](const Variable& x, const Variable& y) -> Variable {
        return x / frc::autodiff::hypot(x, y);
      },
      [](const Variable& x, const Variable& y) -> Variable {
        return y / frc::autodiff::hypot(x, y);
      })};
}

Variable log(double x) {  // NOLINT
  return log(Constant(x));
}

Variable log(const Variable& x) {  // NOLINT
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      x, [](double x) { return std::log(x); },
      [](const Variable& x) -> Variable { return 1.0 / x; })};
}

Variable log10(double x) {  // NOLINT
  return log10(Constant(x));
}

Variable log10(const Variable& x) {  // NOLINT
  static constexpr double ln10 = 2.3025850929940456840179914546843L;

  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      x, [](double x) { return std::log10(x); },
      [](const Variable& x) -> Variable { return 1.0 / (ln10 * x); })};
}

Variable pow(double base, const Variable& power) {  // NOLINT
  return pow(Constant(base), power);
}

Variable pow(const Variable& base, double power) {  // NOLINT
  return pow(base, Constant(power));
}

Variable pow(const Variable& base, const Variable& power) {  // NOLINT
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushBinary(
      base, power,
      [](double base, double power) { return std::pow(base, power); },
      [](const Variable& base, const Variable& power) -> Variable {
        return frc::autodiff::pow(base, power - 1) * power;
      },
      [](const Variable& base, const Variable& power) -> Variable {
        // Since x * std::log(x) -> 0 as x -> 0
        if (base.Value() == 0.0) {
          return Constant(0.0);
        } else {
          return frc::autodiff::pow(base, power - 1) * base *
                 frc::autodiff::log(base);
        }
      })};
}

Variable sin(double x) {  // NOLINT
  return sin(Constant(x));
}

Variable sin(const Variable& x) {  // NOLINT
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      x, [](double x) { return std::sin(x); },
      [](const Variable& x) -> Variable { return frc::autodiff::cos(x); })};
}

Variable sinh(double x) {  // NOLINT
  return sinh(Constant(x));
}

Variable sinh(const Variable& x) {  // NOLINT
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      x, [](double x) { return std::sinh(x); },
      [](const Variable& x) -> Variable { return frc::autodiff::cosh(x); })};
}

Variable sqrt(double x) {  // NOLINT
  return sqrt(Constant(x));
}

Variable sqrt(const Variable& x) {  // NOLINT
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      x, [](double x) { return std::sqrt(x); },
      [](const Variable& x) -> Variable {
        return 1.0 / (2.0 * frc::autodiff::sqrt(x));
      })};
}

Variable tan(double x) {  // NOLINT
  return tan(Constant(x));
}

Variable tan(const Variable& x) {  // NOLINT
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      x, [](double x) { return std::tan(x); },
      [](const Variable& x) -> Variable {
        return 1.0 / (frc::autodiff::cos(x) * frc::autodiff::cos(x));
      })};
}

Variable tanh(double x) {  // NOLINT
  return tanh(Constant(x));
}

Variable tanh(const Variable& x) {  // NOLINT
  auto& tape = Tape::GetCurrentTape();
  return Variable{tape.PushUnary(
      x, [](double x) { return std::tanh(x); },
      [](const Variable& x) -> Variable {
        return 1.0 / (frc::autodiff::cosh(x) * frc::autodiff::cosh(x));
      })};
}

}  // namespace frc::autodiff
