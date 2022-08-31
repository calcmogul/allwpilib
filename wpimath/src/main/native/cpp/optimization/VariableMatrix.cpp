// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/optimization/VariableMatrix.h"

namespace frc {

VariableMatrix::VariableMatrix(int rows, int cols)
    : m_rows{rows}, m_cols{cols} {
  m_storage.resize(rows * cols);
}

VariableMatrix::VariableMatrix(double value) : m_rows{1}, m_cols{1} {
  m_storage.emplace_back(value);
}

VariableMatrix& VariableMatrix::operator=(double value) {
  assert(Rows() == 1 && Cols() == 1);

  Autodiff(0, 0) = value;

  return *this;
}

VariableMatrix::VariableMatrix(const autodiff::Variable& variable)
    : m_rows{1}, m_cols{1} {
  m_storage.emplace_back(variable);
}

VariableMatrix::VariableMatrix(autodiff::Variable&& variable)
    : m_rows{1}, m_cols{1} {
  m_storage.emplace_back(std::move(variable));
}

VariableMatrix::VariableMatrix(const VariableBlock<VariableMatrix>& values)
    : m_rows{values.Rows()}, m_cols{values.Cols()} {
  for (int row = 0; row < Rows(); ++row) {
    for (int col = 0; col < Cols(); ++col) {
      m_storage.emplace_back(values.Autodiff(row, col));
    }
  }
}

VariableMatrix::VariableMatrix(
    const VariableBlock<const VariableMatrix>& values)
    : m_rows{values.Rows()}, m_cols{values.Cols()} {
  for (int row = 0; row < Rows(); ++row) {
    for (int col = 0; col < Cols(); ++col) {
      m_storage.emplace_back(values.Autodiff(row, col));
    }
  }
}

VariableBlock<VariableMatrix> VariableMatrix::operator()(int row, int col) {
  assert(row < Rows() && col < Cols());
  return Block(row, col, 1, 1);
}

VariableBlock<const VariableMatrix> VariableMatrix::operator()(int row,
                                                               int col) const {
  assert(row < Rows() && col < Cols());
  return Block(row, col, 1, 1);
}

VariableBlock<VariableMatrix> VariableMatrix::operator()(int row) {
  return Block(row, 0, 1, 1);
}

VariableBlock<const VariableMatrix> VariableMatrix::operator()(int row) const {
  return Block(row, 0, 1, 1);
}

VariableBlock<VariableMatrix> VariableMatrix::Block(int rowOffset,
                                                    int colOffset,
                                                    int blockRows,
                                                    int blockCols) {
  return VariableBlock{*this, rowOffset, colOffset, blockRows, blockCols};
}

const VariableBlock<const VariableMatrix> VariableMatrix::Block(
    int rowOffset, int colOffset, int blockRows, int blockCols) const {
  return VariableBlock{*this, rowOffset, colOffset, blockRows, blockCols};
}

VariableBlock<VariableMatrix> VariableMatrix::Row(int row) {
  return Block(row, 0, 1, Cols());
}

VariableBlock<const VariableMatrix> VariableMatrix::Row(int row) const {
  return Block(row, 0, 1, Cols());
}

VariableBlock<VariableMatrix> VariableMatrix::Col(int col) {
  return Block(0, col, Rows(), 1);
}

VariableBlock<const VariableMatrix> VariableMatrix::Col(int col) const {
  return Block(0, col, Rows(), 1);
}

WPILIB_DLLEXPORT VariableMatrix operator*(const VariableMatrix& lhs,
                                          const VariableMatrix& rhs) {
  assert(lhs.Cols() == rhs.Rows());

  VariableMatrix result{lhs.Rows(), rhs.Cols()};

  for (int i = 0; i < lhs.Rows(); ++i) {
    for (int j = 0; j < rhs.Cols(); ++j) {
      autodiff::Variable sum = 0.0;
      for (int k = 0; k < lhs.Cols(); ++k) {
        sum += lhs.Autodiff(i, k) * rhs.Autodiff(k, j);
      }
      result.Autodiff(i, j) = sum;
    }
  }

  return result;
}

WPILIB_DLLEXPORT VariableMatrix operator*(const VariableMatrix& lhs,
                                          double rhs) {
  VariableMatrix result{lhs.Rows(), lhs.Cols()};

  autodiff::Variable rhsVar{rhs};
  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = lhs.Autodiff(row, col) * rhsVar;
    }
  }

  return result;
}

WPILIB_DLLEXPORT VariableMatrix operator*(double lhs,
                                          const VariableMatrix& rhs) {
  VariableMatrix result{rhs.Rows(), rhs.Cols()};

  autodiff::Variable lhsVar{lhs};
  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = rhs.Autodiff(row, col) * lhsVar;
    }
  }

  return result;
}

VariableMatrix& VariableMatrix::operator*=(const VariableMatrix& rhs) {
  assert(Cols() == rhs.Rows() && Cols() == rhs.Cols());

  for (int i = 0; i < Rows(); ++i) {
    for (int j = 0; j < rhs.Cols(); ++j) {
      autodiff::Variable sum = 0.0;
      for (int k = 0; k < Cols(); ++k) {
        sum += Autodiff(i, k) * rhs.Autodiff(k, j);
      }
      Autodiff(i, j) = sum;
    }
  }

  return *this;
}

VariableMatrix& VariableMatrix::operator*=(double rhs) {
  for (int row = 0; row < Rows(); ++row) {
    for (int col = 0; col < Cols(); ++col) {
      Autodiff(row, col) *= autodiff::Variable{rhs};
    }
  }

  return *this;
}

WPILIB_DLLEXPORT VariableMatrix operator/(const VariableMatrix& lhs,
                                          double rhs) {
  VariableMatrix result{lhs.Rows(), lhs.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) =
          lhs.Autodiff(row, col) / autodiff::Variable{rhs};
    }
  }

  return result;
}

VariableMatrix& VariableMatrix::operator/=(double rhs) {
  for (int row = 0; row < Rows(); ++row) {
    for (int col = 0; col < Cols(); ++col) {
      Autodiff(row, col) /= autodiff::Variable{rhs};
    }
  }

  return *this;
}

WPILIB_DLLEXPORT VariableMatrix operator+(const VariableMatrix& lhs,
                                          const VariableMatrix& rhs) {
  VariableMatrix result{lhs.Rows(), lhs.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) =
          lhs.Autodiff(row, col) + rhs.Autodiff(row, col);
    }
  }

  return result;
}

VariableMatrix& VariableMatrix::operator+=(const VariableMatrix& rhs) {
  for (int row = 0; row < Rows(); ++row) {
    for (int col = 0; col < Cols(); ++col) {
      Autodiff(row, col) += rhs.Autodiff(row, col);
    }
  }

  return *this;
}

WPILIB_DLLEXPORT VariableMatrix operator-(const VariableMatrix& lhs,
                                          const VariableMatrix& rhs) {
  VariableMatrix result{lhs.Rows(), lhs.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) =
          lhs.Autodiff(row, col) - rhs.Autodiff(row, col);
    }
  }

  return result;
}

VariableMatrix& VariableMatrix::operator-=(const VariableMatrix& rhs) {
  for (int row = 0; row < Rows(); ++row) {
    for (int col = 0; col < Cols(); ++col) {
      Autodiff(row, col) -= rhs.Autodiff(row, col);
    }
  }

  return *this;
}

WPILIB_DLLEXPORT VariableMatrix operator-(const VariableMatrix& lhs) {
  VariableMatrix result{lhs.Rows(), lhs.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = -lhs.Autodiff(row, col);
    }
  }

  return result;
}

VariableMatrix VariableMatrix::Transpose() const {
  VariableMatrix result{Cols(), Rows()};

  for (int row = 0; row < Rows(); ++row) {
    for (int col = 0; col < Cols(); ++col) {
      result.Autodiff(col, row) = Autodiff(row, col);
    }
  }

  return result;
}

int VariableMatrix::Rows() const {
  return m_rows;
}

int VariableMatrix::Cols() const {
  return m_cols;
}

double VariableMatrix::Value(int row, int col) const {
  return m_storage[row * Cols() + col].Value();
}

double VariableMatrix::Value(int index) const {
  return m_storage[index].Value();
}

Eigen::MatrixXd VariableMatrix::Value() const {
  Eigen::MatrixXd result{Rows(), Cols()};

  for (int row = 0; row < Rows(); ++row) {
    for (int col = 0; col < Cols(); ++col) {
      result(row, col) = Value(row, col);
    }
  }

  return result;
}

autodiff::Variable& VariableMatrix::Autodiff(int row, int col) {
  return m_storage[row * Cols() + col];
}

const autodiff::Variable& VariableMatrix::Autodiff(int row, int col) const {
  return m_storage[row * Cols() + col];
}

VariableMatrix abs(const VariableMatrix& x) {  // NOLINT
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = abs(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix acos(const VariableMatrix& x) {  // NOLINT
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = acos(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix asin(const VariableMatrix& x) {  // NOLINT
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = asin(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix atan(const VariableMatrix& x) {  // NOLINT
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = atan(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix atan2(const VariableMatrix& y,  // NOLINT
                     const VariableMatrix& x) {
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) =
          atan2(y.Autodiff(row, col), x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix cos(const VariableMatrix& x) {  // NOLINT
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = cos(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix cosh(const VariableMatrix& x) {  // NOLINT
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = cosh(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix erf(const VariableMatrix& x) {  // NOLINT
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = erf(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix exp(const VariableMatrix& x) {  // NOLINT
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = exp(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix hypot(const VariableMatrix& x,  // NOLINT
                     const VariableMatrix& y) {
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) =
          hypot(x.Autodiff(row, col), y.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix log(const VariableMatrix& x) {  // NOLINT
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = log(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix log10(const VariableMatrix& x) {  // NOLINT
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = log10(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix pow(const VariableMatrix& base,
                   const VariableMatrix& power) {  // NOLINT
  VariableMatrix result{base.Rows(), base.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) =
          pow(base.Autodiff(row, col), power.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix sin(const VariableMatrix& x) {  // NOLINT
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = sin(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix sinh(const VariableMatrix& x) {  // NOLINT
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = sinh(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix sqrt(const VariableMatrix& x) {  // NOLINT
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = sqrt(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix tan(const VariableMatrix& x) {  // NOLINT
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = tan(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

VariableMatrix tanh(const VariableMatrix& x) {  // NOLINT
  VariableMatrix result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = tanh(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

}  // namespace frc
