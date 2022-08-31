// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/optimization/VariableBlock.h"

namespace frc {

template <typename Mat>
VariableBlock<Mat>::VariableBlock(Mat& mat, int rowOffset, int colOffset,
                                  int blockRows, int blockCols)
    : m_mat{&mat},
      m_rowOffset{rowOffset},
      m_colOffset{colOffset},
      m_blockRows{blockRows},
      m_blockCols{blockCols} {}

template <typename Mat>
VariableBlock<Mat>& VariableBlock<Mat>::operator=(double value) {
  assert(Rows() == 1 && Cols() == 1);

  Autodiff(0, 0) = value;

  return *this;
}

template <typename Mat>
VariableBlock<Mat> VariableBlock<Mat>::operator()(int row, int col) {
  assert(row < Rows() && col < Cols());
  return VariableBlock{(*m_mat)(m_rowOffset + row, m_colOffset + col)};
}

template <typename Mat>
VariableBlock<const Mat> VariableBlock<Mat>::operator()(int row,
                                                        int col) const {
  assert(row < Rows() && col < Cols());
  return VariableBlock{(*m_mat)(m_rowOffset + row, m_colOffset + col)};
}

template <typename Mat>
VariableBlock<Mat> VariableBlock<Mat>::operator()(int row) {
  return VariableBlock{(*m_mat)(row)};
}

template <typename Mat>
VariableBlock<const Mat> VariableBlock<Mat>::operator()(int row) const {
  return VariableBlock{(*m_mat)(row)};
}

template <typename Mat>
VariableBlock<Mat> VariableBlock<Mat>::Block(int rowOffset, int colOffset,
                                             int blockRows, int blockCols) {
  return VariableBlock{*m_mat, rowOffset, colOffset, blockRows, blockCols};
}

template <typename Mat>
const VariableBlock<const Mat> VariableBlock<Mat>::Block(int rowOffset,
                                                         int colOffset,
                                                         int blockRows,
                                                         int blockCols) const {
  return VariableBlock{*m_mat, rowOffset, colOffset, blockRows, blockCols};
}

template <typename Mat>
VariableBlock<Mat> VariableBlock<Mat>::Row(int row) {
  return Block(row, 0, 1, Cols());
}

template <typename Mat>
VariableBlock<const Mat> VariableBlock<Mat>::Row(int row) const {
  return Block(row, 0, 1, Cols());
}

template <typename Mat>
VariableBlock<Mat> VariableBlock<Mat>::Col(int col) {
  return Block(0, col, Rows(), 1);
}

template <typename Mat>
VariableBlock<const Mat> VariableBlock<Mat>::Col(int col) const {
  return Block(0, col, Rows(), 1);
}

template <typename Mat>
VariableBlock<Mat>& VariableBlock<Mat>::operator*=(
    const VariableBlock<Mat>& rhs) {
  assert(Cols() == rhs.Rows() && Cols() == rhs.Cols());

  for (int i = 0; i < Rows(); ++i) {
    for (int j = 0; j < rhs.Cols(); ++j) {
      autodiff::Variable sum;
      for (int k = 0; k < Cols(); ++k) {
        sum += Autodiff(i, k) * rhs.Autodiff(k, j);
      }
      Autodiff(i, j) = sum;
    }
  }

  return *this;
}

template <typename Mat>
VariableBlock<Mat>& VariableBlock<Mat>::operator*=(double rhs) {
  for (int row = 0; row < Rows(); ++row) {
    for (int col = 0; col < Cols(); ++col) {
      Autodiff(row, col) *= autodiff::Variable{rhs};
    }
  }

  return *this;
}

template <typename Mat>
VariableBlock<Mat>& VariableBlock<Mat>::operator/=(double rhs) {
  for (int row = 0; row < Rows(); ++row) {
    for (int col = 0; col < Cols(); ++col) {
      Autodiff(row, col) /= autodiff::Variable{rhs};
    }
  }

  return *this;
}

template <typename Mat>
VariableBlock<Mat>& VariableBlock<Mat>::operator+=(
    const VariableBlock<Mat>& rhs) {
  for (int row = 0; row < Rows(); ++row) {
    for (int col = 0; col < Cols(); ++col) {
      Autodiff(row, col) += rhs.Autodiff(row, col);
    }
  }

  return *this;
}

template <typename Mat>
VariableBlock<Mat>& VariableBlock<Mat>::operator-=(
    const VariableBlock<Mat>& rhs) {
  for (int row = 0; row < Rows(); ++row) {
    for (int col = 0; col < Cols(); ++col) {
      Autodiff(row, col) -= rhs.Autodiff(row, col);
    }
  }

  return *this;
}

template <typename Mat>
Mat VariableBlock<Mat>::Transpose() const {
  Mat result{Cols(), Rows()};

  for (int row = 0; row < Rows(); ++row) {
    for (int col = 0; col < Cols(); ++col) {
      result.Autodiff(col, row) = Autodiff(row, col);
    }
  }

  return result;
}

template <typename Mat>
int VariableBlock<Mat>::Rows() const {
  return m_blockRows;
}

template <typename Mat>
int VariableBlock<Mat>::Cols() const {
  return m_blockCols;
}

template <typename Mat>
double VariableBlock<Mat>::Value(int row, int col) const {
  return m_mat->Autodiff(m_rowOffset + row, m_colOffset + col).Value();
}

template <typename Mat>
double VariableBlock<Mat>::Value(int index) const {
  return m_mat
      ->Autodiff(m_rowOffset + index / m_blockCols,
                 m_colOffset + index % m_blockCols)
      .Value();
}

template <typename Mat>
Eigen::MatrixXd VariableBlock<Mat>::Value() const {
  Eigen::MatrixXd result{Rows(), Cols()};

  for (int row = 0; row < Rows(); ++row) {
    for (int col = 0; col < Cols(); ++col) {
      result(row, col) = Value(row, col);
    }
  }

  return result;
}

template <typename Mat>
autodiff::Variable& VariableBlock<Mat>::Autodiff(int row, int col) {
  return m_mat->Autodiff(m_rowOffset + row, m_colOffset + col);
}

template <typename Mat>
const autodiff::Variable& VariableBlock<Mat>::Autodiff(int row, int col) const {
  return m_mat->Autodiff(m_rowOffset + row, m_colOffset + col);
}

template <typename Mat>
Mat abs(const VariableBlock<Mat>& x) {  // NOLINT
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = abs(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat acos(const VariableBlock<Mat>& x) {  // NOLINT
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = acos(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat asin(const VariableBlock<Mat>& x) {  // NOLINT
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = asin(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat atan(const VariableBlock<Mat>& x) {  // NOLINT
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = atan(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat atan2(const VariableBlock<Mat>& y,  // NOLINT
          const VariableBlock<Mat>& x) {
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) =
          atan2(y.Autodiff(row, col), x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat cos(const VariableBlock<Mat>& x) {  // NOLINT
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = cos(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat cosh(const VariableBlock<Mat>& x) {  // NOLINT
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = cosh(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat erf(const VariableBlock<Mat>& x) {  // NOLINT
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = erf(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat exp(const VariableBlock<Mat>& x) {  // NOLINT
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = exp(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat hypot(const VariableBlock<Mat>& x,  // NOLINT
          const VariableBlock<Mat>& y) {
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) =
          hypot(x.Autodiff(row, col), y.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat log(const VariableBlock<Mat>& x) {  // NOLINT
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = log(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat log10(const VariableBlock<Mat>& x) {  // NOLINT
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = log10(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat pow(const VariableBlock<Mat>& base,
        const VariableBlock<Mat>& power) {  // NOLINT
  Mat result{base.Rows(), base.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) =
          pow(base.Autodiff(row, col), power.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat sin(const VariableBlock<Mat>& x) {  // NOLINT
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = sin(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat sinh(const VariableBlock<Mat>& x) {  // NOLINT
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = sinh(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat sqrt(const VariableBlock<Mat>& x) {  // NOLINT
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = sqrt(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat tan(const VariableBlock<Mat>& x) {  // NOLINT
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = tan(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

template <typename Mat>
Mat tanh(const VariableBlock<Mat>& x) {  // NOLINT
  Mat result{x.Rows(), x.Cols()};

  for (int row = 0; row < result.Rows(); ++row) {
    for (int col = 0; col < result.Cols(); ++col) {
      result.Autodiff(row, col) = tanh(x.Autodiff(row, col));  // NOLINT
    }
  }

  return result;
}

}  // namespace frc
