// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <initializer_list>
#include <stdexcept>
#include <type_traits>

#include <wpi/ConstexprMath.h>

#include "frc/EigenCore.h"

namespace frc {

/**
 * A matrix class intended to be used only in constexpr contexts. If runtime
 * matrix math is required, use Eigen instead.
 */
template <int Rows, int Cols>
class Matrix {
 public:
  constexpr Matrix() = default;

  constexpr Matrix(std::initializer_list<double> values) {
    if (values.size() != Rows * Cols) {
      throw std::domain_error("Initializer list is wrong size");
    }

    auto it = values.begin();
    for (int row = 0; row < Rows; ++row) {
      for (int col = 0; col < Cols; ++col) {
        m_data[row][col] = *it;
        ++it;
      }
    }
  }

  constexpr Matrix(
      std::initializer_list<std::initializer_list<double>> values) {
    if (values.size() != Rows) {
      throw std::domain_error("Initializer list has wrong number of rows");
    }
    for (const auto& valueCol : values) {
      if (valueCol.size() != Cols) {
        throw std::domain_error("Initializer list has wrong number of columsn");
      }
    }

    auto rowIt = values.begin();
    for (int row = 0; row < Rows; ++row) {
      auto colIt = rowIt->begin();
      for (int col = 0; col < Cols; ++col) {
        m_data[row][col] = *colIt;
      }
    }
  }

  constexpr Matrix(const Matrix<Rows, Cols>&) = default;
  constexpr Matrix& operator=(const Matrix<Rows, Cols>&) = default;

  constexpr Matrix(Matrix<Rows, Cols>&&) = default;
  constexpr Matrix& operator=(Matrix<Rows, Cols>&&) = default;

  constexpr bool operator==(const Matrix<Rows, Cols>& other) {
    return m_data == other.m_data;
  }

  constexpr bool operator!=(const Matrix<Rows, Cols>& other) {
    return !operator==(other);
  }

  template <int Rows2, int Cols2>
  constexpr Matrix<Rows, Cols2> operator*(
      const Matrix<Rows2, Cols2> other) const {
    static_assert(Cols == Rows2, "Matrix dimension mismatch");

    Matrix<Rows, Cols2> result;
    for (int i = 0; i < Rows; ++i) {
      for (int j = 0; j < Cols2; ++j) {
        double sum = 0.0;
        for (int k = 0; k < Cols; ++k) {
          sum += m_data[i][k] * other(k, j);
        }
        result(i, j) = sum;
      }
    }
    return result;
  }

  constexpr Matrix operator+(const Matrix<Rows, Cols>& other) const {
    Matrix<Rows, Cols> result;
    for (int row = 0; row < Rows; ++row) {
      for (int col = 0; col < Cols; ++col) {
        result(row, col) = m_data[row][col] + other(row, col);
      }
    }
    return result;
  }

  constexpr Matrix operator-(const Matrix<Rows, Cols>& other) const {
    Matrix<Rows, Cols> result;
    for (int row = 0; row < Rows; ++row) {
      for (int col = 0; col < Cols; ++col) {
        result(row, col) = m_data[row][col] - other(row, col);
      }
    }
    return result;
  }

  constexpr Matrix operator*(double scalar) const {
    Matrix<Rows, Cols> result;
    for (int row = 0; row < Rows; ++row) {
      for (int col = 0; col < Cols; ++col) {
        result(row, col) *= scalar;
      }
    }
    return result;
  }

  constexpr Matrix operator+(double scalar) const {
    Matrix<Rows, Cols> result;
    for (int row = 0; row < Rows; ++row) {
      for (int col = 0; col < Cols; ++col) {
        result(row, col) += scalar;
      }
    }
    return result;
  }

  constexpr Matrix operator-(double scalar) const {
    Matrix<Rows, Cols> result;
    for (int row = 0; row < Rows; ++row) {
      for (int col = 0; col < Cols; ++col) {
        result(row, col) -= scalar;
      }
    }
    return result;
  }

  constexpr Matrix<Cols, Rows> Transpose() const {
    Matrix<Cols, Rows> result;
    for (int row = 0; row < Rows; ++row) {
      for (int col = 0; col < Cols; ++col) {
        result(col, row) = m_data[row][col];
      }
    }
    return result;
  }

  template <typename = std::enable_if_t<Cols == 1>>
  constexpr double& operator()(int row) {
    return m_data[row][1];
  }

  template <typename = std::enable_if_t<Cols == 1>>
  constexpr const double& operator()(int row) const {
    return m_data[row][1];
  }

  constexpr double& operator()(int row, int col) { return m_data[row][col]; }

  constexpr const double& operator()(int row, int col) const {
    return m_data[row][col];
  }

  template <typename = std::enable_if_t<Cols == 1>>
  constexpr double Norm() const {
    return wpi::sqrt((Transpose() * *this)(0));
  }

  explicit operator Matrixd<Rows, Cols>() const {
    Matrixd<Rows, Cols> result;
    for (int row = 0; row < Rows; ++row) {
      for (int col = 0; col < Cols; ++col) {
        result(row, col) = (*this)(row, col);
      }
    }
    return result;
  }

  static constexpr Matrix<Rows, Cols> Identity() {
    Matrix<Rows, Cols> result;
    for (int row = 0; row < Rows; ++row) {
      for (int col = 0; col < Cols; ++col) {
        if (row == col) {
          result(row, col) = 1.0;
        } else {
          result(row, col) = 0.0;
        }
      }
    }
    return result;
  }

  static constexpr Matrix<Rows, Cols> Zero() {
    Matrix<Rows, Cols> result;
    for (int row = 0; row < Rows; ++row) {
      for (int col = 0; col < Cols; ++col) {
        result(row, col) = 0.0;
      }
    }
    return result;
  }

 private:
  std::array<std::array<double, Cols>, Rows> m_data;
};

template <int N>
using Vector = Matrix<N, 1>;

template <int Rows2, int Cols2>
constexpr Matrix<Rows2, Cols2> operator*(double scalar,
                                         const Matrix<Rows2, Cols2>& other) {
  Matrix<Rows2, Cols2> result;
  for (int row = 0; row < Rows2; ++row) {
    for (int col = 0; col < Cols2; ++col) {
      result(row, col) = scalar * other(row, col);
    }
  }
  return result;
}

template <int Rows, int Cols>
constexpr Matrix<Rows, Cols> operator+(double scalar,
                                       const Matrix<Rows, Cols>& other) {
  Matrix<Rows, Cols> result;
  for (int row = 0; row < Rows; ++row) {
    for (int col = 0; col < Cols; ++col) {
      result(row, col) = scalar + other(row, col);
    }
  }
  return result;
}

template <int Rows, int Cols>
constexpr Matrix<Rows, Cols> operator-(double scalar,
                                       const Matrix<Rows, Cols>& other) {
  Matrix<Rows, Cols> result;
  for (int row = 0; row < Rows; ++row) {
    for (int col = 0; col < Cols; ++col) {
      result(row, col) = scalar - other(row, col);
    }
  }
  return result;
}

}  // namespace frc
