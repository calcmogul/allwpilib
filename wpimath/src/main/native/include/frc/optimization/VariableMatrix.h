// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cassert>
#include <utility>

#include <wpi/SmallVector.h>
#include <wpi/SymbolExports.h>

#include "frc/EigenCore.h"
#include "frc/autodiff/Variable.h"
#include "frc/optimization/VariableBlock.h"

namespace frc {

class WPILIB_DLLEXPORT VariableMatrix {
 public:
  VariableMatrix(int rows, int cols);

  VariableMatrix(double value);  // NOLINT

  VariableMatrix& operator=(double value);

  template <int _Rows, int _Cols>
  VariableMatrix(const frc::Matrixd<_Rows, _Cols>& values)  // NOLINT
      : m_rows{_Rows}, m_cols{_Cols} {
    m_storage.reserve(_Rows * _Cols);
    for (size_t row = 0; row < _Rows; ++row) {
      for (size_t col = 0; col < _Cols; ++col) {
        m_storage.emplace_back(values(row, col));
      }
    }
  }

  template <int _Rows, int _Cols>
  VariableMatrix& operator=(const frc::Matrixd<_Rows, _Cols>& values) {
    for (size_t row = 0; row < _Rows; ++row) {
      for (size_t col = 0; col < _Cols; ++col) {
        Autodiff(row, col) = values(row, col);
      }
    }

    return *this;
  }

  template <int _Rows, int _Cols>
  explicit VariableMatrix(frc::Matrixd<_Rows, _Cols>&& values)
      : m_rows{_Rows}, m_cols{_Cols} {
    m_storage.clear();
    m_storage.reserve(_Rows * _Cols);
    for (size_t row = 0; row < _Rows; ++row) {
      for (size_t col = 0; col < _Cols; ++col) {
        m_storage.emplace_back(values(row, col));
      }
    }
  }

  template <int _Rows, int _Cols>
  VariableMatrix& operator=(frc::Matrixd<_Rows, _Cols>&& values) {
    for (size_t row = 0; row < _Rows; ++row) {
      for (size_t col = 0; col < _Cols; ++col) {
        Autodiff(row, col) = values(row, col);
      }
    }

    return *this;
  }

  template <int _Rows, int _Cols>
  explicit VariableMatrix(
      const Eigen::Matrix<autodiff::Variable, _Rows, _Cols>& values)
      : m_rows{_Rows}, m_cols{_Cols} {
    m_storage.clear();
    m_storage.reserve(_Rows * _Cols);
    for (size_t row = 0; row < _Rows; ++row) {
      for (size_t col = 0; col < _Cols; ++col) {
        m_storage.emplace_back(values(row, col));
      }
    }
  }

  template <int _Rows, int _Cols>
  VariableMatrix& operator=(
      const Eigen::Matrix<autodiff::Variable, _Rows, _Cols>& values) {
    m_storage = values;
    return *this;
  }

  template <int _Rows, int _Cols>
  explicit VariableMatrix(
      Eigen::Matrix<autodiff::Variable, _Rows, _Cols>&& values)
      : m_rows{_Rows}, m_cols{_Cols} {
    m_storage.clear();
    m_storage.reserve(_Rows * _Cols);
    for (size_t row = 0; row < _Rows; ++row) {
      for (size_t col = 0; col < _Cols; ++col) {
        m_storage.emplace_back(values(row, col));
      }
    }
  }

  template <int _Rows, int _Cols>
  VariableMatrix& operator=(
      Eigen::Matrix<autodiff::Variable, _Rows, _Cols>&& values) {
    m_storage = std::move(values);
    return *this;
  }

  VariableMatrix(const autodiff::Variable& variable);  // NOLINT

  VariableMatrix(autodiff::Variable&& variable);  // NOLINT

  VariableMatrix(const VariableBlock<VariableMatrix>& values);  // NOLINT

  VariableMatrix(const VariableBlock<const VariableMatrix>& values);  // NOLINT

  VariableBlock<VariableMatrix> operator()(int row, int col);

  VariableBlock<const VariableMatrix> operator()(int row, int col) const;

  VariableBlock<VariableMatrix> operator()(int row);

  VariableBlock<const VariableMatrix> operator()(int row) const;

  /**
   * Returns a block slice of the variable matrix.
   *
   * @param rowOffset The row offset of the block selection.
   * @param colOffset The column offset of the block selection.
   * @param blockRows The number of rows in the block selection.
   * @param blockCols The number of columns in the block selection.
   */
  VariableBlock<VariableMatrix> Block(int rowOffset, int colOffset,
                                      int blockRows, int blockCols);

  /**
   * Returns a block slice of the variable matrix.
   *
   * @param rowOffset The row offset of the block selection.
   * @param colOffset The column offset of the block selection.
   * @param blockRows The number of rows in the block selection.
   * @param blockCols The number of columns in the block selection.
   */
  const VariableBlock<const VariableMatrix> Block(int rowOffset, int colOffset,
                                                  int blockRows,
                                                  int blockCols) const;

  /**
   * Returns a row slice of the variable matrix.
   *
   * @param row The row to slice.
   */
  VariableBlock<VariableMatrix> Row(int row);

  /**
   * Returns a row slice of the variable matrix.
   *
   * @param row The row to slice.
   */
  VariableBlock<const VariableMatrix> Row(int row) const;

  /**
   * Returns a column slice of the variable matrix.
   *
   * @param col The column to slice.
   */
  VariableBlock<VariableMatrix> Col(int col);

  /**
   * Returns a column slice of the variable matrix.
   *
   * @param col The column to slice.
   */
  VariableBlock<const VariableMatrix> Col(int col) const;

  /**
   * Matrix multiplication operator.
   *
   * @param lhs Operator left-hand side.
   * @param rhs Operator right-hand side.
   */
  friend VariableMatrix operator*(const VariableMatrix& lhs,
                                  const VariableMatrix& rhs);

  /**
   * Matrix-scalar multiplication operator.
   *
   * @param lhs Operator left-hand side.
   * @param rhs Operator right-hand side.
   */
  friend VariableMatrix operator*(const VariableMatrix& lhs, double rhs);

  /**
   * Scalar-matrix multiplication operator.
   *
   * @param lhs Operator left-hand side.
   * @param rhs Operator right-hand side.
   */
  friend VariableMatrix operator*(double lhs, const VariableMatrix& rhs);

  /**
   * Compound matrix multiplication-assignment operator.
   *
   * @param rhs Variable to multiply.
   */
  VariableMatrix& operator*=(const VariableMatrix& rhs);

  /**
   * Compound matrix multiplication-assignment operator (only enabled when lhs
   * is a scalar).
   *
   * @param rhs Variable to multiply.
   */
  VariableMatrix& operator*=(double rhs);

  /**
   * Binary division operator.
   *
   * @param lhs Operator left-hand side.
   * @param rhs Operator right-hand side.
   */
  friend VariableMatrix operator/(const VariableMatrix& lhs, double rhs);

  /**
   * Compound matrix division-assignment operator (only enabled when lhs
   * is a scalar).
   *
   * @param rhs Variable to divide.
   */
  VariableMatrix& operator/=(double rhs);

  /**
   * Binary addition operator.
   *
   * @param lhs Operator left-hand side.
   * @param rhs Operator right-hand side.
   */
  friend VariableMatrix operator+(const VariableMatrix& lhs,
                                  const VariableMatrix& rhs);

  /**
   * Compound addition-assignment operator.
   *
   * @param rhs Variable to add.
   */
  VariableMatrix& operator+=(const VariableMatrix& rhs);

  /**
   * Binary subtraction operator.
   *
   * @param lhs Operator left-hand side.
   * @param rhs Operator right-hand side.
   */
  friend VariableMatrix operator-(const VariableMatrix& lhs,
                                  const VariableMatrix& rhs);

  /**
   * Compound subtraction-assignment operator.
   *
   * @param rhs Variable to subtract.
   */
  VariableMatrix& operator-=(const VariableMatrix& rhs);

  /**
   * Unary minus operator.
   *
   * @param lhs Operand for unary minus.
   */
  friend VariableMatrix operator-(const VariableMatrix& lhs);

  /**
   * Returns the transpose of the variable matrix.
   */
  VariableMatrix Transpose() const;

  /**
   * Returns number of rows in the matrix.
   */
  int Rows() const;

  /**
   * Returns number of columns in the matrix.
   */
  int Cols() const;

  /**
   * Returns an element of the variable matrix.
   *
   * @param row The row of the element to return.
   * @param col The column of the element to return.
   */
  double Value(int row, int col) const;

  /**
   * Returns a row of the variable column vector.
   *
   * @param index The index of the element to return.
   */
  double Value(int index) const;

  /**
   * Returns the contents of the variable matrix.
   */
  Eigen::MatrixXd Value() const;

  /**
   * Returns the autodiff variable backing a matrix entry.
   */
  autodiff::Variable& Autodiff(int row, int col);

  /**
   * Returns the autodiff variable backing a matrix entry.
   */
  const autodiff::Variable& Autodiff(int row, int col) const;

 private:
  wpi::SmallVector<autodiff::Variable, 32> m_storage;
  int m_rows = 0;
  int m_cols = 0;
};

/**
 * std::abs() for VariableMatrices.
 *
 * The function is applied element-wise to the argument.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT VariableMatrix abs(const VariableMatrix& x);  // NOLINT

/**
 * std::acos() for VariableMatrices.
 *
 * The function is applied element-wise to the argument.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT VariableMatrix acos(const VariableMatrix& x);  // NOLINT

/**
 * std::asin() for VariableMatrices.
 *
 * The function is applied element-wise to the argument.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT VariableMatrix asin(const VariableMatrix& x);  // NOLINT

/**
 * std::atan() for VariableMatrices.
 *
 * The function is applied element-wise to the argument.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT VariableMatrix atan(const VariableMatrix& x);  // NOLINT

/**
 * std::atan2() for VariableMatrices.
 *
 * The function is applied element-wise to the arguments.
 *
 * @param y The y argument.
 * @param x The x argument.
 */
WPILIB_DLLEXPORT VariableMatrix atan2(const VariableMatrix& y,  // NOLINT
                                      const VariableMatrix& x);

/**
 * std::cos() for VariableMatrices.
 *
 * The function is applied element-wise to the argument.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT VariableMatrix cos(const VariableMatrix& x);  // NOLINT

/**
 * std::cosh() for VariableMatrices.
 *
 * The function is applied element-wise to the argument.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT VariableMatrix cosh(const VariableMatrix& x);  // NOLINT

/**
 * std::erf() for VariableMatrices.
 *
 * The function is applied element-wise to the argument.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT VariableMatrix erf(const VariableMatrix& x);  // NOLINT

/**
 * std::exp() for VariableMatrices.
 *
 * The function is applied element-wise to the argument.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT VariableMatrix exp(const VariableMatrix& x);  // NOLINT

/**
 * std::hypot() for VariableMatrices.
 *
 * The function is applied element-wise to the arguments.
 *
 * @param x The x argument.
 * @param y The y argument.
 */
WPILIB_DLLEXPORT VariableMatrix hypot(const VariableMatrix& x,  // NOLINT
                                      const VariableMatrix& y);

/**
 * std::log() for VariableMatrices.
 *
 * The function is applied element-wise to the argument.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT VariableMatrix log(const VariableMatrix& x);  // NOLINT

/**
 * std::log10() for VariableMatrices.
 *
 * The function is applied element-wise to the argument.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT VariableMatrix log10(const VariableMatrix& x);  // NOLINT

/**
 * std::pow() for VariableMatrices.
 *
 * The function is applied element-wise to the arguments.
 *
 * @param base The base.
 * @param power The power.
 */
WPILIB_DLLEXPORT VariableMatrix pow(const VariableMatrix& base,  // NOLINT
                                    const VariableMatrix& power);

/**
 * std::sin() for VariableMatrices.
 *
 * The function is applied element-wise to the argument.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT VariableMatrix sin(const VariableMatrix& x);  // NOLINT

/**
 * std::sinh() for VariableMatrices.
 *
 * The function is applied element-wise to the argument.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT VariableMatrix sinh(const VariableMatrix& x);  // NOLINT

/**
 * std::sqrt() for VariableMatrices.
 *
 * The function is applied element-wise to the argument.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT VariableMatrix sqrt(const VariableMatrix& x);  // NOLINT

/**
 * std::tan() for VariableMatrices.
 *
 * The function is applied element-wise to the argument.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT VariableMatrix tan(const VariableMatrix& x);  // NOLINT

/**
 * std::tanh() for VariableMatrices.
 *
 * The function is applied element-wise to the argument.
 *
 * @param x The argument.
 */
WPILIB_DLLEXPORT VariableMatrix tanh(const VariableMatrix& x);  // NOLINT

}  // namespace frc
