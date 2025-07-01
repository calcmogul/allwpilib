// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.autodiff;

import java.util.ArrayList;
import java.util.function.UnaryOperator;
import org.ejml.simple.SimpleMatrix;

/** A matrix of autodiff variables. */
public class VariableMatrix {
  private ArrayList<Variable> m_storage = new ArrayList<>();
  private int m_rows;
  private int m_cols;

  /** Constructs an empty VariableMatrix. */
  public VariableMatrix() {}

  /**
   * Constructs a VariableMatrix column vector with the given rows.
   *
   * @param rows The number of matrix rows.
   */
  public VariableMatrix(int rows) {
    m_rows = rows;
    m_cols = 1;
    for (int row = 0; row < rows; ++row) {
      m_storage.add(new Variable());
    }
  }

  /**
   * Constructs a zero-initialized VariableMatrix with the given dimensions.
   *
   * @param rows The number of matrix rows.
   * @param cols The number of matrix columns.
   */
  public VariableMatrix(int rows, int cols) {
    m_rows = rows;
    m_cols = cols;
    for (int index = 0; index < rows * cols; ++index) {
      m_storage.add(new Variable());
    }
  }

  /**
   * Constructs a scalar VariableMatrix from a nested list of doubles.
   *
   * <p>This overload is for Python bindings only.
   *
   * @param list The nested list of Variables.
   */
  public VariableMatrix(double[][] list) {
    // Get row and column counts for destination matrix
    m_rows = list.length;
    m_cols = 0;
    if (list.length > 0) {
      m_cols = list[0].length;
    }

    // Assert the first and latest column counts are the same
    for (var row : list) {
      assert list[0].length == row.length;
    }

    for (var row : list) {
      for (var elem : row) {
        m_storage.add(new Variable(elem));
      }
    }
  }

  /**
   * Constructs a scalar VariableMatrix from a nested list of Variables.
   *
   * <p>This overload is for Python bindings only.
   *
   * @param list The nested list of Variables.
   */
  public VariableMatrix(Variable[][] list) {
    // Get row and column counts for destination matrix
    m_rows = list.length;
    m_cols = 0;
    if (list.length > 0) {
      m_cols = list[0].length;
    }

    // Assert the first and latest column counts are the same
    for (var row : list) {
      assert list[0].length == row.length;
    }

    for (var row : list) {
      for (var elem : row) {
        m_storage.add(elem);
      }
    }
  }

  /**
   * Constructs a VariableMatrix from an Eigen matrix.
   *
   * @param values EJML matrix of values.
   */
  public VariableMatrix(SimpleMatrix values) {
    m_rows = values.numRows();
    m_cols = values.numCols();
    for (int row = 0; row < values.numRows(); ++row) {
      for (int col = 0; col < values.numCols(); ++col) {
        m_storage.add(new Variable(values.get(row, col)));
      }
    }
  }

  /**
   * Assigns an EJML matrix to a VariableMatrix.
   *
   * @param values EJML matrix of values.
   * @return This VariableMatrix.
   */
  public VariableMatrix set(SimpleMatrix values) {
    assert rows() == values.numRows() && cols() == values.numCols();

    for (int row = 0; row < values.numRows(); ++row) {
      for (int col = 0; col < values.numCols(); ++col) {
        set(row, col, values.get(row, col));
      }
    }

    return this;
  }

  /**
   * Assigns a double to the matrix.
   *
   * <p>This only works for matrices with one row and one column.
   *
   * @param value Value to assign.
   * @return This VariableMatrix.
   */
  public VariableMatrix set(double value) {
    assert rows() == 1 && cols() == 1;

    get(0, 0).setValue(value);

    return this;
  }

  /**
   * Sets the VariableMatrix's internal values.
   *
   * @param values EJML matrix of values.
   */
  public void setValue(SimpleMatrix values) {
    assert rows() == values.numRows() && cols() == values.numCols();

    for (int row = 0; row < values.numRows(); ++row) {
      for (int col = 0; col < values.numCols(); ++col) {
        get(row, col).setValue(values.get(row, col));
      }
    }
  }

  /**
   * Constructs a scalar VariableMatrix from a Variable.
   *
   * @param variable Variable.
   */
  public VariableMatrix(Variable variable) {
    m_rows = 1;
    m_cols = 1;
    m_storage.add(variable);
  }

  /**
   * Constructs a VariableMatrix from a VariableBlock.
   *
   * @param values VariableBlock of values.
   */
  public VariableMatrix(VariableBlock values) {
    m_rows = values.rows();
    m_cols = values.cols();
    for (int row = 0; row < rows(); ++row) {
      for (int col = 0; col < cols(); ++col) {
        m_storage.add(values.get(row, col));
      }
    }
  }

  /**
   * Returns a block pointing to the given row and column.
   *
   * @param row The block row.
   * @param col The block column.
   * @return A block pointing to the given row and column.
   */
  public Variable get(int row, int col) {
    assert row >= 0 && row < rows();
    assert col >= 0 && col < cols();
    return m_storage.get(row * cols() + col);
  }

  /**
   * Returns a block pointing to the given row.
   *
   * @param row The block row.
   * @return A block pointing to the given row.
   */
  public Variable get(int row) {
    assert row >= 0 && row < rows() * cols();
    return m_storage.get(row);
  }

  /**
   * Sets an element to the given value.
   *
   * @param row The row.
   * @param col The column.
   * @param value The value.
   */
  public void set(int row, int col, Variable value) {
    assert row >= 0 && row < rows();
    assert col >= 0 && col < cols();
    m_storage.set(row * cols() + col, value);
  }

  /**
   * Sets an element to the given value.
   *
   * @param row The row.
   * @param col The column.
   * @param value The value.
   */
  public void set(int row, int col, double value) {
    assert row >= 0 && row < rows();
    assert col >= 0 && col < cols();
    m_storage.set(row * cols() + col, new Variable(value));
  }

  /**
   * Sets an element to the given value.
   *
   * @param row The row.
   * @param value The value.
   */
  public void set(int row, Variable value) {
    assert row >= 0 && row < rows() * cols();
    m_storage.set(row, value);
  }

  /**
   * Returns a block of the variable matrix.
   *
   * @param rowOffset The row offset of the block selection.
   * @param colOffset The column offset of the block selection.
   * @param blockRows The number of rows in the block selection.
   * @param blockCols The number of columns in the block selection.
   * @return A block of the variable matrix.
   */
  public VariableBlock block(int rowOffset, int colOffset, int blockRows, int blockCols) {
    assert rowOffset >= 0 && rowOffset <= rows();
    assert colOffset >= 0 && colOffset <= cols();
    assert blockRows >= 0 && blockRows <= rows() - rowOffset;
    assert blockCols >= 0 && blockCols <= cols() - colOffset;
    return new VariableBlock(this, rowOffset, colOffset, blockRows, blockCols);
  }

  /**
   * Returns a slice of the variable matrix.
   *
   * @param rowSlice The row slice.
   * @param colSlice The column slice.
   * @return A slice of the variable matrix.
   */
  public VariableBlock slice(Slice rowSlice, Slice colSlice) {
    int rowSliceLength = rowSlice.adjust(rows());
    int colSliceLength = colSlice.adjust(cols());
    return new VariableBlock(this, rowSlice, rowSliceLength, colSlice, colSliceLength);
  }

  /**
   * Returns a slice of the variable matrix.
   *
   * <p>The given slices aren't adjusted. This overload is for Python bindings only.
   *
   * @param row_slice The row slice.
   * @param row_slice_length The row slice length.
   * @param col_slice The column slice.
   * @param col_slice_length The column slice length.
   * @return A slice of the variable matrix.
   */
  public VariableBlock slice(
      Slice rowSlice, int rowSliceLength, Slice colSlice, int colSliceLength) {
    return new VariableBlock(this, rowSlice, rowSliceLength, colSlice, colSliceLength);
  }

  /**
   * Returns a segment of the variable vector.
   *
   * @param offset The offset of the segment.
   * @param length The length of the segment.
   * @return A segment of the variable vector.
   */
  public VariableBlock segment(int offset, int length) {
    assert offset >= 0 && offset < rows() * cols();
    assert length >= 0 && length <= rows() * cols() - offset;
    return block(offset, 0, length, 1);
  }

  /**
   * Returns a row slice of the variable matrix.
   *
   * @param row The row to slice.
   * @return A row slice of the variable matrix.
   */
  public VariableBlock row(int row) {
    assert row >= 0 && row < rows();
    return block(row, 0, 1, cols());
  }

  /**
   * Returns a column slice of the variable matrix.
   *
   * @param col The column to slice.
   * @return A column slice of the variable matrix.
   */
  public VariableBlock col(int col) {
    assert col >= 0 && col < cols();
    return block(0, col, rows(), 1);
  }

  /**
   * Matrix multiplication operator.
   *
   * @param rhs Operator right-hand side.
   */
  public VariableMatrix times(VariableMatrix rhs) {
    assert cols() == rhs.rows();

    var result = new VariableMatrix(rows(), rhs.cols());

    for (int i = 0; i < rows(); ++i) {
      for (int j = 0; j < rhs.cols(); ++j) {
        var sum = new Variable();
        for (int k = 0; k < cols(); ++k) {
          sum = sum.plus(get(i, k).times(rhs.get(k, j)));
        }
        result.set(i, j, sum);
      }
    }

    return result;
  }

  /**
   * Matrix-scalar multiplication operator.
   *
   * @param rhs Operator right-hand side.
   */
  public VariableMatrix times(double rhs) {
    var result = new VariableMatrix(rows(), cols());

    for (int row = 0; row < result.rows(); ++row) {
      for (int col = 0; col < result.cols(); ++col) {
        result.set(row, col, get(row, col).times(new Variable(rhs)));
      }
    }

    return result;
  }

  /**
   * Matrix-scalar multiplication operator.
   *
   * @param rhs Operator right-hand side.
   */
  public VariableMatrix times(Variable rhs) {
    var result = new VariableMatrix(rows(), cols());

    for (int row = 0; row < result.rows(); ++row) {
      for (int col = 0; col < result.cols(); ++col) {
        result.set(row, col, get(row, col).times(rhs));
      }
    }

    return result;
  }

  /**
   * Binary division operator.
   *
   * @param rhs Operator right-hand side.
   * @return Result of division.
   */
  public VariableMatrix div(Variable rhs) {
    var result = new VariableMatrix(rows(), cols());

    for (int row = 0; row < result.rows(); ++row) {
      for (int col = 0; col < result.cols(); ++col) {
        result.set(row, col, get(row, col).div(rhs));
      }
    }

    return result;
  }

  /**
   * Binary division operator.
   *
   * @param rhs Operator right-hand side.
   * @return Result of division.
   */
  public VariableMatrix div(double rhs) {
    var result = new VariableMatrix(rows(), cols());

    for (int row = 0; row < result.rows(); ++row) {
      for (int col = 0; col < result.cols(); ++col) {
        result.set(row, col, get(row, col).div(new Variable(rhs)));
      }
    }

    return result;
  }

  /**
   * Binary addition operator.
   *
   * @param rhs Operator right-hand side.
   * @return Result of addition.
   */
  public VariableMatrix plus(VariableMatrix rhs) {
    assert rows() == rhs.rows() && cols() == rhs.cols();

    var result = new VariableMatrix(rows(), cols());

    for (int row = 0; row < result.rows(); ++row) {
      for (int col = 0; col < result.cols(); ++col) {
        result.set(row, col, get(row, col).plus(rhs.get(row, col)));
      }
    }

    return result;
  }

  /**
   * Binary addition operator.
   *
   * @param rhs Operator right-hand side.
   * @return Result of addition.
   */
  public VariableMatrix plus(VariableBlock rhs) {
    assert rows() == rhs.rows() && cols() == rhs.cols();

    var result = new VariableMatrix(rows(), cols());

    for (int row = 0; row < result.rows(); ++row) {
      for (int col = 0; col < result.cols(); ++col) {
        result.set(row, col, get(row, col).plus(rhs.get(row, col)));
      }
    }

    return result;
  }

  /**
   * Binary addition operator.
   *
   * @param rhs Operator right-hand side.
   * @return Result of addition.
   */
  public VariableMatrix plus(SimpleMatrix rhs) {
    assert rows() == rhs.numRows() && cols() == rhs.numCols();

    var result = new VariableMatrix(rows(), cols());

    for (int row = 0; row < result.rows(); ++row) {
      for (int col = 0; col < result.cols(); ++col) {
        result.set(row, col, get(row, col).plus(new Variable(rhs.get(row, col))));
      }
    }

    return result;
  }

  /**
   * Binary subtraction operator.
   *
   * @param rhs Operator right-hand side.
   * @return Result of subtraction.
   */
  public VariableMatrix minus(VariableMatrix rhs) {
    assert rows() == rhs.rows() && cols() == rhs.cols();

    var result = new VariableMatrix(rows(), cols());

    for (int row = 0; row < result.rows(); ++row) {
      for (int col = 0; col < result.cols(); ++col) {
        result.set(row, col, get(row, col).minus(rhs.get(row, col)));
      }
    }

    return result;
  }

  /**
   * Binary subtraction operator.
   *
   * @param rhs Operator right-hand side.
   * @return Result of subtraction.
   */
  public VariableMatrix minus(VariableBlock rhs) {
    assert rows() == rhs.rows() && cols() == rhs.cols();

    var result = new VariableMatrix(rows(), cols());

    for (int row = 0; row < result.rows(); ++row) {
      for (int col = 0; col < result.cols(); ++col) {
        result.set(row, col, get(row, col).minus(rhs.get(row, col)));
      }
    }

    return result;
  }

  /**
   * Binary subtraction operator.
   *
   * @param rhs Operator right-hand side.
   * @return Result of subtraction.
   */
  public VariableMatrix minus(SimpleMatrix rhs) {
    assert rows() == rhs.numRows() && cols() == rhs.numCols();

    var result = new VariableMatrix(rows(), cols());

    for (int row = 0; row < result.rows(); ++row) {
      for (int col = 0; col < result.cols(); ++col) {
        result.set(row, col, get(row, col).minus(rhs.get(row, col)));
      }
    }

    return result;
  }

  /** Unary minus operator. */
  public VariableMatrix unaryMinus() {
    var result = new VariableMatrix(rows(), cols());

    for (int row = 0; row < result.rows(); ++row) {
      for (int col = 0; col < result.cols(); ++col) {
        result.set(row, col, get(row, col).unaryMinus());
      }
    }

    return result;
  }

  /**
   * Returns the transpose of the variable matrix.
   *
   * @return The transpose of the variable matrix.
   */
  public VariableMatrix T() {
    var result = new VariableMatrix(cols(), rows());

    for (int row = 0; row < rows(); ++row) {
      for (int col = 0; col < cols(); ++col) {
        result.set(col, row, get(row, col));
      }
    }

    return result;
  }

  /**
   * Returns the number of rows in the matrix.
   *
   * @return The number of rows in the matrix.
   */
  public int rows() {
    return m_rows;
  }

  /**
   * Returns the number of columns in the matrix.
   *
   * @return The number of columns in the matrix.
   */
  public int cols() {
    return m_cols;
  }

  /**
   * Returns an element of the variable matrix.
   *
   * @param row The row of the element to return.
   * @param col The column of the element to return.
   * @return An element of the variable matrix.
   */
  public double value(int row, int col) {
    assert row >= 0 && row < rows();
    assert col >= 0 && col < cols();
    return m_storage.get(row * cols() + col).value();
  }

  /**
   * Returns a row of the variable column vector.
   *
   * @param index The index of the element to return.
   * @return A row of the variable column vector.
   */
  public double value(int index) {
    assert index >= 0 && index < rows() * cols();
    return m_storage.get(index).value();
  }

  /**
   * Returns the contents of the variable matrix.
   *
   * @return The contents of the variable matrix.
   */
  public SimpleMatrix value() {
    var result = new SimpleMatrix(rows(), cols());

    for (int row = 0; row < rows(); ++row) {
      for (int col = 0; col < cols(); ++col) {
        result.set(row, col, value(row, col));
      }
    }

    return result;
  }

  /**
   * Transforms the matrix coefficient-wise with an unary operator.
   *
   * @param unary_op The unary operator to use for the transform operation.
   * @return Result of the unary operator.
   */
  public VariableMatrix cwiseTransform(UnaryOperator<Variable> unaryOp) {
    var result = new VariableMatrix(rows(), cols());

    for (int row = 0; row < rows(); ++row) {
      for (int col = 0; col < cols(); ++col) {
        result.set(row, col, unaryOp.apply(get(row, col)));
      }
    }

    return result;
  }

  /**
   * Returns a variable matrix filled with zeroes.
   *
   * @param rows The number of matrix rows.
   * @param cols The number of matrix columns.
   * @return A variable matrix filled with zeroes.
   */
  public static VariableMatrix zero(int rows, int cols) {
    var result = new VariableMatrix(rows, cols);

    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < cols; ++col) {
        result.set(row, col, 0.0);
      }
    }

    return result;
  }

  /**
   * Returns a variable matrix filled with ones.
   *
   * @param rows The number of matrix rows.
   * @param cols The number of matrix columns.
   * @return A variable matrix filled with ones.
   */
  public static VariableMatrix ones(int rows, int cols) {
    var result = new VariableMatrix(rows, cols);

    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < cols; ++col) {
        result.set(row, col, 1.0);
      }
    }

    return result;
  }
}
