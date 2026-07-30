// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package wpilib.robot;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N7;
import org.wpilib.math.util.Nat;

public final class MatrixPowBenchmark {
  private MatrixPowBenchmark() {
    // Utility class.
  }

  /** Matrix power benchmark. */
  public static Matrix<N7, N7> matrixPow() {
    // Jordan block with eigenvalue 2, which has a principal square root
    var A = new Matrix<>(Nat.N7(), Nat.N7());
    for (int i = 0; i < 7; ++i) {
      A.set(i, i, 2);
    }
    for (int col = 1; col < 7; ++col) {
      A.set(col - 1, col, 1);
    }
    return A.pow(0.5);
  }
}
