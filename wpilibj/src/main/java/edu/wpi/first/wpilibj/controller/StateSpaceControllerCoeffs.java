/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import edu.wpi.first.wpilibj.math.Matrix;
import edu.wpi.first.wpilibj.math.Num;
import edu.wpi.first.wpilibj.math.numbers.N1;

@SuppressWarnings({"unused", "ParameterName", "ClassTypeParameterName", "MemberName"})
public class StateSpaceControllerCoeffs<States extends Num, Inputs extends Num, Outputs extends Num> {
  private final Matrix<Inputs, States> K;
  private final Matrix<Inputs, States> Kff;
  private final Matrix<Inputs, N1> Umin;
  private final Matrix<Inputs, N1> Umax;

  public StateSpaceControllerCoeffs(Matrix<Inputs, States> K, Matrix<Inputs, States> Kff,
                                    Matrix<Inputs, N1> Umin, Matrix<Inputs, N1> Umax) {
    this.K = K;
    this.Kff = Kff;
    this.Umin = Umin;
    this.Umax = Umax;
  }

  public Matrix<Inputs, States> getK() {
    return K;
  }

  public Matrix<Inputs, States> getKff() {
    return Kff;
  }

  public Matrix<Inputs, N1> getUmin() {
    return Umin;
  }

  public Matrix<Inputs, N1> getUmax() {
    return Umax;
  }
}
