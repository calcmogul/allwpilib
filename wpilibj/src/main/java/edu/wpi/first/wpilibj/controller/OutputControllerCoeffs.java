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

public final class OutputControllerCoeffs<Inputs extends Num, Outputs extends Num> {
  private Matrix<Inputs, Outputs> K;
  private Matrix<Inputs, N1> Umin;
  private Matrix<Inputs, N1> Umax;

  public OutputControllerCoeffs(Matrix<Inputs, Outputs> K,
                                Matrix<Inputs, N1> Umin,
                                Matrix<Inputs, N1> Umax) {
    this.K = K;
    this.Umin = Umin;
    this.Umax = Umax;
  }

  public Matrix<Inputs, Outputs> getK() {
    return K;
  }

  public Matrix<Inputs, N1> getUmin() {
    return Umin;
  }

  public Matrix<Inputs, N1> getUmax() {
    return Umax;
  }

  public double getUmax(int i) {
    return getUmax().get(i, 0);
  }

  public double getUmin(int i) {
    return getUmin().get(i, 0);
  }
}
