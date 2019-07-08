/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import edu.wpi.first.wpilibj.math.Matrix;
import edu.wpi.first.wpilibj.math.Num;

public final class PeriodVariantObserverCoeffs<States extends Num, Inputs extends Num, Outputs extends Num> {
  private Matrix<States, States> Qcontinuous;
  private Matrix<Outputs, Outputs> Rcontinuous;
  private Matrix<States, States> PsteadyState;

  public PeriodVariantObserverCoeffs(Matrix<States, States> Qcontinuous,
                                     Matrix<Outputs, Outputs> Rcontinuous,
                                     Matrix<States, States> PsteadyState) {
    this.Qcontinuous = Qcontinuous;
    this.Rcontinuous = Rcontinuous;
    this.PsteadyState = PsteadyState;
  }

  public Matrix<States, States> getQcontinuous() {
    return Qcontinuous;
  }

  public Matrix<Outputs, Outputs> getRcontinuous() {
    return Rcontinuous;
  }

  public Matrix<States, States> getPsteadyState() {
    return PsteadyState;
  }
}
