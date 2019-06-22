/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import edu.wpi.first.wpilibj.math.Matrix;
import edu.wpi.first.wpilibj.math.Num;

@SuppressWarnings({"unused", "ClassTypeParameterName", "MemberName", "ParameterName"})
public class StateSpaceObserverCoeffs<States extends Num, Inputs extends Num, Outputs extends Num> {
  private final Matrix<States, Outputs> K;

  public StateSpaceObserverCoeffs(Matrix<States, Outputs> K) {
    this.K = K;
  }

  public Matrix<States, Outputs> getK() {
    return K;
  }
}
