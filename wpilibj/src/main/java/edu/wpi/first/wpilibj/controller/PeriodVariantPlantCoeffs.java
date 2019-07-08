/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import edu.wpi.first.wpilibj.math.Matrix;
import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public final class PeriodVariantPlantCoeffs<States extends Num, Inputs extends Num, Outputs extends Num> {

  private Nat<States> states;
  private Nat<Inputs> inputs;
  private Nat<Outputs> outputs;

  private Matrix<States, States> Acontinuous;
  private Matrix<States, Inputs> Bcontinuous;
  private Matrix<Outputs, States> C;
  private Matrix<Outputs, Inputs> D;

  public PeriodVariantPlantCoeffs(Nat<States> states, Nat<Inputs> inputs, Nat<Outputs> outputs,
                                  Matrix<States, States> Acontinuous, Matrix<States, Inputs> Bcontinuous,
                                  Matrix<Outputs, States> C, Matrix<Outputs, Inputs> D) {
    this.states = states;
    this.inputs = inputs;
    this.outputs = outputs;
    this.Acontinuous = Acontinuous;
    this.Bcontinuous = Bcontinuous;
    this.C = C;
    this.D = D;
  }

  public Matrix<States, States> getAcontinuous() {
    return Acontinuous;
  }

  public Matrix<States, Inputs> getBcontinuous() {
    return Bcontinuous;
  }

  public Matrix<Outputs, States> getC() {
    return C;
  }

  public Matrix<Outputs, Inputs> getD() {
    return D;
  }

  public Nat<States> getStates() {
    return states;
  }

  public Nat<Inputs> getInputs() {
    return inputs;
  }

  public Nat<Outputs> getOutputs() {
    return outputs;
  }
}
