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

@SuppressWarnings({"MemberName", "ClassTypeParameterName", "ParameterName"})
public final class StateSpacePlantCoeffs<States extends Num, Inputs extends Num, Outputs extends Num> {
  private final Nat<States> states;
  private final Nat<Inputs> inputs;
  private final Nat<Outputs> outputs;

  private final Matrix<States, States> A;
  private final Matrix<States, Inputs> B;
  private final Matrix<Outputs, States> C;
  private final Matrix<Outputs, Inputs> D;

  public StateSpacePlantCoeffs(Nat<States> states, Nat<Inputs> inputs, Nat<Outputs> outputs, Matrix<States, States> A, Matrix<States, Inputs> B, Matrix<Outputs, States> C, Matrix<Outputs, Inputs> D) {
    this.states = states;
    this.inputs = inputs;
    this.outputs = outputs;
    this.A = A;
    this.B = B;
    this.C = C;
    this.D = D;
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

  public Matrix<States, States> getA() {
    return A;
  }

  public Matrix<States, Inputs> getB() {
    return B;
  }

  public Matrix<Outputs, States> getC() {
    return C;
  }

  public Matrix<Outputs, Inputs> getD() {
    return D;
  }
}
