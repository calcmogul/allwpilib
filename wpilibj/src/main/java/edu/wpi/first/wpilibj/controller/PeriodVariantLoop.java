/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import edu.wpi.first.wpilibj.math.Matrix;
import edu.wpi.first.wpilibj.math.MatrixUtils;
import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;
import edu.wpi.first.wpilibj.math.numbers.N1;

public class PeriodVariantLoop<States extends Num, Inputs extends Num, Outputs extends Num> {
  protected final Nat<States> kStates;
  protected final Nat<Inputs> kInputs;
  protected final Nat<Outputs> kOutputs;
  protected PeriodVariantPlant<States, Inputs, Outputs> m_plant;
  protected PeriodVariantController<States, Inputs, Outputs> m_controller;
  protected PeriodVariantKalmanFilter<States, Inputs, Outputs> m_observer;

  protected Matrix<States, N1> m_nextR;

  public PeriodVariantLoop(PeriodVariantPlant<States, Inputs, Outputs> plant,
                           PeriodVariantController<States, Inputs, Outputs> controller,
                           PeriodVariantKalmanFilter<States, Inputs, Outputs> observer) {
    m_plant = plant;
    m_controller = controller;
    m_observer = observer;
    kStates = m_plant.getStates();
    kInputs = m_plant.getInputs();
    kOutputs = m_plant.getOutputs();
  }

  public PeriodVariantLoop(PeriodVariantPlantCoeffs<States, Inputs, Outputs> plantCoeffs,
                           StateSpaceControllerCoeffs<States, Inputs, Outputs> controllerCoeffs,
                           PeriodVariantKalmanFilterCoeffs<States, Inputs, Outputs> observerCoeffs) {
    m_plant = new PeriodVariantPlant<>(plantCoeffs);
    m_controller = new PeriodVariantController<>(controllerCoeffs, m_plant);
    m_observer = new PeriodVariantKalmanFilter<>(observerCoeffs, m_plant);
    kStates = m_plant.getStates();
    kInputs = m_plant.getInputs();
    kOutputs = m_plant.getOutputs();
  }

  public void enable() {
    m_controller.enable();
  }

  public void disable() {
    m_controller.disable();
  }

  public Matrix<States, N1> getXhat() {
    return m_observer.getXhat();
  }

  public double getXhat(int i) {
    return m_observer.getXhat(i);
  }

  public Matrix<States, N1> getNextR() {
    return m_nextR;
  }

  public double getNextR(int i) {
    return getNextR().get(i, 0);
  }

  public Matrix<Inputs, N1> getU() {
    return m_controller.getU();
  }

  public double getU(int i) {
    return m_controller.getU(i);
  }

  public void setXhat(Matrix<States, N1> xHat) {
    m_observer.setXhat(xHat);
  }

  public void setXhat(int i, double value) {
    m_observer.setXhat(i, value);
  }

  public void setNextR(Matrix<States, N1> nextR) {
    m_nextR = nextR;
  }

  public PeriodVariantPlant<States, Inputs, Outputs> getPlant() {
    return m_plant;
  }

  public PeriodVariantController<States, Inputs, Outputs> getController() {
    return m_controller;
  }

  public PeriodVariantKalmanFilter<States, Inputs, Outputs> getObserver() {
    return m_observer;
  }

  public void reset() {
    m_plant.reset();
    m_controller.reset();
    m_observer.reset();
    m_nextR = MatrixUtils.zeros(kStates);
  }

  public Matrix<States, N1> getError() {
    return m_controller.getR().minus(m_observer.getXhat());
  }

  public void correct(Matrix<Outputs, N1> y) {
    m_observer.correct(m_controller.getU(), y);
  }

  public void predict(double dt) {
    m_controller.update(m_nextR, m_observer.getXhat());
    m_observer.predict(m_controller.getU(), dt);
  }

  public void setIndex(int index) {
    m_plant.setIndex(index);
    m_controller.setIndex(index);
    m_observer.setIndex(index);
  }

  public int getIndex() {
    return m_plant.getIndex();
  }
}
