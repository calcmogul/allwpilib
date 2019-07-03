package edu.wpi.first.wpilibj.controller;

import edu.wpi.first.wpilibj.math.Matrix;
import edu.wpi.first.wpilibj.math.MatrixUtils;
import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;
import edu.wpi.first.wpilibj.math.numbers.N1;

public class StateSpaceLoop<States extends Num, Inputs extends Num, Outputs extends Num> {
  protected final Nat<States> kStates;
  protected final Nat<Inputs> kInputs;
  protected final Nat<Outputs> kOutputs;

  protected StateSpacePlant<States, Inputs, Outputs> m_plant;
  protected StateSpaceController<States, Inputs, Outputs> m_controller;
  protected StateSpaceObserver<States, Inputs, Outputs> m_observer;

  protected Matrix<States, N1> m_nextR;

  public StateSpaceLoop(StateSpacePlantCoeffs<States, Inputs, Outputs> plantCoeffs,
                        StateSpaceControllerCoeffs<States, Inputs, Outputs> controllerCoeffs,
                        StateSpaceObserverCoeffs<States, Inputs, Outputs> observerCoeffs) {
    kStates = plantCoeffs.getStates();
    kInputs = plantCoeffs.getInputs();
    kOutputs = plantCoeffs.getOutputs();

    m_nextR = MatrixUtils.zeros(kStates);

    m_plant = new StateSpacePlant<>(plantCoeffs);
    m_controller = new StateSpaceController<>(m_plant, controllerCoeffs);
    m_observer = new StateSpaceObserver<>(m_plant, observerCoeffs);
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
    return m_nextR.get(i, 0);
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

  public StateSpacePlant<States, Inputs, Outputs> getPlant() {
    return m_plant;
  }

  public StateSpaceController<States, Inputs, Outputs> getController() {
    return m_controller;
  }

  public StateSpaceObserver<States, Inputs, Outputs> getObserver() {
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

  public void predict() {
    m_observer.predict(m_controller.getU());
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
