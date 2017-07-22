/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <utility>

namespace frc {

template <int Inputs, int Outputs>
OutputLoop<Inputs, Outputs>::OutputLoop(
    const OutputControllerCoeffs<Inputs, Outputs>& controllerCoeffs)
    : m_controller(controllerCoeffs) {
  Reset();
}

template <int Inputs, int Outputs>
OutputLoop<Inputs, Outputs>::OutputLoop(
    OutputController<Inputs, Outputs>&& controller)
    : m_controller(std::move(controller)) {
  Reset();
}

template <int Inputs, int Outputs>
void OutputLoop<Inputs, Outputs>::Enable() {
  m_controller.Enable();
}

template <int Inputs, int Outputs>
void OutputLoop<Inputs, Outputs>::Disable() {
  m_controller.Disable();
}

template <int Inputs, int Outputs>
const Eigen::Matrix<double, Outputs, 1>& OutputLoop<Inputs, Outputs>::NextR()
    const {
  return m_nextR;
}

template <int Inputs, int Outputs>
double OutputLoop<Inputs, Outputs>::NextR(int i) const {
  return NextR()(i, 0);
}

template <int Inputs, int Outputs>
const Eigen::Matrix<double, Inputs, 1>& OutputLoop<Inputs, Outputs>::U() const {
  return m_controller.U();
}

template <int Inputs, int Outputs>
double OutputLoop<Inputs, Outputs>::U(int i) const {
  return m_controller.U(i);
}

template <int Inputs, int Outputs>
void OutputLoop<Inputs, Outputs>::SetNextR(
    const Eigen::Matrix<double, Outputs, 1>& nextR) {
  m_nextR = nextR;
}

template <int Inputs, int Outputs>
const OutputController<Inputs, Outputs>&
OutputLoop<Inputs, Outputs>::GetController() const {
  return m_controller;
}

template <int Inputs, int Outputs>
void OutputLoop<Inputs, Outputs>::Reset() {
  m_controller.Reset();
  m_nextR.setZero();
  m_y.setZero();
}

template <int Inputs, int Outputs>
const Eigen::Matrix<double, Outputs, 1> OutputLoop<Inputs, Outputs>::Error()
    const {
  return m_controller.R() - m_y;
}

template <int Inputs, int Outputs>
void OutputLoop<Inputs, Outputs>::Update(
    const Eigen::Matrix<double, Outputs, 1>& y) {
  m_y = y;
  m_controller.Update(m_nextR, m_y);
}

template <int Inputs, int Outputs>
void OutputLoop<Inputs, Outputs>::SetIndex(int index) {
  m_controller.SetIndex(index);
}

template <int Inputs, int Outputs>
int OutputLoop<Inputs, Outputs>::GetIndex() const {
  return m_controller.GetIndex();
}

}  // namespace frc
