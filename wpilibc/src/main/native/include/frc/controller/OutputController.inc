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
OutputController<Inputs, Outputs>::OutputController(
    const OutputControllerCoeffs<Inputs, Outputs>& controllerCoeffs) {
  AddCoefficients(controllerCoeffs);
}

template <int Inputs, int Outputs>
void OutputController<Inputs, Outputs>::Enable() {
  m_enabled = true;
}

template <int Inputs, int Outputs>
void OutputController<Inputs, Outputs>::Disable() {
  m_enabled = false;
  m_u.setZero();
}

template <int Inputs, int Outputs>
const Eigen::Matrix<double, Inputs, Outputs>&
OutputController<Inputs, Outputs>::K() const {
  return GetCoefficients().K;
}

template <int Inputs, int Outputs>
double OutputController<Inputs, Outputs>::K(int i, int j) const {
  return K()(i, j);
}

template <int Inputs, int Outputs>
const Eigen::Matrix<double, Outputs, 1>& OutputController<Inputs, Outputs>::R()
    const {
  return m_r;
}

template <int Inputs, int Outputs>
double OutputController<Inputs, Outputs>::R(int i) const {
  return R()(i, 0);
}

template <int Inputs, int Outputs>
const Eigen::Matrix<double, Inputs, 1>& OutputController<Inputs, Outputs>::U()
    const {
  return m_u;
}

template <int Inputs, int Outputs>
double OutputController<Inputs, Outputs>::U(int i) const {
  return U()(i, 0);
}

template <int Inputs, int Outputs>
void OutputController<Inputs, Outputs>::Reset() {
  m_r.setZero();
  m_u.setZero();
}

template <int Inputs, int Outputs>
void OutputController<Inputs, Outputs>::Update(
    const Eigen::Matrix<double, Outputs, 1>& y) {
  if (m_enabled) {
    m_u = K() * (m_r - y);
    CapU();
  }
}

template <int Inputs, int Outputs>
void OutputController<Inputs, Outputs>::Update(
    const Eigen::Matrix<double, Outputs, 1>& nextR,
    const Eigen::Matrix<double, Outputs, 1>& y) {
  if (m_enabled) {
    m_u = K() * (m_r - y);
    CapU();
    m_r = nextR;
  }
}

template <int Inputs, int Outputs>
void OutputController<Inputs, Outputs>::CapU() {
  for (int i = 0; i < Inputs; ++i) {
    if (U(i) > GetCoefficients().Umax(i)) {
      m_u(i, 0) = GetCoefficients().Umax(i);
    } else if (U(i) < GetCoefficients().Umin(i)) {
      m_u(i, 0) = GetCoefficients().Umin(i);
    }
  }
}

template <int Inputs, int Outputs>
void OutputController<Inputs, Outputs>::AddCoefficients(
    const OutputControllerCoeffs<Inputs, Outputs>& coefficients) {
  m_coefficients.emplace_back(coefficients);
}

template <int Inputs, int Outputs>
const OutputControllerCoeffs<Inputs, Outputs>&
OutputController<Inputs, Outputs>::GetCoefficients(int index) const {
  return m_coefficients[index];
}

template <int Inputs, int Outputs>
const OutputControllerCoeffs<Inputs, Outputs>&
OutputController<Inputs, Outputs>::GetCoefficients() const {
  return m_coefficients[m_index];
}

template <int Inputs, int Outputs>
void OutputController<Inputs, Outputs>::SetIndex(int index) {
  if (index < 0) {
    m_index = 0;
  } else if (index >= static_cast<int>(m_coefficients.size())) {
    m_index = static_cast<int>(m_coefficients.size()) - 1;
  } else {
    m_index = index;
  }
}

template <int Inputs, int Outputs>
int OutputController<Inputs, Outputs>::GetIndex() const {
  return m_index;
}

}  // namespace frc
