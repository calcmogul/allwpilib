// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/estimator/AngleStatistics.h"

namespace frc {

template <>
Eigen::VectorXd AngleMean<Eigen::Dynamic, Eigen::Dynamic>(
    const Eigen::MatrixXd& sigmas, const Eigen::VectorXd& Wm,
    int angleStatesIdx) {
  double sumSin = (sigmas.row(angleStatesIdx).unaryExpr([](auto it) {
                    return std::sin(it);
                  }) *
                   Wm)
                      .sum();
  double sumCos = (sigmas.row(angleStatesIdx).unaryExpr([](auto it) {
                    return std::cos(it);
                  }) *
                   Wm)
                      .sum();

  Eigen::VectorXd ret = sigmas * Wm;
  ret[angleStatesIdx] = std::atan2(sumSin, sumCos);
  return ret;
}

template <>
std::function<Eigen::VectorXd(const Eigen::MatrixXd&, const Eigen::VectorXd&)>
AngleMean<Eigen::Dynamic, Eigen::Dynamic>(int angleStateIdx) {
  return [=](auto sigmas, auto Wm) {
    return AngleMean<Eigen::Dynamic, Eigen::Dynamic>(sigmas, Wm, angleStateIdx);
  };
}

}  // namespace frc
