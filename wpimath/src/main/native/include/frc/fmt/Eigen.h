// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <fmt/format.h>

#include "Eigen/Core"

template <int Rows, int Cols, int... Args>
struct fmt::formatter<Eigen::Matrix<double, Rows, Cols, Args...>>
    : fmt::formatter<double> {
  template <typename FormatContext>
  auto format(const Eigen::Matrix<double, Rows, Cols, Args...>& mat,
              FormatContext& ctx) {
    auto out = ctx.out();

    for (int i = 0; i < Rows; ++i) {
      for (int j = 0; j < Cols; ++j) {
        out = fmt::format_to(out, "  ");
        out = fmt::formatter<double>::format(mat(i, j), ctx);
      }

      if (i < Rows - 1) {
        out = fmt::format_to(out, "\n");
      }
    }

    return out;
  }
};
