// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stacktrace>
#include <stdexcept>
#include <string>
#include <utility>

namespace wpi {

class Exception : std::exception {
 public:
  explicit Exception(std::string_view message,
                     std::stacktrace st = std::stacktrace::current())
      : m_message{std::move(message)}, m_stacktrace{std::move(st)} {}

  const char* what() const noexcept override { return m_message.c_str(); }

  const std::stacktrace& trace() const noexcept { return m_stacktrace; }

 private:
  std::string m_message;
  std::stacktrace m_stacktrace;
};

}  // namespace wpi
