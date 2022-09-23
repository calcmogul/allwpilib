// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <utility>

namespace wpi {

template <typename F>
class ScopeExit {
 public:
  explicit ScopeExit(F&& f) noexcept : m_f{f} {}

  ~ScopeExit() { m_f(); }

  ScopeExit(ScopeExit&& other) noexcept : m_f(std::move(other.m_f)) {}

  ScopeExit(const ScopeExit&) = delete;
  ScopeExit& operator=(const ScopeExit&) = delete;

 private:
  F m_f;
};

}  // namespace wpi
