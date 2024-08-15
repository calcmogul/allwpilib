// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// THIS FILE WAS AUTO-GENERATED BY ./wpilibNewCommands/generate_hids.py. DO NOT MODIFY

#include "frc2/command/button/CommandPS5Controller.h"

using namespace frc2;

CommandPS5Controller::CommandPS5Controller(int port)
    : CommandGenericHID(port), m_hid{frc::PS5Controller(port)} {}

frc::PS5Controller& CommandPS5Controller::GetHID() {
  return m_hid;
}

Trigger CommandPS5Controller::Square(frc::EventLoop* loop) const {
  return Button(frc::PS5Controller::Button::kSquare, loop);
}

Trigger CommandPS5Controller::Cross(frc::EventLoop* loop) const {
  return Button(frc::PS5Controller::Button::kCross, loop);
}

Trigger CommandPS5Controller::Circle(frc::EventLoop* loop) const {
  return Button(frc::PS5Controller::Button::kCircle, loop);
}

Trigger CommandPS5Controller::Triangle(frc::EventLoop* loop) const {
  return Button(frc::PS5Controller::Button::kTriangle, loop);
}

Trigger CommandPS5Controller::L1(frc::EventLoop* loop) const {
  return Button(frc::PS5Controller::Button::kL1, loop);
}

Trigger CommandPS5Controller::R1(frc::EventLoop* loop) const {
  return Button(frc::PS5Controller::Button::kR1, loop);
}

Trigger CommandPS5Controller::L2(frc::EventLoop* loop) const {
  return Button(frc::PS5Controller::Button::kL2, loop);
}

Trigger CommandPS5Controller::R2(frc::EventLoop* loop) const {
  return Button(frc::PS5Controller::Button::kR2, loop);
}

Trigger CommandPS5Controller::Create(frc::EventLoop* loop) const {
  return Button(frc::PS5Controller::Button::kCreate, loop);
}

Trigger CommandPS5Controller::Options(frc::EventLoop* loop) const {
  return Button(frc::PS5Controller::Button::kOptions, loop);
}

Trigger CommandPS5Controller::L3(frc::EventLoop* loop) const {
  return Button(frc::PS5Controller::Button::kL3, loop);
}

Trigger CommandPS5Controller::R3(frc::EventLoop* loop) const {
  return Button(frc::PS5Controller::Button::kR3, loop);
}

Trigger CommandPS5Controller::PS(frc::EventLoop* loop) const {
  return Button(frc::PS5Controller::Button::kPS, loop);
}

Trigger CommandPS5Controller::Touchpad(frc::EventLoop* loop) const {
  return Button(frc::PS5Controller::Button::kTouchpad, loop);
}

double CommandPS5Controller::GetLeftX() const {
  return m_hid.GetLeftX();
}

double CommandPS5Controller::GetLeftY() const {
  return m_hid.GetLeftY();
}

double CommandPS5Controller::GetRightX() const {
  return m_hid.GetRightX();
}

double CommandPS5Controller::GetRightY() const {
  return m_hid.GetRightY();
}

double CommandPS5Controller::GetL2Axis() const {
  return m_hid.GetL2Axis();
}

double CommandPS5Controller::GetR2Axis() const {
  return m_hid.GetR2Axis();
}