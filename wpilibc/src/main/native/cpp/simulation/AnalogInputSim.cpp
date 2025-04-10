// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/simulation/AnalogInputSim.h"

#include <memory>

#include <hal/simulation/AnalogInData.h>

#include "frc/AnalogInput.h"

using namespace frc;
using namespace frc::sim;

AnalogInputSim::AnalogInputSim(const AnalogInput& analogInput)
    : m_index{analogInput.GetChannel()} {}

AnalogInputSim::AnalogInputSim(int channel) : m_index{channel} {}

std::unique_ptr<CallbackStore> AnalogInputSim::RegisterInitializedCallback(
    NotifyCallback callback, bool initialNotify) {
  auto store = std::make_unique<CallbackStore>(
      m_index, -1, callback, &HALSIM_CancelAnalogInInitializedCallback);
  store->SetUid(HALSIM_RegisterAnalogInInitializedCallback(
      m_index, &CallbackStoreThunk, store.get(), initialNotify));
  return store;
}

bool AnalogInputSim::GetInitialized() const {
  return HALSIM_GetAnalogInInitialized(m_index);
}

void AnalogInputSim::SetInitialized(bool initialized) {
  HALSIM_SetAnalogInInitialized(m_index, initialized);
}

std::unique_ptr<CallbackStore> AnalogInputSim::RegisterAverageBitsCallback(
    NotifyCallback callback, bool initialNotify) {
  auto store = std::make_unique<CallbackStore>(
      m_index, -1, callback, &HALSIM_CancelAnalogInAverageBitsCallback);
  store->SetUid(HALSIM_RegisterAnalogInAverageBitsCallback(
      m_index, &CallbackStoreThunk, store.get(), initialNotify));
  return store;
}

int AnalogInputSim::GetAverageBits() const {
  return HALSIM_GetAnalogInAverageBits(m_index);
}

void AnalogInputSim::SetAverageBits(int averageBits) {
  HALSIM_SetAnalogInAverageBits(m_index, averageBits);
}

std::unique_ptr<CallbackStore> AnalogInputSim::RegisterOversampleBitsCallback(
    NotifyCallback callback, bool initialNotify) {
  auto store = std::make_unique<CallbackStore>(
      m_index, -1, callback, &HALSIM_CancelAnalogInOversampleBitsCallback);
  store->SetUid(HALSIM_RegisterAnalogInOversampleBitsCallback(
      m_index, &CallbackStoreThunk, store.get(), initialNotify));
  return store;
}

int AnalogInputSim::GetOversampleBits() const {
  return HALSIM_GetAnalogInOversampleBits(m_index);
}

void AnalogInputSim::SetOversampleBits(int oversampleBits) {
  HALSIM_SetAnalogInOversampleBits(m_index, oversampleBits);
}

std::unique_ptr<CallbackStore> AnalogInputSim::RegisterVoltageCallback(
    NotifyCallback callback, bool initialNotify) {
  auto store = std::make_unique<CallbackStore>(
      m_index, -1, callback, &HALSIM_CancelAnalogInVoltageCallback);
  store->SetUid(HALSIM_RegisterAnalogInVoltageCallback(
      m_index, &CallbackStoreThunk, store.get(), initialNotify));
  return store;
}

double AnalogInputSim::GetVoltage() const {
  return HALSIM_GetAnalogInVoltage(m_index);
}

void AnalogInputSim::SetVoltage(double voltage) {
  HALSIM_SetAnalogInVoltage(m_index, voltage);
}

std::unique_ptr<CallbackStore>
AnalogInputSim::RegisterAccumulatorInitializedCallback(NotifyCallback callback,
                                                       bool initialNotify) {
  auto store = std::make_unique<CallbackStore>(
      m_index, -1, callback,
      &HALSIM_CancelAnalogInAccumulatorInitializedCallback);
  store->SetUid(HALSIM_RegisterAnalogInAccumulatorInitializedCallback(
      m_index, &CallbackStoreThunk, store.get(), initialNotify));
  return store;
}

bool AnalogInputSim::GetAccumulatorInitialized() const {
  return HALSIM_GetAnalogInAccumulatorInitialized(m_index);
}

void AnalogInputSim::SetAccumulatorInitialized(bool accumulatorInitialized) {
  HALSIM_SetAnalogInAccumulatorInitialized(m_index, accumulatorInitialized);
}

std::unique_ptr<CallbackStore> AnalogInputSim::RegisterAccumulatorValueCallback(
    NotifyCallback callback, bool initialNotify) {
  auto store = std::make_unique<CallbackStore>(
      m_index, -1, callback, &HALSIM_CancelAnalogInAccumulatorValueCallback);
  store->SetUid(HALSIM_RegisterAnalogInAccumulatorValueCallback(
      m_index, &CallbackStoreThunk, store.get(), initialNotify));
  return store;
}

int64_t AnalogInputSim::GetAccumulatorValue() const {
  return HALSIM_GetAnalogInAccumulatorValue(m_index);
}

void AnalogInputSim::SetAccumulatorValue(int64_t accumulatorValue) {
  HALSIM_SetAnalogInAccumulatorValue(m_index, accumulatorValue);
}

std::unique_ptr<CallbackStore> AnalogInputSim::RegisterAccumulatorCountCallback(
    NotifyCallback callback, bool initialNotify) {
  auto store = std::make_unique<CallbackStore>(
      m_index, -1, callback, &HALSIM_CancelAnalogInAccumulatorCountCallback);
  store->SetUid(HALSIM_RegisterAnalogInAccumulatorCountCallback(
      m_index, &CallbackStoreThunk, store.get(), initialNotify));
  return store;
}

int64_t AnalogInputSim::GetAccumulatorCount() const {
  return HALSIM_GetAnalogInAccumulatorCount(m_index);
}

void AnalogInputSim::SetAccumulatorCount(int64_t accumulatorCount) {
  HALSIM_SetAnalogInAccumulatorCount(m_index, accumulatorCount);
}

std::unique_ptr<CallbackStore>
AnalogInputSim::RegisterAccumulatorCenterCallback(NotifyCallback callback,
                                                  bool initialNotify) {
  auto store = std::make_unique<CallbackStore>(
      m_index, -1, callback, &HALSIM_CancelAnalogInAccumulatorCenterCallback);
  store->SetUid(HALSIM_RegisterAnalogInAccumulatorCenterCallback(
      m_index, &CallbackStoreThunk, store.get(), initialNotify));
  return store;
}

int AnalogInputSim::GetAccumulatorCenter() const {
  return HALSIM_GetAnalogInAccumulatorCenter(m_index);
}

void AnalogInputSim::SetAccumulatorCenter(int accumulatorCenter) {
  HALSIM_SetAnalogInAccumulatorCenter(m_index, accumulatorCenter);
}

std::unique_ptr<CallbackStore>
AnalogInputSim::RegisterAccumulatorDeadbandCallback(NotifyCallback callback,
                                                    bool initialNotify) {
  auto store = std::make_unique<CallbackStore>(
      m_index, -1, callback, &HALSIM_CancelAnalogInAccumulatorDeadbandCallback);
  store->SetUid(HALSIM_RegisterAnalogInAccumulatorDeadbandCallback(
      m_index, &CallbackStoreThunk, store.get(), initialNotify));
  return store;
}

int AnalogInputSim::GetAccumulatorDeadband() const {
  return HALSIM_GetAnalogInAccumulatorDeadband(m_index);
}

void AnalogInputSim::SetAccumulatorDeadband(int accumulatorDeadband) {
  HALSIM_SetAnalogInAccumulatorDeadband(m_index, accumulatorDeadband);
}

void AnalogInputSim::ResetData() {
  HALSIM_ResetAnalogInData(m_index);
}
