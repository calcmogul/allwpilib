/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011-2017. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>
#include <string>
#include <type_traits>

#include "SmartDashboard/SendableChooserBase.h"
#include "llvm/StringMap.h"
#include "llvm/StringRef.h"
#include "tables/ITable.h"

namespace frc {

/**
 * The {@link SendableChooser} class is a useful tool for presenting a selection
 * of options to the {@link SmartDashboard}.
 *
 * <p>For instance, you may wish to be able to select between multiple
 * autonomous modes. You can do this by putting every possible {@link Command}
 * you want to run as an autonomous into a {@link SendableChooser} and then put
 * it into the {@link SmartDashboard} to have a list of options appear on the
 * laptop.  Once autonomous starts, simply ask the {@link SendableChooser} what
 * the selected value is.</p>
 *
 * @tparam T The type of values to be stored
 * @see SmartDashboard
 */
template <class T>
class SendableChooser : public SendableChooserBase {
 public:
  virtual ~SendableChooser() = default;

  void AddObject(llvm::StringRef name, T&& object);
  void AddDefault(llvm::StringRef name, T&& object);

  template <class U, typename = std::enable_if_t<
                         !std::is_same<T, std::unique_ptr<U>>::value>>
  T GetSelected();

  template <class U, typename = std::enable_if_t<
                         std::is_same<T, std::unique_ptr<U>>::value>>
  U* GetSelected();

  void InitTable(std::shared_ptr<ITable> subtable) override;

 private:
  llvm::StringMap<T> m_choices;
};

}  // namespace frc

#include "SendableChooser.inc"
