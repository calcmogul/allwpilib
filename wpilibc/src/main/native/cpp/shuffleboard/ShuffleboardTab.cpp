// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/shuffleboard/ShuffleboardTab.h"

#include <string>
#include <vector>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

#include "frc/shuffleboard/LayoutType.h"
#include "frc/shuffleboard/ShuffleboardComponentBase.h"
#include "frc/shuffleboard/ShuffleboardValue.h"

using namespace frc;

ShuffleboardTab::ShuffleboardTab(ShuffleboardRoot& root, std::string_view title)
    : ShuffleboardValue(title), ShuffleboardContainer(title), m_root(root) {}

ShuffleboardRoot& ShuffleboardTab::GetRoot() {
  return m_root;
}

void ShuffleboardTab::BuildInto(std::shared_ptr<nt::NetworkTable> parentTable,
                                std::shared_ptr<nt::NetworkTable> metaTable) {
  auto tabTable = parentTable->GetSubTable(GetTitle());
  tabTable->GetEntry(".type").SetString("ShuffleboardTab");
  for (auto& component : GetComponents()) {
    component->BuildInto(tabTable,
                         metaTable->GetSubTable(component->GetTitle()));
  }
}
