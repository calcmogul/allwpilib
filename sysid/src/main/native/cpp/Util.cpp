// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/Util.h"

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include <IconsFontAwesome6.h>
#include <imgui.h>

void sysid::CreateTooltip(const char* text) {
  ImGui::SameLine();
  ImGui::TextDisabled(" (?)");

  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}

void sysid::CreateErrorTooltip(const char* text) {
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f),
                     ICON_FA_TRIANGLE_EXCLAMATION);

  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", text);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}

void sysid::CreateErrorPopup(bool& isError, std::string_view errorMessage) {
  if (isError) {
    ImGui::OpenPopup("Exception Caught!");
  }

  // Handle exceptions.
  ImGui::SetNextWindowSize(ImVec2(480.f, 0.0f));
  if (ImGui::BeginPopupModal("Exception Caught!")) {
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s",
                       errorMessage.data());
    ImGui::PopTextWrapPos();
    if (ImGui::Button("Close")) {
      ImGui::CloseCurrentPopup();
      isError = false;
    }
    ImGui::EndPopup();
  }
}

void sysid::SaveFile(std::string_view contents,
                     const std::filesystem::path& path) {
  // Create the path if it doesn't already exist.
  std::filesystem::create_directories(path.root_directory());

  // Open a fd_ostream to write to file.
  std::ofstream ostream{path.string()};

  // Check error code.
  if (!ostream.is_open()) {
    throw std::runtime_error("Cannot write to file");
  }

  // Write contents.
  ostream << contents;
}
