// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#ifndef WPIUTIL_WPI_BASE64_H_
#define WPIUTIL_WPI_BASE64_H_

#include <stdint.h>

#include <cstddef>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace wpi {

size_t Base64Decode(std::string_view encoded, std::string* plain);

size_t Base64Decode(std::string_view encoded, std::vector<uint8_t>* plain);

void Base64Encode(std::string_view plain, std::string* encoded);

void Base64Encode(std::span<const uint8_t> plain, std::string* encoded);

}  // namespace wpi

#endif  // WPIUTIL_WPI_BASE64_H_
