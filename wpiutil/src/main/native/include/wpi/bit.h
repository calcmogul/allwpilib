// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <stdint.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <type_traits>

namespace wpi {

template <typename value_type>
[[nodiscard]]
inline value_type byteswap(value_type value) {
  static_assert(std::has_unique_object_representations_v<value_type>,
                "value_type may not have padding bits");
  auto value_representation =
      std::bit_cast<std::array<std::byte, sizeof(value_type)>>(value);
  std::ranges::reverse(value_representation);
  return std::bit_cast<value_type>(value_representation);
  return value;
}

namespace detail {

/// Read a value of a particular endianness from memory.
template <typename value_type, std::endian endian>
[[nodiscard]]
inline value_type read(const void* memory) {
  value_type ret;
  std::memcpy(&ret, memory, sizeof(value_type));
  if constexpr (endian != std::endian::native) {
    return byteswap(ret);
  } else {
    return ret;
  }
}

/// Write a value to memory with a particular endianness.
template <typename value_type, std::endian endian>
inline void write(void* memory, value_type value) {
  if constexpr (endian != std::endian::native) {
    value = byteswap(value);
  }
  std::memcpy(memory, &value, sizeof(value_type));
}

}  // namespace detail

[[nodiscard]]
inline uint16_t read16le(const void* ptr) {
  return detail::read<uint16_t, std::endian::little>(ptr);
}
[[nodiscard]]
inline uint32_t read32le(const void* ptr) {
  return detail::read<uint32_t, std::endian::little>(ptr);
}
[[nodiscard]]
inline uint64_t read64le(const void* ptr) {
  return detail::read<uint64_t, std::endian::little>(ptr);
}
[[nodiscard]]
inline uint16_t read16be(const void* ptr) {
  return detail::read<uint16_t, std::endian::big>(ptr);
}
[[nodiscard]]
inline uint32_t read32be(const void* ptr) {
  return detail::read<uint32_t, std::endian::big>(ptr);
}
[[nodiscard]]
inline uint64_t read64be(const void* ptr) {
  return detail::read<uint64_t, std::endian::big>(ptr);
}

inline void write16le(void* ptr, uint16_t value) {
  detail::write<uint16_t, std::endian::little>(ptr, value);
}
inline void write32le(void* ptr, uint32_t value) {
  detail::write<uint32_t, std::endian::little>(ptr, value);
}
inline void write64le(void* ptr, uint64_t value) {
  detail::write<uint64_t, std::endian::little>(ptr, value);
}
inline void write16be(void* ptr, uint16_t value) {
  detail::write<uint16_t, std::endian::big>(ptr, value);
}
inline void write32be(void* ptr, uint32_t value) {
  detail::write<uint32_t, std::endian::big>(ptr, value);
}
inline void write64be(void* ptr, uint64_t value) {
  detail::write<uint64_t, std::endian::big>(ptr, value);
}

}  // namespace wpi
