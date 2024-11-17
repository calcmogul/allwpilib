// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <utility>
#include <vector>

namespace wpi {

/**
 * Implements a table of key-value pairs with linear interpolation between
 * values.
 *
 * If there's no matching key, the value returned will be a linear interpolation
 * between the keys before and after the provided one.
 *
 * @tparam Key   The key type.
 * @tparam Value The value type.
 */
template <typename Key, typename Value>
class interpolating_map {
 public:
  /**
   * Inserts a key-value pair.
   *
   * @param key   The key.
   * @param value The value.
   */
  constexpr void insert(const Key& key, const Value& value) {
    std::pair item{key, value};
    m_container.insert(
        std::upper_bound(
            m_container.begin(), m_container.end(), key,
            [](const Key& lhs, const auto& rhs) { return lhs < rhs.first; }),
        item);
  }

  /**
   * Inserts a key-value pair.
   *
   * @param key   The key.
   * @param value The value.
   */
  constexpr void insert(Key&& key, Value&& value) {
    std::pair item{key, value};
    m_container.insert(
        std::upper_bound(
            m_container.begin(), m_container.end(), key,
            [](const Key& lhs, const auto& rhs) { return lhs < rhs.first; }),
        item);
  }

  /**
   * Returns the value associated with a given key.
   *
   * If there's no matching key, the value returned will be a linear
   * interpolation between the keys before and after the provided one.
   *
   * @param key The key.
   */
  constexpr Value operator[](const Key& key) const {
    using const_iterator = typename decltype(m_container)::const_iterator;

    // Get iterator to upper bound key-value pair for the given key
    const_iterator upper = std::upper_bound(
        m_container.begin(), m_container.end(), key,
        [](const Key& lhs, const auto& rhs) { return lhs < rhs.first; });

    // If key > largest key in table, return value for largest table key
    if (upper == m_container.end()) {
      return (--upper)->second;
    }

    // If key <= smallest key in table, return value for smallest table key
    if (upper == m_container.begin()) {
      return upper->second;
    }

    // Get iterator to lower bound key-value pair
    const_iterator lower = upper;
    --lower;

    // Perform linear interpolation between lower and upper bound
    const double delta = (key - lower->first) / (upper->first - lower->first);
    return delta * upper->second + (1.0 - delta) * lower->second;
  }

  /**
   * Clears the contents.
   */
  constexpr void clear() { m_container.clear(); }

 private:
  std::vector<std::pair<Key, Value>> m_container;
};

}  // namespace wpi
