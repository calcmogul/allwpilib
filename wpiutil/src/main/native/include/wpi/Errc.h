// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//===- llvm/Support/Errc.h - Defines the wpi::errc enum --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// While std::error_code works OK on all platforms we use, there are some
// some problems with std::errc that can be avoided by using our own
// enumeration:
//
// * std::errc is a namespace in some implementations. That means that ADL
//   doesn't work and it is sometimes necessary to write std::make_error_code
//   or in templates:
//   using std::make_error_code;
//   make_error_code(...);
//
//   with this enum it is safe to always just use make_error_code.
//
// * Some implementations define fewer names than others. This header has
//   the intersection of all the ones we support.
//
// * std::errc is just marked with is_error_condition_enum. This means that
//   common patterns like AnErrorCode == errc::no_such_file_or_directory take
//   4 virtual calls instead of two comparisons.
//===----------------------------------------------------------------------===//

#ifndef WPIUTIL_WPI_ERRC_H_
#define WPIUTIL_WPI_ERRC_H_

#include <system_error>

namespace wpi {
enum class errc {
  argument_list_too_long = static_cast<int>(std::errc::argument_list_too_long),
  argument_out_of_domain = static_cast<int>(std::errc::argument_out_of_domain),
  bad_address = static_cast<int>(std::errc::bad_address),
  bad_file_descriptor = static_cast<int>(std::errc::bad_file_descriptor),
  broken_pipe = static_cast<int>(std::errc::broken_pipe),
  device_or_resource_busy =
      static_cast<int>(std::errc::device_or_resource_busy),
  directory_not_empty = static_cast<int>(std::errc::directory_not_empty),
  executable_format_error =
      static_cast<int>(std::errc::executable_format_error),
  file_exists = static_cast<int>(std::errc::file_exists),
  file_too_large = static_cast<int>(std::errc::file_too_large),
  filename_too_long = static_cast<int>(std::errc::filename_too_long),
  function_not_supported = static_cast<int>(std::errc::function_not_supported),
  illegal_byte_sequence = static_cast<int>(std::errc::illegal_byte_sequence),
  inappropriate_io_control_operation =
      static_cast<int>(std::errc::inappropriate_io_control_operation),
  interrupted = static_cast<int>(std::errc::interrupted),
  invalid_argument = static_cast<int>(std::errc::invalid_argument),
  invalid_seek = static_cast<int>(std::errc::invalid_seek),
  io_error = static_cast<int>(std::errc::io_error),
  is_a_directory = static_cast<int>(std::errc::is_a_directory),
  no_child_process = static_cast<int>(std::errc::no_child_process),
  no_lock_available = static_cast<int>(std::errc::no_lock_available),
  no_space_on_device = static_cast<int>(std::errc::no_space_on_device),
  no_such_device_or_address =
      static_cast<int>(std::errc::no_such_device_or_address),
  no_such_device = static_cast<int>(std::errc::no_such_device),
  no_such_file_or_directory =
      static_cast<int>(std::errc::no_such_file_or_directory),
  no_such_process = static_cast<int>(std::errc::no_such_process),
  not_a_directory = static_cast<int>(std::errc::not_a_directory),
  not_enough_memory = static_cast<int>(std::errc::not_enough_memory),
  not_supported = static_cast<int>(std::errc::not_supported),
  operation_not_permitted =
      static_cast<int>(std::errc::operation_not_permitted),
  permission_denied = static_cast<int>(std::errc::permission_denied),
  read_only_file_system = static_cast<int>(std::errc::read_only_file_system),
  resource_deadlock_would_occur =
      static_cast<int>(std::errc::resource_deadlock_would_occur),
  resource_unavailable_try_again =
      static_cast<int>(std::errc::resource_unavailable_try_again),
  result_out_of_range = static_cast<int>(std::errc::result_out_of_range),
  too_many_files_open_in_system =
      static_cast<int>(std::errc::too_many_files_open_in_system),
  too_many_files_open = static_cast<int>(std::errc::too_many_files_open),
  too_many_links = static_cast<int>(std::errc::too_many_links)
};

inline std::error_code make_error_code(errc E) {
  return std::error_code(static_cast<int>(E), std::generic_category());
}
}  // namespace wpi

namespace std {
template <>
struct is_error_code_enum<wpi::errc> : std::true_type {};
}  // namespace std
#endif  // WPIUTIL_WPI_ERRC_H_
