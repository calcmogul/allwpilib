// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) 2016 Nic Holthaus
//
// The MIT License (MIT)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// ATTRIBUTION:
// Parts of this work have been adapted from:
// http://stackoverflow.com/questions/35069778/create-comparison-trait-for-template-classes-whose-parameters-are-in-a-different
// http://stackoverflow.com/questions/28253399/check-traits-for-all-variadic-template-arguments/28253503
// http://stackoverflow.com/questions/36321295/rational-approximation-of-square-root-of-stdratio-at-compile-time?noredirect=1#comment60266601_36321295

/// @file  units.h
/// @brief Complete implementation of `units` - a compile-time, header-only,
///        unit conversion library built on C++20 with no dependencies.

#pragma once

//--------------------
//  INCLUDES
//--------------------

#include <stdint.h>

#include <chrono>
#include <cmath>
#include <concepts>
#include <limits>
#include <numbers>
#include <ratio>
#include <type_traits>

#if defined(UNIT_LIB_ENABLE_IOSTREAM)
#include <iostream>
#include <locale>
#include <string>
#endif
#if __has_include(<fmt/format.h>) && !defined(UNIT_LIB_DISABLE_FMT)
#include <locale>
#include <string>

#include <fmt/format.h>
#endif

#include <gcem.hpp>

//------------------------------
//  STRING FORMATTER
//------------------------------

namespace units::detail {
template <typename T>
std::string to_string(const T& t) {
  std::string str{std::to_string(t)};
  int offset{1};

  // Remove trailing decimal points for integer value units. Locale-aware!
  struct lconv* lc;
  lc = localeconv();
  char decimalPoint = *lc->decimal_point;
  if (str.find_last_not_of('0') == str.find(decimalPoint)) {
    offset = 0;
  }
  str.erase(str.find_last_not_of('0') + offset, std::string::npos);
  return str;
}
}  // namespace units::detail

namespace units {

template <typename T>
constexpr const char* name(const T&);

template <typename T>
constexpr const char* abbreviation(const T&);

}  // namespace units

//------------------------------
//  MACROS
//------------------------------

/**
 * @def UNIT_ADD_UNIT_TAGS(namespaceName, nameSingular, namePlural,
 *                         abbreviation, definition)
 * @brief Helper macro for generating the boiler-plate code generating the tags
 *        of a new unit.
 * @details The macro generates singular, plural, and abbreviated forms of the
 *          unit definition (e.g. `meter`, `meters`, and `m`), as aliases for
 *          the unit tag.
 * @param namespaceName namespace in which the new units will be encapsulated.
 * @param nameSingular singular version of the unit name, e.g. 'meter'
 * @param namePlural plural version of the unit name, e.g. 'meters'
 * @param abbreviation abbreviated unit name, e.g. 'm'
 * @param definition the variadic parameter is used for the definition of the
 *                   unit (e.g. `unit<std::ratio<1>,
 *                   units::category::length_unit>`)
 * @note a variadic template is used for the definition to allow templates with
 *       commas to be easily expanded. All the variadic 'arguments' should
 *       together comprise the unit definition.
 */
#define UNIT_ADD_UNIT_TAGS(namespaceName, nameSingular, namePlural, \
                           abbreviation, /*definition*/...)         \
  namespace namespaceName {                                         \
  /** @name Units (full names plural) */                            \
  /** @{ */                                                         \
  using namePlural = __VA_ARGS__;                                   \
  /** @} */                                                         \
  /** @name Units (full names singular) */                          \
  /** @{ */                                                         \
  using nameSingular = namePlural;                                  \
  /** @} */                                                         \
  /** @name Units (abbreviated) */                                  \
  /** @{ */                                                         \
  using abbreviation = namePlural;                                  \
  /** @} */                                                         \
  }

/**
 * @def UNIT_ADD_CUSTOM_TYPE_UNIT_DEFINITION(namespaceName, nameSingular,
 *                                           underlyingType)
 * @brief Macro for generating the boiler-plate code for a unit_t type
 *        definition with a non-default underlying type.
 * @details The macro generates the definition of the unit container types, e.g.
 *          `meter_t`
 * @param namespaceName namespace in which the new units will be encapsulated.
 * @param nameSingular singular version of the unit name, e.g. 'meter'
 * @param underlyingType the underlying type
 */
#define UNIT_ADD_CUSTOM_TYPE_UNIT_DEFINITION(namespaceName, nameSingular, \
                                             underlyingType)              \
  namespace namespaceName {                                               \
  /** @name Unit Containers */                                            \
  /** @{ */                                                               \
  using nameSingular##_t = unit_t<nameSingular, underlyingType>;          \
  /** @} */                                                               \
  }

/**
 * @def UNIT_ADD_IO(namespaceName,nameSingular, abbreviation)
 * @brief Macro for generating the boiler-plate code needed for I/O for a new
 *        unit.
 * @details The macro generates the code to insert units into an ostream. It
 *          prints both the value and abbreviation of the unit when invoked.
 * @param namespaceName namespace in which the new units will be encapsulated.
 * @param nameSingular singular version of the unit name, e.g. 'meter'
 * @param abbrev abbreviated unit name, e.g. 'm'
 * @note When UNIT_LIB_ENABLE_IOSTREAM isn't defined, the macro does not
 *       generate any code
 */
#if __has_include(<fmt/format.h>) && !defined(UNIT_LIB_DISABLE_FMT)
#define UNIT_ADD_IO(namespaceName, nameSingular, abbrev)               \
  }                                                                    \
  template <>                                                          \
  struct fmt::formatter<units::namespaceName::nameSingular##_t>        \
      : fmt::formatter<double> {                                       \
    template <typename FormatContext>                                  \
    auto format(const units::namespaceName::nameSingular##_t& obj,     \
                FormatContext& ctx) -> decltype(ctx.out()) {           \
      auto out = ctx.out();                                            \
      out = fmt::formatter<double>::format(obj(), ctx);                \
      return fmt::format_to(out, " " #abbrev);                         \
    }                                                                  \
  };                                                                   \
  namespace units {                                                    \
  namespace namespaceName {                                            \
  inline std::string to_string(const nameSingular##_t& obj) {          \
    return units::detail::to_string(obj()) + std::string(" " #abbrev); \
  }                                                                    \
  }
#elif defined(UNIT_LIB_ENABLE_IOSTREAM)
#define UNIT_ADD_IO(namespaceName, nameSingular, abbrev)               \
  namespace namespaceName {                                            \
  inline std::ostream& operator<<(std::ostream& os,                    \
                                  const nameSingular##_t& obj) {       \
    os << obj() << " " #abbrev;                                        \
    return os;                                                         \
  }                                                                    \
  inline std::string to_string(const nameSingular##_t& obj) {          \
    return units::detail::to_string(obj()) + std::string(" " #abbrev); \
  }                                                                    \
  }
#endif

/**
 * @def UNIT_ADD_NAME(namespaceName, nameSingular, abbreviation)
 * @brief Macro for generating constexpr names/abbreviations for units.
 * @details The macro generates names for units. E.g. name() of 1_m would be
 *          "meter", and abbreviation would be "m".
 * @param namespaceName namespace in which the new units will be encapsulated.
 *        All literal values are placed in the `units::literals` namespace.
 * @param nameSingular singular version of the unit name, e.g. 'meter'
 * @param abbreviation abbreviated unit name, e.g. 'm'
 */
#define UNIT_ADD_NAME(namespaceName, nameSingular, abbrev)                    \
  template <>                                                                 \
  inline constexpr const char* name(const namespaceName::nameSingular##_t&) { \
    return #nameSingular;                                                     \
  }                                                                           \
  template <>                                                                 \
  inline constexpr const char* abbreviation(                                  \
      const namespaceName::nameSingular##_t&) {                               \
    return #abbrev;                                                           \
  }

/**
 * @def UNIT_ADD_LITERALS(namespaceName, nameSingular, abbreviation)
 * @brief Macro for generating user-defined literals for units.
 * @details The macro generates user-defined literals for units. A literal
 *          suffix is created using the abbreviation (e.g. `10.0_m`).
 * @param namespaceName namespace in which the new units will be encapsulated.
 *        All literal values are placed in the `units::literals` namespace.
 * @param nameSingular singular version of the unit name, e.g. 'meter'
 * @param abbreviation abbreviated unit name, e.g. 'm'
 */
#define UNIT_ADD_LITERALS(namespaceName, nameSingular, abbreviation)          \
  namespace literals {                                                        \
  inline constexpr namespaceName::nameSingular##_t operator""_##abbreviation( \
      long double d) {                                                        \
    return namespaceName::nameSingular##_t(                                   \
        static_cast<namespaceName::nameSingular##_t::underlying_type>(d));    \
  }                                                                           \
  inline constexpr namespaceName::nameSingular##_t operator""_##abbreviation( \
      unsigned long long d) { /* NOLINT */                                    \
    return namespaceName::nameSingular##_t(                                   \
        static_cast<namespaceName::nameSingular##_t::underlying_type>(d));    \
  }                                                                           \
  }

/**
 * @def UNIT_ADD(namespaceName, nameSingular, namePlural, abbreviation,
 *               definition)
 * @brief Macro for generating the boiler-plate code needed for a new unit.
 * @details The macro generates singular, plural, and abbreviated forms of the
 *          unit definition (e.g. `meter`, `meters`, and `m`), as well as the
 *          appropriately named unit container (e.g. `meter_t`). A literal
 *          suffix is created using the abbreviation (e.g. `10.0_m`). It also
 *          defines a class-specific cout function which prints both the value
 *          and abbreviation of the unit when invoked.
 * @param namespaceName namespace in which the new units will be encapsulated.
 *        All literal values are placed in the `units::literals` namespace.
 * @param nameSingular singular version of the unit name, e.g. 'meter'
 * @param namePlural plural version of the unit name, e.g. 'meters'
 * @param abbreviation abbreviated unit name, e.g. 'm'
 * @param definition the variadic parameter is used for the definition of the
 *        unit (e.g. `unit<std::ratio<1>, units::category::length_unit>`)
 * @note a variadic template is used for the definition to allow templates with
 *       commas to be easily expanded. All the variadic 'arguments' should
 *       together comprise the unit definition.
 */
#define UNIT_ADD(namespaceName, nameSingular, namePlural, abbreviation,     \
                 /*definition*/...)                                         \
  UNIT_ADD_UNIT_TAGS(namespaceName, nameSingular, namePlural, abbreviation, \
                     __VA_ARGS__)                                           \
  namespace namespaceName {                                                 \
  /** @name Unit Containers */                                              \
  /** @{ */                                                                 \
  using nameSingular##_t = unit_t<nameSingular>;                            \
  /** @} */                                                                 \
  }                                                                         \
  UNIT_ADD_NAME(namespaceName, nameSingular, abbreviation)                  \
  UNIT_ADD_IO(namespaceName, nameSingular, abbreviation)                    \
  UNIT_ADD_LITERALS(namespaceName, nameSingular, abbreviation)

/**
 * @def UNIT_ADD_WITH_CUSTOM_TYPE(namespaceName, nameSingular, namePlural,
 *                                abbreviation, underlyingType, definition)
 * @brief Macro for generating the boiler-plate code needed for a new unit with
 *        a non-default underlying type.
 * @details The macro generates singular, plural, and abbreviated forms of the
 *          unit definition (e.g. `meter`, `meters`, and `m`), as well as the
 *          appropriately named unit container (e.g. `meter_t`). A literal
 *          suffix is created using the abbreviation (e.g. `10.0_m`). It also
 *          defines a class-specific cout function which prints both the value
 *          and abbreviation of the unit when invoked.
 * @param namespaceName namespace in which the new units will be encapsulated.
 *        All literal values are placed in the `units::literals` namespace.
 * @param nameSingular singular version of the unit name, e.g. 'meter'
 * @param namePlural plural version of the unit name, e.g. 'meters'
 * @param abbreviation abbreviated unit name, e.g. 'm'
 * @param underlyingType the underlying type, e.g. 'int' or 'float'
 * @param definition the variadic parameter is used for the definition of the
 *        unit (e.g. `unit<std::ratio<1>, units::category::length_unit>`)
 * @note a variadic template is used for the definition to allow templates with
 *       commas to be easily expanded. All the variadic 'arguments' should
 *       together comprise the unit definition.
 */
#define UNIT_ADD_WITH_CUSTOM_TYPE(namespaceName, nameSingular, namePlural,  \
                                  abbreviation, underlyingType,             \
                                  /*definition*/...)                        \
  UNIT_ADD_UNIT_TAGS(namespaceName, nameSingular, namePlural, abbreviation, \
                     __VA_ARGS__)                                           \
  UNIT_ADD_CUSTOM_TYPE_UNIT_DEFINITION(namespaceName, nameSingular,         \
                                       underlyingType)                      \
  UNIT_ADD_IO(namespaceName, nameSingular, abbreviation)                    \
  UNIT_ADD_LITERALS(namespaceName, nameSingular, abbreviation)

/**
 * @def UNIT_ADD_DECIBEL(namespaceName, nameSingular, abbreviation)
 * @brief Macro to create decibel container and literals for an existing unit
 *        type.
 * @details This macro generates the decibel unit container, cout overload, and
 *          literal definitions.
 * @param namespaceName namespace in which the new units will be encapsulated.
 *        All literal values are placed in the `units::literals` namespace.
 * @param nameSingular singular version of the base unit name, e.g. 'watt'
 * @param abbreviation abbreviated decibel unit name, e.g. 'dBW'
 */
#define UNIT_ADD_DECIBEL(namespaceName, nameSingular, abbreviation) \
  namespace namespaceName {                                         \
  /** @name Unit Containers */                                      \
  /** @{ */                                                         \
  using abbreviation##_t =                                          \
      unit_t<nameSingular, double, units::decibel_scale<double>>;   \
  /** @} */                                                         \
  }                                                                 \
  UNIT_ADD_IO(namespaceName, abbreviation, abbreviation)            \
  UNIT_ADD_LITERALS(namespaceName, abbreviation, abbreviation)

/**
 * @def UNIT_ADD_CATEGORY_TRAIT(unitCategory, baseUnit)
 * @brief Macro to create the `is_category_unit` type trait.
 * @details This trait allows users to test whether a given type matches an
 *          intended category. This macro comprises all the boiler-plate code
 *          necessary to do so.
 * @param unitCategory The name of the category of unit, e.g. length or mass.
 */
#define UNIT_ADD_CATEGORY_TRAIT_DETAIL(unitCategory)                        \
  namespace traits {                                                        \
  /** @cond */                                                              \
  namespace detail {                                                        \
                                                                            \
  template <typename T>                                                     \
  struct is_##unitCategory##_unit_impl : std::false_type {};                \
                                                                            \
  template <typename C, typename U, typename P, typename T>                 \
  struct is_##unitCategory##_unit_impl<units::unit<C, U, P, T>>             \
      : std::is_same<units::traits::base_unit_of<                           \
                         typename units::unit<C, U, P, T>::base_unit_type>, \
                     units::category::unitCategory##_unit>::type {};        \
                                                                            \
  template <typename U, typename S, Scale<S> N>                             \
  struct is_##unitCategory##_unit_impl<units::unit_t<U, S, N>>              \
      : std::is_same<units::traits::base_unit_of<                           \
                         typename units::unit_t<U, S, N>::unit_type>,       \
                     units::category::unitCategory##_unit>::type {};        \
  }                                                                         \
  /** @endcond */                                                           \
  }

#define UNIT_ADD_IS_UNIT_CATEGORY_TRAIT(unitCategory)       \
  namespace traits {                                        \
                                                            \
  template <typename T>                                     \
  inline constexpr bool is_##unitCategory##_unit_v =        \
      units::traits::detail::is_##unitCategory##_unit_impl< \
          std::decay_t<T>>::value;                          \
  }

#define UNIT_ADD_CATEGORY_TRAIT(unitCategory)                                  \
  UNIT_ADD_CATEGORY_TRAIT_DETAIL(unitCategory)                                 \
  /** @ingroup TypeTraits */                                                   \
  /**                                                                          \
   * @brief Trait which tests whether a type represents a unit of unitCategory \
   * @details Inherits from `std::true_type` or `std::false_type`. Use         \
   *          `is_##unitCategory##_unit_v<T>` to test the unit represents a    \
   *          unitCategory quantity.                                           \
   * @tparam T one or more types to test                                       \
   */                                                                          \
  UNIT_ADD_IS_UNIT_CATEGORY_TRAIT(unitCategory)

/**
 * @def UNIT_ADD_WITH_METRIC_PREFIXES(nameSingular, namePlural, abbreviation,
 *                                    definition)
 * @brief Macro for generating the boiler-plate code needed for a new unit,
 *        including its metric prefixes from femto to peta.
 * @details See UNIT_ADD. In addition to generating the unit definition and
 *          containers '(e.g. `meters` and 'meter_t', it also creates
 *          corresponding units with metric suffixes such as `millimeters`, and
 *          `millimeter_t`), as well as the literal suffixes (e.g. `10.0_mm`).
 * @param namespaceName namespace in which the new units will be encapsulated.
 *        All literal values are placed in the `units::literals` namespace.
 * @param nameSingular singular version of the unit name, e.g. 'meter'
 * @param namePlural plural version of the unit name, e.g. 'meters'
 * @param abbreviation abbreviated unit name, e.g. 'm'
 * @param definition the variadic parameter is used for the definition of the
 *        unit (e.g. `unit<std::ratio<1>, units::category::length_unit>`)
 * @note a variadic template is used for the definition to allow templates with
 *       commas to be easily expanded. All the variadic 'arguments' should
 *       together comprise the unit definition.
 */
#define UNIT_ADD_WITH_METRIC_PREFIXES(namespaceName, nameSingular, namePlural, \
                                      abbreviation, /*definition*/...)         \
  UNIT_ADD(namespaceName, nameSingular, namePlural, abbreviation, __VA_ARGS__) \
  UNIT_ADD(namespaceName, femto##nameSingular, femto##namePlural,              \
           f##abbreviation, femto<namePlural>)                                 \
  UNIT_ADD(namespaceName, pico##nameSingular, pico##namePlural,                \
           p##abbreviation, pico<namePlural>)                                  \
  UNIT_ADD(namespaceName, nano##nameSingular, nano##namePlural,                \
           n##abbreviation, nano<namePlural>)                                  \
  UNIT_ADD(namespaceName, micro##nameSingular, micro##namePlural,              \
           u##abbreviation, micro<namePlural>)                                 \
  UNIT_ADD(namespaceName, milli##nameSingular, milli##namePlural,              \
           m##abbreviation, milli<namePlural>)                                 \
  UNIT_ADD(namespaceName, centi##nameSingular, centi##namePlural,              \
           c##abbreviation, centi<namePlural>)                                 \
  UNIT_ADD(namespaceName, deci##nameSingular, deci##namePlural,                \
           d##abbreviation, deci<namePlural>)                                  \
  UNIT_ADD(namespaceName, deca##nameSingular, deca##namePlural,                \
           da##abbreviation, deca<namePlural>)                                 \
  UNIT_ADD(namespaceName, hecto##nameSingular, hecto##namePlural,              \
           h##abbreviation, hecto<namePlural>)                                 \
  UNIT_ADD(namespaceName, kilo##nameSingular, kilo##namePlural,                \
           k##abbreviation, kilo<namePlural>)                                  \
  UNIT_ADD(namespaceName, mega##nameSingular, mega##namePlural,                \
           M##abbreviation, mega<namePlural>)                                  \
  UNIT_ADD(namespaceName, giga##nameSingular, giga##namePlural,                \
           G##abbreviation, giga<namePlural>)                                  \
  UNIT_ADD(namespaceName, tera##nameSingular, tera##namePlural,                \
           T##abbreviation, tera<namePlural>)                                  \
  UNIT_ADD(namespaceName, peta##nameSingular, peta##namePlural,                \
           P##abbreviation, peta<namePlural>)

/**
 * @def UNIT_ADD_WITH_METRIC_AND_BINARY_PREFIXES(nameSingular, namePlural,
 *                                               abbreviation, definition)
 * @brief Macro for generating the boiler-plate code needed for a new unit,
 *        including its metric prefixes from femto to peta, and binary prefixes
 *        from kibi to exbi.
 * @details See UNIT_ADD. In addition to generating the unit definition and
 *          containers '(e.g. `bytes` and 'byte_t', it also creates
 *          corresponding units with metric suffixes such as `millimeters`, and
 *          `millimeter_t`), as well as the literal suffixes (e.g. `10.0_B`).
 * @param namespaceName namespace in which the new units will be encapsulated.
 *        All literal values are placed in the `units::literals` namespace.
 * @param nameSingular singular version of the unit name, e.g. 'byte'
 * @param namePlural plural version of the unit name, e.g. 'bytes'
 * @param abbreviation abbreviated unit name, e.g. 'B'
 * @param definition the variadic parameter is used for the definition of the
 *        unit (e.g. `unit<std::ratio<1>, units::category::data_unit>`)
 * @note a variadic template is used for the definition to allow templates with
 *       commas to be easily expanded. All the variadic 'arguments' should
 *       together comprise the unit definition.
 */
#define UNIT_ADD_WITH_METRIC_AND_BINARY_PREFIXES(                             \
    namespaceName, nameSingular, namePlural, abbreviation, /*definition*/...) \
  UNIT_ADD_WITH_METRIC_PREFIXES(namespaceName, nameSingular, namePlural,      \
                                abbreviation, __VA_ARGS__)                    \
  UNIT_ADD(namespaceName, kibi##nameSingular, kibi##namePlural,               \
           Ki##abbreviation, kibi<namePlural>)                                \
  UNIT_ADD(namespaceName, mebi##nameSingular, mebi##namePlural,               \
           Mi##abbreviation, mebi<namePlural>)                                \
  UNIT_ADD(namespaceName, gibi##nameSingular, gibi##namePlural,               \
           Gi##abbreviation, gibi<namePlural>)                                \
  UNIT_ADD(namespaceName, tebi##nameSingular, tebi##namePlural,               \
           Ti##abbreviation, tebi<namePlural>)                                \
  UNIT_ADD(namespaceName, pebi##nameSingular, pebi##namePlural,               \
           Pi##abbreviation, pebi<namePlural>)                                \
  UNIT_ADD(namespaceName, exbi##nameSingular, exbi##namePlural,               \
           Ei##abbreviation, exbi<namePlural>)

//--------------------
//  UNITS NAMESPACE
//--------------------

/**
 * @namespace units
 * @brief Unit Conversion Library namespace
 */
namespace units {

//----------------------------------
//  DOXYGEN
//----------------------------------

/**
 * @defgroup Units Unit API
 */

/**
 * @defgroup UnitContainers Unit Containers
 * @ingroup Units
 * @brief Defines a series of classes which contain dimensioned values. Unit
 *        containers store a value, and support various arithmetic operations.
 */

/**
 * @defgroup UnitTypes Unit Types
 * @ingroup Units
 * @brief Defines a series of classes which represent units. These types are
 *        tags used by the conversion function, to create compound units, or to
 *        create `unit_t` types. By themselves, they are not containers and have
 *        no stored value.
 */

/**
 * @defgroup UnitManipulators Unit Manipulators
 * @ingroup Units
 * @brief Defines a series of classes used to manipulate unit types, such as
 *        `inverse<>`, `squared<>`, and metric prefixes. Unit manipulators can
 *        be chained together, e.g. `inverse<squared<pico<time::seconds>>>` to
 *        represent picoseconds^-2.
 */

/**
 * @defgroup CompileTimeUnitManipulators Compile-time Unit Manipulators
 * @ingroup Units
 * @brief Defines a series of classes used to manipulate `unit_value_t` types at
 *        compile-time, such as `unit_value_add<>`, `unit_value_sqrt<>`, etc.
 *        Compile-time manipulators can be chained together, e.g.
 *        `unit_value_sqrt<unit_value_add<unit_value_power<a, 2>,
 *        unit_value_power<b, 2>>>` to represent `c = std::sqrt(a^2 + b^2).
 */

/**
 * @defgroup UnitMath Unit Math
 * @ingroup Units
 * @brief Defines a collection of unit-enabled, strongly-typed versions of
 *        `<cmath>` functions.
 * @details Includes most C++11 extensions.
 */

/**
 * @defgroup Conversion Explicit Conversion
 * @ingroup Units
 * @brief Functions used to convert values of one logical type to another.
 */

/**
 * @defgroup TypeTraits Type Traits
 * @ingroup Units
 * @brief Defines a series of classes to obtain unit type information at
 *        compile-time.
 */

//------------------------------
//  RATIO TRAITS
//------------------------------

/**
 * @ingroup TypeTraits
 * @{
 */

/**
 * @brief Concept that tests whether a type represents a std::ratio.
 *
 * A type represents a ratio if it has static numerator and denominator members.
 */
template <class U>
concept is_ratio = requires {
  U::num;
  U::den;
};

//------------------------------
//  UNIT TRAITS
//------------------------------

/** @cond */  // DOXYGEN IGNORE
namespace detail {

/**
 * @brief helper type to identify base units.
 * @details A non-templated base class for `base_unit` which enables RTTI
 *          testing.
 */
struct _base_unit_t {};

}  // namespace detail
/** @endcond */  // END DOXYGEN IGNORE

/**
 * @ingroup TypeTraits
 * @brief Concept which tests if a class is a `base_unit` type.
 */
template <class T>
concept is_base_unit = std::derived_from<T, units::detail::_base_unit_t>;

/** @cond */  // DOXYGEN IGNORE
namespace detail {

/**
 * @brief helper type to identify units.
 * @details A non-templated base class for `unit` which enables RTTI testing.
 */
struct _unit {};

template <intmax_t Num, intmax_t Den = 1>
using meter_ratio = std::ratio<Num, Den>;

}  // namespace detail
/** @endcond */  // END DOXYGEN IGNORE

/**
 * @ingroup TypeTraits
 * @brief Concept which tests if a class is a `unit`
 */
template <class T>
concept is_unit = std::derived_from<T, units::detail::_unit>;

/** @} */  // end of TypeTraits

//------------------------------
//  BASE UNIT CLASS
//------------------------------

/**
 * @ingroup UnitTypes
 * @brief Class representing SI base unit types.
 * @details Base units are represented by a combination of `std::ratio` template
 *          parameters, each describing the exponent of the type of unit they
 *          represent. Example: meters per second would be described by a +1
 *          exponent for meters, and a -1 exponent for seconds, thus:
 *          `base_unit<std::ratio<1>, std::ratio<0>, std::ratio<-1>>`
 * @tparam Meter `std::ratio` representing the exponent value for meters.
 * @tparam Kilogram `std::ratio` representing the exponent value for kilograms.
 * @tparam Second `std::ratio` representing the exponent value for seconds.
 * @tparam Radian `std::ratio` representing the exponent value for radians.
 *         Although radians are not SI base units, they are included because
 *         radians are described by the SI as mm⁻¹, which would make them
 *         indistinguishable from scalars.
 * @tparam Ampere `std::ratio` representing the exponent value for amperes.
 * @tparam Kelvin `std::ratio` representing the exponent value for Kelvin.
 * @tparam Mole `std::ratio` representing the exponent value for moles.
 * @tparam Candela `std::ratio` representing the exponent value for candelas.
 * @tparam Byte `std::ratio` representing the exponent value for bytes.
 * @sa category for type aliases for SI base_unit types.
 */
template <class Meter = detail::meter_ratio<0>, class Kilogram = std::ratio<0>,
          class Second = std::ratio<0>, class Radian = std::ratio<0>,
          class Ampere = std::ratio<0>, class Kelvin = std::ratio<0>,
          class Mole = std::ratio<0>, class Candela = std::ratio<0>,
          class Byte = std::ratio<0>>
struct base_unit : units::detail::_base_unit_t {
  static_assert(is_ratio<Meter>,
                "Template parameter `Meter` must be a `std::ratio` "
                "representing the exponent of meters the unit has");
  static_assert(is_ratio<Kilogram>,
                "Template parameter `Kilogram` must be a `std::ratio` "
                "representing the exponent of kilograms the unit has");
  static_assert(is_ratio<Second>,
                "Template parameter `Second` must be a `std::ratio` "
                "representing the exponent of seconds the unit has");
  static_assert(is_ratio<Ampere>,
                "Template parameter `Ampere` must be a `std::ratio` "
                "representing the exponent of amperes the unit has");
  static_assert(is_ratio<Kelvin>,
                "Template parameter `Kelvin` must be a `std::ratio` "
                "representing the exponent of kelvin the unit has");
  static_assert(is_ratio<Candela>,
                "Template parameter `Candela` must be a `std::ratio` "
                "representing the exponent of candelas the unit has");
  static_assert(is_ratio<Mole>,
                "Template parameter `Mole` must be a `std::ratio` representing "
                "the exponent of moles the unit has");
  static_assert(is_ratio<Radian>,
                "Template parameter `Radian` must be a `std::ratio` "
                "representing the exponent of radians the unit has");
  static_assert(is_ratio<Byte>,
                "Template parameter `Byte` must be a `std::ratio` representing "
                "the exponent of bytes the unit has");

  using meter_ratio = Meter;
  using kilogram_ratio = Kilogram;
  using second_ratio = Second;
  using radian_ratio = Radian;
  using ampere_ratio = Ampere;
  using kelvin_ratio = Kelvin;
  using mole_ratio = Mole;
  using candela_ratio = Candela;
  using byte_ratio = Byte;
};

template <typename Units>
inline constexpr bool is_dimensionless_unit_v =
    std::same_as<typename Units::base_unit_type::meter_ratio,
                 detail::meter_ratio<0>> &&
    std::same_as<typename Units::base_unit_type::second_ratio, std::ratio<0>> &&
    std::same_as<typename Units::base_unit_type::radian_ratio, std::ratio<0>> &&
    std::same_as<typename Units::base_unit_type::ampere_ratio, std::ratio<0>> &&
    std::same_as<typename Units::base_unit_type::kelvin_ratio, std::ratio<0>> &&
    std::same_as<typename Units::base_unit_type::mole_ratio, std::ratio<0>> &&
    std::same_as<typename Units::base_unit_type::candela_ratio,
                 std::ratio<0>> &&
    std::same_as<typename Units::base_unit_type::byte_ratio, std::ratio<0>>;

template <typename Units>
inline constexpr bool is_dimensionless_unit_t_v =
    is_dimensionless_unit_v<typename Units::unit_type>;

//------------------------------
//  UNIT CATEGORIES
//------------------------------

/**
 * @brief namespace representing the implemented base and derived unit types.
 *        These will not generally be needed by library users.
 * @sa  base_unit for the definition of the category parameters.
 */
namespace category {

// SCALAR (DIMENSIONLESS) TYPES

/**
 * Represents a quantity with no dimension.
 */
using scalar_unit = base_unit<>;

/**
 * Represents a quantity with no dimension.
 */
using dimensionless_unit = base_unit<>;

// SI BASE UNIT TYPES:
// METERS, KILOGRAMS, SECONDS, RADIANS, AMPERES, KELVIN, MOLE, CANDELA, BYTE
//--- CATEGORY

/**
 * Represents an SI base unit of length
 */
using length_unit = base_unit<detail::meter_ratio<1>>;

/**
 * Represents an SI base unit of mass
 */
using mass_unit = base_unit<detail::meter_ratio<0>, std::ratio<1>>;

/**
 * Represents an SI base unit of time
 */
using time_unit =
    base_unit<detail::meter_ratio<0>, std::ratio<0>, std::ratio<1>>;

/**
 * Represents an SI base unit of angle
 */
using angle_unit = base_unit<detail::meter_ratio<0>, std::ratio<0>,
                             std::ratio<0>, std::ratio<1>>;

/**
 * Represents an SI base unit of current
 */
using current_unit = base_unit<detail::meter_ratio<0>, std::ratio<0>,
                               std::ratio<0>, std::ratio<0>, std::ratio<1>>;

/**
 * Represents an SI base unit of temperature
 */
using temperature_unit =
    base_unit<detail::meter_ratio<0>, std::ratio<0>, std::ratio<0>,
              std::ratio<0>, std::ratio<0>, std::ratio<1>>;

/**
 * Represents an SI base unit of amount of substance
 */
using substance_unit =
    base_unit<detail::meter_ratio<0>, std::ratio<0>, std::ratio<0>,
              std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>;

/**
 * Represents an SI base unit of luminous intensity
 */
using luminous_intensity_unit =
    base_unit<detail::meter_ratio<0>, std::ratio<0>, std::ratio<0>,
              std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>,
              std::ratio<1>>;

// SI DERIVED UNIT TYPES:
// METERS, KILOGRAMS, SECONDS, RADIANS, AMPERES, KELVIN, MOLE, CANDELA, BYTE
//--- CATEGORY

/**
 * Represents an SI derived unit of solid angle
 */
using solid_angle_unit = base_unit<detail::meter_ratio<0>, std::ratio<0>,
                                   std::ratio<0>, std::ratio<2>, std::ratio<0>,
                                   std::ratio<0>, std::ratio<0>, std::ratio<0>>;

/**
 * Represents an SI derived unit of frequency
 */
using frequency_unit =
    base_unit<detail::meter_ratio<0>, std::ratio<0>, std::ratio<-1>>;

/**
 * Represents an SI derived unit of velocity
 */
using velocity_unit =
    base_unit<detail::meter_ratio<1>, std::ratio<0>, std::ratio<-1>>;

/**
 * Represents an SI derived unit of angular velocity
 */
using angular_velocity_unit = base_unit<detail::meter_ratio<0>, std::ratio<0>,
                                        std::ratio<-1>, std::ratio<1>>;

/**
 * Represents an SI derived unit of acceleration
 */
using acceleration_unit =
    base_unit<detail::meter_ratio<1>, std::ratio<0>, std::ratio<-2>>;

/**
 * Represents an SI derived unit of angular acceleration
 */
using angular_acceleration_unit =
    base_unit<detail::meter_ratio<0>, std::ratio<0>, std::ratio<-2>,
              std::ratio<1>>;

/**
 * Represents an SI derived unit of angular jerk
 */
using angular_jerk_unit = base_unit<detail::meter_ratio<0>, std::ratio<0>,
                                    std::ratio<-3>, std::ratio<1>>;

/**
 * Represents an SI derived unit of force
 */
using force_unit =
    base_unit<detail::meter_ratio<1>, std::ratio<1>, std::ratio<-2>>;

/**
 * Represents an SI derived unit of pressure
 */
using pressure_unit =
    base_unit<detail::meter_ratio<-1>, std::ratio<1>, std::ratio<-2>>;

/**
 * Represents an SI derived unit of charge
 */
using charge_unit = base_unit<detail::meter_ratio<0>, std::ratio<0>,
                              std::ratio<1>, std::ratio<0>, std::ratio<1>>;

/**
 * Represents an SI derived unit of energy
 */
using energy_unit =
    base_unit<detail::meter_ratio<2>, std::ratio<1>, std::ratio<-2>>;

/**
 * Represents an SI derived unit of power
 */
using power_unit =
    base_unit<detail::meter_ratio<2>, std::ratio<1>, std::ratio<-3>>;

/**
 * Represents an SI derived unit of voltage
 */
using voltage_unit = base_unit<detail::meter_ratio<2>, std::ratio<1>,
                               std::ratio<-3>, std::ratio<0>, std::ratio<-1>>;

/**
 * Represents an SI derived unit of capacitance
 */
using capacitance_unit = base_unit<detail::meter_ratio<-2>, std::ratio<-1>,
                                   std::ratio<4>, std::ratio<0>, std::ratio<2>>;

/**
 * Represents an SI derived unit of impedance
 */
using impedance_unit = base_unit<detail::meter_ratio<2>, std::ratio<1>,
                                 std::ratio<-3>, std::ratio<0>, std::ratio<-2>>;

/**
 * Represents an SI derived unit of conductance
 */
using conductance_unit = base_unit<detail::meter_ratio<-2>, std::ratio<-1>,
                                   std::ratio<3>, std::ratio<0>, std::ratio<2>>;

/**
 * Represents an SI derived unit of magnetic flux
 */
using magnetic_flux_unit =
    base_unit<detail::meter_ratio<2>, std::ratio<1>, std::ratio<-2>,
              std::ratio<0>, std::ratio<-1>>;

/**
 * Represents an SI derived unit of magnetic field strength
 */
using magnetic_field_strength_unit =
    base_unit<detail::meter_ratio<0>, std::ratio<1>, std::ratio<-2>,
              std::ratio<0>, std::ratio<-1>>;

/**
 * Represents an SI derived unit of inductance
 */
using inductance_unit =
    base_unit<detail::meter_ratio<2>, std::ratio<1>, std::ratio<-2>,
              std::ratio<0>, std::ratio<-2>>;

/**
 * Represents an SI derived unit of luminous flux
 */
using luminous_flux_unit =
    base_unit<detail::meter_ratio<0>, std::ratio<0>, std::ratio<0>,
              std::ratio<2>, std::ratio<0>, std::ratio<0>, std::ratio<0>,
              std::ratio<1>>;

/**
 * Represents an SI derived unit of illuminance
 */
using illuminance_unit = base_unit<detail::meter_ratio<-2>, std::ratio<0>,
                                   std::ratio<0>, std::ratio<2>, std::ratio<0>,
                                   std::ratio<0>, std::ratio<0>, std::ratio<1>>;

/**
 * Represents an SI derived unit of radioactivity
 */
using radioactivity_unit =
    base_unit<detail::meter_ratio<0>, std::ratio<0>, std::ratio<-1>>;

// OTHER UNIT TYPES:
// METERS, KILOGRAMS, SECONDS, RADIANS, AMPERES, KELVIN, MOLE, CANDELA, BYTE
//--- CATEGORY

/**
 * Represents an SI derived unit of torque
 */
using torque_unit =
    base_unit<detail::meter_ratio<2>, std::ratio<1>, std::ratio<-2>>;

/**
 * Represents an SI derived unit of area
 */
using area_unit = base_unit<detail::meter_ratio<2>>;

/**
 * Represents an SI derived unit of volume
 */
using volume_unit = base_unit<detail::meter_ratio<3>>;

/**
 * Represents an SI derived unit of density
 */
using density_unit = base_unit<detail::meter_ratio<-3>, std::ratio<1>>;

/**
 * Represents a unit of concentration
 */
using concentration_unit = base_unit<>;

/**
 * Represents a unit of data size
 */
using data_unit =
    base_unit<detail::meter_ratio<0>, std::ratio<0>, std::ratio<0>,
              std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>,
              std::ratio<0>, std::ratio<1>>;

/**
 * Represents a unit of data transfer rate
 */
using data_transfer_rate_unit =
    base_unit<detail::meter_ratio<0>, std::ratio<0>, std::ratio<-1>,
              std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>,
              std::ratio<0>, std::ratio<1>>;

}  // namespace category

//------------------------------
//  UNIT CLASSES
//------------------------------

/** @cond */  // DOXYGEN IGNORE
/**
 * @brief unit type template specialization for units derived from base units.
 */
template <class, class, class, class>
struct unit;
template <class Conversion, class... Exponents, class PiExponent,
          class Translation>
struct unit<Conversion, base_unit<Exponents...>, PiExponent, Translation>
    : units::detail::_unit {
  static_assert(is_ratio<Conversion>,
                "Template parameter `Conversion` must be a `std::ratio` "
                "representing the conversion factor to `BaseUnit`.");
  static_assert(is_ratio<PiExponent>,
                "Template parameter `PiExponent` must be a `std::ratio` "
                "representing the exponents of Pi the unit has.");
  static_assert(is_ratio<Translation>,
                "Template parameter `Translation` must be a `std::ratio` "
                "representing an "
                "additive translation required by the unit conversion.");

  using base_unit_type = typename units::base_unit<Exponents...>;
  using conversion_ratio = Conversion;
  using translation_ratio = Translation;
  using pi_exponent_ratio = PiExponent;
};
/** @endcond */  // END DOXYGEN IGNORE

/**
 * @brief Type representing an arbitrary unit.
 * @ingroup UnitTypes
 * @details `unit` types are used as tags for the `conversion` function. They
 *          are *not* containers (see `unit_t` for a container class).
 *
 * Each unit is defined by:
 *
 * - A `std::ratio` defining the conversion factor to the base unit type. (e.g.
 *   `std::ratio<1,12>` for inches to feet)
 * - A base unit that the unit is derived from (or a unit category. Must be of
 *   type `unit` or `base_unit`)
 * - An exponent representing factors of PI required by the conversion. (e.g.
 *   `std::ratio<-1>` for a radians to degrees conversion)
 * - a ratio representing a datum translation required for the conversion (e.g.
 *   `std::ratio<32>` for a fahrenheit to celsius conversion)
 *
 * Typically, a specific unit, like `meters`, would be implemented as a type
 * alias of `unit`, i.e.
 * `using meters = unit<std::ratio<1>, units::category::length_unit`, or
 * `using inches = unit<std::ratio<1,12>, feet>`.
 *
 * @tparam Conversion  std::ratio representing scalar multiplication factor.
 * @tparam BaseUnit Unit type which this unit is derived from. May be a
 *         `base_unit`, or another `unit`.
 * @tparam PiExponent std::ratio representing the exponent of pi required by the
 *         conversion.
 * @tparam Translation std::ratio representing any datum translation required by
 *         the conversion.
 */
template <class Conversion, class BaseUnit, class PiExponent = std::ratio<0>,
          class Translation = std::ratio<0>>
struct unit : units::detail::_unit {
  static_assert(is_unit<BaseUnit>,
                "Template parameter `BaseUnit` must be a `unit` type.");
  static_assert(is_ratio<Conversion>,
                "Template parameter `Conversion` must be a `std::ratio` "
                "representing the conversion factor to `BaseUnit`.");
  static_assert(is_ratio<PiExponent>,
                "Template parameter `PiExponent` must be a `std::ratio` "
                "representing the exponents of Pi the unit has.");

  using base_unit_type = typename BaseUnit::base_unit_type;
  using conversion_ratio =
      typename std::ratio_multiply<typename BaseUnit::conversion_ratio,
                                   Conversion>;
  using pi_exponent_ratio =
      typename std::ratio_add<typename BaseUnit::pi_exponent_ratio, PiExponent>;
  using translation_ratio = typename std::ratio_add<
      std::ratio_multiply<typename BaseUnit::conversion_ratio, Translation>,
      typename BaseUnit::translation_ratio>;
};

//------------------------------
//  BASE UNIT MANIPULATORS
//------------------------------

/** @cond */  // DOXYGEN IGNORE
namespace detail {

/**
 * @brief base_unit_of trait implementation
 * @details recursively seeks base_unit type that a unit is derived from. Since
 *          units can be derived from other units, the `base_unit_type` typedef
 *          may not represent this value.
 */
template <class>
struct base_unit_of_impl;

template <class Conversion, class BaseUnit, class PiExponent, class Translation>
struct base_unit_of_impl<unit<Conversion, BaseUnit, PiExponent, Translation>>
    : base_unit_of_impl<BaseUnit> {};

template <class... Exponents>
struct base_unit_of_impl<base_unit<Exponents...>> {
  using type = base_unit<Exponents...>;
};

template <>
struct base_unit_of_impl<void> {
  using type = void;
};

}  // namespace detail
/** @endcond */  // END DOXYGEN IGNORE

namespace traits {

/**
 * @brief Trait which returns the `base_unit` type that a unit is originally
 *        derived from.
 * @details Since units can be derived from other `unit` types in addition to
 *          `base_unit` types, the `base_unit_type` typedef will not always be a
 *          `base_unit` (or unit category). Since compatible
 */
template <class U>
using base_unit_of = typename units::detail::base_unit_of_impl<U>::type;

}  // namespace traits

/** @cond */  // DOXYGEN IGNORE
namespace detail {

/**
 * @brief implementation of base_unit_multiply
 * @details 'multiples' (adds exponent ratios of) two base unit types. Base
 *          units can be found using `base_unit_of`.
 */
template <class, class>
struct base_unit_multiply_impl;

template <class... Exponents1, class... Exponents2>
struct base_unit_multiply_impl<base_unit<Exponents1...>,
                               base_unit<Exponents2...>> {
  using type = base_unit<std::ratio_add<Exponents1, Exponents2>...>;
};

/**
 * @brief represents type of two base units multiplied together
 */
template <class U1, class U2>
using base_unit_multiply = typename base_unit_multiply_impl<U1, U2>::type;

/**
 * @brief implementation of base_unit_divide
 * @details 'dived' (subtracts exponent ratios of) two base unit types. Base
 *          units can be found using `base_unit_of`.
 */
template <class, class>
struct base_unit_divide_impl;

template <class... Exponents1, class... Exponents2>
struct base_unit_divide_impl<base_unit<Exponents1...>,
                             base_unit<Exponents2...>> {
  using type = base_unit<std::ratio_subtract<Exponents1, Exponents2>...>;
};

/**
 * @brief represents the resulting type of `base_unit` U1 divided by U2.
 */
template <class U1, class U2>
using base_unit_divide = typename base_unit_divide_impl<U1, U2>::type;

/**
 * @brief implementation of inverse_base
 * @details multiplies all `base_unit` exponent ratios by -1. The resulting type
 *          represents the inverse base unit of the given `base_unit` type.
 */
template <class>
struct inverse_base_impl;

template <class... Exponents>
struct inverse_base_impl<base_unit<Exponents...>> {
  using type = base_unit<std::ratio_multiply<Exponents, std::ratio<-1>>...>;
};

/**
 * @brief represent the inverse type of `class U`
 * @details E.g. if `U` is `length_unit`, then `inverse<U>` will represent
 *          `length_unit⁻¹`.
 */
template <class U>
using inverse_base = typename inverse_base_impl<U>::type;

/**
 * @brief implementation of `squared_base`
 * @details multiplies all the exponent ratios of the given class by 2. The
 *          resulting type is equivalent to the given type squared.
 */
template <class U>
struct squared_base_impl;

template <class... Exponents>
struct squared_base_impl<base_unit<Exponents...>> {
  using type = base_unit<std::ratio_multiply<Exponents, std::ratio<2>>...>;
};

/**
 * @brief represents the type of a `base_unit` squared.
 * @details E.g. `squared<length_unit>` will represent `length_unit²`.
 */
template <class U>
using squared_base = typename squared_base_impl<U>::type;

/**
 * @brief implementation of `cubed_base`
 * @details multiplies all the exponent ratios of the given class by 3. The
 *          resulting type is equivalent to the given type cubed.
 */
template <class U>
struct cubed_base_impl;

template <class... Exponents>
struct cubed_base_impl<base_unit<Exponents...>> {
  using type = base_unit<std::ratio_multiply<Exponents, std::ratio<3>>...>;
};

/**
 * @brief represents the type of a `base_unit` cubed.
 * @details E.g. `cubed<length_unit>` will represent `length_unit³`.
 */
template <class U>
using cubed_base = typename cubed_base_impl<U>::type;

/**
 * @brief implementation of `sqrt_base`
 * @details divides all the exponent ratios of the given class by 2. The
 *          resulting type is equivalent to the square root of the given type.
 */
template <class U>
struct sqrt_base_impl;

template <class... Exponents>
struct sqrt_base_impl<base_unit<Exponents...>> {
  using type = base_unit<std::ratio_divide<Exponents, std::ratio<2>>...>;
};

/**
 * @brief represents the square-root type of a `base_unit`.
 * @details E.g. `sqrt<length_unit>` will represent `length_unit^(1/2)`.
 */
template <class U>
using sqrt_base = typename sqrt_base_impl<U>::type;

/**
 * @brief    implementation of `cbrt_base`
 * @details    divides all the exponent ratios of the given class by 3.
 *The resulting type is equivalent to the given type's cube-root.
 */
template <class U>
struct cbrt_base_impl;

template <class... Exponents>
struct cbrt_base_impl<base_unit<Exponents...>> {
  using type = base_unit<std::ratio_divide<Exponents, std::ratio<3>>...>;
};

/**
 * @brief represents the cube-root type of a `base_unit` .
 * @details E.g. `cbrt<length_unit>` will represent `length_unit^(1/3)`.
 */
template <class U>
using cbrt_base = typename cbrt_base_impl<U>::type;

}  // namespace detail
/** @endcond */  // END DOXYGEN IGNORE

//------------------------------
//  UNIT MANIPULATORS
//------------------------------

/** @cond */  // DOXYGEN IGNORE
namespace detail {

/**
 * @brief implementation of `unit_multiply`.
 * @details multiplies two units. The base unit becomes the base units of each
 *          with their exponents added together. The conversion factors of each
 *          are multiplied by each other. Pi exponent ratios are added, and
 *          datum translations are removed.
 */
template <class Unit1, class Unit2>
struct unit_multiply_impl {
  using type = unit<
      std::ratio_multiply<typename Unit1::conversion_ratio,
                          typename Unit2::conversion_ratio>,
      base_unit_multiply<traits::base_unit_of<typename Unit1::base_unit_type>,
                         traits::base_unit_of<typename Unit2::base_unit_type>>,
      std::ratio_add<typename Unit1::pi_exponent_ratio,
                     typename Unit2::pi_exponent_ratio>,
      std::ratio<0>>;
};

/**
 * @brief represents the type of two units multiplied together.
 * @details recalculates conversion and exponent ratios at compile-time.
 */
template <class U1, class U2>
using unit_multiply = typename unit_multiply_impl<U1, U2>::type;

/**
 * @brief implementation of `unit_divide`.
 * @details divides two units. The base unit becomes the base units of each with
 *          their exponents subtracted from each other. The conversion factors
 *          of each are divided by each other. Pi exponent ratios are
 *          subtracted, and datum translations are removed.
 */
template <class Unit1, class Unit2>
struct unit_divide_impl {
  using type = unit<
      std::ratio_divide<typename Unit1::conversion_ratio,
                        typename Unit2::conversion_ratio>,
      base_unit_divide<traits::base_unit_of<typename Unit1::base_unit_type>,
                       traits::base_unit_of<typename Unit2::base_unit_type>>,
      std::ratio_subtract<typename Unit1::pi_exponent_ratio,
                          typename Unit2::pi_exponent_ratio>,
      std::ratio<0>>;
};

/**
 * @brief represents the type of two units divided by each other.
 * @details recalculates conversion and exponent ratios at compile-time.
 */
template <class U1, class U2>
using unit_divide = typename unit_divide_impl<U1, U2>::type;

/**
 * @brief implementation of `inverse`
 * @details inverts a unit (equivalent to 1/unit). The `base_unit` and pi
 *          exponents are all multiplied by -1. The conversion ratio numerator
 *          and denominator are swapped. Datum translation ratios are removed.
 */
template <class Unit>
struct inverse_impl {
  using type = unit<
      std::ratio<Unit::conversion_ratio::den, Unit::conversion_ratio::num>,
      inverse_base<traits::base_unit_of<typename Unit::base_unit_type>>,
      std::ratio_multiply<typename Unit::pi_exponent_ratio, std::ratio<-1>>,
      // inverses are rates or change, the translation factor goes away.
      std::ratio<0>>;
};

}  // namespace detail
/** @endcond */  // END DOXYGEN IGNORE

/**
 * @brief represents the inverse unit type of `class U`.
 * @ingroup UnitManipulators
 * @tparam U  `unit` type to invert.
 * @details E.g. `inverse<meters>` will represent meters^-1 (i.e. 1/meters).
 */
template <class U>
using inverse = typename units::detail::inverse_impl<U>::type;

/** @cond */  // DOXYGEN IGNORE
namespace detail {

/**
 * @brief implementation of `squared`
 * @details Squares the conversion ratio, `base_unit` exponents, pi exponents,
 *          and removes datum translation ratios.
 */
template <class Unit>
struct squared_impl {
  static_assert(is_unit<Unit>,
                "Template parameter `Unit` must be a `unit` type.");
  using Conversion = typename Unit::conversion_ratio;
  using type =
      unit<std::ratio_multiply<Conversion, Conversion>,
           squared_base<traits::base_unit_of<typename Unit::base_unit_type>>,
           std::ratio_multiply<typename Unit::pi_exponent_ratio, std::ratio<2>>,
           typename Unit::translation_ratio>;
};

}  // namespace detail
/** @endcond */  // END DOXYGEN IGNORE

/**
 * @brief represents the unit type of `class U` squared
 * @ingroup UnitManipulators
 * @tparam U `unit` type to square.
 * @details E.g. `square<meters>` will represent meters^2.
 */
template <class U>
using squared = typename units::detail::squared_impl<U>::type;

/** @cond */  // DOXYGEN IGNORE
namespace detail {

/**
 * @brief implementation of `cubed`
 * @details Cubes the conversion ratio, `base_unit` exponents, pi exponents, and
 *          removes datum translation ratios.
 */
template <class Unit>
struct cubed_impl {
  static_assert(is_unit<Unit>,
                "Template parameter `Unit` must be a `unit` type.");
  using Conversion = typename Unit::conversion_ratio;
  using type =
      unit<std::ratio_multiply<Conversion,
                               std::ratio_multiply<Conversion, Conversion>>,
           cubed_base<traits::base_unit_of<typename Unit::base_unit_type>>,
           std::ratio_multiply<typename Unit::pi_exponent_ratio, std::ratio<3>>,
           typename Unit::translation_ratio>;
};

}  // namespace detail
/** @endcond */  // END DOXYGEN IGNORE

/**
 * @brief represents the type of `class U` cubed.
 * @ingroup UnitManipulators
 * @tparam U `unit` type to cube.
 * @details E.g. `cubed<meters>` will represent meters^3.
 */
template <class U>
using cubed = typename units::detail::cubed_impl<U>::type;

/** @cond */  // DOXYGEN IGNORE
namespace detail {

//----------------------------------
//  RATIO_SQRT IMPLEMENTATION
//----------------------------------

using Zero = std::ratio<0>;
using One = std::ratio<1>;
template <typename R>
using Square = std::ratio_multiply<R, R>;

// Find the largest std::integer N such that Predicate<N>::value is true.
template <template <intmax_t N> class Predicate, typename enabled = void>
struct BinarySearch {
  template <intmax_t N>
  struct SafeDouble_ {
    static constexpr const intmax_t value = 2 * N;
    static_assert(value > 0, "Overflows when computing 2 * N");
  };

  template <intmax_t Lower, intmax_t Upper, typename Condition1 = void,
            typename Condition2 = void>
  struct DoubleSidedSearch_
      : DoubleSidedSearch_<
            Lower, Upper, std::integral_constant<bool, (Upper - Lower == 1)>,
            std::integral_constant<
                bool, ((Upper - Lower > 1 &&
                        Predicate<Lower + (Upper - Lower) / 2>::value))>> {};

  template <intmax_t Lower, intmax_t Upper>
  struct DoubleSidedSearch_<Lower, Upper, std::false_type, std::false_type>
      : DoubleSidedSearch_<Lower, Lower + (Upper - Lower) / 2> {};

  template <intmax_t Lower, intmax_t Upper, typename Condition2>
  struct DoubleSidedSearch_<Lower, Upper, std::true_type, Condition2>
      : std::integral_constant<intmax_t, Lower> {};

  template <intmax_t Lower, intmax_t Upper, typename Condition1>
  struct DoubleSidedSearch_<Lower, Upper, Condition1, std::true_type>
      : DoubleSidedSearch_<Lower + (Upper - Lower) / 2, Upper> {};

  template <intmax_t Lower, class enabled1 = void>
  struct SingleSidedSearch_
      : SingleSidedSearch_<
            Lower, std::integral_constant<
                       bool, Predicate<SafeDouble_<Lower>::value>::value>> {};

  template <intmax_t Lower>
  struct SingleSidedSearch_<Lower, std::false_type>
      : DoubleSidedSearch_<Lower, SafeDouble_<Lower>::value> {};

  template <intmax_t Lower>
  struct SingleSidedSearch_<Lower, std::true_type>
      : SingleSidedSearch_<SafeDouble_<Lower>::value> {};

  static constexpr const intmax_t value = SingleSidedSearch_<1>::value;
};

template <template <intmax_t N> class Predicate>
  requires(!Predicate<1>::value)
struct BinarySearch<Predicate> : std::integral_constant<intmax_t, 0> {};

// Find largest integer N such that N ≤ √(R)
template <typename R>
struct Integer {
  template <intmax_t N>
  using Predicate_ =
      std::ratio_less_equal<std::ratio<N>, std::ratio_divide<R, std::ratio<N>>>;
  static constexpr const intmax_t value = BinarySearch<Predicate_>::value;
};

template <typename R>
struct IsPerfectSquare {
  static constexpr const intmax_t DenSqrt_ = Integer<std::ratio<R::den>>::value;
  static constexpr const intmax_t NumSqrt_ = Integer<std::ratio<R::num>>::value;
  static constexpr const bool value =
      (DenSqrt_ * DenSqrt_ == R::den && NumSqrt_ * NumSqrt_ == R::num);
  using Sqrt = std::ratio<NumSqrt_, DenSqrt_>;
};

// Represents √(P) - Q.
template <typename Tp, typename Tq>
struct Remainder {
  using P = Tp;
  using Q = Tq;
};

// Represents 1/R = I + Rem where R is a Remainder.
template <typename R>
struct Reciprocal {
  using P_ = typename R::P;
  using Q_ = typename R::Q;
  using Den_ = std::ratio_subtract<P_, Square<Q_>>;
  using A_ = std::ratio_divide<Q_, Den_>;
  using B_ = std::ratio_divide<P_, Square<Den_>>;
  static constexpr const intmax_t I_ =
      (A_::num +
       Integer<std::ratio_multiply<B_, Square<std::ratio<A_::den>>>>::value) /
      A_::den;
  using I = std::ratio<I_>;
  using Rem = Remainder<B_, std::ratio_subtract<I, A_>>;
};

// Expands sqrt(R) to continued fraction:
//
//   f(x) = C1 + 1/(C2 + 1/(C3 + 1/(... + 1/(Cn + x))))
//        = (Ux + V)/(Wx + 1)
//
//   √(R) = f(Rem)
//
// The error is
//
//   |f(Rem) - V| = |(U - WV)x/(Wx + 1)| ≤ |U - WV| Rem ≤ |U - WV|/I'
//
// where I' is the integer part of reciprocal of Rem.
template <typename Tr, intmax_t N>
struct ContinuedFraction {
  template <typename T>
  using Abs_ = std::conditional_t<std::ratio_less<T, Zero>::value,
                                  std::ratio_subtract<Zero, T>, T>;

  using R = Tr;
  using Last_ = ContinuedFraction<R, N - 1>;
  using Reciprocal_ = Reciprocal<typename Last_::Rem>;
  using Rem = typename Reciprocal_::Rem;
  using I_ = typename Reciprocal_::I;
  using Den_ = std::ratio_add<typename Last_::W, I_>;
  using U = std::ratio_divide<typename Last_::V, Den_>;
  using V = std::ratio_divide<
      std::ratio_add<typename Last_::U,
                     std::ratio_multiply<typename Last_::V, I_>>,
      Den_>;
  using W = std::ratio_divide<One, Den_>;
  using Error =
      Abs_<std::ratio_divide<std::ratio_subtract<U, std::ratio_multiply<V, W>>,
                             typename Reciprocal<Rem>::I>>;
};

template <typename Tr>
struct ContinuedFraction<Tr, 1> {
  using R = Tr;
  using U = One;
  using V = std::ratio<Integer<R>::value>;
  using W = Zero;
  using Rem = Remainder<R, V>;
  using Error = std::ratio_divide<One, typename Reciprocal<Rem>::I>;
};

template <typename R, typename Eps, intmax_t N = 1, typename enabled = void>
struct Sqrt_ : Sqrt_<R, Eps, N + 1> {};

template <typename R, typename Eps, intmax_t N>
  requires std::ratio_less_equal_v<typename ContinuedFraction<R, N>::Error, Eps>
struct Sqrt_<R, Eps, N> {
  using type = typename ContinuedFraction<R, N>::V;
};

template <typename R, typename Eps, typename enabled = void>
struct Sqrt {
  static_assert(std::ratio_greater_equal<R, Zero>::value,
                "R can't be negative");
};

template <typename R, typename Eps>
  requires std::ratio_greater_equal_v<R, Zero> && IsPerfectSquare<R>::value
struct Sqrt<R, Eps> {
  using type = typename IsPerfectSquare<R>::Sqrt;
};

template <typename R, typename Eps>
  requires std::ratio_greater_equal<R, Zero>::value &&
           (!IsPerfectSquare<R>::value)
struct Sqrt<R, Eps> : Sqrt_<R, Eps> {};

}  // namespace detail
/** @endcond */  // END DOXYGEN IGNORE

/**
 * @ingroup TypeTraits
 * @brief Calculate square root of a ratio at compile-time
 * @details Calculates a rational approximation of the square root of the ratio.
 *          The error in the calculation is bounded by 1/epsilon (Eps). E.g. for
 *          the default value of 10000000000, the maximum error will be
 *          a/10000000000, or 1e-8, or said another way, the error will be on
 *          the order of 10⁻⁹. Since these calculations are done at compile
 *          time, it is advisable to set epsilon to the highest value that does
 *          not cause an integer overflow in the calculation. If you can't
 *          compile `ratio_sqrt` due to overflow errors, reducing the value of
 *          epsilon sufficiently will correct the problem.
 *
 *          `ratio_sqrt` is guaranteed to converge for all values of `Ratio`
 *          which do not overflow.
 * @note This function provides a rational approximation, _NOT_ an exact value.
 * @tparam Ratio ratio to take the square root of. This can represent any
 *         rational value, _not_ just integers or values with integer roots.
 * @tparam Eps Value of epsilon, which represents the inverse of the maximum
 *         allowable error. This value should be chosen to be as high as
 *         possible before integer overflow errors occur in the compiler.
 */
template <typename Ratio, intmax_t Eps = 10000000000>
using ratio_sqrt =
    typename units::detail::Sqrt<Ratio, std::ratio<1, Eps>>::type;

/** @cond */  // DOXYGEN IGNORE
namespace detail {

/**
 * @brief implementation of `sqrt`
 * @details square roots the conversion ratio, `base_unit` exponents, pi
 *          exponents, and removes datum translation ratios.
 */
template <class Unit, intmax_t Eps>
struct sqrt_impl {
  static_assert(is_unit<Unit>,
                "Template parameter `Unit` must be a `unit` type.");
  using Conversion = typename Unit::conversion_ratio;
  using type =
      unit<ratio_sqrt<Conversion, Eps>,
           sqrt_base<traits::base_unit_of<typename Unit::base_unit_type>>,
           std::ratio_divide<typename Unit::pi_exponent_ratio, std::ratio<2>>,
           typename Unit::translation_ratio>;
};

}  // namespace detail
/** @endcond */  // END DOXYGEN IGNORE

/**
 * @ingroup UnitManipulators
 * @brief represents the square root of type `class U`.
 * @details Calculates a rational approximation of the square root of the unit.
 *          The error in the calculation is bounded by 1/epsilon (Eps). E.g. for
 *          the default value of 10000000000, the maximum error will be
 *          a/10000000000, or 1e-8, or said another way, the error will be on
 *          the order of 10⁻⁹. Since these calculations are done at compile
 *          time, it is advisable to set epsilon to the highest value that does
 *          not cause an integer overflow in the calculation. If you can't
 *          compile `ratio_sqrt` due to overflow errors, reducing the value of
 *          epsilon sufficiently will correct the problem.
 *
 *          `ratio_sqrt` is guaranteed to converge for all values of `Ratio`
 *          which do not overflow.
 * @tparam U `unit` type to take the square root of.
 * @tparam Eps Value of epsilon, which represents the inverse of the maximum
 *         allowable error. This value should be chosen to be as high as
 *         possible before integer overflow errors occur in the compiler.
 * @note USE WITH CAUTION. The is an approximate value. In general,
 *       squared<sqrt<meter>> != meter, i.e. the operation is not reversible,
 *       and it will result in propagated approximations. Use only when
 *       absolutely necessary.
 */
template <class U, intmax_t Eps = 10000000000>
using square_root = typename units::detail::sqrt_impl<U, Eps>::type;

//------------------------------
//  COMPOUND UNITS
//------------------------------

/** @cond */  // DOXYGEN IGNORE
namespace detail {

/**
 * @brief implementation of compound_unit
 * @details multiplies a variadic list of units together, and is inherited from
 *          the resulting type.
 */
template <class U, class... Us>
struct compound_impl;

template <class U>
struct compound_impl<U> {
  using type = U;
};

template <class U1, class U2, class... Us>
struct compound_impl<U1, U2, Us...>
    : compound_impl<unit_multiply<U1, U2>, Us...> {};

}  // namespace detail
/** @endcond */  // END DOXYGEN IGNORE

/**
 * @brief Represents a unit type made up from other units.
 * @details Compound units are formed by multiplying the units of all the types
 *          provided in the template argument. Types provided must inherit from
 *          `unit`. A compound unit can be formed from any number of other
 *          units, and unit manipulators like `inverse` and `squared` are
 *          supported. E.g. to specify acceleration, on could create `using
 *          acceleration = compound_unit<length::meters,
 *          inverse<squared<seconds>>;`
 * @tparam U... units which, when multiplied together, form the desired compound
 *         unit.
 * @ingroup UnitTypes
 */
template <class U, class... Us>
using compound_unit = typename units::detail::compound_impl<U, Us...>::type;

//------------------------------
//  PREFIXES
//------------------------------

/** @cond */  // DOXYGEN IGNORE
namespace detail {

/**
 * @brief prefix applicator.
 * @details creates a unit type from a prefix and a unit
 */
template <class Ratio, class Unit>
struct prefix {
  static_assert(is_ratio<Ratio>,
                "Template parameter `Ratio` must be a `std::ratio`.");
  static_assert(is_unit<Unit>,
                "Template parameter `Unit` must be a `unit` type.");
  using type = typename units::unit<Ratio, Unit>;
};

template <class Ratio, class Unit>
using prefix_t = typename prefix<Ratio, Unit>::type;

/// Recursive exponential implementation
template <int N, class U>
struct power_of_ratio {
  using type = std::ratio_multiply<U, typename power_of_ratio<N - 1, U>::type>;
};

/// End recursion
template <class U>
struct power_of_ratio<1, U> {
  using type = U;
};

}  // namespace detail
/** @endcond */  // END DOXYGEN IGNORE

/**
 * @ingroup UnitManipulators
 * @{
 * @ingroup Decimal Prefixes
 * @{
 */

/**
 * Represents the type of `class U` with the metric 'atto' prefix appended.
 *
 * @details E.g. atto<meters> represents meters×10⁻¹⁸
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using atto = units::detail::prefix_t<std::atto, U>;

/**
 * Represents the type of `class U` with the metric 'femto' prefix appended.
 *
 * @details E.g. femto<meters> represents meters×10⁻¹⁵
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using femto = units::detail::prefix_t<std::femto, U>;

/**
 * Represents the type of `class U` with the metric 'pico' prefix appended.
 *
 * @details E.g. pico<meters> represents meters×10⁻¹²
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using pico = units::detail::prefix_t<std::pico, U>;

/**
 * Represents the type of `class U` with the metric 'nano' prefix appended.
 *
 * @details E.g. nano<meters> represents meters×10⁻⁹
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using nano = units::detail::prefix_t<std::nano, U>;

/**
 * Represents the type of `class U` with the metric 'micro' prefix appended.
 *
 * @details E.g. micro<meters> represents meters×10⁻⁶
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using micro = units::detail::prefix_t<std::micro, U>;

/**
 * Represents the type of `class U` with the metric 'milli' prefix appended.
 *
 * @details E.g. milli<meters> represents meters×10⁻³
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using milli = units::detail::prefix_t<std::milli, U>;

/**
 * Represents the type of `class U` with the metric 'centi' prefix appended.
 *
 * @details E.g. centi<meters> represents meters×10⁻²
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using centi = units::detail::prefix_t<std::centi, U>;

/**
 * Represents the type of `class U` with the metric 'deci' prefix appended.
 *
 * @details E.g. deci<meters> represents meters×10⁻¹
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using deci = units::detail::prefix_t<std::deci, U>;

/**
 * Represents the type of `class U` with the metric 'deca' prefix appended.
 *
 * @details E.g. deca<meters> represents meters×10¹
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using deca = units::detail::prefix_t<std::deca, U>;

/**
 * Represents the type of `class U` with the metric 'hecto' prefix appended.
 *
 * @details E.g. hecto<meters> represents meters×10²
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using hecto = units::detail::prefix_t<std::hecto, U>;

/**
 * Represents the type of `class U` with the metric 'kilo' prefix appended.
 *
 * @details E.g. kilo<meters> represents meters×10³
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using kilo = units::detail::prefix_t<std::kilo, U>;

/**
 * Represents the type of `class U` with the metric 'mega' prefix appended.
 *
 * @details E.g. mega<meters> represents meters×10⁶
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using mega = units::detail::prefix_t<std::mega, U>;

/**
 * Represents the type of `class U` with the metric 'giga' prefix appended.
 *
 * @details E.g. giga<meters> represents meters×10⁹
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using giga = units::detail::prefix_t<std::giga, U>;

/**
 * Represents the type of `class U` with the metric 'tera' prefix appended.
 *
 * @details E.g. tera<meters> represents meters×10¹²
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using tera = units::detail::prefix_t<std::tera, U>;

/**
 * Represents the type of `class U` with the metric 'peta' prefix appended.
 *
 * @details E.g. peta<meters> represents meters×10¹⁵
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using peta = units::detail::prefix_t<std::peta, U>;

/**
 * Represents the type of `class U` with the metric 'exa' prefix appended.
 *
 * @details E.g. exa<meters> represents meters×10¹⁸
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using exa = units::detail::prefix_t<std::exa, U>;
/** @} @} */

/**
 * @ingroup UnitManipulators
 * @{
 * @ingroup Binary Prefixes
 * @{
 */

/**
 * Represents the type of `class U` with the binary 'kibi' prefix appended.
 *
 * @details E.g. kibi<bytes> represents bytes×2¹⁰
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using kibi = units::detail::prefix_t<std::ratio<1024>, U>;

/**
 * Represents the type of `class U` with the binary 'mibi' prefix appended.
 *
 * @details E.g. mebi<bytes> represents bytes×2²⁰
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using mebi = units::detail::prefix_t<std::ratio<1048576>, U>;

/**
 * Represents the type of `class U` with the binary 'gibi' prefix appended.
 *
 * @details E.g. gibi<bytes> represents bytes×2³⁰
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using gibi = units::detail::prefix_t<std::ratio<1073741824>, U>;

/**
 * Represents the type of `class U` with the binary 'tebi' prefix appended.
 *
 * @details E.g. tebi<bytes> represents bytes×2⁴⁰
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using tebi = units::detail::prefix_t<std::ratio<1099511627776>, U>;

/**
 * Represents the type of `class U` with the binary 'pebi' prefix appended.
 *
 * @details E.g. pebi<bytes> represents bytes×2⁵⁰
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using pebi = units::detail::prefix_t<std::ratio<1125899906842624>, U>;

/**
 * Represents the type of `class U` with the binary 'exbi' prefix appended.
 *
 * @details E.g. exbi<bytes> represents bytes×2⁶⁰
 * @tparam U unit type to apply the prefix to.
 */
template <class U>
using exbi = units::detail::prefix_t<std::ratio<1152921504606846976>, U>;
/** @} @} */

//------------------------------
//  CONVERSION TRAITS
//------------------------------

/**
 * @ingroup TypeTraits
 * @brief Trait which checks whether two units can be converted to each other
 * @details Inherits from `std::true_type` or `std::false_type`. Use
 *          `is_convertible_unit_v<U1, U2>` to test whether `class U1` is
 *          convertible to `class U2`. Note: convertible has both the semantic
 *          meaning, (i.e. meters can be converted to feet), and the C++ meaning
 *          of conversion (type meters can be converted to type feet).
 *          Conversion is always symmetric, so if U1 is convertible to U2, then
 *          U2 will be convertible to U1.
 * @tparam U1 Unit to convert from.
 * @tparam U2 Unit to convert to.
 * @sa is_convertible_unit_t
 */
template <class U1, class U2>
concept is_convertible_unit = requires {
  requires std::same_as<traits::base_unit_of<typename U1::base_unit_type>,
                        traits::base_unit_of<typename U2::base_unit_type>>;
};

//------------------------------
//  CONVERSION FUNCTION
//------------------------------

/** @cond */  // DOXYGEN IGNORE
namespace detail {

constexpr inline double pow(double x, uint64_t y) {
  return y == 0 ? 1.0 : x * std::pow(x, y - 1);
}

constexpr inline double abs(double x) {
  return x < 0 ? -x : x;
}

/// Convert dispatch for units which are both the same
template <class UnitFrom, class UnitTo, class Ratio, class PiRatio,
          class Translation, typename T>
static inline constexpr T convert(const T& value, std::true_type,
                                  std::false_type, std::false_type) noexcept {
  return value;
}

/// Convert dispatch for units which are both the same
template <class UnitFrom, class UnitTo, class Ratio, class PiRatio,
          class Translation, typename T>
static inline constexpr T convert(const T& value, std::true_type,
                                  std::false_type, std::true_type) noexcept {
  return value;
}

/// Convert dispatch for units which are both the same
template <class UnitFrom, class UnitTo, class Ratio, class PiRatio,
          class Translation, typename T>
static inline constexpr T convert(const T& value, std::true_type,
                                  std::true_type, std::false_type) noexcept {
  return value;
}

/// Convert dispatch for units which are both the same
template <class UnitFrom, class UnitTo, class Ratio, class PiRatio,
          class Translation, typename T>
static inline constexpr T convert(const T& value, std::true_type,
                                  std::true_type, std::true_type) noexcept {
  return value;
}

/// Convert dispatch for units of different types w/ no translation and no PI
template <class UnitFrom, class UnitTo, class Ratio, class PiRatio,
          class Translation, typename T>
static inline constexpr T convert(const T& value, std::false_type,
                                  std::false_type, std::false_type) noexcept {
  return ((value * Ratio::num) / Ratio::den);
}

/// Convert dispatch for units of different types w/ no translation, but has
/// PI in numerator
// constepxr with PI in numerator
template <class UnitFrom, class UnitTo, class Ratio, class PiRatio,
          class Translation, typename T>
  requires(PiRatio::num / PiRatio::den >= 1 && PiRatio::num % PiRatio::den == 0)
static inline constexpr T convert(const T& value, std::false_type,
                                  std::true_type, std::false_type) noexcept {
  return ((value * std::pow(std::numbers::pi, PiRatio::num / PiRatio::den) *
           Ratio::num) /
          Ratio::den);
}

/// Convert dispatch for units of different types w/ no translation, but has
/// PI in denominator
// constexpr with PI in denominator
template <class UnitFrom, class UnitTo, class Ratio, class PiRatio,
          class Translation, typename T>
  requires(PiRatio::num / PiRatio::den <= -1 &&
           PiRatio::num % PiRatio::den == 0)
static inline constexpr T convert(const T& value, std::false_type,
                                  std::true_type, std::false_type) noexcept {
  return (value * Ratio::num) /
         (Ratio::den *
          std::pow(std::numbers::pi, -PiRatio::num / PiRatio::den));
}

/// Convert dispatch for units of different types w/ no translation, but has
/// PI in numerator
// Not constexpr - uses std::pow
template <class UnitFrom, class UnitTo, class Ratio, class PiRatio,
          class Translation, typename T>
  requires((PiRatio::num / PiRatio::den) < 1 &&
           (PiRatio::num / PiRatio::den) > -1)
static inline T convert(const T& value, std::false_type, std::true_type,
                        std::false_type) noexcept {
  return ((value * std::pow(std::numbers::pi, PiRatio::num / PiRatio::den) *
           Ratio::num) /
          Ratio::den);
}

/// Convert dispatch for units of different types with a translation, but no
/// PI
template <class UnitFrom, class UnitTo, class Ratio, class PiRatio,
          class Translation, typename T>
static inline constexpr T convert(const T& value, std::false_type,
                                  std::false_type, std::true_type) noexcept {
  return ((value * Ratio::num) / Ratio::den) +
         (static_cast<double>(Translation::num) / Translation::den);
}

/// Convert dispatch for units of different types with a translation AND PI
template <class UnitFrom, class UnitTo, class Ratio, class PiRatio,
          class Translation, typename T>
static inline constexpr T convert(const T& value, const std::false_type,
                                  const std::true_type,
                                  const std::true_type) noexcept {
  return ((value * std::pow(std::numbers::pi, PiRatio::num / PiRatio::den) *
           Ratio::num) /
          Ratio::den) +
         (static_cast<double>(Translation::num) / Translation::den);
}

}  // namespace detail
/** @endcond */  // END DOXYGEN IGNORE

/**
 * @ingroup Conversion
 * @brief converts a <i>value</i> from one type to another.
 * @details Converts a <i>value</i> of a built-in arithmetic type to another
 *          unit. This does not change the type of <i>value</i>, only what it
 *          contains. E.g. @code double result = convert<length::meters,
 *          length::feet>(1.0);  // result == 3.28084 @endcode
 * @sa unit_t for implicit conversion of unit containers.
 * @tparam UnitFrom unit tag to convert <i>value</i> from. Must be a `unit` type
 *         (i.e. is_unit<UnitFrom> == true), and must be convertible to
 *         `UnitTo` (i.e. is_convertible_unit_v<UnitFrom, UnitTo> == true).
 * @tparam UnitTo unit tag to convert <i>value</i> to. Must be a `unit` type
 *         (i.e. is_unit<UnitTo> == true), and must be convertible from
 *         `UnitFrom` (i.e. is_convertible_unit_v<UnitFrom, UnitTo> == true).
 * @tparam T type of <i>value</i>. It is inferred from <i>value</i>, and is
 *         expected to be a built-in arithmetic type.
 * @param[in] value Arithmetic value to convert from `UnitFrom` to `UnitTo`. The
 *            value should represent a quantity in units of `UnitFrom`.
 * @returns value, converted from units of `UnitFrom` to `UnitTo`.
 */
template <class UnitFrom, class UnitTo, typename T = double>
static inline constexpr T convert(const T& value) noexcept {
  static_assert(is_unit<UnitFrom>,
                "Template parameter `UnitFrom` must be a `unit` type.");
  static_assert(is_unit<UnitTo>,
                "Template parameter `UnitTo` must be a `unit` type.");
  static_assert(is_convertible_unit<UnitFrom, UnitTo>,
                "Units are not compatible.");

  using Ratio = std::ratio_divide<typename UnitFrom::conversion_ratio,
                                  typename UnitTo::conversion_ratio>;
  using PiRatio = std::ratio_subtract<typename UnitFrom::pi_exponent_ratio,
                                      typename UnitTo::pi_exponent_ratio>;
  using Translation = std::ratio_divide<
      std::ratio_subtract<typename UnitFrom::translation_ratio,
                          typename UnitTo::translation_ratio>,
      typename UnitTo::conversion_ratio>;

  using isSame =
      typename std::is_same<std::decay_t<UnitFrom>, std::decay_t<UnitTo>>::type;
  using piRequired =
      std::integral_constant<bool, !(std::is_same_v<std::ratio<0>, PiRatio>)>;
  using translationRequired =
      std::integral_constant<bool,
                             !(std::is_same_v<std::ratio<0>, Translation>)>;

  return units::detail::convert<UnitFrom, UnitTo, Ratio, PiRatio, Translation,
                                T>(value, isSame{}, piRequired{},
                                   translationRequired{});
}

//----------------------------------
//  SCALE TRAITS
//----------------------------------

/**
 * @ingroup TypeTraits
 * @brief Trait which tests that `class T` meets the requirements for a scale
 * @details A nonlinear scale must:
 *
 * - be default constructible
 * - have an `operator()` member which returns the nonlinear value stored in the
 *   scale
 * - have an accessible `m_value` member type which stores the linearized value
 *   in the scale.
 *
 * Linear/nonlinear scales are used by `units::unit` to store values and scale
 * them if they represent things like dB.
 */
template <class T, class Ret>
concept Scale = requires(T t) {
  requires std::is_default_constructible_v<T> && std::is_trivial_v<T>;
  // invocable with required return type
  { t() } -> std::same_as<Ret>;
  // has m_value member
  t.m_value;
};

//------------------------------
//  UNIT_T TYPE TRAITS
//------------------------------

/**
 * @ingroup TypeTraits
 * @brief Trait which tests whether two container types derived from `unit_t`
 *        are convertible to each other
 * @details Inherits from `std::true_type` or `std::false_type`. Use
 *          `is_convertible_unit_t_v<U1, U2>` to test whether `class U1` is
 *          convertible to `class U2`. Note: convertible has both the semantic
 *          meaning, (i.e. meters can be converted to feet), and the C++ meaning
 *          of conversion (type meters can be converted to type feet).
 *          Conversion is always symmetric, so if U1 is convertible to U2, then
 *          U2 will be convertible to U1.
 * @tparam U1 Unit to convert from.
 * @tparam U2 Unit to convert to.
 * @sa is_convertible_unit
 */
template <class U1, class U2>
concept is_convertible_unit_t = requires {
  typename U1::unit_type;
  typename U2::unit_type;
  requires std::same_as<traits::base_unit_of<typename U1::base_unit_type>,
                        traits::base_unit_of<typename U2::base_unit_type>>;
};

//----------------------------------
//  UNIT TYPE
//----------------------------------

/** @cond */  // DOXYGEN IGNORE
// Forward declaration
template <typename T>
struct linear_scale;

template <typename T>
struct decibel_scale;

namespace detail {

/**
 * @brief helper type to identify units.
 * @details A non-templated base class for `unit` which enables RTTI testing.
 */
struct _unit_t {};

}  // namespace detail
/** @endcond */  // END DOXYGEN IGNORE

/**
 * @ingroup TypeTraits
 * @brief Concept which tests if a class is a `unit_t`
 */
template <class T>
concept is_unit_t = std::derived_from<T, units::detail::_unit_t>;

/**
 * @ingroup UnitContainers
 * @brief Container for values which represent quantities of a given unit.
 * @details Stores a value which represents a quantity in the given units. Unit
 *          containers (except scalar values) are *not* convertible to built-in
 *          C++ types, in order to provide type safety in dimensional analysis.
 *          Unit containers *are* implicitly convertible to other compatible
 *          unit container types. Unit containers support various types of
 *          arithmetic operations, depending on their scale type.
 *
 *         The value of a `unit_t` can only be changed on construction, or by
 *         assignment from another `unit_t` type. If necessary, the underlying
 *         value can be accessed using `operator()`:
 * @code
 * meter_t m(5.0);
 * double val = m(); // val == 5.0
 * @endcode.
 * @tparam Units unit tag for which type of units the `unit_t` represents (e.g.
 *         meters)
 * @tparam T underlying type of the storage. Defaults to double.
 * @tparam Scale optional scale class for the units. Defaults to linear (i.e.
 *         does not scale the unit value). Examples of nonlinear scales could be
 *         logarithmic, decibel, or richter scales. Scales must adhere to the
 *         Scale concept.
 * @sa
 * - \ref lengthContainers "length unit containers"
 * - \ref massContainers "mass unit containers"
 * - \ref timeContainers "time unit containers"
 * - \ref angleContainers "angle unit containers"
 * - \ref currentContainers "current unit containers"
 * - \ref temperatureContainers "temperature unit containers"
 * - \ref substanceContainers "substance unit containers"
 * - \ref luminousIntensityContainers "luminous intensity unit containers"
 * - \ref solidAngleContainers "solid angle unit containers"
 * - \ref frequencyContainers "frequency unit containers"
 * - \ref velocityContainers "velocity unit containers"
 * - \ref angularVelocityContainers "angular velocity unit containers"
 * - \ref accelerationContainers "acceleration unit containers"
 * - \ref forceContainers "force unit containers"
 * - \ref pressureContainers "pressure unit containers"
 * - \ref chargeContainers "charge unit containers"
 * - \ref energyContainers "energy unit containers"
 * - \ref powerContainers "power unit containers"
 * - \ref voltageContainers "voltage unit containers"
 * - \ref capacitanceContainers "capacitance unit containers"
 * - \ref impedanceContainers "impedance unit containers"
 * - \ref magneticFluxContainers "magnetic flux unit containers"
 * - \ref magneticFieldStrengthContainers "magnetic field strength unit
 *   containers"
 * - \ref inductanceContainers "inductance unit containers"
 * - \ref luminousFluxContainers "luminous flux unit containers"
 * - \ref illuminanceContainers "illuminance unit containers"
 * - \ref radiationContainers "radiation unit containers"
 * - \ref torqueContainers "torque unit containers"
 * - \ref areaContainers "area unit containers"
 * - \ref volumeContainers "volume unit containers"
 * - \ref densityContainers "density unit containers"
 * - \ref concentrationContainers "concentration unit containers"
 * - \ref constantContainers "constant unit containers"
 */
template <class Units, typename T = double, Scale<T> S = linear_scale<T>>
class unit_t : public S, units::detail::_unit_t {
  static_assert(is_unit<Units>,
                "Template parameter `Units` must be a unit tag. Check that you "
                "aren't using a unit type (_t).");

 protected:
  using nls = S;
  using nls::m_value;

 public:
  /**
   * Type of the scale of the unit_t (e.g. linear_scale)
   */
  using scale_type = S;

  /**
   * Type of the underlying storage of the unit_t (e.g. double)
   */
  using underlying_type = T;

  /**
   * Type of `unit` the `unit_t` represents (e.g. meters)
   */
  using unit_type = Units;

  /**
   * @ingroup Constructors
   * @brief default constructor.
   */
  constexpr unit_t() = default;

  /**
   * @brief constructor
   * @details constructs a new unit_t using the scale's constructor.
   * @param[in] value unit value magnitude.
   * @param[in] args additional constructor arguments are forwarded to the scale
   *            constructor. Which args are required depends on which scale is
   *            used. For the default (linear) scale, no additional args are
   *            necessary.
   */
  template <class... Args>
  inline explicit constexpr unit_t(const T value, const Args&... args) noexcept
      : nls(value, args...) {}

  /**
   * @brief constructor
   * @details enable implicit conversions from T types ONLY for linear scalar
   *          units
   * @param[in] value value of the unit_t
   */
  template <class Ty>
    requires is_dimensionless_unit_v<Units> && std::is_arithmetic_v<Ty>
  // NOLINTNEXTLINE
  inline constexpr unit_t(const Ty value) noexcept : nls(value) {}

  /**
   * @brief chrono constructor
   * @details enable implicit conversions from std::chrono::duration types ONLY
   *          for time units
   * @param[in] value value of the unit_t
   */
  template <class Rep, class Period>
    requires std::is_arithmetic_v<Rep> && is_ratio<Period>
  // NOLINTNEXTLINE
  inline constexpr unit_t(
      const std::chrono::duration<Rep, Period>& value) noexcept
      : nls(units::convert<unit<std::ratio<1, 1000000000>, category::time_unit>,
                           Units>(static_cast<T>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(value)
                .count()))) {}

  /**
   * @brief copy constructor (converting)
   * @details performs implicit unit conversions if required.
   * @param[in] rhs unit to copy.
   */
  template <class UnitsRhs, typename Ty, Scale<Ty> NlsRhs>
  // NOLINTNEXTLINE
  inline constexpr unit_t(const unit_t<UnitsRhs, Ty, NlsRhs>& rhs) noexcept
      : nls(units::convert<UnitsRhs, Units, T>(rhs.m_value),
            std::true_type() /*store linear value*/) {}

  /**
   * @brief assignment
   * @details performs implicit unit conversions if required
   * @param[in] rhs unit to copy.
   */
  template <class UnitsRhs, typename Ty, Scale<Ty> NlsRhs>
  inline unit_t& operator=(const unit_t<UnitsRhs, Ty, NlsRhs>& rhs) noexcept {
    nls::m_value = units::convert<UnitsRhs, Units, T>(rhs.m_value);
    return *this;
  }

  /**
   * @brief assignment
   * @details performs implicit conversions from built-in types ONLY for scalar
   *          units
   * @param[in] rhs value to copy.
   */
  template <class Ty>
    requires is_dimensionless_unit_v<Units> && std::is_arithmetic_v<Ty>
  inline unit_t& operator=(const Ty& rhs) noexcept {
    nls::m_value = rhs;
    return *this;
  }

  /**
   * @brief less-than
   * @details compares the linearized value of two units. Performs unit
   *          conversions if necessary.
   * @param[in] rhs right-hand side unit for the comparison
   * @returns true IFF the value of `this` is less than the value of `rhs`
   */
  template <class UnitsRhs, typename Ty, Scale<Ty> NlsRhs>
  inline constexpr bool operator<(
      const unit_t<UnitsRhs, Ty, NlsRhs>& rhs) const noexcept {
    return (nls::m_value < units::convert<UnitsRhs, Units>(rhs.m_value));
  }

  /**
   * @brief less-than or equal
   * @details compares the linearized value of two units. Performs unit
   *          conversions if necessary.
   * @param[in] rhs right-hand side unit for the comparison
   * @returns true IFF the value of `this` is less than or equal to the value of
   *          `rhs`
   */
  template <class UnitsRhs, typename Ty, Scale<Ty> NlsRhs>
  inline constexpr bool operator<=(
      const unit_t<UnitsRhs, Ty, NlsRhs>& rhs) const noexcept {
    return (nls::m_value <= units::convert<UnitsRhs, Units>(rhs.m_value));
  }

  /**
   * @brief greater-than
   * @details compares the linearized value of two units. Performs unit
   *          conversions if necessary.
   * @param[in] rhs right-hand side unit for the comparison
   * @returns true IFF the value of `this` is greater than the value of `rhs`
   */
  template <class UnitsRhs, typename Ty, Scale<Ty> NlsRhs>
  inline constexpr bool operator>(
      const unit_t<UnitsRhs, Ty, NlsRhs>& rhs) const noexcept {
    return (nls::m_value > units::convert<UnitsRhs, Units>(rhs.m_value));
  }

  /**
   * @brief greater-than or equal
   * @details compares the linearized value of two units. Performs unit
   *          conversions if necessary.
   * @param[in] rhs right-hand side unit for the comparison
   * @returns true IFF the value of `this` is greater than or equal to the value
   *          of `rhs`
   */
  template <class UnitsRhs, typename Ty, Scale<Ty> NlsRhs>
  inline constexpr bool operator>=(
      const unit_t<UnitsRhs, Ty, NlsRhs>& rhs) const noexcept {
    return (nls::m_value >= units::convert<UnitsRhs, Units>(rhs.m_value));
  }

  /**
   * @brief equality
   * @details compares the linearized value of two units. Performs unit
   *          conversions if necessary.
   * @param[in] rhs right-hand side unit for the comparison
   * @returns true IFF the value of `this` exactly equal to the value of rhs.
   * @note This may not be suitable for all applications when the
   *       underlying_type of unit_t is a double.
   */
  template <class UnitsRhs, typename Ty, Scale<Ty> NlsRhs>
    requires std::floating_point<T> || std::floating_point<Ty>
  inline constexpr bool operator==(
      const unit_t<UnitsRhs, Ty, NlsRhs>& rhs) const noexcept {
    return detail::abs(nls::m_value -
                       units::convert<UnitsRhs, Units>(rhs.m_value)) <
               std::numeric_limits<T>::epsilon() *
                   detail::abs(nls::m_value +
                               units::convert<UnitsRhs, Units>(rhs.m_value)) ||
           detail::abs(nls::m_value -
                       units::convert<UnitsRhs, Units>(rhs.m_value)) <
               (std::numeric_limits<T>::min)();
  }

  template <class UnitsRhs, std::integral Ty, Scale<Ty> NlsRhs>
    requires std::integral<T>
  inline constexpr bool operator==(
      const unit_t<UnitsRhs, Ty, NlsRhs>& rhs) const noexcept {
    return nls::m_value == units::convert<UnitsRhs, Units>(rhs.m_value);
  }

  /**
   * @brief inequality
   * @details compares the linearized value of two units. Performs unit
   *          conversions if necessary.
   * @param[in] rhs right-hand side unit for the comparison
   * @returns true IFF the value of `this` is not equal to the value of rhs.
   * @note This may not be suitable for all applications when the
   *       underlying_type of unit_t is a double.
   */
  template <class UnitsRhs, typename Ty, Scale<Ty> NlsRhs>
  inline constexpr bool operator!=(
      const unit_t<UnitsRhs, Ty, NlsRhs>& rhs) const noexcept {
    return !(*this == rhs);
  }

  /**
   * @brief unit value
   * @returns value of the unit in it's underlying, non-safe type.
   */
  inline constexpr underlying_type value() const noexcept {
    return static_cast<underlying_type>(*this);
  }

  /**
   * @brief unit value
   * @returns value of the unit converted to an arithmetic, non-safe type.
   */
  template <typename Ty>
    requires std::is_arithmetic_v<Ty>
  inline constexpr Ty to() const noexcept {
    return static_cast<Ty>(*this);
  }

  /**
   * @brief linearized unit value
   * @returns linearized value of unit which has a nonlinear scale. For `unit_t`
   *          types with linear scales, this is equivalent to `value`.
   */
  template <typename Ty>
    requires std::is_arithmetic_v<Ty>
  inline constexpr Ty toLinearized() const noexcept {
    return static_cast<Ty>(m_value);
  }

  /**
   * @brief conversion
   * @details Converts to a different unit container. Units can be converted to
   *          other containers implicitly, but this can be used in cases where
   *          explicit notation of a conversion is beneficial, or where an
   *          r-value container is needed.
   * @tparam U unit (not unit_t) to convert to
   * @returns a unit container with the specified units containing the
   *          equivalent value to *this.
   */
  template <class U>
  inline constexpr unit_t<U> convert() const noexcept {
    static_assert(is_unit<U>, "Template parameter `U` must be a unit type.");
    return unit_t<U>(*this);
  }

  /**
   * @brief implicit type conversion.
   * @details only enabled for scalar unit types.
   */
  template <class Ty>
    requires is_dimensionless_unit_v<Units> && std::is_arithmetic_v<Ty>
  // NOLINTNEXTLINE
  inline constexpr operator Ty() const noexcept {
    // this conversion also resolves any PI exponents, by converting from a
    // non-zero PI ratio to a zero-pi ratio.
    return static_cast<Ty>(
        units::convert<Units,
                       unit<std::ratio<1>, units::category::scalar_unit>>(
            (*this)()));
  }

  /**
   * @brief explicit type conversion.
   * @details only enabled for non-dimensionless unit types.
   */
  template <class Ty>
    requires(!is_dimensionless_unit_v<Units>) && std::is_arithmetic_v<Ty>
  inline constexpr explicit operator Ty() const noexcept {
    return static_cast<Ty>((*this)());
  }

  /**
   * @brief chrono implicit type conversion.
   * @details only enabled for time unit types.
   */
  template <typename U = Units>
    requires units::is_convertible_unit<
        U, unit<std::ratio<1>, category::time_unit>>
  // NOLINTNEXTLINE
  inline constexpr operator std::chrono::nanoseconds() const noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double, std::nano>(
            units::convert<
                Units, unit<std::ratio<1, 1000000000>, category::time_unit>>(
                (*this)())));
  }

  /**
   * @brief returns the unit name
   */
  inline constexpr const char* name() const noexcept {
    return units::name(*this);
  }

  /**
   * @brief returns the unit abbreviation
   */
  inline constexpr const char* abbreviation() const noexcept {
    return units::abbreviation(*this);
  }

 public:
  template <class U, typename Ty, Scale<Ty> Nlt>
  friend class unit_t;
};

//------------------------------
//  UNIT_T NON-MEMBER FUNCTIONS
//------------------------------

/**
 * @ingroup UnitContainers
 * @brief Constructs a unit container from an arithmetic type.
 * @details make_unit can be used to construct a unit container from an
 *          arithmetic type, as an alternative to using the explicit
 *          constructor. Unlike the explicit constructor it forces the user to
 *          explicitly specify the units.
 * @tparam UnitType Type to construct.
 * @tparam Ty Arithmetic type.
 * @param[in] value Arithmetic value that represents a quantity in units of
 *            `UnitType`.
 */
template <class UnitType, typename T>
  requires std::is_arithmetic_v<T>
inline constexpr UnitType make_unit(const T value) noexcept {
  static_assert(is_unit<UnitType>,
                "Template parameter `UnitType` must be a unit type (_t).");

  return UnitType(value);
}

#if defined(UNIT_LIB_ENABLE_IOSTREAM)
template <class Units, typename T, Scale<T> S>
inline std::ostream& operator<<(std::ostream& os,
                                const unit_t<Units, T, S>& obj) noexcept {
  using BaseUnits = unit<std::ratio<1>, typename Units::base_unit_type>;
  os << convert<Units, BaseUnits>(obj());

  if constexpr (Units::base_unit_type::meter_ratio::num != 0) {
    os << " m";
  }
  if constexpr (Units::base_unit_type::meter_ratio::num != 0 &&
                Units::base_unit_type::meter_ratio::num != 1) {
    os << "^" << Units::base_unit_type::meter_ratio::num;
  }
  if constexpr (Units::base_unit_type::meter_ratio::den != 1) {
    os << "/" << Units::base_unit_type::meter_ratio::den;
  }

  if constexpr (Units::base_unit_type::kilogram_ratio::num != 0) {
    os << " kg";
  }
  if constexpr (Units::base_unit_type::kilogram_ratio::num != 0 &&
                Units::base_unit_type::kilogram_ratio::num != 1) {
    os << "^" << Units::base_unit_type::kilogram_ratio::num;
  }
  if constexpr (Units::base_unit_type::kilogram_ratio::den != 1) {
    os << "/" << Units::base_unit_type::kilogram_ratio::den;
  }

  if constexpr (Units::base_unit_type::second_ratio::num != 0) {
    os << " s";
  }
  if constexpr (Units::base_unit_type::second_ratio::num != 0 &&
                Units::base_unit_type::second_ratio::num != 1) {
    os << "^" << Units::base_unit_type::second_ratio::num;
  }
  if constexpr (Units::base_unit_type::second_ratio::den != 1) {
    os << "/" << Units::base_unit_type::second_ratio::den;
  }

  if constexpr (Units::base_unit_type::ampere_ratio::num != 0) {
    os << " A";
  }
  if constexpr (Units::base_unit_type::ampere_ratio::num != 0 &&
                Units::base_unit_type::ampere_ratio::num != 1) {
    os << "^" << Units::base_unit_type::ampere_ratio::num;
  }
  if constexpr (Units::base_unit_type::ampere_ratio::den != 1) {
    os << "/" << Units::base_unit_type::ampere_ratio::den;
  }

  if constexpr (Units::base_unit_type::kelvin_ratio::num != 0) {
    os << " K";
  }
  if constexpr (Units::base_unit_type::kelvin_ratio::num != 0 &&
                Units::base_unit_type::kelvin_ratio::num != 1) {
    os << "^" << Units::base_unit_type::kelvin_ratio::num;
  }
  if constexpr (Units::base_unit_type::kelvin_ratio::den != 1) {
    os << "/" << Units::base_unit_type::kelvin_ratio::den;
  }

  if constexpr (Units::base_unit_type::mole_ratio::num != 0) {
    os << " mol";
  }
  if constexpr (Units::base_unit_type::mole_ratio::num != 0 &&
                Units::base_unit_type::mole_ratio::num != 1) {
    os << "^" << Units::base_unit_type::mole_ratio::num;
  }
  if constexpr (Units::base_unit_type::mole_ratio::den != 1) {
    os << "/" << Units::base_unit_type::mole_ratio::den;
  }

  if constexpr (Units::base_unit_type::candela_ratio::num != 0) {
    os << " cd";
  }
  if constexpr (Units::base_unit_type::candela_ratio::num != 0 &&
                Units::base_unit_type::candela_ratio::num != 1) {
    os << "^" << Units::base_unit_type::candela_ratio::num;
  }
  if constexpr (Units::base_unit_type::candela_ratio::den != 1) {
    os << "/" << Units::base_unit_type::candela_ratio::den;
  }

  if constexpr (Units::base_unit_type::radian_ratio::num != 0) {
    os << " rad";
  }
  if constexpr (Units::base_unit_type::radian_ratio::num != 0 &&
                Units::base_unit_type::radian_ratio::num != 1) {
    os << "^" << Units::base_unit_type::radian_ratio::num;
  }
  if constexpr (Units::base_unit_type::radian_ratio::den != 1) {
    os << "/" << Units::base_unit_type::radian_ratio::den;
  }

  if constexpr (Units::base_unit_type::byte_ratio::num != 0) {
    os << " b";
  }
  if constexpr (Units::base_unit_type::byte_ratio::num != 0 &&
                Units::base_unit_type::byte_ratio::num != 1) {
    os << "^" << Units::base_unit_type::byte_ratio::num;
  }
  if constexpr (Units::base_unit_type::byte_ratio::den != 1) {
    os << "/" << Units::base_unit_type::byte_ratio::den;
  }

  return os;
}
#endif

template <class Units, typename T, Scale<T> S, typename RhsType>
inline unit_t<Units, T, S>& operator+=(unit_t<Units, T, S>& lhs,
                                       const RhsType& rhs) noexcept {
  static_assert(is_convertible_unit_t<unit_t<Units, T, S>, RhsType> ||
                    (is_dimensionless_unit_v<decltype(lhs)> &&
                     std::is_arithmetic_v<RhsType>),
                "parameters are not compatible units.");

  lhs = lhs + rhs;
  return lhs;
}

template <class Units, typename T, Scale<T> S, typename RhsType>
inline unit_t<Units, T, S>& operator-=(unit_t<Units, T, S>& lhs,
                                       const RhsType& rhs) noexcept {
  static_assert(is_convertible_unit_t<unit_t<Units, T, S>, RhsType> ||
                    (is_dimensionless_unit_v<decltype(lhs)> &&
                     std::is_arithmetic_v<RhsType>),
                "parameters are not compatible units.");

  lhs = lhs - rhs;
  return lhs;
}

template <class Units, typename T, Scale<T> S, typename RhsType>
inline unit_t<Units, T, S>& operator*=(unit_t<Units, T, S>& lhs,
                                       const RhsType& rhs) noexcept {
  static_assert(
      (is_dimensionless_unit_v<RhsType> || std::is_arithmetic_v<RhsType>),
      "right-hand side parameter must be dimensionless.");

  lhs = lhs * rhs;
  return lhs;
}

template <class Units, typename T, Scale<T> S, typename RhsType>
inline unit_t<Units, T, S>& operator/=(unit_t<Units, T, S>& lhs,
                                       const RhsType& rhs) noexcept {
  static_assert(
      (is_dimensionless_unit_v<RhsType> || std::is_arithmetic_v<RhsType>),
      "right-hand side parameter must be dimensionless.");

  lhs = lhs / rhs;
  return lhs;
}

//------------------------------
//  UNIT_T UNARY OPERATORS
//------------------------------

// unary addition: +T
template <class Units, typename T, Scale<T> S>
constexpr inline unit_t<Units, T, S> operator+(
    const unit_t<Units, T, S>& u) noexcept {
  return u;
}

// prefix increment: ++T
template <class Units, typename T, Scale<T> S>
inline unit_t<Units, T, S>& operator++(unit_t<Units, T, S>& u) noexcept {
  u = unit_t<Units, T, S>(u() + 1);
  return u;
}

// postfix increment: T++
template <class Units, typename T, Scale<T> S>
inline unit_t<Units, T, S> operator++(unit_t<Units, T, S>& u, int) noexcept {
  auto ret = u;
  u = unit_t<Units, T, S>(u() + 1);
  return ret;
}

// unary addition: -T
template <class Units, typename T, Scale<T> S>
constexpr inline unit_t<Units, T, S> operator-(
    const unit_t<Units, T, S>& u) noexcept {
  return unit_t<Units, T, S>(-u());
}

// prefix increment: --T
template <class Units, typename T, Scale<T> S>
inline unit_t<Units, T, S>& operator--(unit_t<Units, T, S>& u) noexcept {
  u = unit_t<Units, T, S>(u() - 1);
  return u;
}

// postfix increment: T--
template <class Units, typename T, Scale<T> S>
inline unit_t<Units, T, S> operator--(unit_t<Units, T, S>& u, int) noexcept {
  auto ret = u;
  u = unit_t<Units, T, S>(u() - 1);
  return ret;
}

//------------------------------
//  UNIT_CAST
//------------------------------

/**
 * @ingroup Conversion
 * @brief Casts a unit container to an arithmetic type.
 * @details unit_cast can be used to remove the strong typing from a unit class,
 *          and convert it to a built-in arithmetic type. This may be useful for
 *          compatibility with libraries and legacy code that don't support
 *          `unit_t` types. E.g
 * @code
 * meter_t unitVal(5);
 * double value = units::unit_cast<double>(unitVal);  // value = 5.0
 * @endcode
 * @tparam T Type to cast the unit type to. Must be a built-in arithmetic type.
 * @param value Unit value to cast.
 * @sa unit_t::to
 */
template <typename T, typename Units>
  requires std::is_arithmetic_v<T> && is_unit<Units>
inline constexpr T unit_cast(const Units& value) noexcept {
  return static_cast<T>(value);
}

//------------------------------
//  SCALE TRAITS
//------------------------------

// Forward declaration
template <typename T>
struct decibel_scale;

/**
 * @ingroup TypeTraits
 * @brief Concept which tests whether a type represents a unit_t with a linear
 * scale.
 */
template <typename T>
concept has_linear_scale =
    std::derived_from<T, units::linear_scale<typename T::underlying_type>>;

/**
 * @ingroup TypeTraits
 * @brief Concept which tests whether a type represents a unit_t with a decibel
 * scale.
 */
template <typename T>
concept has_decibel_scale =
    std::derived_from<T, units::decibel_scale<typename T::underlying_type>>;

/**
 * @ingroup TypeTraits
 * @brief Concept which tests whether two types has the same scale.
 */
template <typename T1, typename T2>
concept is_same_scale =
    std::same_as<typename T1::scale_type, typename T2::scale_type>;

//----------------------------------
//  NONLINEAR SCALES
//----------------------------------

// Non-linear transforms are used to pre and post scale units which are defined
// in terms of non- linear functions of their current value. A good example of a
// nonlinear scale would be a logarithmic or decibel scale.

//------------------------------
//  LINEAR SCALE
//------------------------------

/**
 * @brief unit_t scale which is linear
 * @details Represents units on a linear scale. This is the appropriate unit_t
 * scale for almost all units almost all of the time.
 * @tparam T underlying storage type
 * @sa unit_t
 */
template <typename T>
struct linear_scale {
  constexpr linear_scale() = default;

  constexpr linear_scale(const linear_scale&) = default;
  linear_scale& operator=(const linear_scale&) = default;
  constexpr linear_scale(linear_scale&&) = default;
  linear_scale& operator=(linear_scale&&) = default;

  template <class... Args>
  // NOLINTNEXTLINE
  constexpr linear_scale(const T& value, Args&&...) noexcept : m_value(value) {}

  constexpr T operator()() const noexcept { return m_value; }

  /**
   * Linearized value.
   */
  T m_value;
};

//----------------------------------
//  SCALAR (LINEAR) UNITS
//----------------------------------

// Scalar units are the *ONLY* units implicitly convertible to/from built-in
// types.
namespace dimensionless {

using scalar = unit<std::ratio<1>, units::category::scalar_unit>;
using dimensionless = unit<std::ratio<1>, units::category::dimensionless_unit>;

using scalar_t = unit_t<scalar>;
using dimensionless_t = scalar_t;

}  // namespace dimensionless

// Ignore the redeclaration of the default template parameters
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4348)
#endif

UNIT_ADD_CATEGORY_TRAIT(scalar)
UNIT_ADD_CATEGORY_TRAIT(dimensionless)

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

//------------------------------
//  LINEAR ARITHMETIC
//------------------------------

template <class UnitTypeLhs, class UnitTypeRhs>
  requires(!is_same_scale<UnitTypeLhs, UnitTypeRhs>)
constexpr inline int operator+(const UnitTypeLhs& /* lhs */,
                               const UnitTypeRhs& /* rhs */) noexcept {
  static_assert(is_same_scale<UnitTypeLhs, UnitTypeRhs>,
                "Cannot add units with different linear/nonlinear scales.");
  return 0;
}

/**
 * Addition operator for unit_t types with a linear_scale.
 */
template <class UnitTypeLhs, class UnitTypeRhs>
  requires has_linear_scale<UnitTypeLhs> && has_linear_scale<UnitTypeRhs>
inline constexpr UnitTypeLhs operator+(const UnitTypeLhs& lhs,
                                       const UnitTypeRhs& rhs) noexcept {
  using UnitsLhs = typename UnitTypeLhs::unit_type;
  using UnitsRhs = typename UnitTypeRhs::unit_type;
  return UnitTypeLhs(lhs() + convert<UnitsRhs, UnitsLhs>(rhs()));
}

/**
 * Addition operator for scalar unit_t types with a linear_scale. Scalar types
 * can be implicitly converted to built-in types.
 */
template <typename T>
  requires std::is_arithmetic_v<T>
inline constexpr dimensionless::scalar_t operator+(
    const dimensionless::scalar_t& lhs, T rhs) noexcept {
  return dimensionless::scalar_t(lhs() + rhs);
}

/**
 * Addition operator for scalar unit_t types with a linear_scale. Scalar types
 * can be implicitly converted to built-in types.
 */
template <typename T>
  requires std::is_arithmetic_v<T>
inline constexpr dimensionless::scalar_t operator+(
    T lhs, const dimensionless::scalar_t& rhs) noexcept {
  return dimensionless::scalar_t(lhs + rhs());
}

/**
 * Subtraction operator for unit_t types with a linear_scale.
 */
template <class UnitTypeLhs, class UnitTypeRhs>
  requires has_linear_scale<UnitTypeLhs> && has_linear_scale<UnitTypeRhs>
inline constexpr UnitTypeLhs operator-(const UnitTypeLhs& lhs,
                                       const UnitTypeRhs& rhs) noexcept {
  using UnitsLhs = typename UnitTypeLhs::unit_type;
  using UnitsRhs = typename UnitTypeRhs::unit_type;
  return UnitTypeLhs(lhs() - convert<UnitsRhs, UnitsLhs>(rhs()));
}

/**
 * Subtraction operator for scalar unit_t types with a linear_scale. Scalar
 * types can be implicitly converted to built-in types.
 */
template <typename T>
  requires std::is_arithmetic_v<T>
inline constexpr dimensionless::scalar_t operator-(
    const dimensionless::scalar_t& lhs, T rhs) noexcept {
  return dimensionless::scalar_t(lhs() - rhs);
}

/**
 * Subtraction operator for scalar unit_t types with a linear_scale. Scalar
 * types can be implicitly converted to built-in types.
 */
template <typename T>
  requires std::is_arithmetic_v<T>
inline constexpr dimensionless::scalar_t operator-(
    T lhs, const dimensionless::scalar_t& rhs) noexcept {
  return dimensionless::scalar_t(lhs - rhs());
}

/**
 * Multiplication type for convertible unit_t types with a linear scale.
 * @returns the multiplied value, with the same type as left-hand side unit.
 */
template <class UnitTypeLhs, class UnitTypeRhs>
  requires is_convertible_unit_t<UnitTypeLhs, UnitTypeRhs> &&
           has_linear_scale<UnitTypeLhs> && has_linear_scale<UnitTypeRhs>
inline constexpr auto operator*(const UnitTypeLhs& lhs,
                                const UnitTypeRhs& rhs) noexcept
    -> unit_t<compound_unit<squared<typename UnitTypeLhs::unit_type>>> {
  using UnitsLhs = typename UnitTypeLhs::unit_type;
  using UnitsRhs = typename UnitTypeRhs::unit_type;
  return unit_t<compound_unit<squared<typename UnitTypeLhs::unit_type>>>(
      lhs() * convert<UnitsRhs, UnitsLhs>(rhs()));
}

/**
 * Multiplication type for non-convertible unit_t types with a linear scale.
 * @returns the multiplied value, whose type is a compound unit of the left and
 *          right hand side values.
 */
template <class UnitTypeLhs, class UnitTypeRhs>
  requires(!is_convertible_unit_t<UnitTypeLhs, UnitTypeRhs>) &&
          has_linear_scale<UnitTypeLhs> && has_linear_scale<UnitTypeRhs> &&
          (!is_dimensionless_unit_t_v<UnitTypeLhs>) &&
          (!is_dimensionless_unit_t_v<UnitTypeRhs>)
inline constexpr auto operator*(const UnitTypeLhs& lhs,
                                const UnitTypeRhs& rhs) noexcept
    -> unit_t<compound_unit<typename UnitTypeLhs::unit_type,
                            typename UnitTypeRhs::unit_type>> {
  using UnitsLhs = typename UnitTypeLhs::unit_type;
  using UnitsRhs = typename UnitTypeRhs::unit_type;
  return unit_t<compound_unit<UnitsLhs, UnitsRhs>>(lhs() * rhs());
}

/**
 * Multiplication by a dimensionless unit for unit_t types with a linear scale.
 */
template <class UnitTypeLhs, typename UnitTypeRhs>
  requires has_linear_scale<UnitTypeLhs> && has_linear_scale<UnitTypeRhs> &&
           (!is_dimensionless_unit_t_v<UnitTypeLhs>) &&
           is_dimensionless_unit_t_v<UnitTypeRhs>
inline constexpr UnitTypeLhs operator*(const UnitTypeLhs& lhs,
                                       const UnitTypeRhs& rhs) noexcept {
  // The cast makes sure factors of PI are handled as expected
  return UnitTypeLhs(lhs() * static_cast<double>(rhs));
}

/**
 * Multiplication by a dimensionless unit for unit_t types with a linear scale.
 */
template <class UnitTypeLhs, typename UnitTypeRhs>
  requires has_linear_scale<UnitTypeLhs> && has_linear_scale<UnitTypeRhs> &&
           is_dimensionless_unit_t_v<UnitTypeLhs> &&
           (!is_dimensionless_unit_t_v<UnitTypeRhs>)
inline constexpr UnitTypeRhs operator*(const UnitTypeLhs& lhs,
                                       const UnitTypeRhs& rhs) noexcept {
  // The cast makes sure factors of PI are handled as expected
  return UnitTypeRhs(static_cast<double>(lhs) * rhs());
}

/**
 * Multiplication by a scalar for unit_t types with a linear scale.
 */
template <class UnitTypeLhs, typename T>
  requires std::is_arithmetic_v<T> && has_linear_scale<UnitTypeLhs>
inline constexpr UnitTypeLhs operator*(const UnitTypeLhs& lhs, T rhs) noexcept {
  return UnitTypeLhs(lhs() * rhs);
}

/**
 * Multiplication by a scalar for unit_t types with a linear scale.
 */
template <class UnitTypeRhs, typename T>
  requires std::is_arithmetic_v<T> && has_linear_scale<UnitTypeRhs>
inline constexpr UnitTypeRhs operator*(T lhs, const UnitTypeRhs& rhs) noexcept {
  return UnitTypeRhs(lhs * rhs());
}

/**
 * Division for convertible unit_t types with a linear scale. @returns the lhs
 * divided by rhs value, whose type is a scalar
 */
template <class UnitTypeLhs, class UnitTypeRhs>
  requires is_convertible_unit_t<UnitTypeLhs, UnitTypeRhs> &&
           has_linear_scale<UnitTypeLhs> && has_linear_scale<UnitTypeRhs>
inline constexpr dimensionless::scalar_t operator/(
    const UnitTypeLhs& lhs, const UnitTypeRhs& rhs) noexcept {
  using UnitsLhs = typename UnitTypeLhs::unit_type;
  using UnitsRhs = typename UnitTypeRhs::unit_type;
  return dimensionless::scalar_t(lhs() / convert<UnitsRhs, UnitsLhs>(rhs()));
}

/**
 * Division for non-convertible unit_t types with a linear scale.
 * @returns the lhs divided by the rhs, with a compound unit type of lhs/rhs
 */
template <class UnitTypeLhs, class UnitTypeRhs>
  requires(!is_convertible_unit_t<UnitTypeLhs, UnitTypeRhs>) &&
          has_linear_scale<UnitTypeLhs> && has_linear_scale<UnitTypeRhs> &&
          (!is_dimensionless_unit_t_v<UnitTypeLhs>) &&
          (!is_dimensionless_unit_t_v<UnitTypeRhs>)
inline constexpr auto operator/(const UnitTypeLhs& lhs,
                                const UnitTypeRhs& rhs) noexcept
    -> unit_t<compound_unit<typename UnitTypeLhs::unit_type,
                            inverse<typename UnitTypeRhs::unit_type>>> {
  using UnitsLhs = typename UnitTypeLhs::unit_type;
  using UnitsRhs = typename UnitTypeRhs::unit_type;
  return unit_t<compound_unit<UnitsLhs, inverse<UnitsRhs>>>(lhs() / rhs());
}

/**
 * Division by a dimensionless unit for unit_t types with a linear scale
 */
template <class UnitTypeLhs, class UnitTypeRhs>
  requires has_linear_scale<UnitTypeLhs> && has_linear_scale<UnitTypeRhs> &&
           (!is_dimensionless_unit_t_v<UnitTypeLhs>) &&
           is_dimensionless_unit_t_v<UnitTypeRhs>
inline constexpr UnitTypeLhs operator/(const UnitTypeLhs& lhs,
                                       const UnitTypeRhs& rhs) noexcept {
  return UnitTypeLhs(lhs() / static_cast<double>(rhs));
}

/**
 * Division of a dimensionless unit  by a unit_t type with a linear scale
 */
template <class UnitTypeLhs, class UnitTypeRhs>
  requires has_linear_scale<UnitTypeLhs> && has_linear_scale<UnitTypeRhs> &&
           is_dimensionless_unit_t_v<UnitTypeLhs> &&
           (!is_dimensionless_unit_t_v<UnitTypeRhs>)
inline constexpr auto operator/(const UnitTypeLhs& lhs,
                                const UnitTypeRhs& rhs) noexcept
    -> unit_t<inverse<typename UnitTypeRhs::unit_type>> {
  return unit_t<inverse<typename UnitTypeRhs::unit_type>>(
      static_cast<double>(lhs) / rhs());
}

/**
 * Division by a scalar for unit_t types with a linear scale
 */
template <class UnitTypeLhs, typename T>
  requires std::is_arithmetic_v<T> && has_linear_scale<UnitTypeLhs>
inline constexpr UnitTypeLhs operator/(const UnitTypeLhs& lhs, T rhs) noexcept {
  return UnitTypeLhs(lhs() / rhs);
}

/**
 * Division of a scalar  by a unit_t type with a linear scale
 */
template <class UnitTypeRhs, typename T>
  requires std::is_arithmetic_v<T> && has_linear_scale<UnitTypeRhs>
inline constexpr auto operator/(T lhs, const UnitTypeRhs& rhs) noexcept
    -> unit_t<inverse<typename UnitTypeRhs::unit_type>> {
  using UnitsRhs = typename UnitTypeRhs::unit_type;
  return unit_t<inverse<UnitsRhs>>(lhs / rhs());
}

//----------------------------------
//  SCALAR COMPARISONS
//----------------------------------

template <typename Units>
  requires units::is_dimensionless_unit_v<Units>
constexpr bool operator==(const double lhs, const Units& rhs) noexcept {
  return detail::abs(lhs - static_cast<double>(rhs)) <
             std::numeric_limits<double>::epsilon() *
                 detail::abs(lhs + static_cast<double>(rhs)) ||
         detail::abs(lhs - static_cast<double>(rhs)) <
             (std::numeric_limits<double>::min)();
}

template <typename Units>
  requires units::is_dimensionless_unit_v<Units>
constexpr bool operator==(const Units& lhs, const double rhs) noexcept {
  return detail::abs(static_cast<double>(lhs) - rhs) <
             std::numeric_limits<double>::epsilon() *
                 detail::abs(static_cast<double>(lhs) + rhs) ||
         detail::abs(static_cast<double>(lhs) - rhs) <
             (std::numeric_limits<double>::min)();
}

template <typename Units>
  requires units::is_dimensionless_unit_v<Units>
constexpr bool operator!=(const double lhs, const Units& rhs) noexcept {
  return !(lhs == static_cast<double>(rhs));
}

template <typename Units>
  requires units::is_dimensionless_unit_v<Units>
constexpr bool operator!=(const Units& lhs, const double rhs) noexcept {
  return !(static_cast<double>(lhs) == rhs);
}

template <typename Units>
  requires units::is_dimensionless_unit_v<Units>
constexpr bool operator>=(const double lhs, const Units& rhs) noexcept {
  return std::isgreaterequal(lhs, static_cast<double>(rhs));
}

template <typename Units>
  requires units::is_dimensionless_unit_v<Units>
constexpr bool operator>=(const Units& lhs, const double rhs) noexcept {
  return std::isgreaterequal(static_cast<double>(lhs), rhs);
}

template <typename Units>
  requires units::is_dimensionless_unit_v<Units>
constexpr bool operator>(const double lhs, const Units& rhs) noexcept {
  return lhs > static_cast<double>(rhs);
}

template <typename Units>
  requires units::is_dimensionless_unit_v<Units>
constexpr bool operator>(const Units& lhs, const double rhs) noexcept {
  return static_cast<double>(lhs) > rhs;
}

template <typename Units>
  requires units::is_dimensionless_unit_v<Units>
constexpr bool operator<=(const double lhs, const Units& rhs) noexcept {
  return std::islessequal(lhs, static_cast<double>(rhs));
}

template <typename Units>
  requires units::is_dimensionless_unit_v<Units>
constexpr bool operator<=(const Units& lhs, const double rhs) noexcept {
  return std::islessequal(static_cast<double>(lhs), rhs);
}

template <typename Units>
  requires units::is_dimensionless_unit_v<Units>
constexpr bool operator<(const double lhs, const Units& rhs) noexcept {
  return lhs < static_cast<double>(rhs);
}

template <typename Units>
  requires units::is_dimensionless_unit_v<Units>
constexpr bool operator<(const Units& lhs, const double rhs) noexcept {
  return static_cast<double>(lhs) < rhs;
}

//----------------------------------
//  POW
//----------------------------------

/** @cond */  // DOXYGEN IGNORE
namespace detail {

/// Recursive exponential implementation
template <int N, class U>
struct power_of_unit {
  using type = typename units::detail::unit_multiply<
      U, typename power_of_unit<N - 1, U>::type>;
};

/// End recursion
template <class U>
struct power_of_unit<1, U> {
  using type = U;
};

}  // namespace detail
/** @endcond */  // END DOXYGEN IGNORE

namespace math {

/**
 * @brief computes the value of <i>value</i> raised to the <i>power</i>
 * @details Only implemented for linear_scale units. <i>Power</i> must be known
 *          at compile time, so the resulting unit type can be deduced.
 * @tparam power exponential power to raise <i>value</i> by.
 * @param[in] value `unit_t` derived type to raise to the given <i>power</i>
 * @returns new unit_t, raised to the given exponent
 */
template <int power, class UnitType>
  requires has_linear_scale<UnitType>
inline auto pow(const UnitType& value) noexcept
    -> unit_t<typename units::detail::power_of_unit<
                  power, typename UnitType::unit_type>::type,
              typename UnitType::underlying_type,
              linear_scale<typename UnitType::underlying_type>> {
  return unit_t<typename units::detail::power_of_unit<
                    power, typename UnitType::unit_type>::type,
                typename UnitType::underlying_type,
                linear_scale<typename UnitType::underlying_type>>(
      std::pow(value(), power));
}

/**
 * @brief computes the value of <i>value</i> raised to the <i>power</i> as a
 *        constexpr
 * @details Only implemented for linear_scale units. <i>Power</i> must be known
 *          at compile time, so the resulting unit type can be deduced.
 *          Additionally, the power must be <i>a positive, integral, value</i>.
 * @tparam power exponential power to raise <i>value</i> by.
 * @param[in] value `unit_t` derived type to raise to the given <i>power</i>
 * @returns new unit_t, raised to the given exponent
 */
template <int power, class UnitType>
  requires has_linear_scale<UnitType>
inline constexpr auto cpow(const UnitType& value) noexcept
    -> unit_t<typename units::detail::power_of_unit<
                  power, typename UnitType::unit_type>::type,
              typename UnitType::underlying_type,
              linear_scale<typename UnitType::underlying_type>> {
  static_assert(
      power >= 0,
      "cpow cannot accept negative numbers. Try units::math::pow instead.");
  return unit_t<typename units::detail::power_of_unit<
                    power, typename UnitType::unit_type>::type,
                typename UnitType::underlying_type,
                linear_scale<typename UnitType::underlying_type>>(
      detail::pow(value(), power));
}

}  // namespace math

//------------------------------
//  DECIBEL SCALE
//------------------------------

/**
 * @brief unit_t scale for representing decibel values.
 * @details internally stores linearized values. `operator()` returns the value
 *          in dB.
 * @tparam T underlying storage type
 * @sa unit_t
 */
template <typename T>
struct decibel_scale {
  inline constexpr decibel_scale() = default;
  inline constexpr decibel_scale(const decibel_scale&) = default;
  inline ~decibel_scale() = default;
  inline decibel_scale& operator=(const decibel_scale&) = default;
  inline constexpr decibel_scale(decibel_scale&&) = default;
  inline decibel_scale& operator=(decibel_scale&&) = default;

  // NOLINTNEXTLINE
  inline constexpr decibel_scale(const T value) noexcept
      : m_value(std::pow(10, value / 10)) {}

  template <class... Args>
  inline constexpr decibel_scale(const T value, std::true_type,
                                 Args&&...) noexcept
      : m_value(value) {}

  inline constexpr T operator()() const noexcept {
    return 10 * std::log10(m_value);
  }

  /**
   * Linearized value.
   */
  T m_value;
};

//------------------------------
//  SCALAR (DECIBEL) UNITS
//------------------------------

/**
 * @brief namespace for unit types and containers for units that have no
 *        dimension (scalar units)
 * @sa See unit_t for more information on unit type containers.
 */
namespace dimensionless {

using dB_t = unit_t<scalar, double, decibel_scale<double>>;
using dBi_t = dB_t;

}  // namespace dimensionless

#if defined(UNIT_LIB_ENABLE_IOSTREAM)
namespace dimensionless {

inline std::ostream& operator<<(std::ostream& os, const dB_t& obj) {
  os << obj() << " dB";
  return os;
}

}  // namespace dimensionless
#endif

}  // namespace units

#if __has_include(<fmt/format.h>) && !defined(UNIT_LIB_DISABLE_FMT)
template <>
struct fmt::formatter<units::dimensionless::dB_t> : fmt::formatter<double> {
  template <typename FormatContext>
  auto format(const units::dimensionless::dB_t& obj, FormatContext& ctx)
      -> decltype(ctx.out()) {
    auto out = ctx.out();
    out = fmt::formatter<double>::format(obj(), ctx);
    return fmt::format_to(out, " dB");
  }
};
#endif

namespace units {

//------------------------------
//  DECIBEL ARITHMETIC
//------------------------------

/**
 * Addition for convertible unit_t types with a decibel_scale
 */
template <class UnitTypeLhs, class UnitTypeRhs>
  requires has_decibel_scale<UnitTypeLhs> && has_decibel_scale<UnitTypeRhs>
constexpr inline auto operator+(const UnitTypeLhs& lhs,
                                const UnitTypeRhs& rhs) noexcept
    -> unit_t<compound_unit<squared<typename UnitTypeLhs::unit_type>>,
              typename UnitTypeLhs::underlying_type,
              decibel_scale<typename UnitTypeLhs::underlying_type>> {
  using LhsUnits = typename UnitTypeLhs::unit_type;
  using RhsUnits = typename UnitTypeRhs::unit_type;
  using underlying_type = typename UnitTypeLhs::underlying_type;

  return unit_t<compound_unit<squared<LhsUnits>>, underlying_type,
                decibel_scale<underlying_type>>(
      lhs.template toLinearized<underlying_type>() *
          convert<RhsUnits, LhsUnits>(
              rhs.template toLinearized<underlying_type>()),
      std::true_type());
}

/**
 * Addition between unit_t types with a decibel_scale and dimensionless dB units
 */
template <class UnitTypeLhs>
  requires has_decibel_scale<UnitTypeLhs> &&
           (!is_dimensionless_unit_v<UnitTypeLhs>)
constexpr inline UnitTypeLhs operator+(
    const UnitTypeLhs& lhs, const dimensionless::dB_t& rhs) noexcept {
  using underlying_type = typename UnitTypeLhs::underlying_type;
  return UnitTypeLhs(lhs.template toLinearized<underlying_type>() *
                         rhs.template toLinearized<underlying_type>(),
                     std::true_type());
}

/**
 * Addition between unit_t types with a decibel_scale and dimensionless dB units
 */
template <class UnitTypeRhs>
  requires has_decibel_scale<UnitTypeRhs> &&
           (!is_dimensionless_unit_v<UnitTypeRhs>)
constexpr inline UnitTypeRhs operator+(const dimensionless::dB_t& lhs,
                                       const UnitTypeRhs& rhs) noexcept {
  using underlying_type = typename UnitTypeRhs::underlying_type;
  return UnitTypeRhs(lhs.template toLinearized<underlying_type>() *
                         rhs.template toLinearized<underlying_type>(),
                     std::true_type());
}

/**
 * Subtraction for convertible unit_t types with a decibel_scale
 */
template <class UnitTypeLhs, class UnitTypeRhs>
  requires has_decibel_scale<UnitTypeLhs> && has_decibel_scale<UnitTypeRhs>
constexpr inline auto operator-(const UnitTypeLhs& lhs,
                                const UnitTypeRhs& rhs) noexcept
    -> unit_t<compound_unit<typename UnitTypeLhs::unit_type,
                            inverse<typename UnitTypeRhs::unit_type>>,
              typename UnitTypeLhs::underlying_type,
              decibel_scale<typename UnitTypeLhs::underlying_type>> {
  using LhsUnits = typename UnitTypeLhs::unit_type;
  using RhsUnits = typename UnitTypeRhs::unit_type;
  using underlying_type = typename UnitTypeLhs::underlying_type;

  return unit_t<compound_unit<LhsUnits, inverse<RhsUnits>>, underlying_type,
                decibel_scale<underlying_type>>(
      lhs.template toLinearized<underlying_type>() /
          convert<RhsUnits, LhsUnits>(
              rhs.template toLinearized<underlying_type>()),
      std::true_type());
}

/**
 * Subtraction between unit_t types with a decibel_scale and dimensionless dB
 * units
 */
template <class UnitTypeLhs>
  requires has_decibel_scale<UnitTypeLhs> &&
           (!is_dimensionless_unit_v<UnitTypeLhs>)
constexpr inline UnitTypeLhs operator-(
    const UnitTypeLhs& lhs, const dimensionless::dB_t& rhs) noexcept {
  using underlying_type = typename UnitTypeLhs::underlying_type;
  return UnitTypeLhs(lhs.template toLinearized<underlying_type>() /
                         rhs.template toLinearized<underlying_type>(),
                     std::true_type());
}

/**
 * Subtraction between unit_t types with a decibel_scale and dimensionless dB
 * units
 */
template <class UnitTypeRhs>
  requires has_decibel_scale<UnitTypeRhs> &&
           (!is_dimensionless_unit_v<UnitTypeRhs>)
constexpr inline auto operator-(const dimensionless::dB_t& lhs,
                                const UnitTypeRhs& rhs) noexcept
    -> unit_t<inverse<typename UnitTypeRhs::unit_type>,
              typename UnitTypeRhs::underlying_type,
              decibel_scale<typename UnitTypeRhs::underlying_type>> {
  using RhsUnits = typename UnitTypeRhs::unit_type;
  using underlying_type = typename RhsUnits::underlying_type;

  return unit_t<inverse<RhsUnits>, underlying_type,
                decibel_scale<underlying_type>>(
      lhs.template toLinearized<underlying_type>() /
          rhs.template toLinearized<underlying_type>(),
      std::true_type());
}

//----------------------------------
//  UNIT RATIO CLASS
//----------------------------------

/** @cond */  // DOXYGEN IGNORE
namespace detail {

template <class Units>
struct _unit_value_t {};

}  // namespace detail
/** @endcond */  // END DOXYGEN IGNORE

//------------------------------------------------------------------------------
//  COMPILE-TIME UNIT VALUES AND ARITHMETIC
//------------------------------------------------------------------------------

/**
 * @ingroup UnitContainers
 * @brief Stores a rational unit value as a compile-time constant
 * @details unit_value_t is useful for performing compile-time arithmetic on
 *          known unit quantities.
 * @tparam Units units represented by the `unit_value_t`
 * @tparam Num numerator of the represented value.
 * @tparam Denom denominator of the represented value.
 * @note This is intentionally identical in concept to a `std::ratio`.
 *
 */
template <typename Units, uintmax_t Num, uintmax_t Denom = 1>
struct unit_value_t : units::detail::_unit_value_t<Units> {
  using unit_type = Units;
  using ratio = std::ratio<Num, Denom>;

  static_assert(is_unit<Units>,
                "Template parameter `Units` must be a unit type.");
  static constexpr const unit_t<Units> value() {
    return unit_t<Units>(static_cast<double>(ratio::num) / ratio::den);
  }
};

namespace traits {

/**
 * @ingroup TypeTraits
 * @brief Trait which tests whether a type is a unit_value_t representing the
 *        given unit type.
 * @details e.g. `is_unit_value_t<meters, myType>::value` would test that
 *          `myType` is a `unit_value_t<meters>`.
 * @tparam Units units that the `unit_value_t` is supposed to have.
 * @tparam T type to test.
 */
template <typename T, typename Units = typename T::unit_type>
struct is_unit_value_t
    : std::integral_constant<
          bool, std::is_base_of_v<units::detail::_unit_value_t<Units>, T>> {};

template <typename T, typename Units = typename T::unit_type>
inline constexpr bool is_unit_value_t_v = is_unit_value_t<T, Units>::value;

/**
 * @ingroup TypeTraits
 * @brief Trait which tests whether type T is a unit_value_t with a unit type in
 *        the given category.
 * @details e.g. `is_unit_value_t_category<units::category::length,
 *          unit_value_t<feet>>::value` would be true
 */
template <typename Category, typename T>
struct is_unit_value_t_category
    : std::integral_constant<
          bool,
          std::is_same_v<units::traits::base_unit_of<typename T::unit_type>,
                         Category>> {
  static_assert(is_base_unit<Category>,
                "Template parameter `Category` must be a `base_unit` type.");
};

template <typename Category, typename T>
inline constexpr bool is_unit_value_t_category_v =
    is_unit_value_t_category<Category, T>::value;

}  // namespace traits

/** @cond */  // DOXYGEN IGNORE
namespace detail {

// Base class for common arithmetic
template <class U1, class U2>
struct unit_value_arithmetic {
  static_assert(traits::is_unit_value_t<U1>::value,
                "Template parameter `U1` must be a `unit_value_t` type.");
  static_assert(traits::is_unit_value_t<U2>::value,
                "Template parameter `U2` must be a `unit_value_t` type.");

  using _UNIT1 = typename U1::unit_type;
  using _UNIT2 = typename U2::unit_type;
  using _CONV1 = typename _UNIT1::conversion_ratio;
  using _CONV2 = typename _UNIT2::conversion_ratio;
  using _RATIO1 = typename U1::ratio;
  using _RATIO2 = typename U2::ratio;
  using _RATIO2CONV =
      typename std::ratio_divide<std::ratio_multiply<_RATIO2, _CONV2>, _CONV1>;
  using _PI_EXP = std::ratio_subtract<typename _UNIT2::pi_exponent_ratio,
                                      typename _UNIT1::pi_exponent_ratio>;
};

}  // namespace detail
/** @endcond */  // END DOXYGEN IGNORE

/**
 * @ingroup CompileTimeUnitManipulators
 * @brief adds two unit_value_t types at compile-time
 * @details The resulting unit will the the `unit_type` of `U1`
 * @tparam U1 left-hand `unit_value_t`
 * @tparam U2 right-hand `unit_value_t`
 * @note very similar in concept to `std::ratio_add`
 */
template <class U1, class U2>
struct unit_value_add : units::detail::unit_value_arithmetic<U1, U2>,
                        units::detail::_unit_value_t<typename U1::unit_type> {
  /** @cond */  // DOXYGEN IGNORE
  using Base = units::detail::unit_value_arithmetic<U1, U2>;
  using unit_type = typename Base::_UNIT1;
  using ratio =
      std::ratio_add<typename Base::_RATIO1, typename Base::_RATIO2CONV>;

  static_assert(
      is_convertible_unit<typename Base::_UNIT1, typename Base::_UNIT2>,
      "Unit types are not compatible.");
  /** @endcond */  // END DOXYGEN IGNORE

  /**
   * @brief Value of sum
   * @details Returns the calculated value of the sum of `U1` and `U2`, in the
   *          same units as `U1`.
   * @returns Value of the sum in the appropriate units.
   */
  static constexpr const unit_t<unit_type> value() noexcept {
    using UsePi = std::integral_constant<bool, Base::_PI_EXP::num != 0>;
    return value(UsePi());
  }

  /** @cond */  // DOXYGEN IGNORE
  // value if PI isn't involved
  static constexpr const unit_t<unit_type> value(std::false_type) noexcept {
    return unit_t<unit_type>(static_cast<double>(ratio::num) / ratio::den);
  }

  // value if PI *is* involved
  static constexpr const unit_t<unit_type> value(std::true_type) noexcept {
    return unit_t<unit_type>(
        (static_cast<double>(Base::_RATIO1::num) / Base::_RATIO1::den) +
        (static_cast<double>(Base::_RATIO2CONV::num) / Base::_RATIO2CONV::den) *
            std::pow(std::numbers::pi,
                     (static_cast<double>(Base::_PI_EXP::num) /
                      Base::_PI_EXP::den)));
  }
  /** @endcond */  // END DOXYGEN IGNORE
};

/**
 * @ingroup CompileTimeUnitManipulators
 * @brief subtracts two unit_value_t types at compile-time
 * @details The resulting unit will the the `unit_type` of `U1`
 * @tparam U1 left-hand `unit_value_t`
 * @tparam U2 right-hand `unit_value_t`
 * @note very similar in concept to `std::ratio_subtract`
 */
template <class U1, class U2>
struct unit_value_subtract
    : units::detail::unit_value_arithmetic<U1, U2>,
      units::detail::_unit_value_t<typename U1::unit_type> {
  /** @cond */  // DOXYGEN IGNORE
  using Base = units::detail::unit_value_arithmetic<U1, U2>;

  using unit_type = typename Base::_UNIT1;
  using ratio =
      std::ratio_subtract<typename Base::_RATIO1, typename Base::_RATIO2CONV>;

  static_assert(
      is_convertible_unit<typename Base::_UNIT1, typename Base::_UNIT2>,
      "Unit types are not compatible.");
  /** @endcond */  // END DOXYGEN IGNORE

  /**
   * @brief Value of difference
   * @details Returns the calculated value of the difference of `U1` and `U2`,
   *          in the same units as `U1`.
   * @returns Value of the difference in the appropriate units.
   */
  static constexpr const unit_t<unit_type> value() noexcept {
    using UsePi = std::integral_constant<bool, Base::_PI_EXP::num != 0>;
    return value(UsePi());
  }

  /** @cond */  // DOXYGEN IGNORE
  // value if PI isn't involved
  static constexpr const unit_t<unit_type> value(std::false_type) noexcept {
    return unit_t<unit_type>(static_cast<double>(ratio::num) / ratio::den);
  }

  // value if PI *is* involved
  static constexpr const unit_t<unit_type> value(std::true_type) noexcept {
    return unit_t<unit_type>(
        (static_cast<double>(Base::_RATIO1::num) / Base::_RATIO1::den) -
        (static_cast<double>(Base::_RATIO2CONV::num) / Base::_RATIO2CONV::den) *
            std::pow(std::numbers::pi,
                     (static_cast<double>(Base::_PI_EXP::num) /
                      Base::_PI_EXP::den)));
  }
  /** @endcond */  // END DOXYGEN IGNORE  };
};

/**
 * @ingroup CompileTimeUnitManipulators
 * @brief multiplies two unit_value_t types at compile-time
 * @details The resulting unit will the the `unit_type` of `U1 * U2`
 * @tparam U1 left-hand `unit_value_t`
 * @tparam U2 right-hand `unit_value_t`
 * @note very similar in concept to `std::ratio_multiply`
 */
template <class U1, class U2>
struct unit_value_multiply
    : units::detail::unit_value_arithmetic<U1, U2>,
      units::detail::_unit_value_t<typename std::conditional<
          is_convertible_unit<typename U1::unit_type, typename U2::unit_type>,
          compound_unit<squared<typename U1::unit_type>>,
          compound_unit<typename U1::unit_type,
                        typename U2::unit_type>>::type> {
  /** @cond */  // DOXYGEN IGNORE
  using Base = units::detail::unit_value_arithmetic<U1, U2>;

  using unit_type = std::conditional_t<
      is_convertible_unit<typename Base::_UNIT1, typename Base::_UNIT2>,
      compound_unit<squared<typename Base::_UNIT1>>,
      compound_unit<typename Base::_UNIT1, typename Base::_UNIT2>>;
  using ratio = std::conditional_t<
      is_convertible_unit<typename Base::_UNIT1, typename Base::_UNIT2>,
      std::ratio_multiply<typename Base::_RATIO1, typename Base::_RATIO2CONV>,
      std::ratio_multiply<typename Base::_RATIO1, typename Base::_RATIO2>>;
  /** @endcond */  // END DOXYGEN IGNORE

  /**
   * @brief Value of product
   * @details Returns the calculated value of the product of `U1` and `U2`, in
   *          units of `U1 x U2`.
   * @returns Value of the product in the appropriate units.
   */
  static constexpr const unit_t<unit_type> value() noexcept {
    using UsePi = std::integral_constant<bool, Base::_PI_EXP::num != 0>;
    return value(UsePi());
  }

  /** @cond */  // DOXYGEN IGNORE
  // value if PI isn't involved
  static constexpr const unit_t<unit_type> value(std::false_type) noexcept {
    return unit_t<unit_type>(static_cast<double>(ratio::num) / ratio::den);
  }

  // value if PI *is* involved
  static constexpr const unit_t<unit_type> value(std::true_type) noexcept {
    return unit_t<unit_type>(
        (static_cast<double>(ratio::num) / ratio::den) *
        std::pow(std::numbers::pi, (static_cast<double>(Base::_PI_EXP::num) /
                                    Base::_PI_EXP::den)));
  }
  /** @endcond */  // END DOXYGEN IGNORE
};

/**
 * @ingroup CompileTimeUnitManipulators
 * @brief divides two unit_value_t types at compile-time
 * @details The resulting unit will the the `unit_type` of `U1`
 * @tparam U1 left-hand `unit_value_t`
 * @tparam U2 right-hand `unit_value_t`
 * @note very similar in concept to `std::ratio_divide`
 */
template <class U1, class U2>
struct unit_value_divide
    : units::detail::unit_value_arithmetic<U1, U2>,
      units::detail::_unit_value_t<typename std::conditional<
          is_convertible_unit<typename U1::unit_type, typename U2::unit_type>,
          dimensionless::scalar,
          compound_unit<typename U1::unit_type,
                        inverse<typename U2::unit_type>>>::type> {
  /** @cond */  // DOXYGEN IGNORE
  using Base = units::detail::unit_value_arithmetic<U1, U2>;

  using unit_type = std::conditional_t<
      is_convertible_unit<typename Base::_UNIT1, typename Base::_UNIT2>,
      dimensionless::scalar,
      compound_unit<typename Base::_UNIT1, inverse<typename Base::_UNIT2>>>;
  using ratio = std::conditional_t<
      is_convertible_unit<typename Base::_UNIT1, typename Base::_UNIT2>,
      std::ratio_divide<typename Base::_RATIO1, typename Base::_RATIO2CONV>,
      std::ratio_divide<typename Base::_RATIO1, typename Base::_RATIO2>>;
  /** @endcond */  // END DOXYGEN IGNORE

  /**
   * @brief Value of quotient
   * @details Returns the calculated value of the quotient of `U1` and `U2`, in
   *          units of `U1 x U2`.
   * @returns Value of the quotient in the appropriate units.
   */
  static constexpr const unit_t<unit_type> value() noexcept {
    using UsePi = std::integral_constant<bool, Base::_PI_EXP::num != 0>;
    return value(UsePi());
  }

  /** @cond */  // DOXYGEN IGNORE
  // value if PI isn't involved
  static constexpr const unit_t<unit_type> value(std::false_type) noexcept {
    return unit_t<unit_type>(static_cast<double>(ratio::num) / ratio::den);
  }

  // value if PI *is* involved
  static constexpr const unit_t<unit_type> value(std::true_type) noexcept {
    return unit_t<unit_type>(
        (static_cast<double>(ratio::num) / ratio::den) *
        std::pow(std::numbers::pi, (static_cast<double>(Base::_PI_EXP::num) /
                                    Base::_PI_EXP::den)));
  }
  /** @endcond */  // END DOXYGEN IGNORE
};

/**
 * @ingroup CompileTimeUnitManipulators
 * @brief raises unit_value_to a power at compile-time
 * @details The resulting unit will the `unit_type` of `U1` squared
 * @tparam U1 `unit_value_t` to take the exponentiation of.
 * @note very similar in concept to `units::math::pow`
 */
template <class U1, int power>
struct unit_value_power
    : units::detail::unit_value_arithmetic<U1, U1>,
      units::detail::_unit_value_t<typename units::detail::power_of_unit<
          power, typename U1::unit_type>::type> {
  /** @cond */  // DOXYGEN IGNORE
  using Base = units::detail::unit_value_arithmetic<U1, U1>;

  using unit_type =
      typename units::detail::power_of_unit<power, typename Base::_UNIT1>::type;
  using ratio =
      typename units::detail::power_of_ratio<power,
                                             typename Base::_RATIO1>::type;
  using pi_exponent =
      std::ratio_multiply<std::ratio<power>,
                          typename Base::_UNIT1::pi_exponent_ratio>;
  /** @endcond */  // END DOXYGEN IGNORE

  /**
   * @brief Value of exponentiation
   * @details Returns the calculated value of the exponentiation of `U1`, in
   *          units of `U1^power`.
   * @returns Value of the exponentiation in the appropriate units.
   */
  static constexpr const unit_t<unit_type> value() noexcept {
    using UsePi = std::integral_constant<bool, Base::_PI_EXP::num != 0>;
    return value(UsePi());
  }

  /** @cond */  // DOXYGEN IGNORE
  // value if PI isn't involved
  static constexpr const unit_t<unit_type> value(std::false_type) noexcept {
    return unit_t<unit_type>(static_cast<double>(ratio::num) / ratio::den);
  }

  // value if PI *is* involved
  static constexpr const unit_t<unit_type> value(std::true_type) noexcept {
    return unit_t<unit_type>(
        (static_cast<double>(ratio::num) / ratio::den) *
        std::pow(std::numbers::pi,
                 (static_cast<double>(pi_exponent::num) / pi_exponent::den)));
  }
  /** @endcond */  // END DOXYGEN IGNORE  };
};

/**
 * @ingroup CompileTimeUnitManipulators
 * @brief calculates square root of unit_value_t at compile-time
 * @details The resulting unit will the square root `unit_type` of `U1`
 * @tparam U1 `unit_value_t` to take the square root of.
 * @note very similar in concept to `units::ratio_sqrt`
 */
template <class U1, intmax_t Eps = 10000000000>
struct unit_value_sqrt
    : units::detail::unit_value_arithmetic<U1, U1>,
      units::detail::_unit_value_t<square_root<typename U1::unit_type, Eps>> {
  /** @cond */  // DOXYGEN IGNORE
  using Base = units::detail::unit_value_arithmetic<U1, U1>;

  using unit_type = square_root<typename Base::_UNIT1, Eps>;
  using ratio = ratio_sqrt<typename Base::_RATIO1, Eps>;
  using pi_exponent = ratio_sqrt<typename Base::_UNIT1::pi_exponent_ratio, Eps>;
  /** @endcond */  // END DOXYGEN IGNORE

  /**
   * @brief Value of square root
   * @details Returns the calculated value of the square root of `U1`, in units
   *          of `U1^1/2`.
   * @returns Value of the square root in the appropriate units.
   */
  static constexpr const unit_t<unit_type> value() noexcept {
    using UsePi = std::integral_constant<bool, Base::_PI_EXP::num != 0>;
    return value(UsePi());
  }

  /** @cond */  // DOXYGEN IGNORE
  // value if PI isn't involved
  static constexpr const unit_t<unit_type> value(std::false_type) noexcept {
    return unit_t<unit_type>(static_cast<double>(ratio::num) / ratio::den);
  }

  // value if PI *is* involved
  static constexpr const unit_t<unit_type> value(std::true_type) noexcept {
    return unit_t<unit_type>(
        (static_cast<double>(ratio::num) / ratio::den) *
        std::pow(std::numbers::pi,
                 (static_cast<double>(pi_exponent::num) / pi_exponent::den)));
  }
  /** @endcond */  // END DOXYGEN IGNORE
};

//----------------------------------
//  UNIT-ENABLED CMATH FUNCTIONS
//----------------------------------

/**
 * @brief namespace for unit-enabled versions of the `<cmath>` library
 * @details Includes trigonometric functions, exponential/log functions,
 *          rounding functions, etc.
 * @sa See `unit_t` for more information on unit type containers.
 */
namespace math {

//----------------------------------
//  MIN/MAX FUNCTIONS
//----------------------------------
// XXX: min/max are defined here instead of math.h to avoid a conflict with
// the "_min" user-defined literal in time.h.

template <class UnitTypeLhs, class UnitTypeRhs>
UnitTypeLhs(min)(const UnitTypeLhs& lhs, const UnitTypeRhs& rhs) {
  static_assert(is_convertible_unit_t<UnitTypeLhs, UnitTypeRhs>,
                "Unit types are not compatible.");
  UnitTypeLhs r(rhs);
  return (lhs < r ? lhs : r);
}

template <class UnitTypeLhs, class UnitTypeRhs>
UnitTypeLhs(max)(const UnitTypeLhs& lhs, const UnitTypeRhs& rhs) {
  static_assert(is_convertible_unit_t<UnitTypeLhs, UnitTypeRhs>,
                "Unit types are not compatible.");
  UnitTypeLhs r(rhs);
  return (lhs > r ? lhs : r);
}

}  // namespace math
}  // namespace units

namespace units::literals {}
using namespace units::literals;  // NOLINT

#if __has_include(<fmt/format.h>) && !defined(UNIT_LIB_DISABLE_FMT)
#include "units/formatter.h"
#endif
