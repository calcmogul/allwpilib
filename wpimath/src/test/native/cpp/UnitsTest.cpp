// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <array>
#include <chrono>
#include <concepts>
#include <numbers>
#include <string>
#include <type_traits>

#include <gtest/gtest.h>
#include <wpi/print.h>

#include "units/acceleration.h"
#include "units/angle.h"
#include "units/angular_acceleration.h"
#include "units/angular_jerk.h"
#include "units/angular_velocity.h"
#include "units/area.h"
#include "units/capacitance.h"
#include "units/charge.h"
#include "units/concentration.h"
#include "units/conductance.h"
#include "units/constants.h"
#include "units/current.h"
#include "units/data.h"
#include "units/data_transfer_rate.h"
#include "units/density.h"
#include "units/dimensionless.h"
#include "units/energy.h"
#include "units/force.h"
#include "units/frequency.h"
#include "units/illuminance.h"
#include "units/impedance.h"
#include "units/inductance.h"
#include "units/length.h"
#include "units/luminous_flux.h"
#include "units/luminous_intensity.h"
#include "units/magnetic_field_strength.h"
#include "units/magnetic_flux.h"
#include "units/mass.h"
#include "units/math.h"
#include "units/power.h"
#include "units/pressure.h"
#include "units/radiation.h"
#include "units/solid_angle.h"
#include "units/substance.h"
#include "units/temperature.h"
#include "units/time.h"
#include "units/torque.h"
#include "units/velocity.h"
#include "units/voltage.h"
#include "units/volume.h"

using namespace units::literals;

namespace {

class TypeTraitsTest : public testing::Test {};

class UnitManipulatorsTest : public testing::Test {};

class UnitContainerTest : public testing::Test {};

class UnitConversionTest : public testing::Test {};

class UnitMathTest : public testing::Test {};

class CompileTimeArithmeticTest : public testing::Test {};

class ConstexprTest : public testing::Test {};

class CaseStudiesTest : public testing::Test {
 protected:
  struct RightTriangle {
    using a = units::unit_value_t<units::meters, 3>;
    using b = units::unit_value_t<units::meters, 4>;
    using c = units::unit_value_sqrt<units::unit_value_add<
        units::unit_value_power<a, 2>, units::unit_value_power<b, 2>>>;
  };
};
}  // namespace

TEST_F(TypeTraitsTest, IsRatio) {
  EXPECT_TRUE(units::is_ratio<std::ratio<1>>);
  EXPECT_FALSE(units::is_ratio<double>);
}

TEST_F(TypeTraitsTest, RatioSqrt) {
  using rt2 = units::ratio_sqrt<std::ratio<2>>;
  EXPECT_LT(std::abs(std::sqrt(2.0) - rt2::num / static_cast<double>(rt2::den)),
            5e-9);

  using rt4 = units::ratio_sqrt<std::ratio<4>>;
  EXPECT_LT(std::abs(std::sqrt(4.0) - rt4::num / static_cast<double>(rt4::den)),
            5e-9);

  using rt10 = units::ratio_sqrt<std::ratio<10>>;
  EXPECT_LT(
      std::abs(std::sqrt(10.0) - rt10::num / static_cast<double>(rt10::den)),
      5e-9);

  using rt30 = units::ratio_sqrt<std::ratio<30>>;
  EXPECT_LT(
      std::abs(std::sqrt(30.0) - rt30::num / static_cast<double>(rt30::den)),
      5e-9);

  using rt61 = units::ratio_sqrt<std::ratio<61>>;
  EXPECT_LT(
      std::abs(std::sqrt(61.0) - rt61::num / static_cast<double>(rt61::den)),
      5e-9);

  using rt100 = units::ratio_sqrt<std::ratio<100>>;
  EXPECT_LT(
      std::abs(std::sqrt(100.0) - rt100::num / static_cast<double>(rt100::den)),
      5e-9);

  using rt1000 = units::ratio_sqrt<std::ratio<1000>>;
  EXPECT_LT(std::abs(std::sqrt(1000.0) -
                     rt1000::num / static_cast<double>(rt1000::den)),
            5e-9);

  using rt10000 = units::ratio_sqrt<std::ratio<10000>>;
  EXPECT_LT(std::abs(std::sqrt(10000.0) -
                     rt10000::num / static_cast<double>(rt10000::den)),
            5e-9);
}

TEST_F(TypeTraitsTest, IsUnitValue) {
  EXPECT_FALSE(units::is_unit<std::ratio<1>>);
  EXPECT_FALSE(units::is_unit<double>);
  EXPECT_TRUE(units::is_unit<units::meters>);
  EXPECT_TRUE(units::is_unit<units::feet>);
  EXPECT_TRUE(units::is_unit<units::degrees_squared>);
  EXPECT_FALSE(units::is_unit<units::meter_t>);
}

TEST_F(TypeTraitsTest, IsUnitType) {
  EXPECT_FALSE(units::is_unit_t<std::ratio<1>>);
  EXPECT_FALSE(units::is_unit_t<double>);
  EXPECT_FALSE(units::is_unit_t<units::meters>);
  EXPECT_FALSE(units::is_unit_t<units::feet>);
  EXPECT_FALSE(units::is_unit_t<units::degrees_squared>);
  EXPECT_TRUE(units::is_unit_t<units::meter_t>);
}

TEST_F(TypeTraitsTest, UnitTraits) {
  EXPECT_TRUE(!(std::same_as<void, units::meters::conversion_ratio>));
}

TEST_F(TypeTraitsTest, UnitTypeTraits) {
  EXPECT_TRUE((std::same_as<double, units::meter_t::underlying_type>));
}

TEST_F(TypeTraitsTest, IsConvertibleUnit) {
  EXPECT_TRUE((units::is_convertible_unit<units::meters, units::meters>));
  EXPECT_TRUE(
      (units::is_convertible_unit<units::meters, units::astronicalUnits>));
  EXPECT_TRUE((units::is_convertible_unit<units::meters, units::parsecs>));

  EXPECT_TRUE((units::is_convertible_unit<units::meters, units::meters>));
  EXPECT_TRUE(
      (units::is_convertible_unit<units::astronicalUnits, units::meters>));
  EXPECT_TRUE((units::is_convertible_unit<units::parsecs, units::meters>));
  EXPECT_TRUE((units::is_convertible_unit<units::years, units::weeks>));

  EXPECT_FALSE((units::is_convertible_unit<units::meters, units::seconds>));
  EXPECT_FALSE((units::is_convertible_unit<units::seconds, units::meters>));
  EXPECT_FALSE((units::is_convertible_unit<units::years, units::meters>));
}

TEST_F(TypeTraitsTest, Inverse) {
  double test;

  using htz = units::inverse<units::seconds>;
  bool shouldBeTrue = std::same_as<units::htz, units::hertz>;
  EXPECT_TRUE(shouldBeTrue);

  test = units::convert<units::inverse<units::celsius>,
                        units::inverse<units::fahrenheit>>(1.0);
  EXPECT_NEAR(5.0 / 9.0, test, 5.0e-5);

  test = units::convert<units::inverse<units::kelvin>,
                        units::inverse<units::fahrenheit>>(6.0);
  EXPECT_NEAR(10.0 / 3.0, test, 5.0e-5);
}

TEST_F(TypeTraitsTest, BaseUnitOf) {
  using base = units::traits::base_unit_of<units::years>;
  bool shouldBeTrue = std::same_as<base, units::category::time_unit>;

  EXPECT_TRUE(shouldBeTrue);
}

TEST_F(TypeTraitsTest, HasLinearScale) {
  EXPECT_TRUE((units::has_linear_scale<units::scalar_t>));
  EXPECT_TRUE((units::has_linear_scale<units::meter_t>));
  EXPECT_TRUE((units::has_linear_scale<units::foot_t>));
  EXPECT_TRUE((units::has_linear_scale<units::watt_t>));
  EXPECT_TRUE((units::has_linear_scale<units::meters_per_second_t>));
  EXPECT_FALSE((units::has_linear_scale<units::dB_t>));
}

TEST_F(TypeTraitsTest, HasDecibelScale) {
  EXPECT_FALSE((units::has_decibel_scale<units::scalar_t>));
  EXPECT_FALSE((units::has_decibel_scale<units::meter_t>));
  EXPECT_FALSE((units::has_decibel_scale<units::foot_t>));
  EXPECT_TRUE((units::has_decibel_scale<units::dB_t>));
  EXPECT_TRUE((units::has_decibel_scale<units::dBW_t>));
}

TEST_F(TypeTraitsTest, IsSameScale) {
  EXPECT_TRUE((units::is_same_scale<units::scalar_t, units ::dimensionless_t>));
  EXPECT_TRUE((units::is_same_scale<units::dB_t, units::dBW_t>));
  EXPECT_FALSE((units::is_same_scale<units::dB_t, units::scalar_t>));
}

TEST_F(TypeTraitsTest, IsDimensionlessUnit) {
  EXPECT_TRUE((units::traits::is_dimensionless_unit_v<units::scalar_t>));
  EXPECT_TRUE((units::traits::is_dimensionless_unit_v<const units::scalar_t>));
  EXPECT_TRUE((units::traits::is_dimensionless_unit_v<const units::scalar_t&>));
  EXPECT_TRUE((units::traits::is_dimensionless_unit_v<units::dimensionless_t>));
  EXPECT_TRUE((units::traits::is_dimensionless_unit_v<units::dB_t>));
  EXPECT_TRUE((units::traits::is_dimensionless_unit_v<units::ppm_t>));
  EXPECT_FALSE((units::traits::is_dimensionless_unit_v<units::meter_t>));
  EXPECT_FALSE((units::traits::is_dimensionless_unit_v<units::dBW_t>));
}

TEST_F(TypeTraitsTest, IsLengthUnit) {
  EXPECT_TRUE((units::traits::is_length_unit_v<units::meter>));
  EXPECT_TRUE((units::traits::is_length_unit_v<units::cubit>));
  EXPECT_FALSE((units::traits::is_length_unit_v<units::year>));
  EXPECT_FALSE((units::traits::is_length_unit_v<double>));

  EXPECT_TRUE((units::traits::is_length_unit_v<units::meter_t>));
  EXPECT_TRUE((units::traits::is_length_unit_v<const units::meter_t>));
  EXPECT_TRUE((units::traits::is_length_unit_v<const units::meter_t&>));
  EXPECT_TRUE((units::traits::is_length_unit_v<units::cubit_t>));
  EXPECT_FALSE((units::traits::is_length_unit_v<units::year_t>));
}

TEST_F(TypeTraitsTest, IsMassUnit) {
  EXPECT_TRUE((units::traits::is_mass_unit_v<units::kilogram>));
  EXPECT_TRUE((units::traits::is_mass_unit_v<units::stone>));
  EXPECT_FALSE((units::traits::is_mass_unit_v<units::meter>));
  EXPECT_FALSE((units::traits::is_mass_unit_v<double>));

  EXPECT_TRUE((units::traits::is_mass_unit_v<units::kilogram_t>));
  EXPECT_TRUE((units::traits::is_mass_unit_v<const units::kilogram_t>));
  EXPECT_TRUE((units::traits::is_mass_unit_v<const units::kilogram_t&>));
  EXPECT_TRUE((units::traits::is_mass_unit_v<units::stone_t>));
  EXPECT_FALSE((units::traits::is_mass_unit_v<units::meter_t>));
}

TEST_F(TypeTraitsTest, IsTimeUnit) {
  EXPECT_TRUE((units::traits::is_time_unit_v<units::second>));
  EXPECT_TRUE((units::traits::is_time_unit_v<units::year>));
  EXPECT_FALSE((units::traits::is_time_unit_v<units::meter>));
  EXPECT_FALSE((units::traits::is_time_unit_v<double>));

  EXPECT_TRUE((units::traits::is_time_unit_v<units::second_t>));
  EXPECT_TRUE((units::traits::is_time_unit_v<const units::second_t>));
  EXPECT_TRUE((units::traits::is_time_unit_v<const units::second_t&>));
  EXPECT_TRUE((units::traits::is_time_unit_v<units::year_t>));
  EXPECT_FALSE((units::traits::is_time_unit_v<units::meter_t>));
}

TEST_F(TypeTraitsTest, IsAngleUnit) {
  EXPECT_TRUE((units::traits::is_angle_unit_v<units::angle::radian>));
  EXPECT_TRUE((units::traits::is_angle_unit_v<units::angle::degree>));
  EXPECT_FALSE((units::traits::is_angle_unit_v<units::watt>));
  EXPECT_FALSE((units::traits::is_angle_unit_v<double>));

  EXPECT_TRUE((units::traits::is_angle_unit_v<units::angle::radian_t>));
  EXPECT_TRUE((units::traits::is_angle_unit_v<const units::angle::radian_t>));
  EXPECT_TRUE((units::traits::is_angle_unit_v<const units::angle::radian_t&>));
  EXPECT_TRUE((units::traits::is_angle_unit_v<units::angle::degree_t>));
  EXPECT_FALSE((units::traits::is_angle_unit_v<units::watt_t>));
}

TEST_F(TypeTraitsTest, IsCurrentUnit) {
  EXPECT_TRUE((units::traits::is_current_unit_v<units::current::ampere>));
  EXPECT_FALSE((units::traits::is_current_unit_v<units::volt>));
  EXPECT_FALSE((units::traits::is_current_unit_v<double>));

  EXPECT_TRUE((units::traits::is_current_unit_v<units::current::ampere_t>));
  EXPECT_TRUE(
      (units::traits::is_current_unit_v<const units::current::ampere_t>));
  EXPECT_TRUE(
      (units::traits::is_current_unit_v<const units::current::ampere_t&>));
  EXPECT_FALSE((units::traits::is_current_unit_v<units::volt_t>));
}

TEST_F(TypeTraitsTest, IsTemperatureUnit) {
  EXPECT_TRUE((units::traits::is_temperature_unit_v<units::fahrenheit>));
  EXPECT_TRUE((units::traits::is_temperature_unit_v<units::kelvin>));
  EXPECT_FALSE((units::traits::is_temperature_unit_v<units::cubit>));
  EXPECT_FALSE((units::traits::is_temperature_unit_v<double>));

  EXPECT_TRUE((units::traits::is_temperature_unit_v<units::fahrenheit_t>));
  EXPECT_TRUE(
      (units::traits::is_temperature_unit_v<const units::fahrenheit_t>));
  EXPECT_TRUE(
      (units::traits::is_temperature_unit_v<const units::fahrenheit_t&>));
  EXPECT_TRUE((units::traits::is_temperature_unit_v<units::kelvin_t>));
  EXPECT_FALSE((units::traits::is_temperature_unit_v<units::cubit_t>));
}

TEST_F(TypeTraitsTest, IsSubstanceUnit) {
  EXPECT_TRUE((units::traits::is_substance_unit_v<units::substance::mol>));
  EXPECT_FALSE((units::traits::is_substance_unit_v<units::year>));
  EXPECT_FALSE((units::traits::is_substance_unit_v<double>));

  EXPECT_TRUE((units::traits::is_substance_unit_v<units::substance::mole_t>));
  EXPECT_TRUE(
      (units::traits::is_substance_unit_v<const units::substance::mole_t>));
  EXPECT_TRUE(
      (units::traits::is_substance_unit_v<const units::substance::mole_t&>));
  EXPECT_FALSE((units::traits::is_substance_unit_v<units::year_t>));
}

TEST_F(TypeTraitsTest, IsLuminousIntensityUnit) {
  EXPECT_TRUE((units::traits::is_luminous_intensity_unit_v<units::candela>));
  EXPECT_FALSE(
      (units::traits::is_luminous_intensity_unit_v<units::radiation::rad>));
  EXPECT_FALSE((units::traits::is_luminous_intensity_unit_v<double>));

  EXPECT_TRUE((units::traits::is_luminous_intensity_unit_v<units::candela_t>));
  EXPECT_TRUE(
      (units::traits::is_luminous_intensity_unit_v<const units::candela_t>));
  EXPECT_TRUE(
      (units::traits::is_luminous_intensity_unit_v<const units::candela_t&>));
  EXPECT_FALSE((units::traits::is_luminous_intensity_unit_v<units::rad_t>));
}

TEST_F(TypeTraitsTest, IsSolidAngleUnit) {
  EXPECT_TRUE((units::traits::is_solid_angle_unit_v<units::steradian>));
  EXPECT_TRUE((units::traits::is_solid_angle_unit_v<units::degree_squared>));
  EXPECT_FALSE((units::traits::is_solid_angle_unit_v<units::angle::degree>));
  EXPECT_FALSE((units::traits::is_solid_angle_unit_v<double>));

  EXPECT_TRUE((units::traits::is_solid_angle_unit_v<units::steradian_t>));
  EXPECT_TRUE((units::traits::is_solid_angle_unit_v<const units::steradian_t>));
  EXPECT_TRUE(
      (units::traits::is_solid_angle_unit_v<const units::degree_squared_t&>));
  EXPECT_FALSE((units::traits::is_solid_angle_unit_v<units::angle::degree_t>));
}

TEST_F(TypeTraitsTest, IsFrequencyUnit) {
  EXPECT_TRUE((units::traits::is_frequency_unit_v<units::hertz>));
  EXPECT_FALSE((units::traits::is_frequency_unit_v<units::second>));
  EXPECT_FALSE((units::traits::is_frequency_unit_v<double>));

  EXPECT_TRUE((units::traits::is_frequency_unit_v<units::hertz_t>));
  EXPECT_TRUE((units::traits::is_frequency_unit_v<const units::hertz_t>));
  EXPECT_TRUE((units::traits::is_frequency_unit_v<const units::hertz_t&>));
  EXPECT_FALSE((units::traits::is_frequency_unit_v<units::second_t>));
}

TEST_F(TypeTraitsTest, IsVelocityUnit) {
  EXPECT_TRUE((units::traits::is_velocity_unit_v<units::meters_per_second>));
  EXPECT_TRUE((units::traits::is_velocity_unit_v<units::miles_per_hour>));
  EXPECT_FALSE(
      (units::traits::is_velocity_unit_v<units::meters_per_second_squared>));
  EXPECT_FALSE((units::traits::is_velocity_unit_v<double>));

  EXPECT_TRUE((units::traits::is_velocity_unit_v<units::meters_per_second_t>));
  EXPECT_TRUE(
      (units::traits::is_velocity_unit_v<const units::meters_per_second_t>));
  EXPECT_TRUE(
      (units::traits::is_velocity_unit_v<const units::meters_per_second_t&>));
  EXPECT_TRUE((units::traits::is_velocity_unit_v<units::miles_per_hour_t>));
  EXPECT_FALSE(
      (units::traits::is_velocity_unit_v<units::meters_per_second_squared_t>));
}

TEST_F(TypeTraitsTest, IsAccelerationUnit) {
  EXPECT_TRUE((
      units::traits::is_acceleration_unit_v<units::meters_per_second_squared>));
  EXPECT_TRUE((units::traits::is_acceleration_unit_v<
               units::acceleration::standard_gravity>));
  EXPECT_FALSE((units::traits::is_acceleration_unit_v<units::inch>));
  EXPECT_FALSE((units::traits::is_acceleration_unit_v<double>));
}

TEST_F(TypeTraitsTest, IsForceUnit) {
  EXPECT_TRUE((units::traits::is_force_unit_v<units::force::newton>));
  EXPECT_TRUE((units::traits::is_force_unit_v<units::force::dynes>));
  EXPECT_FALSE((units::traits::is_force_unit_v<units::meter>));
  EXPECT_FALSE((units::traits::is_force_unit_v<double>));

  EXPECT_TRUE((units::traits::is_force_unit_v<units::force::newton_t>));
  EXPECT_TRUE((units::traits::is_force_unit_v<const units::force::newton_t>));
  EXPECT_TRUE((units::traits::is_force_unit_v<const units::force::newton_t&>));
  EXPECT_TRUE((units::traits::is_force_unit_v<units::force::dyne_t>));
  EXPECT_FALSE((units::traits::is_force_unit_v<units::watt_t>));
}

TEST_F(TypeTraitsTest, IsPressureUnit) {
  EXPECT_TRUE((units::traits::is_pressure_unit_v<units::pressure::pascals>));
  EXPECT_TRUE((units::traits::is_pressure_unit_v<units::atmosphere>));
  EXPECT_FALSE((units::traits::is_pressure_unit_v<units::year>));
  EXPECT_FALSE((units::traits::is_pressure_unit_v<double>));

  EXPECT_TRUE((units::traits::is_pressure_unit_v<units::pascal_t>));
  EXPECT_TRUE((units::traits::is_pressure_unit_v<const units::pascal_t>));
  EXPECT_TRUE((units::traits::is_pressure_unit_v<const units::pascal_t&>));
  EXPECT_TRUE((units::traits::is_pressure_unit_v<units::atmosphere_t>));
  EXPECT_FALSE((units::traits::is_pressure_unit_v<units::year_t>));
}

TEST_F(TypeTraitsTest, IsChargeUnit) {
  EXPECT_TRUE((units::traits::is_charge_unit_v<units::coulomb>));
  EXPECT_FALSE((units::traits::is_charge_unit_v<units::watt>));
  EXPECT_FALSE((units::traits::is_charge_unit_v<double>));

  EXPECT_TRUE((units::traits::is_charge_unit_v<units::coulomb_t>));
  EXPECT_TRUE((units::traits::is_charge_unit_v<const units::coulomb_t>));
  EXPECT_TRUE((units::traits::is_charge_unit_v<const units::coulomb_t&>));
  EXPECT_FALSE((units::traits::is_charge_unit_v<units::watt_t>));
}

TEST_F(TypeTraitsTest, IsEnergyUnit) {
  EXPECT_TRUE((units::traits::is_energy_unit_v<units::joule>));
  EXPECT_TRUE((units::traits::is_energy_unit_v<units::calorie>));
  EXPECT_FALSE((units::traits::is_energy_unit_v<units::watt>));
  EXPECT_FALSE((units::traits::is_energy_unit_v<double>));

  EXPECT_TRUE((units::traits::is_energy_unit_v<units::joule_t>));
  EXPECT_TRUE((units::traits::is_energy_unit_v<const units::joule_t>));
  EXPECT_TRUE((units::traits::is_energy_unit_v<const units::joule_t&>));
  EXPECT_TRUE((units::traits::is_energy_unit_v<units::calorie_t>));
  EXPECT_FALSE((units::traits::is_energy_unit_v<units::watt_t>));
}

TEST_F(TypeTraitsTest, IsPowerUnit) {
  EXPECT_TRUE((units::traits::is_power_unit_v<units::watt>));
  EXPECT_FALSE((units::traits::is_power_unit_v<units::henry>));
  EXPECT_FALSE((units::traits::is_power_unit_v<double>));

  EXPECT_TRUE((units::traits::is_power_unit_v<units::watt_t>));
  EXPECT_TRUE((units::traits::is_power_unit_v<const units::watt_t>));
  EXPECT_TRUE((units::traits::is_power_unit_v<const units::watt_t&>));
  EXPECT_FALSE((units::traits::is_power_unit_v<units::henry_t>));
}

TEST_F(TypeTraitsTest, IsVoltageUnit) {
  EXPECT_TRUE((units::traits::is_voltage_unit_v<units::volt>));
  EXPECT_FALSE((units::traits::is_voltage_unit_v<units::henry>));
  EXPECT_FALSE((units::traits::is_voltage_unit_v<double>));

  EXPECT_TRUE((units::traits::is_voltage_unit_v<units::volt_t>));
  EXPECT_TRUE((units::traits::is_voltage_unit_v<const units::volt_t>));
  EXPECT_TRUE((units::traits::is_voltage_unit_v<const units::volt_t&>));
  EXPECT_FALSE((units::traits::is_voltage_unit_v<units::henry_t>));
}

TEST_F(TypeTraitsTest, IsCapacitanceUnit) {
  EXPECT_TRUE((units::traits::is_capacitance_unit_v<units::farad>));
  EXPECT_FALSE((units::traits::is_capacitance_unit_v<units::ohm>));
  EXPECT_FALSE((units::traits::is_capacitance_unit_v<double>));

  EXPECT_TRUE((units::traits::is_capacitance_unit_v<units::farad_t>));
  EXPECT_TRUE((units::traits::is_capacitance_unit_v<const units::farad_t>));
  EXPECT_TRUE((units::traits::is_capacitance_unit_v<const units::farad_t&>));
  EXPECT_FALSE((units::traits::is_capacitance_unit_v<units::ohm_t>));
}

TEST_F(TypeTraitsTest, IsImpedanceUnit) {
  EXPECT_TRUE((units::traits::is_impedance_unit_v<units::ohm>));
  EXPECT_FALSE((units::traits::is_impedance_unit_v<units::farad>));
  EXPECT_FALSE((units::traits::is_impedance_unit_v<double>));

  EXPECT_TRUE((units::traits::is_impedance_unit_v<units::ohm_t>));
  EXPECT_TRUE((units::traits::is_impedance_unit_v<const units::ohm_t>));
  EXPECT_TRUE((units::traits::is_impedance_unit_v<const units::ohm_t&>));
  EXPECT_FALSE((units::traits::is_impedance_unit_v<units::farad_t>));
}

TEST_F(TypeTraitsTest, IsConductanceUnit) {
  EXPECT_TRUE((units::traits::is_conductance_unit_v<units::siemens>));
  EXPECT_FALSE((units::traits::is_conductance_unit_v<units::volt>));
  EXPECT_FALSE((units::traits::is_conductance_unit_v<double>));

  EXPECT_TRUE((units::traits::is_conductance_unit_v<units::siemens_t>));
  EXPECT_TRUE((units::traits::is_conductance_unit_v<const units::siemens_t>));
  EXPECT_TRUE((units::traits::is_conductance_unit_v<const units::siemens_t&>));
  EXPECT_FALSE((units::traits::is_conductance_unit_v<units::volt_t>));
}

TEST_F(TypeTraitsTest, IsMagneticFluxUnit) {
  EXPECT_TRUE((units::traits::is_magnetic_flux_unit_v<units::weber>));
  EXPECT_TRUE((units::traits::is_magnetic_flux_unit_v<units::maxwell>));
  EXPECT_FALSE((units::traits::is_magnetic_flux_unit_v<units::inch>));
  EXPECT_FALSE((units::traits::is_magnetic_flux_unit_v<double>));

  EXPECT_TRUE((units::traits::is_magnetic_flux_unit_v<units::weber_t>));
  EXPECT_TRUE((units::traits::is_magnetic_flux_unit_v<const units::weber_t>));
  EXPECT_TRUE((units::traits::is_magnetic_flux_unit_v<const units::weber_t&>));
  EXPECT_TRUE((units::traits::is_magnetic_flux_unit_v<units::maxwell_t>));
  EXPECT_FALSE((units::traits::is_magnetic_flux_unit_v<units::inch_t>));
}

TEST_F(TypeTraitsTest, IsMagneticFieldStrengthUnit) {
  EXPECT_TRUE((units::traits::is_magnetic_field_strength_unit_v<
               units::magnetic_field_strength::tesla>));
  EXPECT_TRUE((units::traits::is_magnetic_field_strength_unit_v<units::gauss>));
  EXPECT_FALSE((units::traits::is_magnetic_field_strength_unit_v<units::volt>));
  EXPECT_FALSE((units::traits::is_magnetic_field_strength_unit_v<double>));

  EXPECT_TRUE(
      (units::traits::is_magnetic_field_strength_unit_v<units::tesla_t>));
  EXPECT_TRUE(
      (units::traits::is_magnetic_field_strength_unit_v<const units::tesla_t>));
  EXPECT_TRUE((
      units::traits::is_magnetic_field_strength_unit_v<const units::tesla_t&>));
  EXPECT_TRUE(
      (units::traits::is_magnetic_field_strength_unit_v<units::gauss_t>));
  EXPECT_FALSE(
      (units::traits::is_magnetic_field_strength_unit_v<units::volt_t>));
}

TEST_F(TypeTraitsTest, IsInductanceUnit) {
  EXPECT_TRUE((units::traits::is_inductance_unit_v<units::henry>));
  EXPECT_FALSE((units::traits::is_inductance_unit_v<units::farad>));
  EXPECT_FALSE((units::traits::is_inductance_unit_v<double>));

  EXPECT_TRUE((units::traits::is_inductance_unit_v<units::henry_t>));
  EXPECT_TRUE((units::traits::is_inductance_unit_v<const units::henry_t>));
  EXPECT_TRUE((units::traits::is_inductance_unit_v<const units::henry_t&>));
  EXPECT_FALSE((units::traits::is_inductance_unit_v<units::farad_t>));
}

TEST_F(TypeTraitsTest, IsLuminousFluxUnit) {
  EXPECT_TRUE((units::traits::is_luminous_flux_unit_v<units::lumen>));
  EXPECT_FALSE((units::traits::is_luminous_flux_unit_v<units::pound>));
  EXPECT_FALSE((units::traits::is_luminous_flux_unit_v<double>));

  EXPECT_TRUE((units::traits::is_luminous_flux_unit_v<units::lumen_t>));
  EXPECT_TRUE((units::traits::is_luminous_flux_unit_v<const units::lumen_t>));
  EXPECT_TRUE((units::traits::is_luminous_flux_unit_v<const units::lumen_t&>));
  EXPECT_FALSE((units::traits::is_luminous_flux_unit_v<units::pound_t>));
}

TEST_F(TypeTraitsTest, IsIlluminanceUnit) {
  EXPECT_TRUE(
      (units::traits::is_illuminance_unit_v<units::illuminance::footcandle>));
  EXPECT_TRUE((units::traits::is_illuminance_unit_v<units::illuminance::lux>));
  EXPECT_FALSE((units::traits::is_illuminance_unit_v<units::meter>));
  EXPECT_FALSE((units::traits::is_illuminance_unit_v<double>));

  EXPECT_TRUE((units::traits::is_illuminance_unit_v<units::footcandle_t>));
  EXPECT_TRUE(
      (units::traits::is_illuminance_unit_v<const units::footcandle_t>));
  EXPECT_TRUE(
      (units::traits::is_illuminance_unit_v<const units::footcandle_t&>));
  EXPECT_TRUE((units::traits::is_illuminance_unit_v<units::lux_t>));
  EXPECT_FALSE((units::traits::is_illuminance_unit_v<units::meter_t>));
}

TEST_F(TypeTraitsTest, IsRadioactivityUnit) {
  EXPECT_TRUE((units::traits::is_radioactivity_unit_v<units::becquerel>));
  EXPECT_FALSE((units::traits::is_radioactivity_unit_v<units::year>));
  EXPECT_FALSE((units::traits::is_radioactivity_unit_v<double>));

  EXPECT_TRUE((units::traits::is_radioactivity_unit_v<units::becquerel_t>));
  EXPECT_TRUE(
      (units::traits::is_radioactivity_unit_v<const units::becquerel_t>));
  EXPECT_TRUE(
      (units::traits::is_radioactivity_unit_v<const units::becquerel_t&>));
  EXPECT_FALSE((units::traits::is_radioactivity_unit_v<units::year_t>));
}

TEST_F(TypeTraitsTest, IsTorqueUnit) {
  EXPECT_TRUE((units::traits::is_torque_unit_v<units::torque::newton_meter>));
  EXPECT_TRUE((units::traits::is_torque_unit_v<units::torque::foot_pound>));
  EXPECT_FALSE((units::traits::is_torque_unit_v<units::volume::cubic_meter>));
  EXPECT_FALSE((units::traits::is_torque_unit_v<double>));

  EXPECT_TRUE((units::traits::is_torque_unit_v<units::torque::newton_meter_t>));
  EXPECT_TRUE(
      (units::traits::is_torque_unit_v<const units::torque::newton_meter_t>));
  EXPECT_TRUE(
      (units::traits::is_torque_unit_v<const units::torque::newton_meter_t&>));
  EXPECT_TRUE((units::traits::is_torque_unit_v<units::torque::foot_pound_t>));
  EXPECT_FALSE((units::traits::is_torque_unit_v<units::volume::cubic_meter_t>));
}

TEST_F(TypeTraitsTest, IsAreaUnit) {
  EXPECT_TRUE((units::traits::is_area_unit_v<units::square_meter>));
  EXPECT_TRUE((units::traits::is_area_unit_v<units::hectare>));
  EXPECT_FALSE((units::traits::is_area_unit_v<units::astronicalUnit>));
  EXPECT_FALSE((units::traits::is_area_unit_v<double>));

  EXPECT_TRUE((units::traits::is_area_unit_v<units::square_meter_t>));
  EXPECT_TRUE((units::traits::is_area_unit_v<const units::square_meter_t>));
  EXPECT_TRUE((units::traits::is_area_unit_v<const units::square_meter_t&>));
  EXPECT_TRUE((units::traits::is_area_unit_v<units::hectare_t>));
  EXPECT_FALSE((units::traits::is_area_unit_v<units::astronicalUnit_t>));
}

TEST_F(TypeTraitsTest, IsVolumeUnit) {
  EXPECT_TRUE((units::traits::is_volume_unit_v<units::cubic_meter>));
  EXPECT_TRUE((units::traits::is_volume_unit_v<units::cubic_foot>));
  EXPECT_FALSE((units::traits::is_volume_unit_v<units::square_feet>));
  EXPECT_FALSE((units::traits::is_volume_unit_v<double>));

  EXPECT_TRUE((units::traits::is_volume_unit_v<units::cubic_meter_t>));
  EXPECT_TRUE((units::traits::is_volume_unit_v<const units::cubic_meter_t>));
  EXPECT_TRUE((units::traits::is_volume_unit_v<const units::cubic_meter_t&>));
  EXPECT_TRUE((units::traits::is_volume_unit_v<units::cubic_inch_t>));
  EXPECT_FALSE((units::traits::is_volume_unit_v<units::foot_t>));
}

TEST_F(TypeTraitsTest, IsDensityUnit) {
  EXPECT_TRUE(
      (units::traits::is_density_unit_v<units::kilograms_per_cubic_meter>));
  EXPECT_TRUE((units::traits::is_density_unit_v<units::ounces_per_cubic_foot>));
  EXPECT_FALSE((units::traits::is_density_unit_v<units::year>));
  EXPECT_FALSE((units::traits::is_density_unit_v<double>));

  EXPECT_TRUE(
      (units::traits::is_density_unit_v<units::kilograms_per_cubic_meter_t>));
  EXPECT_TRUE((units::traits::is_density_unit_v<
               const units::kilograms_per_cubic_meter_t>));
  EXPECT_TRUE((units::traits::is_density_unit_v<
               const units::kilograms_per_cubic_meter_t&>));
  EXPECT_TRUE(
      (units::traits::is_density_unit_v<units::ounces_per_cubic_foot_t>));
  EXPECT_FALSE((units::traits::is_density_unit_v<units::year_t>));
}

TEST_F(TypeTraitsTest, IsDataUnit) {
  EXPECT_TRUE((units::traits::is_data_unit_v<units::bit>));
  EXPECT_TRUE((units::traits::is_data_unit_v<units::byte>));
  EXPECT_TRUE((units::traits::is_data_unit_v<units::exabit>));
  EXPECT_TRUE((units::traits::is_data_unit_v<units::exabyte>));
  EXPECT_FALSE((units::traits::is_data_unit_v<units::year>));
  EXPECT_FALSE((units::traits::is_data_unit_v<double>));

  EXPECT_TRUE((units::traits::is_data_unit_v<units::bit_t>));
  EXPECT_TRUE((units::traits::is_data_unit_v<const units::bit_t>));
  EXPECT_TRUE((units::traits::is_data_unit_v<const units::bit_t&>));
  EXPECT_TRUE((units::traits::is_data_unit_v<units::byte_t>));
  EXPECT_FALSE((units::traits::is_data_unit_v<units::year_t>));
}

TEST_F(TypeTraitsTest, IsDataTransferRateUnit) {
  EXPECT_TRUE((units::traits::is_data_transfer_rate_unit_v<units::Gbps>));
  EXPECT_TRUE((units::traits::is_data_transfer_rate_unit_v<units::GBps>));
  EXPECT_FALSE((units::traits::is_data_transfer_rate_unit_v<units::year>));
  EXPECT_FALSE((units::traits::is_data_transfer_rate_unit_v<double>));

  EXPECT_TRUE((units::traits::is_data_transfer_rate_unit_v<
               units::gigabits_per_second_t>));
  EXPECT_TRUE((units::traits::is_data_transfer_rate_unit_v<
               const units::gigabytes_per_second_t>));
  EXPECT_TRUE((units::traits::is_data_transfer_rate_unit_v<
               const units::gigabytes_per_second_t&>));
  EXPECT_TRUE((units::traits::is_data_transfer_rate_unit_v<
               units::gigabytes_per_second_t>));
  EXPECT_FALSE((units::traits::is_data_transfer_rate_unit_v<units::year_t>));
}

TEST_F(UnitManipulatorsTest, Squared) {
  double test;

  test = units::convert<units::squared<units::meters>, units::square_feet>(
      0.092903);
  EXPECT_NEAR(0.99999956944, test, 5.0e-12);

  // This is actually nonsensical, and should also result in a scalar.
  using scalar_2 = units::squared<units::scalar>;
  bool isSame = std::same_as<typename std::decay_t<units::scalar_t>,
                             typename std::decay_t<units::unit_t<scalar_2>>>;
  EXPECT_TRUE(isSame);
}

TEST_F(UnitManipulatorsTest, Cubed) {
  double test;

  test =
      units::convert<units::cubed<units::meters>, units::cubic_feet>(0.0283168);
  EXPECT_NEAR(0.999998354619, test, 5.0e-13);
}

TEST_F(UnitManipulatorsTest, SquareRoot) {
  double test;

  test =
      units::convert<units::square_root<units::square_kilometer>, units::meter>(
          1.0);
  EXPECT_TRUE(
      (units::is_convertible_unit<
          typename std::decay_t<units::square_root<units::square_kilometer>>,
          units::kilometer>));
  EXPECT_NEAR(1000.0, test, 5.0e-13);
}

TEST_F(UnitManipulatorsTest, CompoundUnit) {
  using acceleration1 =
      units::unit<std::ratio<1>, units::category::acceleration_unit>;
  using acceleration2 =
      units::compound_unit<units::meters, units::inverse<units::seconds>,
                           units::inverse<units::seconds>>;
  using acceleration3 =
      units::unit<std::ratio<1>, units::base_unit<std::ratio<1>, std::ratio<0>,
                                                  std::ratio<-2>>>;
  using acceleration4 =
      units::compound_unit<units::meters,
                           units::inverse<units::squared<units::seconds>>>;
  using acceleration5 =
      units::compound_unit<units::meters,
                           units::squared<units::inverse<units::seconds>>>;

  bool areSame12 = std::same_as<acceleration1, acceleration2>;
  bool areSame23 = std::same_as<acceleration2, acceleration3>;
  bool areSame34 = std::same_as<acceleration3, acceleration4>;
  bool areSame45 = std::same_as<acceleration4, acceleration5>;

  EXPECT_TRUE(areSame12);
  EXPECT_TRUE(areSame23);
  EXPECT_TRUE(areSame34);
  EXPECT_TRUE(areSame45);

  // Test that thing with translations still compile
  using arbitrary1 =
      units::compound_unit<units::meters, units::inverse<units::celsius>>;
  using arbitrary2 = units::compound_unit<units::meters, units::celsius>;
  using arbitrary3 = units::compound_unit<arbitrary1, arbitrary2>;
  EXPECT_TRUE((std::same_as<units::square_meters, arbitrary3>));
}

TEST_F(UnitManipulatorsTest, DimensionalAnalysis) {
  // These look like 'compound units', but the dimensional analysis can be
  // REALLY handy if the unit types aren't know (i.e. they themselves are
  // template parameters), as you can get the resulting unit of the operation.

  using velocity = units::detail::unit_divide<units::meters, units::second>;
  bool shouldBeTrue = std::same_as<units::meters_per_second, units::velocity>;
  EXPECT_TRUE(shouldBeTrue);

  using acceleration1 =
      units::unit<std::ratio<1>, units::category::acceleration_unit>;
  using acceleration2 = units::detail::unit_divide<
      units::meters,
      units::detail::unit_multiply<units::seconds, units::seconds>>;
  shouldBeTrue = std::same_as<acceleration1, acceleration2>;
  EXPECT_TRUE(shouldBeTrue);
}

TEST_F(UnitContainerTest, Trivial) {
  EXPECT_TRUE((std::is_trivial_v<units::meter_t>));
  EXPECT_TRUE((std::is_trivially_assignable_v<units::meter_t, units::meter_t>));
  EXPECT_TRUE((std::is_trivially_constructible_v<units::meter_t>));
  EXPECT_TRUE((std::is_trivially_copy_assignable_v<units::meter_t>));
  EXPECT_TRUE((std::is_trivially_copy_constructible_v<units::meter_t>));
  EXPECT_TRUE((std::is_trivially_copyable_v<units::meter_t>));
  EXPECT_TRUE((std::is_trivially_default_constructible_v<units::meter_t>));
  EXPECT_TRUE((std::is_trivially_destructible_v<units::meter_t>));
  EXPECT_TRUE((std::is_trivially_move_assignable_v<units::meter_t>));
  EXPECT_TRUE((std::is_trivially_move_constructible_v<units::meter_t>));

  EXPECT_TRUE((std::is_trivial_v<units::dB_t>));
  EXPECT_TRUE((std::is_trivially_assignable_v<units::dB_t, units::dB_t>));
  EXPECT_TRUE((std::is_trivially_constructible_v<units::dB_t>));
  EXPECT_TRUE((std::is_trivially_copy_assignable_v<units::dB_t>));
  EXPECT_TRUE((std::is_trivially_copy_constructible_v<units::dB_t>));
  EXPECT_TRUE((std::is_trivially_copyable_v<units::dB_t>));
  EXPECT_TRUE((std::is_trivially_default_constructible_v<units::dB_t>));
  EXPECT_TRUE((std::is_trivially_destructible_v<units::dB_t>));
  EXPECT_TRUE((std::is_trivially_move_assignable_v<units::dB_t>));
  EXPECT_TRUE((std::is_trivially_move_constructible_v<units::dB_t>));
}

TEST_F(UnitContainerTest, MakeUnit) {
  auto dist = units::make_unit<units::meter_t>(5);
  EXPECT_EQ(units::meter_t(5), dist);
}

TEST_F(UnitContainerTest, UnitTypeAddition) {
  // units
  units::meter_t a_m(1.0), c_m;
  units::foot_t b_ft(3.28084);

  double d = units::convert<units::feet, units::meters>(b_ft());
  EXPECT_NEAR(1.0, d, 5.0e-5);

  c_m = a_m + b_ft;
  EXPECT_NEAR(2.0, c_m(), 5.0e-5);

  c_m = b_ft + units::meter_t(3);
  EXPECT_NEAR(4.0, c_m(), 5.0e-5);

  auto e_ft = b_ft + units::meter_t(3);
  EXPECT_NEAR(13.12336, e_ft(), 5.0e-6);

  // scalar
  units::scalar_t sresult = units::scalar_t(1.0) + units::scalar_t(1.0);
  EXPECT_NEAR(2.0, sresult, 5.0e-6);

  sresult = units::scalar_t(1.0) + 1.0;
  EXPECT_NEAR(2.0, sresult, 5.0e-6);

  sresult = 1.0 + units::scalar_t(1.0);
  EXPECT_NEAR(2.0, sresult, 5.0e-6);

  d = units::scalar_t(1.0) + units::scalar_t(1.0);
  EXPECT_NEAR(2.0, d, 5.0e-6);

  d = units::scalar_t(1.0) + 1.0;
  EXPECT_NEAR(2.0, d, 5.0e-6);

  d = 1.0 + units::scalar_t(1.0);
  EXPECT_NEAR(2.0, d, 5.0e-6);
}

TEST_F(UnitContainerTest, UnitTypeUnaryAddition) {
  units::meter_t a_m(1.0);

  EXPECT_EQ(++a_m, units::meter_t(2));
  EXPECT_EQ(a_m++, units::meter_t(2));
  EXPECT_EQ(a_m, units::meter_t(3));
  EXPECT_EQ(+a_m, units::meter_t(3));
  EXPECT_EQ(a_m, units::meter_t(3));

  units::dBW_t b_dBW(1.0);

  EXPECT_EQ(++b_dBW, units::dBW_t(2));
  EXPECT_EQ(b_dBW++, units::dBW_t(2));
  EXPECT_EQ(b_dBW, units::dBW_t(3));
  EXPECT_EQ(+b_dBW, units::dBW_t(3));
  EXPECT_EQ(b_dBW, units::dBW_t(3));
}

TEST_F(UnitContainerTest, UnitTypeSubtraction) {
  units::meter_t a_m(1.0), c_m;
  units::foot_t b_ft(3.28084);

  c_m = a_m - b_ft;
  EXPECT_NEAR(0.0, c_m(), 5.0e-5);

  c_m = b_ft - units::meter_t(1);
  EXPECT_NEAR(0.0, c_m(), 5.0e-5);

  auto e_ft = b_ft - units::meter_t(1);
  EXPECT_NEAR(0.0, e_ft(), 5.0e-6);

  units::scalar_t sresult = units::scalar_t(1.0) - units::scalar_t(1.0);
  EXPECT_NEAR(0.0, sresult, 5.0e-6);

  sresult = units::scalar_t(1.0) - 1.0;
  EXPECT_NEAR(0.0, sresult, 5.0e-6);

  sresult = 1.0 - units::scalar_t(1.0);
  EXPECT_NEAR(0.0, sresult, 5.0e-6);

  double d = units::scalar_t(1.0) - units::scalar_t(1.0);
  EXPECT_NEAR(0.0, d, 5.0e-6);

  d = units::scalar_t(1.0) - 1.0;
  EXPECT_NEAR(0.0, d, 5.0e-6);

  d = 1.0 - units::scalar_t(1.0);
  EXPECT_NEAR(0.0, d, 5.0e-6);
}

TEST_F(UnitContainerTest, UnitTypeUnarySubtraction) {
  units::meter_t a_m(4.0);

  EXPECT_EQ(--a_m, units::meter_t(3));
  EXPECT_EQ(a_m--, units::meter_t(3));
  EXPECT_EQ(a_m, units::meter_t(2));
  EXPECT_EQ(-a_m, units::meter_t(-2));
  EXPECT_EQ(a_m, units::meter_t(2));

  units::dBW_t b_dBW(4.0);

  EXPECT_EQ(--b_dBW, units::dBW_t(3));
  EXPECT_EQ(b_dBW--, units::dBW_t(3));
  EXPECT_EQ(b_dBW, units::dBW_t(2));
  EXPECT_EQ(-b_dBW, units::dBW_t(-2));
  EXPECT_EQ(b_dBW, units::dBW_t(2));
}

TEST_F(UnitContainerTest, UnitTypeMultiplication) {
  units::meter_t a_m(1.0), b_m(2.0);
  units::foot_t a_ft(3.28084);

  auto c_m2 = a_m * b_m;
  EXPECT_NEAR(2.0, c_m2(), 5.0e-5);

  c_m2 = b_m * units::meter_t(2);
  EXPECT_NEAR(4.0, c_m2(), 5.0e-5);

  c_m2 = b_m * a_ft;
  EXPECT_NEAR(2.0, c_m2(), 5.0e-5);

  auto c_m = b_m * 2.0;
  EXPECT_NEAR(4.0, c_m(), 5.0e-5);

  c_m = 2.0 * b_m;
  EXPECT_NEAR(4.0, c_m(), 5.0e-5);

  double convert = units::scalar_t(3.14);
  EXPECT_NEAR(3.14, convert, 5.0e-5);

  units::scalar_t sresult = units::scalar_t(5.0) * units::scalar_t(4.0);
  EXPECT_NEAR(20.0, sresult(), 5.0e-5);

  sresult = units::scalar_t(5.0) * 4.0;
  EXPECT_NEAR(20.0, sresult(), 5.0e-5);

  sresult = 4.0 * units::scalar_t(5.0);
  EXPECT_NEAR(20.0, sresult(), 5.0e-5);

  double result = units::scalar_t(5.0) * units::scalar_t(4.0);
  EXPECT_NEAR(20.0, result, 5.0e-5);

  result = units::scalar_t(5.0) * 4.0;
  EXPECT_NEAR(20.0, result, 5.0e-5);

  result = 4.0 * units::scalar_t(5.0);
  EXPECT_NEAR(20.0, result, 5.0e-5);
}

TEST_F(UnitContainerTest, UnitTypeMixedUnitMultiplication) {
  units::meter_t a_m(1.0);
  units::foot_t b_ft(3.28084);
  units::unit_t<units::inverse<units::meter>> i_m(2.0);

  // resultant unit is square of leftmost unit
  auto c_m2 = a_m * b_ft;
  EXPECT_NEAR(1.0, c_m2(), 5.0e-5);

  auto c_ft2 = b_ft * a_m;
  EXPECT_NEAR(10.7639111056, c_ft2(), 5.0e-7);

  // you can get whatever (compatible) type you want if you ask explicitly
  units::square_meter_t d_m2 = b_ft * a_m;
  EXPECT_NEAR(1.0, d_m2(), 5.0e-5);

  // a unit times a sclar ends up with the same units.
  units::meter_t e_m = a_m * units::scalar_t(3.0);
  EXPECT_NEAR(3.0, e_m(), 5.0e-5);

  e_m = units::scalar_t(4.0) * a_m;
  EXPECT_NEAR(4.0, e_m(), 5.0e-5);

  // unit times its units::inverse results in a scalar
  units::scalar_t s = a_m * i_m;
  EXPECT_NEAR(2.0, s, 5.0e-5);

  c_m2 = b_ft * units::meter_t(2);
  EXPECT_NEAR(2.0, c_m2(), 5.0e-5);

  auto e_ft2 = b_ft * units::meter_t(3);
  EXPECT_NEAR(32.2917333168, e_ft2(), 5.0e-6);

  auto mps =
      units::meter_t(10.0) * units::unit_t<units::inverse<units::seconds>>(1.0);
  EXPECT_EQ(mps, units::meters_per_second_t(10));
}

TEST_F(UnitContainerTest, UnitTypeScalarMultiplication) {
  units::meter_t a_m(1.0);

  auto result_m = units::scalar_t(3.0) * a_m;
  EXPECT_NEAR(3.0, result_m(), 5.0e-5);

  result_m = a_m * units::scalar_t(4.0);
  EXPECT_NEAR(4.0, result_m(), 5.0e-5);

  result_m = 3.0 * a_m;
  EXPECT_NEAR(3.0, result_m(), 5.0e-5);

  result_m = a_m * 4.0;
  EXPECT_NEAR(4.0, result_m(), 5.0e-5);

  bool isSame = std::same_as<decltype(result_m), units::meter_t>;
  EXPECT_TRUE(isSame);
}

TEST_F(UnitContainerTest, UnitTypeDivision) {
  units::meter_t a_m(1.0), b_m(2.0);
  units::foot_t a_ft(3.28084);
  units::second_t a_sec(10.0);
  bool isSame;

  auto c = a_m / a_ft;
  EXPECT_NEAR(1.0, c, 5.0e-5);
  isSame = std::same_as<decltype(c), units::scalar_t>;
  EXPECT_TRUE(isSame);

  c = a_m / b_m;
  EXPECT_NEAR(0.5, c, 5.0e-5);
  isSame = std::same_as<decltype(c), units::scalar_t>;
  EXPECT_TRUE(isSame);

  c = a_ft / a_m;
  EXPECT_NEAR(1.0, c, 5.0e-5);
  isSame = std::same_as<decltype(c), units::scalar_t>;
  EXPECT_TRUE(isSame);

  c = units::scalar_t(1.0) / 2.0;
  EXPECT_NEAR(0.5, c, 5.0e-5);
  isSame = std::same_as<decltype(c), units::scalar_t>;
  EXPECT_TRUE(isSame);

  c = 1.0 / units::scalar_t(2.0);
  EXPECT_NEAR(0.5, c, 5.0e-5);
  isSame = std::same_as<decltype(c), units::scalar_t>;
  EXPECT_TRUE(isSame);

  double d = units::scalar_t(1.0) / 2.0;
  EXPECT_NEAR(0.5, d, 5.0e-5);

  auto e = a_m / a_sec;
  EXPECT_NEAR(0.1, e(), 5.0e-5);
  isSame = std::same_as<decltype(e), units::meters_per_second_t>;
  EXPECT_TRUE(isSame);

  auto f = a_m / 8.0;
  EXPECT_NEAR(0.125, f(), 5.0e-5);
  isSame = std::same_as<decltype(f), units::meter_t>;
  EXPECT_TRUE(isSame);

  auto g = 4.0 / b_m;
  EXPECT_NEAR(2.0, g(), 5.0e-5);
  isSame =
      std::same_as<decltype(g), units::unit_t<units::inverse<units::meters>>>;
  EXPECT_TRUE(isSame);

  auto mph = units::mile_t(60.0) / units::hour_t(1.0);
  units::meters_per_second_t mps = mph;
  EXPECT_NEAR(26.8224, mps(), 5.0e-5);
}

TEST_F(UnitContainerTest, CompoundAssignmentAddition) {
  // units
  units::meter_t a(0.0);
  a += units::meter_t(1.0);

  EXPECT_EQ(units::meter_t(1.0), a);

  a += units::foot_t(units::meter_t(1));

  EXPECT_EQ(units::meter_t(2.0), a);

  // scalars
  units::scalar_t b(0);
  b += units::scalar_t(1.0);

  EXPECT_EQ(units::scalar_t(1.0), b);

  b += 1;

  EXPECT_EQ(units::scalar_t(2.0), b);
}

TEST_F(UnitContainerTest, CompoundAssignmentSubtraction) {
  // units
  units::meter_t a(2.0);
  a -= units::meter_t(1.0);

  EXPECT_EQ(units::meter_t(1.0), a);

  a -= units::foot_t(units::meter_t(1));

  EXPECT_EQ(units::meter_t(0.0), a);

  // scalars
  units::scalar_t b(2);
  b -= units::scalar_t(1.0);

  EXPECT_EQ(units::scalar_t(1.0), b);

  b -= 1;

  EXPECT_EQ(units::scalar_t(0), b);
}

TEST_F(UnitContainerTest, CompoundAssignmentMultiplication) {
  // units
  units::meter_t a(2.0);
  a *= units::scalar_t(2.0);

  EXPECT_EQ(units::meter_t(4.0), a);

  a *= 2.0;

  EXPECT_EQ(units::meter_t(8.0), a);

  // scalars
  units::scalar_t b(2);
  b *= units::scalar_t(2.0);

  EXPECT_EQ(units::scalar_t(4.0), b);

  b *= 2;

  EXPECT_EQ(units::scalar_t(8.0), b);
}

TEST_F(UnitContainerTest, CompoundAssignmentDivision) {
  // units
  units::meter_t a(8.0);
  a /= units::scalar_t(2.0);

  EXPECT_EQ(units::meter_t(4.0), a);

  a /= 2.0;

  EXPECT_EQ(units::meter_t(2.0), a);

  // scalars
  units::scalar_t b(8);
  b /= units::scalar_t(2.0);

  EXPECT_EQ(units::scalar_t(4.0), b);

  b /= 2;

  EXPECT_EQ(units::scalar_t(2.0), b);
}

TEST_F(UnitContainerTest, ScalarTypeImplicitConversion) {
  double test = units::scalar_t(3.0);
  EXPECT_DOUBLE_EQ(3.0, test);

  units::scalar_t testS = 3.0;
  EXPECT_DOUBLE_EQ(3.0, testS);

  units::scalar_t test3(units::ppm_t(10));
  EXPECT_DOUBLE_EQ(0.00001, test3);

  units::scalar_t test4;
  test4 = units::ppm_t(1);
  EXPECT_DOUBLE_EQ(0.000001, test4);
}

TEST_F(UnitContainerTest, ValueMethod) {
  double test = units::meter_t(3.0).value();
  EXPECT_DOUBLE_EQ(3.0, test);

  auto test2 = units::meter_t(4.0).value();
  EXPECT_DOUBLE_EQ(4.0, test2);
  EXPECT_TRUE((std::same_as<decltype(test2), double>));
}

TEST_F(UnitContainerTest, ConvertMethod) {
  double test = units::meter_t(3.0).convert<units::feet>().value();
  EXPECT_NEAR(9.84252, test, 5.0e-6);
}

#ifdef UNIT_LIB_ENABLE_IOSTREAM
TEST_F(UnitContainerTest, Cout) {
  testing::internal::CaptureStdout();
  std::cout << units::degree_t(349.87);
  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("349.87 deg", output.c_str());

  testing::internal::CaptureStdout();
  std::cout << units::meter_t(1.0);
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("1 m", output.c_str());

  testing::internal::CaptureStdout();
  std::cout << units::dB_t(31.0);
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("31 dB", output.c_str());

  testing::internal::CaptureStdout();
  std::cout << units::volt_t(21.79);
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("21.79 V", output.c_str());

  testing::internal::CaptureStdout();
  std::cout << units::dBW_t(12.0);
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("12 dBW", output.c_str());

  testing::internal::CaptureStdout();
  std::cout << units::dBm_t(120.0);
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("120 dBm", output.c_str());

  testing::internal::CaptureStdout();
  std::cout << units::miles_per_hour_t(72.1);
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("72.1 mph", output.c_str());

  // undefined unit
  testing::internal::CaptureStdout();
  std::cout << units::math::cpow<4>(units::meter_t(2));
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("16 m^4", output.c_str());

  testing::internal::CaptureStdout();
  std::cout << units::math::cpow<3>(units::foot_t(2));
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("8 cu_ft", output.c_str());

  testing::internal::CaptureStdout();
  std::cout << std::setprecision(9) << units::math::cpow<4>(units::foot_t(2));
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("0.138095597 m^4", output.c_str());

  // constants
  testing::internal::CaptureStdout();
  std::cout << std::setprecision(8) << units::constants::k_B;
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("1.3806485e-23 m^2 kg s^-2 K^-1", output.c_str());

  testing::internal::CaptureStdout();
  std::cout << std::setprecision(9) << units::constants::mu_B;
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("9.27400999e-24 m^2 A", output.c_str());

  testing::internal::CaptureStdout();
  std::cout << std::setprecision(7) << units::constants::sigma;
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("5.670367e-08 kg s^-3 K^-4", output.c_str());
}
#endif

#if __has_include(<fmt/format.h>) && !defined(UNIT_LIB_DISABLE_FMT)
TEST_F(UnitContainerTest, Fmtlib) {
  testing::internal::CaptureStdout();
  wpi::print("{}", units::degree_t(349.87));
  std::string output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("349.87 deg", output.c_str());

  testing::internal::CaptureStdout();
  wpi::print("{}", units::meter_t(1.0));
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("1 m", output.c_str());

  testing::internal::CaptureStdout();
  wpi::print("{}", units::dB_t(31.0));
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("31 dB", output.c_str());

  testing::internal::CaptureStdout();
  wpi::print("{}", units::volt_t(21.79));
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("21.79 V", output.c_str());

  testing::internal::CaptureStdout();
  wpi::print("{}", units::dBW_t(12.0));
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("12 dBW", output.c_str());

  testing::internal::CaptureStdout();
  wpi::print("{}", units::dBm_t(120.0));
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("120 dBm", output.c_str());

  testing::internal::CaptureStdout();
  wpi::print("{}", units::miles_per_hour_t(72.1));
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("72.1 mph", output.c_str());

  // undefined unit
  testing::internal::CaptureStdout();
  wpi::print("{}", units::math::cpow<4>(units::meter_t(2)));
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("16 m^4", output.c_str());

  testing::internal::CaptureStdout();
  wpi::print("{}", units::math::cpow<3>(units::foot_t(2)));
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("8 cu_ft", output.c_str());

  testing::internal::CaptureStdout();
  wpi::print("{:.9}", units::math::cpow<4>(units::foot_t(2)));
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("0.138095597 m^4", output.c_str());

  // constants
  testing::internal::CaptureStdout();
  wpi::print("{:.8}", units::constants::k_B);
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("1.3806485e-23 m^2 kg s^-2 K^-1", output.c_str());

  testing::internal::CaptureStdout();
  wpi::print("{:.9}", units::constants::mu_B);
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("9.27400999e-24 m^2 A", output.c_str());

  testing::internal::CaptureStdout();
  wpi::print("{:.7}", units::constants::sigma);
  output = testing::internal::GetCapturedStdout();
  EXPECT_STREQ("5.670367e-08 kg s^-3 K^-4", output.c_str());
}
#endif

TEST_F(UnitContainerTest, ToString) {
  units::foot_t a(3.5);
  EXPECT_STREQ("3.5 ft", units::length::to_string(a).c_str());

  units::meter_t b(8);
  EXPECT_STREQ("8 m", units::length::to_string(b).c_str());
}

TEST_F(UnitContainerTest, DISABLED_ToStringLocale) {
  struct lconv* lc;

  // German locale
#if defined(_MSC_VER)
  setlocale(LC_ALL, "de-DE");
#else
  EXPECT_STREQ("de_DE.utf8", setlocale(LC_ALL, "de_DE.utf8"));
#endif

  lc = localeconv();
  char point_de = *lc->decimal_point;
  EXPECT_EQ(point_de, ',');

  units::kilometer_t de = 2_km;
  EXPECT_STREQ("2 km", units::length::to_string(de).c_str());

  de = 2.5_km;
  EXPECT_STREQ("2,5 km", units::length::to_string(de).c_str());

  // US locale
#if defined(_MSC_VER)
  setlocale(LC_ALL, "en-US");
#else
  EXPECT_STREQ("en_US.utf8", setlocale(LC_ALL, "en_US.utf8"));
#endif

  lc = localeconv();
  char point_us = *lc->decimal_point;
  EXPECT_EQ(point_us, '.');

  units::mile_t us = 2_mi;
  EXPECT_STREQ("2 mi", units::length::to_string(us).c_str());

  us = 2.5_mi;
  EXPECT_STREQ("2.5 mi", units::length::to_string(us).c_str());
}

TEST_F(UnitContainerTest, NameAndAbbreviation) {
  units::foot_t a(3.5);
  EXPECT_STREQ("ft", units::abbreviation(a));
  EXPECT_STREQ("ft", a.abbreviation());
  EXPECT_STREQ("foot", a.name());

  units::meter_t b(8);
  EXPECT_STREQ("m", units::abbreviation(b));
  EXPECT_STREQ("m", b.abbreviation());
  EXPECT_STREQ("meter", b.name());
}

TEST_F(UnitContainerTest, Negative) {
  units::meter_t a(5.3);
  units::meter_t b(-5.3);
  EXPECT_NEAR(a.value(), -b.value(), 5.0e-320);
  EXPECT_NEAR(b.value(), -a.value(), 5.0e-320);

  units::dB_t c(2.87);
  units::dB_t d(-2.87);
  EXPECT_NEAR(c.value(), -d.value(), 5.0e-320);
  EXPECT_NEAR(d.value(), -c.value(), 5.0e-320);

  units::ppm_t e = -1 * units::ppm_t(10);
  EXPECT_EQ(e, -units::ppm_t(10));
  EXPECT_NEAR(-0.00001, e, 5.0e-10);
}

TEST_F(UnitContainerTest, Concentration) {
  units::ppb_t a(units::ppm_t(1));
  EXPECT_EQ(ppb_t(1000), a);
  EXPECT_EQ(0.000001, a);
  EXPECT_EQ(0.000001, a.value());

  units::scalar_t b(units::ppm_t(1));
  EXPECT_EQ(0.000001, b);

  units::scalar_t c = units::ppb_t(1);
  EXPECT_EQ(0.000000001, c);
}

TEST_F(UnitContainerTest, dBConversion) {
  units::dBW_t a_dbw(23.1);
  units::watt_t a_w = a_dbw;
  units::dBm_t a_dbm = a_dbw;

  EXPECT_NEAR(204.173794, a_w(), 5.0e-7);
  EXPECT_NEAR(53.1, a_dbm(), 5.0e-7);

  units::milliwatt_t b_mw(100000.0);
  units::watt_t b_w = b_mw;
  units::dBm_t b_dbm = b_mw;
  units::dBW_t b_dbw = b_mw;

  EXPECT_NEAR(100.0, b_w(), 5.0e-7);
  EXPECT_NEAR(50.0, b_dbm(), 5.0e-7);
  EXPECT_NEAR(20.0, b_dbw(), 5.0e-7);
}

TEST_F(UnitContainerTest, dBAddition) {
  bool isSame;

  auto result_dbw = units::dBW_t(10.0) + units::dB_t(30.0);
  EXPECT_NEAR(40.0, result_dbw(), 5.0e-5);
  result_dbw = units::dB_t(12.0) + units::dBW_t(30.0);
  EXPECT_NEAR(42.0, result_dbw(), 5.0e-5);
  isSame = std::same_as<decltype(result_dbw), units::dBW_t>;
  EXPECT_TRUE(isSame);

  auto result_dbm = units::dB_t(30.0) + units::dBm_t(20.0);
  EXPECT_NEAR(50.0, result_dbm(), 5.0e-5);

  // adding dBW to dBW is something you probably shouldn't do, but let's see if
  // it works...
  auto result_dBW2 = units::dBW_t(10.0) + units::dBm_t(40.0);
  EXPECT_NEAR(20.0, result_dBW2(), 5.0e-5);
  isSame = std::same_as<decltype(result_dBW2),
                        units::unit_t<units::squared<units::watts>, double,
                                      decibel_scale<double>>>;
  EXPECT_TRUE(isSame);
}

TEST_F(UnitContainerTest, dBSubtraction) {
  bool isSame;

  auto result_dbw = units::dBW_t(10.0) - units::dB_t(30.0);
  EXPECT_NEAR(-20.0, result_dbw(), 5.0e-5);
  isSame = std::same_as<decltype(result_dbw), units::dBW_t>;
  EXPECT_TRUE(isSame);

  auto result_dbm = units::dBm_t(100.0) - units::dB_t(30.0);
  EXPECT_NEAR(70.0, result_dbm(), 5.0e-5);
  isSame = std::same_as<decltype(result_dbm), units::dBm_t>;
  EXPECT_TRUE(isSame);

  auto result_db = units::dBW_t(100.0) - units::dBW_t(80.0);
  EXPECT_NEAR(20.0, result_db(), 5.0e-5);
  isSame = std::same_as<decltype(result_db), units::dB_t>;
  EXPECT_TRUE(isSame);

  result_db = units::dB_t(100.0) - units::dB_t(80.0);
  EXPECT_NEAR(20.0, result_db(), 5.0e-5);
  isSame = std::same_as<decltype(result_db), units::dB_t>;
  EXPECT_TRUE(isSame);
}

TEST_F(UnitContainerTest, UnitCast) {
  units::meter_t test1(5.7);
  units::hectare_t test2(16);

  double dResult1 = 5.7;

  double dResult2 = 16;
  int iResult2 = 16;

  EXPECT_EQ(dResult1, units::unit_cast<double>(test1));
  EXPECT_EQ(dResult2, units::unit_cast<double>(test2));
  EXPECT_EQ(iResult2, units::unit_cast<int>(test2));

  EXPECT_TRUE(
      (std::same_as<double, decltype(units::unit_cast<double>(test1))>));
  EXPECT_TRUE((std::same_as<int, decltype(units::unit_cast<int>(test2))>));
}

// literal syntax is only supported in GCC 4.7+ and MSVC2015+
TEST_F(UnitContainerTest, Literals) {
  // basic functionality testing
  EXPECT_TRUE((std::same_as<decltype(16.2_m), units::meter_t>));
  EXPECT_TRUE(units::meter_t(16.2) == 16.2_m);
  EXPECT_TRUE(units::meter_t(16) == 16_m);

  EXPECT_TRUE((std::same_as<decltype(11.2_ft), units::foot_t>));
  EXPECT_TRUE(units::foot_t(11.2) == 11.2_ft);
  EXPECT_TRUE(units::foot_t(11) == 11_ft);

  // auto using literal syntax
  auto x = 10.0_m;
  EXPECT_TRUE((std::same_as<decltype(x), units::meter_t>));
  EXPECT_TRUE(units::meter_t(10) == x);

  // conversion using literal syntax
  units::foot_t y = 0.3048_m;
  EXPECT_TRUE(1_ft == y);

  // Pythagorean theorem
  units::meter_t a = 3_m;
  units::meter_t b = 4_m;
  units::meter_t c =
      units::math::sqrt(units::math::pow<2>(a) + units::math::pow<2>(b));
  EXPECT_TRUE(c == 5_m);
}

TEST_F(UnitConversionTest, Length) {
  double test;
  test = units::convert<units::meters, units::nanometers>(0.000000001);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::meters, units::micrometers>(0.000001);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::meters, units::millimeters>(0.001);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::meters, units::centimeters>(0.01);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::meters, units::kilometers>(1000.0);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::meters, units::meters>(1.0);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::meters, units::feet>(0.3048);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::meters, units::miles>(1609.344);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::meters, units::inches>(0.0254);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::meters, units::nauticalMiles>(1852.0);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::meters, units::astronicalUnits>(149597870700.0);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::meters, units::lightyears>(9460730472580800.0);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::meters, units::parsec>(3.08567758e16);
  EXPECT_NEAR(1.0, test, 5.0e7);

  test = units::convert<units::feet, units::feet>(6.3);
  EXPECT_NEAR(6.3, test, 5.0e-5);
  test = units::convert<units::feet, units::inches>(6.0);
  EXPECT_NEAR(72.0, test, 5.0e-5);
  test = units::convert<units::inches, units::feet>(6.0);
  EXPECT_NEAR(0.5, test, 5.0e-5);
  test = units::convert<units::meter, units::feet>(1.0);
  EXPECT_NEAR(3.28084, test, 5.0e-5);
  test = units::convert<units::miles, units::nauticalMiles>(6.3);
  EXPECT_NEAR(5.47455, test, 5.0e-6);
  test = units::convert<units::miles, units::meters>(11.0);
  EXPECT_NEAR(17702.8, test, 5.0e-2);
  test = units::convert<units::meters, units::chains>(1.0);
  EXPECT_NEAR(0.0497097, test, 5.0e-7);
}

TEST_F(UnitConversionTest, Mass) {
  double test;

  test = units::convert<units::kilograms, units::grams>(1.0e-3);
  EXPECT_NEAR(1.0, test, 5.0e-6);
  test = units::convert<units::kilograms, units::micrograms>(1.0e-9);
  EXPECT_NEAR(1.0, test, 5.0e-6);
  test = units::convert<units::kilograms, units::milligrams>(1.0e-6);
  EXPECT_NEAR(1.0, test, 5.0e-6);
  test = units::convert<units::kilograms, units::kilograms>(1.0);
  EXPECT_NEAR(1.0, test, 5.0e-6);
  test = units::convert<units::kilograms, units::metric_tons>(1000.0);
  EXPECT_NEAR(1.0, test, 5.0e-6);
  test = units::convert<units::kilograms, units::pounds>(0.453592);
  EXPECT_NEAR(1.0, test, 5.0e-6);
  test = units::convert<units::kilograms, units::long_tons>(1016.05);
  EXPECT_NEAR(1.0, test, 5.0e-6);
  test = units::convert<units::kilograms, units::short_tons>(907.185);
  EXPECT_NEAR(1.0, test, 5.0e-6);
  test = units::convert<units::kilograms, mass::ounces>(0.0283495);
  EXPECT_NEAR(1.0, test, 5.0e-6);
  test = units::convert<units::kilograms, units::carats>(0.0002);
  EXPECT_NEAR(1.0, test, 5.0e-6);
  test = units::convert<units::slugs, units::kilograms>(1.0);
  EXPECT_NEAR(14.593903, test, 5.0e-7);

  test = units::convert<units::pounds, units::carats>(6.3);
  EXPECT_NEAR(14288.2, test, 5.0e-2);
}

TEST_F(UnitConversionTest, Time) {
  double result = 0;
  double daysPerYear = 365;
  double hoursPerDay = 24;
  double minsPerHour = 60;
  double secsPerMin = 60;
  double daysPerWeek = 7;

  result = 2 * daysPerYear * hoursPerDay * minsPerHour * secsPerMin *
           (1 / minsPerHour) * (1 / secsPerMin) * (1 / hoursPerDay) *
           (1 / daysPerWeek);
  EXPECT_NEAR(104.286, result, 5.0e-4);

  units::year_t twoYears(2.0);
  units::week_t twoYearsInWeeks = twoYears;
  EXPECT_NEAR(week_t(104.286).value(), twoYearsInWeeks.value(), 5.0e-4);

  double test;

  test = units::convert<units::seconds, units::seconds>(1.0);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::seconds, units::nanoseconds>(1.0e-9);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::seconds, units::microseconds>(1.0e-6);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::seconds, units::milliseconds>(1.0e-3);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::seconds, units::minutes>(60.0);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::seconds, units::hours>(3600.0);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::seconds, units::days>(86400.0);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::seconds, units::weeks>(604800.0);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test = units::convert<units::seconds, units::years>(3.154e7);
  EXPECT_NEAR(1.0, test, 5.0e3);

  test = units::convert<units::years, units::weeks>(2.0);
  EXPECT_NEAR(104.2857142857143, test, 5.0e-14);
  test = units::convert<units::hours, units::minutes>(4.0);
  EXPECT_NEAR(240.0, test, 5.0e-14);
  test = units::convert<units::julian_years, units::days>(1.0);
  EXPECT_NEAR(365.25, test, 5.0e-14);
  test = units::convert<units::gregorian_years, units::days>(1.0);
  EXPECT_NEAR(365.2425, test, 5.0e-14);
}

TEST_F(UnitConversionTest, Angle) {
  units::angle::degree_t quarterCircleDeg(90.0);
  units::angle::radian_t quarterCircleRad = quarterCircleDeg;
  EXPECT_NEAR(units::angle::radian_t(std::numbers::pi / 2.0).value(),
              quarterCircleRad.value(), 5.0e-12);

  double test;

  test = units::convert<units::angle::radians, units::angle::radians>(1.0);
  EXPECT_NEAR(1.0, test, 5.0e-20);
  test =
      units::convert<units::angle::radians, units::angle::milliradians>(0.001);
  EXPECT_NEAR(1.0, test, 5.0e-4);
  test =
      units::convert<units::angle::radians, units::angle::degrees>(0.0174533);
  EXPECT_NEAR(1.0, test, 5.0e-7);
  test = units::convert<units::angle::radians, units::angle::arcminutes>(
      0.000290888);
  EXPECT_NEAR(0.99999928265913, test, 5.0e-8);
  test = units::convert<units::angle::radians, units::angle::arcseconds>(
      4.8481e-6);
  EXPECT_NEAR(0.999992407, test, 5.0e-10);
  test = units::convert<units::angle::radians, units::angle::turns>(6.28319);
  EXPECT_NEAR(1.0, test, 5.0e-6);
  test =
      units::convert<units::angle::radians, units::angle::gradians>(0.015708);
  EXPECT_NEAR(1.0, test, 5.0e-6);

  test = units::convert<units::angle::radians, units::angle::radians>(2.1);
  EXPECT_NEAR(2.1, test, 5.0e-6);
  test = units::convert<units::angle::arcseconds, units::angle::gradians>(2.1);
  EXPECT_NEAR(0.000648148, test, 5.0e-6);
  test = units::convert<units::angle::radians, units::angle::degrees>(
      std::numbers::pi);
  EXPECT_NEAR(180.0, test, 5.0e-6);
  test = units::convert<units::angle::degrees, units::angle::radians>(90.0);
  EXPECT_NEAR(std::numbers::pi / 2, test, 5.0e-6);
}

TEST_F(UnitConversionTest, Current) {
  double test;

  test = units::convert<units::current::A, units::current::mA>(2.1);
  EXPECT_NEAR(2100.0, test, 5.0e-6);
}

TEST_F(UnitConversionTest, Temperature) {
  // temp conversion are weird/hard since they involve translations AND scaling.
  double test;

  test = units::convert<units::kelvin, units::kelvin>(72.0);
  EXPECT_NEAR(72.0, test, 5.0e-5);
  test = units::convert<units::fahrenheit, units::fahrenheit>(72.0);
  EXPECT_NEAR(72.0, test, 5.0e-5);
  test = units::convert<units::kelvin, units::fahrenheit>(300.0);
  EXPECT_NEAR(80.33, test, 5.0e-5);
  test = units::convert<units::fahrenheit, units::kelvin>(451.0);
  EXPECT_NEAR(505.928, test, 5.0e-4);
  test = units::convert<units::kelvin, units::celsius>(300.0);
  EXPECT_NEAR(26.85, test, 5.0e-3);
  test = units::convert<units::celsius, units::kelvin>(451.0);
  EXPECT_NEAR(724.15, test, 5.0e-3);
  test = units::convert<units::fahrenheit, units::celsius>(72.0);
  EXPECT_NEAR(22.2222, test, 5.0e-5);
  test = units::convert<units::celsius, units::fahrenheit>(100.0);
  EXPECT_NEAR(212.0, test, 5.0e-5);
  test = units::convert<units::fahrenheit, units::celsius>(32.0);
  EXPECT_NEAR(0.0, test, 5.0e-5);
  test = units::convert<units::celsius, units::fahrenheit>(0.0);
  EXPECT_NEAR(32.0, test, 5.0e-5);
  test = units::convert<units::rankine, units::kelvin>(100.0);
  EXPECT_NEAR(55.5556, test, 5.0e-5);
  test = units::convert<units::kelvin, units::rankine>(100.0);
  EXPECT_NEAR(180.0, test, 5.0e-5);
  test = units::convert<units::fahrenheit, units::rankine>(100.0);
  EXPECT_NEAR(559.67, test, 5.0e-5);
  test = units::convert<units::rankine, units::fahrenheit>(72.0);
  EXPECT_NEAR(-387.67, test, 5.0e-5);
  test = units::convert<units::reaumur, units::kelvin>(100.0);
  EXPECT_NEAR(398.0, test, 5.0e-1);
  test = units::convert<units::reaumur, units::celsius>(80.0);
  EXPECT_NEAR(100.0, test, 5.0e-5);
  test = units::convert<units::celsius, units::reaumur>(212.0);
  EXPECT_NEAR(169.6, test, 5.0e-2);
  test = units::convert<units::reaumur, units::fahrenheit>(80.0);
  EXPECT_NEAR(212.0, test, 5.0e-5);
  test = units::convert<units::fahrenheit, units::reaumur>(37.0);
  EXPECT_NEAR(2.222, test, 5.0e-3);
}

TEST_F(UnitConversionTest, LuminousIntensity) {
  double test;

  test = units::convert<units::candela, units::millicandela>(72.0);
  EXPECT_NEAR(72000.0, test, 5.0e-5);
  test = units::convert<units::millicandela, units::candela>(376.0);
  EXPECT_NEAR(0.376, test, 5.0e-5);
}

TEST_F(UnitConversionTest, SolidAngle) {
  double test;
  bool same;

  same = std::same_as<units::traits::base_unit_of<units::steradians>,
                      units::traits::base_unit_of<units::degrees_squared>>;
  EXPECT_TRUE(same);

  test = units::convert<units::steradians, units::steradians>(72.0);
  EXPECT_NEAR(72.0, test, 5.0e-5);
  test = units::convert<units::steradians, units::degrees_squared>(1.0);
  EXPECT_NEAR(3282.8, test, 5.0e-2);
  test = units::convert<units::steradians, units::spats>(8.0);
  EXPECT_NEAR(0.636619772367582, test, 5.0e-14);
  test = units::convert<units::degrees_squared, units::steradians>(3282.8);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::degrees_squared, units::degrees_squared>(72.0);
  EXPECT_NEAR(72.0, test, 5.0e-5);
  test = units::convert<units::degrees_squared, units::spats>(3282.8);
  EXPECT_NEAR(1.0 / (4 * std::numbers::pi), test, 5.0e-5);
  test = units::convert<units::spats, units::steradians>(
      1.0 / (4 * std::numbers::pi));
  EXPECT_NEAR(1.0, test, 5.0e-14);
  test = units::convert<units::spats, units::degrees_squared>(
      1.0 / (4 * std::numbers::pi));
  EXPECT_NEAR(3282.8, test, 5.0e-2);
  test = units::convert<units::spats, units::spats>(72.0);
  EXPECT_NEAR(72.0, test, 5.0e-5);
}

TEST_F(UnitConversionTest, Frequency) {
  double test;

  test = units::convert<units::hertz, units::kilohertz>(63000.0);
  EXPECT_NEAR(63.0, test, 5.0e-5);
  test = units::convert<units::hertz, units::hertz>(6.3);
  EXPECT_NEAR(6.3, test, 5.0e-5);
  test = units::convert<units::kilohertz, units::hertz>(5.0);
  EXPECT_NEAR(5000.0, test, 5.0e-5);
  test = units::convert<units::megahertz, units::hertz>(1.0);
  EXPECT_NEAR(1.0e6, test, 5.0e-5);
}

TEST_F(UnitConversionTest, Velocity) {
  double test;
  bool same;

  same =
      std::same_as<units::meters_per_second,
                   units::unit<std::ratio<1>, units::category::velocity_unit>>;
  EXPECT_TRUE(same);
  same = units::is_convertible_unit<units::miles_per_hour,
                                    units::meters_per_second>;
  EXPECT_TRUE(same);

  test =
      units::convert<units::meters_per_second, units::miles_per_hour>(1250.0);
  EXPECT_NEAR(2796.17, test, 5.0e-3);
  test = units::convert<units::feet_per_second, units::kilometers_per_hour>(
      2796.17);
  EXPECT_NEAR(3068.181418, test, 5.0e-7);
  test = units::convert<units::knots, units::miles_per_hour>(600.0);
  EXPECT_NEAR(690.468, test, 5.0e-4);
  test = units::convert<units::miles_per_hour, units::feet_per_second>(120.0);
  EXPECT_NEAR(176.0, test, 5.0e-5);
  test = units::convert<units::feet_per_second, units::meters_per_second>(10.0);
  EXPECT_NEAR(3.048, test, 5.0e-5);
}

TEST_F(UnitConversionTest, AngularVelocity) {
  double test;
  bool same;

  same = std::same_as<
      units::radians_per_second,
      units::unit<std::ratio<1>, units::category::angular_velocity_unit>>;
  EXPECT_TRUE(same);
  same = units::is_convertible_unit<rpm, units::radians_per_second>;
  EXPECT_TRUE(same);

  test = units::convert<units::radians_per_second,
                        units::milliarcseconds_per_year>(1.0);
  EXPECT_NEAR(6.504e15, test, 1.0e12);
  test =
      units::convert<units::degrees_per_second, units::radians_per_second>(1.0);
  EXPECT_NEAR(0.0174533, test, 5.0e-8);
  test = units::convert<units::rpm, units::radians_per_second>(1.0);
  EXPECT_NEAR(0.10471975512, test, 5.0e-13);
  test = units::convert<units::milliarcseconds_per_year,
                        units::radians_per_second>(1.0);
  EXPECT_NEAR(1.537e-16, test, 5.0e-20);
}

TEST_F(UnitConversionTest, AngularJerk) {
  double test;
  bool same;

  same = std::same_as<
      units::radians_per_second_cubed,
      units::unit<std::ratio<1>, units::category::angular_jerk_unit>>;
  EXPECT_TRUE(same);
  same = units::is_convertible_unit<units::deg_per_s_cu,
                                    units::radians_per_second_cubed>;
  EXPECT_TRUE(same);

  test = units::convert<units::degrees_per_second_cubed,
                        units::radians_per_second_cubed>(1.0);
  EXPECT_NEAR(0.0174533, test, 5.0e-8);
  test = units::convert<units::turns_per_second_cubed,
                        units::radians_per_second_cubed>(1.0);
  EXPECT_NEAR(6.283185307, test, 5.0e-6);
}

TEST_F(UnitConversionTest, Acceleration) {
  double test;

  test =
      units::convert<units::standard_gravity, units::meters_per_second_squared>(
          1.0);
  EXPECT_NEAR(9.80665, test, 5.0e-10);
}

TEST_F(UnitConversionTest, Force) {
  double test;

  test = units::convert<units::force::newton, units::force::newton>(1.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::force::newton, units::force::pounds>(6.3);
  EXPECT_NEAR(1.4163, test, 5.0e-5);
  test = units::convert<units::force::newton, units::force::dynes>(5.0);
  EXPECT_NEAR(500000.0, test, 5.0e-5);
  test = units::convert<units::force::newtons, units::force::poundals>(2.1);
  EXPECT_NEAR(15.1893, test, 5.0e-5);
  test = units::convert<units::force::newtons, units::force::kiloponds>(173.0);
  EXPECT_NEAR(17.6411, test, 5.0e-5);
  test =
      units::convert<units::force::poundals, units::force::kiloponds>(21.879);
  EXPECT_NEAR(0.308451933, test, 5.0e-10);
}

TEST_F(UnitConversionTest, Area) {
  double test;

  test = units::convert<units::hectares, units::acres>(6.3);
  EXPECT_NEAR(15.5676, test, 5.0e-5);
  test = units::convert<units::square_miles, units::square_kilometers>(10.0);
  EXPECT_NEAR(25.8999, test, 5.0e-5);
  test = units::convert<units::square_inch, units::square_meter>(4.0);
  EXPECT_NEAR(0.00258064, test, 5.0e-9);
  test = units::convert<units::acre, units::square_foot>(5.0);
  EXPECT_NEAR(217800.0, test, 5.0e-5);
  test = units::convert<units::square_meter, units::square_foot>(1.0);
  EXPECT_NEAR(10.7639, test, 5.0e-5);
}

TEST_F(UnitConversionTest, Pressure) {
  double test;

  test = units::convert<units::pascals, units::torr>(1.0);
  EXPECT_NEAR(0.00750062, test, 5.0e-5);
  test = units::convert<units::bar, units::psi>(2.2);
  EXPECT_NEAR(31.9083, test, 5.0e-5);
  test = units::convert<units::atmospheres, units::bar>(4.0);
  EXPECT_NEAR(4.053, test, 5.0e-5);
  test = units::convert<units::torr, units::pascals>(800.0);
  EXPECT_NEAR(106657.89474, test, 5.0e-5);
  test = units::convert<units::psi, units::atmospheres>(38.0);
  EXPECT_NEAR(2.58575, test, 5.0e-5);
  test = units::convert<units::psi, units::pascals>(1.0);
  EXPECT_NEAR(6894.76, test, 5.0e-3);
  test = units::convert<units::pascals, units::bar>(0.25);
  EXPECT_NEAR(2.5e-6, test, 5.0e-5);
  test = units::convert<units::torr, units::atmospheres>(9.0);
  EXPECT_NEAR(0.0118421, test, 5.0e-8);
  test = units::convert<units::bar, units::torr>(12.0);
  EXPECT_NEAR(9000.74, test, 5.0e-3);
  test = units::convert<units::atmospheres, units::psi>(1.0);
  EXPECT_NEAR(14.6959, test, 5.0e-5);
}

TEST_F(UnitConversionTest, Charge) {
  double test;

  test = units::convert<units::coulombs, units::ampere_hours>(4.0);
  EXPECT_NEAR(0.00111111, test, 5.0e-9);
  test = units::convert<units::ampere_hours, units::coulombs>(1.0);
  EXPECT_NEAR(3600.0, test, 5.0e-6);
}

TEST_F(UnitConversionTest, Energy) {
  double test;

  test = units::convert<units::joules, units::calories>(8000.000464);
  EXPECT_NEAR(1912.046, test, 5.0e-4);
  test = units::convert<units::therms, units::joules>(12.0);
  EXPECT_NEAR(1.266e+9, test, 5.0e5);
  test = units::convert<units::megajoules, units::watt_hours>(100.0);
  EXPECT_NEAR(27777.778, test, 5.0e-4);
  test = units::convert<units::kilocalories, units::megajoules>(56.0);
  EXPECT_NEAR(0.234304, test, 5.0e-7);
  test = units::convert<units::kilojoules, units::therms>(56.0);
  EXPECT_NEAR(0.000530904, test, 5.0e-5);
  test = units::convert<units::british_thermal_units, units::kilojoules>(
      18.56399995447);
  EXPECT_NEAR(19.5860568, test, 5.0e-5);
  test = units::convert<units::calories, units::energy::foot_pounds>(
      18.56399995447);
  EXPECT_NEAR(57.28776190423856, test, 5.0e-5);
  test = units::convert<units::megajoules, units::calories>(1.0);
  EXPECT_NEAR(239006.0, test, 5.0e-1);
  test = units::convert<units::kilocalories, units::kilowatt_hours>(2.0);
  EXPECT_NEAR(0.00232444, test, 5.0e-9);
  test = units::convert<units::therms, units::kilocalories>(0.1);
  EXPECT_NEAR(2521.04, test, 5.0e-3);
  test = units::convert<units::watt_hours, units::megajoules>(67.0);
  EXPECT_NEAR(0.2412, test, 5.0e-5);
  test = units::convert<units::british_thermal_units, units::watt_hours>(100.0);
  EXPECT_NEAR(29.3071, test, 5.0e-5);
  test = units::convert<units::calories, units::BTU>(100.0);
  EXPECT_NEAR(0.396567, test, 5.0e-5);
}

TEST_F(UnitConversionTest, Power) {
  double test;

  test = units::convert<units::compound_unit<units::energy::foot_pounds,
                                             units::inverse<units::seconds>>,
                        units::watts>(550.0);
  EXPECT_NEAR(745.7, test, 5.0e-2);
  test = units::convert<units::watts, units::gigawatts>(1000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-4);
  test = units::convert<units::microwatts, units::watts>(200000.0);
  EXPECT_NEAR(0.2, test, 5.0e-4);
  test = units::convert<units::horsepower, units::watts>(100.0);
  EXPECT_NEAR(74570.0, test, 5.0e-1);
  test = units::convert<units::horsepower, units::megawatts>(5.0);
  EXPECT_NEAR(0.0037284994, test, 5.0e-7);
  test = units::convert<units::kilowatts, units::horsepower>(232.0);
  EXPECT_NEAR(311.117, test, 5.0e-4);
  test = units::convert<units::milliwatts, units::horsepower>(1001.0);
  EXPECT_NEAR(0.001342363, test, 5.0e-9);
}

TEST_F(UnitConversionTest, Voltage) {
  double test;

  test = units::convert<units::volts, units::millivolts>(10.0);
  EXPECT_NEAR(10000.0, test, 5.0e-5);
  test = units::convert<units::picovolts, units::volts>(1000000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::nanovolts, units::volts>(1000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::microvolts, units::volts>(1000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::millivolts, units::volts>(1000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::kilovolts, units::volts>(0.001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::megavolts, units::volts>(0.000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::gigavolts, units::volts>(0.000000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::statvolts, units::volts>(299.792458);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::millivolts, units::statvolts>(1000.0);
  EXPECT_NEAR(299.792458, test, 5.0e-5);
  test = units::convert<units::abvolts, units::nanovolts>(0.1);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::microvolts, units::abvolts>(0.01);
  EXPECT_NEAR(1.0, test, 5.0e-5);
}

TEST_F(UnitConversionTest, Capacitance) {
  double test;

  test = units::convert<units::farads, units::millifarads>(10.0);
  EXPECT_NEAR(10000.0, test, 5.0e-5);
  test = units::convert<units::picofarads, units::farads>(1000000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::nanofarads, units::farads>(1000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::microfarads, units::farads>(1000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::millifarads, units::farads>(1000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::kilofarads, units::farads>(0.001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::megafarads, units::farads>(0.000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::gigafarads, units::farads>(0.000000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
}

TEST_F(UnitConversionTest, Impedance) {
  double test;

  test = units::convert<units::ohms, units::milliohms>(10.0);
  EXPECT_NEAR(10000.0, test, 5.0e-5);
  test = units::convert<units::picoohms, units::ohms>(1000000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::nanoohms, units::ohms>(1000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::microohms, units::ohms>(1000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::milliohms, units::ohms>(1000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::kiloohms, units::ohms>(0.001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::megaohms, units::ohms>(0.000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::gigaohms, units::ohms>(0.000000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
}

TEST_F(UnitConversionTest, Conductance) {
  double test;

  test = units::convert<units::siemens, units::millisiemens>(10.0);
  EXPECT_NEAR(10000.0, test, 5.0e-5);
  test = units::convert<units::picosiemens, units::siemens>(1000000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::nanosiemens, units::siemens>(1000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::microsiemens, units::siemens>(1000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::millisiemens, units::siemens>(1000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::kilosiemens, units::siemens>(0.001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::megasiemens, units::siemens>(0.000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::gigasiemens, units::siemens>(0.000000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
}

TEST_F(UnitConversionTest, MagneticFlux) {
  double test;

  test = units::convert<units::webers, units::milliwebers>(10.0);
  EXPECT_NEAR(10000.0, test, 5.0e-5);
  test = units::convert<units::picowebers, units::webers>(1000000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::nanowebers, units::webers>(1000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::microwebers, units::webers>(1000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::milliwebers, units::webers>(1000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::kilowebers, units::webers>(0.001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::megawebers, units::webers>(0.000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::gigawebers, units::webers>(0.000000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::maxwells, units::webers>(100000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::nanowebers, units::maxwells>(10.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
}

TEST_F(UnitConversionTest, MagneticFieldStrength) {
  double test;

  test = units::convert<units::teslas, units::milliteslas>(10.0);
  EXPECT_NEAR(10000.0, test, 5.0e-5);
  test = units::convert<units::picoteslas, units::teslas>(1000000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::nanoteslas, units::teslas>(1000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::microteslas, units::teslas>(1000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::milliteslas, units::teslas>(1000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::kiloteslas, units::teslas>(0.001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::megateslas, units::teslas>(0.000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::gigateslas, units::teslas>(0.000000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::gauss, units::teslas>(10000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::nanoteslas, units::gauss>(100000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
}

TEST_F(UnitConversionTest, Inductance) {
  double test;

  test = units::convert<units::henries, units::millihenries>(10.0);
  EXPECT_NEAR(10000.0, test, 5.0e-5);
  test = units::convert<units::picohenries, units::henries>(1000000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::nanohenries, units::henries>(1000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::microhenries, units::henries>(1000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::millihenries, units::henries>(1000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::kilohenries, units::henries>(0.001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::megahenries, units::henries>(0.000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::gigahenries, units::henries>(0.000000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
}

TEST_F(UnitConversionTest, LuminousFlux) {
  double test;

  test = units::convert<units::lumens, units::millilumens>(10.0);
  EXPECT_NEAR(10000.0, test, 5.0e-5);
  test = units::convert<units::picolumens, units::lumens>(1000000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::nanolumens, units::lumens>(1000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::microlumens, units::lumens>(1000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::millilumens, units::lumens>(1000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::kilolumens, units::lumens>(0.001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::megalumens, units::lumens>(0.000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::gigalumens, units::lumens>(0.000000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
}

TEST_F(UnitConversionTest, Illuminance) {
  double test;

  test = units::convert<units::luxes, units::milliluxes>(10.0);
  EXPECT_NEAR(10000.0, test, 5.0e-5);
  test = units::convert<units::picoluxes, units::luxes>(1000000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::nanoluxes, units::luxes>(1000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::microluxes, units::luxes>(1000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::milliluxes, units::luxes>(1000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::kiloluxes, units::luxes>(0.001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::megaluxes, units::luxes>(0.000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::gigaluxes, units::luxes>(0.000000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);

  test = units::convert<units::footcandles, units::luxes>(0.092903);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::lux, units::lumens_per_square_inch>(
      1550.0031000062);
  EXPECT_NEAR(1.0, test, 5.0e-13);
  test = units::convert<units::phots, units::luxes>(0.0001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
}

TEST_F(UnitConversionTest, Radiation) {
  double test;

  test = units::convert<units::becquerels, units::millibecquerels>(10.0);
  EXPECT_NEAR(10000.0, test, 5.0e-5);
  test =
      units::convert<units::picobecquerels, units::becquerels>(1000000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::nanobecquerels, units::becquerels>(1000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::microbecquerels, units::becquerels>(1000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::millibecquerels, units::becquerels>(1000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::kilobecquerels, units::becquerels>(0.001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::megabecquerels, units::becquerels>(0.000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::gigabecquerels, units::becquerels>(0.000000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);

  test = units::convert<units::grays, units::milligrays>(10.0);
  EXPECT_NEAR(10000.0, test, 5.0e-5);
  test = units::convert<units::picograys, units::grays>(1000000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::nanograys, units::grays>(1000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::micrograys, units::grays>(1000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::milligrays, units::grays>(1000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::kilograys, units::grays>(0.001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::megagrays, units::grays>(0.000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::gigagrays, units::grays>(0.000000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);

  test = units::convert<units::sieverts, units::millisieverts>(10.0);
  EXPECT_NEAR(10000.0, test, 5.0e-5);
  test = units::convert<units::picosieverts, units::sieverts>(1000000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::nanosieverts, units::sieverts>(1000000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::microsieverts, units::sieverts>(1000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::millisieverts, units::sieverts>(1000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::kilosieverts, units::sieverts>(0.001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::megasieverts, units::sieverts>(0.000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::gigasieverts, units::sieverts>(0.000000001);
  EXPECT_NEAR(1.0, test, 5.0e-5);

  test = units::convert<units::becquerels, units::curies>(37.0e9);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::becquerels, units::rutherfords>(1000000.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::rads, units::grays>(100.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
}

TEST_F(UnitConversionTest, Torque) {
  double test;

  test = units::convert<units::torque::foot_pounds, units::newton_meter>(1.0);
  EXPECT_NEAR(1.355817948, test, 5.0e-5);
  test = units::convert<units::inch_pounds, units::newton_meter>(1.0);
  EXPECT_NEAR(0.112984829, test, 5.0e-5);
  test = units::convert<units::foot_poundals, units::newton_meter>(1.0);
  EXPECT_NEAR(4.214011009e-2, test, 5.0e-5);
  test = units::convert<units::meter_kilograms, units::newton_meter>(1.0);
  EXPECT_NEAR(9.80665, test, 5.0e-5);
  test = units::convert<units::inch_pound, units::meter_kilogram>(
      86.79616930855788);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::foot_poundals, units::inch_pound>(2.681170713);
  EXPECT_NEAR(1.0, test, 5.0e-5);
}

TEST_F(UnitConversionTest, Volume) {
  double test;

  test = units::convert<units::cubic_meters, units::cubic_meter>(1.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::cubic_millimeters, units::cubic_meter>(1.0);
  EXPECT_NEAR(1.0e-9, test, 5.0e-5);
  test = units::convert<units::cubic_kilometers, units::cubic_meter>(1.0);
  EXPECT_NEAR(1.0e9, test, 5.0e-5);
  test = units::convert<units::liters, units::cubic_meter>(1.0);
  EXPECT_NEAR(0.001, test, 5.0e-5);
  test = units::convert<units::milliliters, units::cubic_meter>(1.0);
  EXPECT_NEAR(1.0e-6, test, 5.0e-5);
  test = units::convert<units::cubic_inches, units::cubic_meter>(1.0);
  EXPECT_NEAR(1.6387e-5, test, 5.0e-10);
  test = units::convert<units::cubic_feet, units::cubic_meter>(1.0);
  EXPECT_NEAR(0.0283168, test, 5.0e-8);
  test = units::convert<units::cubic_yards, units::cubic_meter>(1.0);
  EXPECT_NEAR(0.764555, test, 5.0e-7);
  test = units::convert<units::cubic_miles, units::cubic_meter>(1.0);
  EXPECT_NEAR(4.168e+9, test, 5.0e5);
  test = units::convert<units::gallons, units::cubic_meter>(1.0);
  EXPECT_NEAR(0.00378541, test, 5.0e-8);
  test = units::convert<units::quarts, units::cubic_meter>(1.0);
  EXPECT_NEAR(0.000946353, test, 5.0e-10);
  test = units::convert<units::pints, units::cubic_meter>(1.0);
  EXPECT_NEAR(0.000473176, test, 5.0e-10);
  test = units::convert<units::cups, units::cubic_meter>(1.0);
  EXPECT_NEAR(0.00024, test, 5.0e-6);
  test = units::convert<units::volume::fluid_ounces, units::cubic_meter>(1.0);
  EXPECT_NEAR(2.9574e-5, test, 5.0e-5);
  test = units::convert<units::barrels, units::cubic_meter>(1.0);
  EXPECT_NEAR(0.158987294928, test, 5.0e-13);
  test = units::convert<units::bushels, units::cubic_meter>(1.0);
  EXPECT_NEAR(0.0352391, test, 5.0e-8);
  test = units::convert<units::cords, units::cubic_meter>(1.0);
  EXPECT_NEAR(3.62456, test, 5.0e-6);
  test = units::convert<units::cubic_fathoms, units::cubic_meter>(1.0);
  EXPECT_NEAR(6.11644, test, 5.0e-6);
  test = units::convert<units::tablespoons, units::cubic_meter>(1.0);
  EXPECT_NEAR(1.4787e-5, test, 5.0e-10);
  test = units::convert<units::teaspoons, units::cubic_meter>(1.0);
  EXPECT_NEAR(4.9289e-6, test, 5.0e-11);
  test = units::convert<units::pinches, units::cubic_meter>(1.0);
  EXPECT_NEAR(616.11519921875e-9, test, 5.0e-20);
  test = units::convert<units::dashes, units::cubic_meter>(1.0);
  EXPECT_NEAR(308.057599609375e-9, test, 5.0e-20);
  test = units::convert<units::drops, units::cubic_meter>(1.0);
  EXPECT_NEAR(82.14869322916e-9, test, 5.0e-9);
  test = units::convert<units::fifths, units::cubic_meter>(1.0);
  EXPECT_NEAR(0.00075708236, test, 5.0e-12);
  test = units::convert<units::drams, units::cubic_meter>(1.0);
  EXPECT_NEAR(3.69669e-6, test, 5.0e-12);
  test = units::convert<units::gills, units::cubic_meter>(1.0);
  EXPECT_NEAR(0.000118294, test, 5.0e-10);
  test = units::convert<units::pecks, units::cubic_meter>(1.0);
  EXPECT_NEAR(0.00880977, test, 5.0e-9);
  test = units::convert<units::sacks, units::cubic_meter>(9.4591978);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::shots, units::cubic_meter>(1.0);
  EXPECT_NEAR(4.43603e-5, test, 5.0e-11);
  test = units::convert<units::strikes, units::cubic_meter>(1.0);
  EXPECT_NEAR(0.07047814033376, test, 5.0e-5);
  test = units::convert<units::volume::fluid_ounces, units::milliliters>(1.0);
  EXPECT_NEAR(29.5735, test, 5.0e-5);
}

TEST_F(UnitConversionTest, Density) {
  double test;

  test = units::convert<units::kilograms_per_cubic_meter,
                        units::kilograms_per_cubic_meter>(1.0);
  EXPECT_NEAR(1.0, test, 5.0e-5);
  test = units::convert<units::grams_per_milliliter,
                        units::kilograms_per_cubic_meter>(1.0);
  EXPECT_NEAR(1000.0, test, 5.0e-5);
  test = units::convert<units::kilograms_per_liter,
                        units::kilograms_per_cubic_meter>(1.0);
  EXPECT_NEAR(1000.0, test, 5.0e-5);
  test = units::convert<units::ounces_per_cubic_foot,
                        units::kilograms_per_cubic_meter>(1.0);
  EXPECT_NEAR(1.001153961, test, 5.0e-10);
  test = units::convert<units::ounces_per_cubic_inch,
                        units::kilograms_per_cubic_meter>(1.0);
  EXPECT_NEAR(1.729994044e3, test, 5.0e-7);
  test = units::convert<units::ounces_per_gallon,
                        units::kilograms_per_cubic_meter>(1.0);
  EXPECT_NEAR(7.489151707, test, 5.0e-10);
  test = units::convert<units::pounds_per_cubic_foot,
                        units::kilograms_per_cubic_meter>(1.0);
  EXPECT_NEAR(16.01846337, test, 5.0e-9);
  test = units::convert<units::pounds_per_cubic_inch,
                        units::kilograms_per_cubic_meter>(1.0);
  EXPECT_NEAR(2.767990471e4, test, 5.0e-6);
  test = units::convert<units::pounds_per_gallon,
                        units::kilograms_per_cubic_meter>(1.0);
  EXPECT_NEAR(119.8264273, test, 5.0e-8);
  test = units::convert<units::slugs_per_cubic_foot,
                        units::kilograms_per_cubic_meter>(1.0);
  EXPECT_NEAR(515.3788184, test, 5.0e-6);
}

TEST_F(UnitConversionTest, Concentration) {
  double test;

  test = units::ppm_t(1.0);
  EXPECT_NEAR(1.0e-6, test, 5.0e-12);
  test = units::ppb_t(1.0);
  EXPECT_NEAR(1.0e-9, test, 5.0e-12);
  test = units::ppt_t(1.0);
  EXPECT_NEAR(1.0e-12, test, 5.0e-12);
  test = units::percent_t(18.0);
  EXPECT_NEAR(0.18, test, 5.0e-12);
}

TEST_F(UnitConversionTest, Data) {
  EXPECT_EQ(8, (units::convert<units::byte, units::bit>(1)));

  EXPECT_EQ(1000, (units::convert<units::kilobytes, units::bytes>(1)));
  EXPECT_EQ(1000, (units::convert<units::megabytes, units::kilobytes>(1)));
  EXPECT_EQ(1000, (units::convert<units::gigabytes, units::megabytes>(1)));
  EXPECT_EQ(1000, (units::convert<units::terabytes, units::gigabytes>(1)));
  EXPECT_EQ(1000, (units::convert<units::petabytes, units::terabytes>(1)));
  EXPECT_EQ(1000, (units::convert<units::exabytes, units::petabytes>(1)));

  EXPECT_EQ(1024, (units::convert<units::kibibytes, units::bytes>(1)));
  EXPECT_EQ(1024, (units::convert<units::mebibytes, units::kibibytes>(1)));
  EXPECT_EQ(1024, (units::convert<units::gibibytes, units::mebibytes>(1)));
  EXPECT_EQ(1024, (units::convert<units::tebibytes, units::gibibytes>(1)));
  EXPECT_EQ(1024, (units::convert<units::pebibytes, units::tebibytes>(1)));
  EXPECT_EQ(1024, (units::convert<units::exbibytes, units::pebibytes>(1)));

  EXPECT_EQ(93750000, (units::convert<units::gigabytes, units::kibibits>(12)));

  EXPECT_EQ(1000, (units::convert<units::kilobits, units::bits>(1)));
  EXPECT_EQ(1000, (units::convert<units::megabits, units::kilobits>(1)));
  EXPECT_EQ(1000, (units::convert<units::gigabits, units::megabits>(1)));
  EXPECT_EQ(1000, (units::convert<units::terabits, units::gigabits>(1)));
  EXPECT_EQ(1000, (units::convert<units::petabits, units::terabits>(1)));
  EXPECT_EQ(1000, (units::convert<units::exabits, units::petabits>(1)));

  EXPECT_EQ(1024, (units::convert<units::kibibits, units::bits>(1)));
  EXPECT_EQ(1024, (units::convert<units::mebibits, units::kibibits>(1)));
  EXPECT_EQ(1024, (units::convert<units::gibibits, units::mebibits>(1)));
  EXPECT_EQ(1024, (units::convert<units::tebibits, units::gibibits>(1)));
  EXPECT_EQ(1024, (units::convert<units::pebibits, units::tebibits>(1)));
  EXPECT_EQ(1024, (units::convert<units::exbibits, units::pebibits>(1)));

  // Source: https://en.wikipedia.org/wiki/Binary_prefix
  EXPECT_NEAR(units::percent_t(2.4),
              units::kibibyte_t(1) / units::kilobyte_t(1) - 1, 0.005);
  EXPECT_NEAR(units::percent_t(4.9),
              units::mebibyte_t(1) / units::megabyte_t(1) - 1, 0.005);
  EXPECT_NEAR(units::percent_t(7.4),
              units::gibibyte_t(1) / units::gigabyte_t(1) - 1, 0.005);
  EXPECT_NEAR(units::percent_t(10.0),
              units::tebibyte_t(1) / units::terabyte_t(1) - 1, 0.005);
  EXPECT_NEAR(units::percent_t(12.6),
              units::pebibyte_t(1) / units::petabyte_t(1) - 1, 0.005);
  EXPECT_NEAR(units::percent_t(15.3),
              units::exbibyte_t(1) / units::exabyte_t(1) - 1, 0.005);
}

TEST_F(UnitConversionTest, DataTransferRate) {
  EXPECT_EQ(
      8, (units::convert<units::bytes_per_second, units::bits_per_second>(1)));

  EXPECT_EQ(
      1000,
      (units::convert<units::kilobytes_per_second, units::bytes_per_second>(
          1)));
  EXPECT_EQ(
      1000,
      (units::convert<units::megabytes_per_second, units::kilobytes_per_second>(
          1)));
  EXPECT_EQ(
      1000,
      (units::convert<units::gigabytes_per_second, units::megabytes_per_second>(
          1)));
  EXPECT_EQ(
      1000,
      (units::convert<units::terabytes_per_second, units::gigabytes_per_second>(
          1)));
  EXPECT_EQ(
      1000,
      (units::convert<units::petabytes_per_second, units::terabytes_per_second>(
          1)));
  EXPECT_EQ(
      1000,
      (units::convert<units::exabytes_per_second, units::petabytes_per_second>(
          1)));

  EXPECT_EQ(
      1024,
      (units::convert<units::kibibytes_per_second, units::bytes_per_second>(
          1)));
  EXPECT_EQ(
      1024,
      (units::convert<units::mebibytes_per_second, units::kibibytes_per_second>(
          1)));
  EXPECT_EQ(
      1024,
      (units::convert<units::gibibytes_per_second, units::mebibytes_per_second>(
          1)));
  EXPECT_EQ(
      1024,
      (units::convert<units::tebibytes_per_second, units::gibibytes_per_second>(
          1)));
  EXPECT_EQ(
      1024,
      (units::convert<units::pebibytes_per_second, units::tebibytes_per_second>(
          1)));
  EXPECT_EQ(
      1024,
      (units::convert<units::exbibytes_per_second, units::pebibytes_per_second>(
          1)));

  EXPECT_EQ(
      93750000,
      (units::convert<units::gigabytes_per_second, units::kibibits_per_second>(
          12)));

  EXPECT_EQ(
      1000,
      (units::convert<units::kilobits_per_second, units::bits_per_second>(1)));
  EXPECT_EQ(
      1000,
      (units::convert<units::megabits_per_second, units::kilobits_per_second>(
          1)));
  EXPECT_EQ(
      1000,
      (units::convert<units::gigabits_per_second, units::megabits_per_second>(
          1)));
  EXPECT_EQ(
      1000,
      (units::convert<units::terabits_per_second, units::gigabits_per_second>(
          1)));
  EXPECT_EQ(
      1000,
      (units::convert<units::petabits_per_second, units::terabits_per_second>(
          1)));
  EXPECT_EQ(
      1000,
      (units::convert<units::exabits_per_second, units::petabits_per_second>(
          1)));

  EXPECT_EQ(
      1024,
      (units::convert<units::kibibits_per_second, units::bits_per_second>(1)));
  EXPECT_EQ(
      1024,
      (units::convert<units::mebibits_per_second, units::kibibits_per_second>(
          1)));
  EXPECT_EQ(
      1024,
      (units::convert<units::gibibits_per_second, units::mebibits_per_second>(
          1)));
  EXPECT_EQ(
      1024,
      (units::convert<units::tebibits_per_second, units::gibibits_per_second>(
          1)));
  EXPECT_EQ(
      1024,
      (units::convert<units::pebibits_per_second, units::tebibits_per_second>(
          1)));
  EXPECT_EQ(
      1024,
      (units::convert<units::exbibits_per_second, units::pebibits_per_second>(
          1)));

  // Source: https://en.wikipedia.org/wiki/Binary_prefix
  EXPECT_NEAR(
      units::percent_t(2.4),
      units::kibibytes_per_second_t(1) / units::kilobytes_per_second_t(1) - 1,
      0.005);
  EXPECT_NEAR(
      units::percent_t(4.9),
      units::mebibytes_per_second_t(1) / units::megabytes_per_second_t(1) - 1,
      0.005);
  EXPECT_NEAR(
      units::percent_t(7.4),
      units::gibibytes_per_second_t(1) / units::gigabytes_per_second_t(1) - 1,
      0.005);
  EXPECT_NEAR(
      units::percent_t(10.0),
      units::tebibytes_per_second_t(1) / units::terabytes_per_second_t(1) - 1,
      0.005);
  EXPECT_NEAR(
      units::percent_t(12.6),
      units::pebibytes_per_second_t(1) / units::petabytes_per_second_t(1) - 1,
      0.005);
  EXPECT_NEAR(
      units::percent_t(15.3),
      units::exbibytes_per_second_t(1) / units::exabytes_per_second_t(1) - 1,
      0.005);
}

TEST_F(UnitConversionTest, Pi) {
  EXPECT_TRUE(
      units::traits::is_dimensionless_unit_v<decltype(units::constants::pi)>);
  EXPECT_TRUE(units::traits::is_dimensionless_unit_v<constants::PI>);

  // implicit conversion/arithmetic
  EXPECT_NEAR(3.14159, units::constants::pi, 5.0e-6);
  EXPECT_NEAR(6.28318531, (2 * units::constants::pi), 5.0e-9);
  EXPECT_NEAR(6.28318531, (units::constants::pi + units::constants::pi),
              5.0e-9);
  EXPECT_NEAR(0.0, (units::constants::pi - units::constants::pi), 5.0e-9);
  EXPECT_NEAR(31.00627668, units::math::cpow<3>(units::constants::pi), 5.0e-10);
  EXPECT_NEAR(0.0322515344, (1.0 / units::math::cpow<3>(units::constants::pi)),
              5.0e-11);
  EXPECT_TRUE(std::numbers::pi == units::constants::pi);
  EXPECT_TRUE(1.0 != units::constants::pi);
  EXPECT_TRUE(4.0 > units::constants::pi);
  EXPECT_TRUE(3.0 < units::constants::pi);
  EXPECT_TRUE(units::constants::pi > 3.0);
  EXPECT_TRUE(units::constants::pi < 4.0);

  // explicit conversion
  EXPECT_NEAR(3.14159, units::constants::pi.value(), 5.0e-6);

  // auto multiplication
  EXPECT_TRUE((std::same_as<units::meter_t, decltype(units::constants::pi *
                                                     units::meter_t(1))>));
  EXPECT_TRUE((std::same_as<units::meter_t, decltype(units::meter_t(1) *
                                                     units::constants::pi)>));

  EXPECT_NEAR(std::numbers::pi,
              (units::constants::pi * units::meter_t(1)).value(), 5.0e-10);
  EXPECT_NEAR(std::numbers::pi,
              (units::meter_t(1) * units::constants::pi).value(), 5.0e-10);

  // explicit multiplication
  units::meter_t a = units::constants::pi * units::meter_t(1);
  units::meter_t b = units::meter_t(1) * units::constants::pi;

  EXPECT_NEAR(std::numbers::pi, a.value(), 5.0e-10);
  EXPECT_NEAR(std::numbers::pi, b.value(), 5.0e-10);

  // auto division
  EXPECT_TRUE((std::same_as<units::hertz_t, decltype(units::constants::pi /
                                                     units::second_t(1))>));
  EXPECT_TRUE((std::same_as<units::second_t, decltype(units::second_t(1) /
                                                      units::constants::pi)>));

  EXPECT_NEAR(std::numbers::pi,
              (units::constants::pi / units::second_t(1)).value(), 5.0e-10);
  EXPECT_NEAR(1.0 / std::numbers::pi,
              (units::second_t(1) / units::constants::pi).value(), 5.0e-10);

  // explicit
  units::hertz_t c = units::constants::pi / units::second_t(1);
  units::second_t d = units::second_t(1) / units::constants::pi;

  EXPECT_NEAR(std::numbers::pi, c.value(), 5.0e-10);
  EXPECT_NEAR(1.0 / std::numbers::pi, d.value(), 5.0e-10);
}

TEST_F(UnitConversionTest, Constants) {
  // Source: NIST "2014 CODATA recommended values"
  EXPECT_NEAR(299792458, units::constants::c(), 5.0e-9);
  EXPECT_NEAR(6.67408e-11, units::constants::G(), 5.0e-17);
  EXPECT_NEAR(6.626070040e-34, units::constants::h(), 5.0e-44);
  EXPECT_NEAR(1.2566370614e-6, units::constants::mu0(), 5.0e-17);
  EXPECT_NEAR(8.854187817e-12, units::constants::epsilon0(), 5.0e-21);
  EXPECT_NEAR(376.73031346177, units::constants::Z0(), 5.0e-12);
  EXPECT_NEAR(8987551787.3681764, units::constants::k_e(), 5.0e-6);
  EXPECT_NEAR(1.6021766208e-19, units::constants::e(), 5.0e-29);
  EXPECT_NEAR(9.10938356e-31, units::constants::m_e(), 5.0e-40);
  EXPECT_NEAR(1.672621898e-27, units::constants::m_p(), 5.0e-37);
  EXPECT_NEAR(9.274009994e-24, units::constants::mu_B(), 5.0e-32);
  EXPECT_NEAR(6.022140857e23, units::constants::N_A(), 5.0e14);
  EXPECT_NEAR(8.3144598, units::constants::R(), 5.0e-8);
  EXPECT_NEAR(1.38064852e-23, units::constants::k_B(), 5.0e-31);
  EXPECT_NEAR(96485.33289, units::constants::F(), 5.0e-5);
  EXPECT_NEAR(5.670367e-8, units::constants::sigma(), 5.0e-14);
}

TEST_F(UnitConversionTest, StdChrono) {
  units::nanosecond_t a = std::chrono::nanoseconds(10);
  EXPECT_EQ(units::nanosecond_t(10), a);
  units::microsecond_t b = std::chrono::microseconds(10);
  EXPECT_EQ(units::microsecond_t(10), b);
  units::millisecond_t c = std::chrono::milliseconds(10);
  EXPECT_EQ(units::millisecond_t(10), c);
  units::second_t d = std::chrono::seconds(1);
  EXPECT_EQ(units::second_t(1), d);
  units::minute_t e = std::chrono::minutes(120);
  EXPECT_EQ(units::minute_t(120), e);
  units::hour_t f = std::chrono::hours(2);
  EXPECT_EQ(units::hour_t(2), f);

  std::chrono::nanoseconds g = units::nanosecond_t(100);
  EXPECT_EQ(std::chrono::duration_cast<std::chrono::nanoseconds>(g).count(),
            100);
  std::chrono::nanoseconds h = units::microsecond_t(2);
  EXPECT_EQ(std::chrono::duration_cast<std::chrono::nanoseconds>(h).count(),
            2000);
  std::chrono::nanoseconds i = units::millisecond_t(1);
  EXPECT_EQ(std::chrono::duration_cast<std::chrono::nanoseconds>(i).count(),
            1000000);
  std::chrono::nanoseconds j = units::second_t(1);
  EXPECT_EQ(std::chrono::duration_cast<std::chrono::nanoseconds>(j).count(),
            1000000000);
  std::chrono::nanoseconds k = units::minute_t(1);
  EXPECT_EQ(std::chrono::duration_cast<std::chrono::nanoseconds>(k).count(),
            60000000000);
  std::chrono::nanoseconds l = units::hour_t(1);
  EXPECT_EQ(std::chrono::duration_cast<std::chrono::nanoseconds>(l).count(),
            3600000000000);
}

TEST_F(UnitConversionTest, SquaredTemperature) {
  using units::squared_celsius =
      units::compound_unit<units::squared<units::celsius>>;
  using units::squared_celsius_t = units::unit_t<units::squared_celsius>;
  const units::squared_celsius_t right(100);
  const units::celsius_t rootRight = units::math::sqrt(right);
  EXPECT_EQ(units::celsius_t(10), rootRight);
}

TEST_F(UnitMathTest, Min) {
  units::meter_t a(1);
  units::foot_t c(1);
  EXPECT_EQ(c, units::math::min(a, c));
}

TEST_F(UnitMathTest, Max) {
  units::meter_t a(1);
  units::foot_t c(1);
  EXPECT_EQ(a, units::math::max(a, c));
}

TEST_F(UnitMathTest, Cos) {
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::scalar_t>,
                            typename std::decay_t<decltype(units::math::cos(
                                units::angle::radian_t(0)))>>));
  EXPECT_NEAR(units::scalar_t(-0.41614683654),
              units::math::cos(units::angle::radian_t(2)), 5.0e-11);
  EXPECT_NEAR(units::scalar_t(-0.70710678118),
              units::math::cos(units::angle::degree_t(135)), 5.0e-11);
}

TEST_F(UnitMathTest, Sin) {
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::scalar_t>,
                            typename std::decay_t<decltype(units::math::sin(
                                units::angle::radian_t(0)))>>));
  EXPECT_NEAR(units::scalar_t(0.90929742682),
              units::math::sin(units::angle::radian_t(2)), 5.0e-11);
  EXPECT_NEAR(units::scalar_t(0.70710678118),
              units::math::sin(units::angle::degree_t(135)), 5.0e-11);
}

TEST_F(UnitMathTest, Tan) {
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::scalar_t>,
                            typename std::decay_t<decltype(units::math::tan(
                                units::angle::radian_t(0)))>>));
  EXPECT_NEAR(units::scalar_t(-2.18503986326),
              units::math::tan(units::angle::radian_t(2)), 5.0e-11);
  EXPECT_NEAR(units::scalar_t(-1.0),
              units::math::tan(units::angle::degree_t(135)), 5.0e-11);
}

TEST_F(UnitMathTest, Acos) {
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::angle::radian_t>,
                            typename std::decay_t<decltype(units::math::acos(
                                units::scalar_t(0)))>>));
  EXPECT_NEAR(units::angle::radian_t(2).value(),
              units::math::acos(units::scalar_t(-0.41614683654)).value(),
              5.0e-11);
  EXPECT_NEAR(units::angle::degree_t(135).value(),
              units::angle::degree_t(units::math::acos(units::scalar_t(
                                         -0.70710678118654752440084436210485)))
                  .value(),
              5.0e-12);
}

TEST_F(UnitMathTest, Asin) {
  EXPECT_TRUE(
      (std::same_as<
          typename std::decay_t<units::angle::radian_t>,
          typename std::decay_t<decltype(std::asin(units::scalar_t(0)))>>));
  EXPECT_NEAR(units::angle::radian_t(1.14159265).value(),
              std::asin(units::scalar_t(0.90929742682)).value(), 5.0e-9);
  EXPECT_NEAR(units::angle::degree_t(45).value(),
              units::angle::degree_t(std::asin(units::scalar_t(
                                         0.70710678118654752440084436210485)))
                  .value(),
              5.0e-12);
}

TEST_F(UnitMathTest, Atan) {
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::angle::radian_t>,
                            typename std::decay_t<decltype(units::math::atan(
                                units::scalar_t(0)))>>));
  EXPECT_NEAR(units::angle::radian_t(-1.14159265).value(),
              units::math::atan(units::scalar_t(-2.18503986326)).value(),
              5.0e-9);
  EXPECT_NEAR(
      units::angle::degree_t(-45).value(),
      units::angle::degree_t(units::math::atan(units::scalar_t(-1.0))).value(),
      5.0e-12);
}

TEST_F(UnitMathTest, Atan2) {
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::angle::radian_t>,
                            typename std::decay_t<decltype(units::math::atan2(
                                units::scalar_t(1), units::scalar_t(1)))>>));
  EXPECT_NEAR(
      units::angle::radian_t(std::numbers::pi / 4).value(),
      units::math::atan2(units::scalar_t(2), units::scalar_t(2)).value(),
      5.0e-12);
  EXPECT_NEAR(units::angle::degree_t(45).value(),
              units::angle::degree_t(
                  units::math::atan2(units::scalar_t(2), units::scalar_t(2)))
                  .value(),
              5.0e-12);

  EXPECT_TRUE((std::same_as<typename std::decay_t<units::angle::radian_t>,
                            typename std::decay_t<decltype(units::math::atan2(
                                units::scalar_t(1), units::scalar_t(1)))>>));
  EXPECT_NEAR(
      units::angle::radian_t(std::numbers::pi / 6).value(),
      units::math::atan2(units::scalar_t(1), units::scalar_t(std::sqrt(3)))
          .value(),
      5.0e-12);
  EXPECT_NEAR(
      units::angle::degree_t(30).value(),
      units::angle::degree_t(
          units::math::atan2(units::scalar_t(1), units::scalar_t(std::sqrt(3))))
          .value(),
      5.0e-12);
}

TEST_F(UnitMathTest, Cosh) {
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::scalar_t>,
                            typename std::decay_t<decltype(units::math::cosh(
                                units::angle::radian_t(0)))>>));
  EXPECT_NEAR(units::scalar_t(3.76219569108),
              units::math::cosh(units::angle::radian_t(2)), 5.0e-11);
  EXPECT_NEAR(units::scalar_t(5.32275215),
              units::math::cosh(units::angle::degree_t(135)), 5.0e-9);
}

TEST_F(UnitMathTest, Sinh) {
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::scalar_t>,
                            typename std::decay_t<decltype(units::math::sinh(
                                units::angle::radian_t(0)))>>));
  EXPECT_NEAR(units::scalar_t(3.62686040785),
              units::math::sinh(units::angle::radian_t(2)), 5.0e-11);
  EXPECT_NEAR(units::scalar_t(5.22797192),
              units::math::sinh(units::angle::degree_t(135)), 5.0e-9);
}

TEST_F(UnitMathTest, Tanh) {
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::scalar_t>,
                            typename std::decay_t<decltype(units::math::tanh(
                                units::angle::radian_t(0)))>>));
  EXPECT_NEAR(units::scalar_t(0.96402758007),
              units::math::tanh(units::angle::radian_t(2)), 5.0e-11);
  EXPECT_NEAR(units::scalar_t(0.98219338),
              units::math::tanh(units::angle::degree_t(135)), 5.0e-11);
}

TEST_F(UnitMathTest, Acosh) {
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::angle::radian_t>,
                            typename std::decay_t<decltype(units::math::acosh(
                                units::scalar_t(0)))>>));
  EXPECT_NEAR(units::angle::radian_t(1.316957896924817).value(),
              units::math::acosh(units::scalar_t(2.0)).value(), 5.0e-11);
  EXPECT_NEAR(
      units::angle::degree_t(75.456129290216893).value(),
      units::angle::degree_t(units::math::acosh(units::scalar_t(2.0))).value(),
      5.0e-12);
}

TEST_F(UnitMathTest, Asinh) {
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::angle::radian_t>,
                            typename std::decay_t<decltype(units::math::asinh(
                                units::scalar_t(0)))>>));
  EXPECT_NEAR(units::angle::radian_t(1.443635475178810).value(),
              units::math::asinh(units::scalar_t(2)).value(), 5.0e-9);
  EXPECT_NEAR(
      units::angle::degree_t(82.714219883108939).value(),
      units::angle::degree_t(units::math::asinh(units::scalar_t(2))).value(),
      5.0e-12);
}

TEST_F(UnitMathTest, Atanh) {
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::angle::radian_t>,
                            typename std::decay_t<decltype(units::math::atanh(
                                units::scalar_t(0)))>>));
  EXPECT_NEAR(units::angle::radian_t(0.549306144334055).value(),
              units::math::atanh(units::scalar_t(0.5)).value(), 5.0e-9);
  EXPECT_NEAR(
      units::angle::degree_t(31.472923730945389).value(),
      units::angle::degree_t(units::math::atanh(units::scalar_t(0.5))).value(),
      5.0e-12);
}

TEST_F(UnitMathTest, Exp) {
  double val = 10.0;
  EXPECT_EQ(std::exp(val), units::math::exp(units::scalar_t(val)));
}

TEST_F(UnitMathTest, Log) {
  double val = 100.0;
  EXPECT_EQ(std::log(val), units::math::log(units::scalar_t(val)));
}

TEST_F(UnitMathTest, Log10) {
  double val = 100.0;
  EXPECT_EQ(std::log10(val), units::math::log10(units::scalar_t(val)));
}

TEST_F(UnitMathTest, Modf) {
  double val = 100.0;
  double modfr1;
  units::scalar_t modfr2;
  EXPECT_EQ(std::modf(val, &modfr1),
            units::math::modf(units::scalar_t(val), &modfr2));
  EXPECT_EQ(modfr1, modfr2);
}

TEST_F(UnitMathTest, Exp2) {
  double val = 10.0;
  EXPECT_EQ(std::exp2(val), units::math::exp2(units::scalar_t(val)));
}

TEST_F(UnitMathTest, Expm1) {
  double val = 10.0;
  EXPECT_EQ(std::expm1(val), units::math::expm1(units::scalar_t(val)));
}

TEST_F(UnitMathTest, Log1p) {
  double val = 10.0;
  EXPECT_EQ(std::log1p(val), units::math::log1p(units::scalar_t(val)));
}

TEST_F(UnitMathTest, Log2) {
  double val = 10.0;
  EXPECT_EQ(std::log2(val), units::math::log2(units::scalar_t(val)));
}

TEST_F(UnitMathTest, Pow) {
  bool isSame;
  units::meter_t value(10.0);

  auto sq = units::math::pow<2>(value);
  EXPECT_NEAR(100.0, sq(), 5.0e-2);
  isSame = std::same_as<decltype(sq), units::square_meter_t>;
  EXPECT_TRUE(isSame);

  auto cube = units::math::pow<3>(value);
  EXPECT_NEAR(1000.0, cube(), 5.0e-2);
  isSame =
      std::same_as<decltype(cube), units::unit_t<units::cubed<units::meter>>>;
  EXPECT_TRUE(isSame);

  auto fourth = units::math::pow<4>(value);
  EXPECT_NEAR(10000.0, fourth(), 5.0e-2);
  isSame = std::same_as<
      decltype(fourth),
      units::unit_t<units::compound_unit<units::squared<units::meter>,
                                         units::squared<units::meter>>>>;
  EXPECT_TRUE(isSame);
}

TEST_F(UnitMathTest, Sqrt) {
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::meter_t>,
                            typename std::decay_t<decltype(units::math::sqrt(
                                units::square_meter_t(4.0)))>>));
  EXPECT_NEAR(units::meter_t(2.0).value(),
              units::math::sqrt(units::square_meter_t(4.0)).value(), 5.0e-9);

  EXPECT_TRUE((std::same_as<typename std::decay_t<units::angle::radian_t>,
                            typename std::decay_t<decltype(units::math::sqrt(
                                units::steradian_t(16.0)))>>));
  EXPECT_NEAR(units::angle::radian_t(4.0).value(),
              units::math::sqrt(units::steradian_t(16.0)).value(), 5.0e-9);

  EXPECT_TRUE(
      (std::is_convertible_v<typename std::decay_t<units::foot_t>,
                             typename std::decay_t<decltype(units::math::sqrt(
                                 units::square_foot_t(10.0)))>>));

  // for rational conversion (i.e. no integral root) let's check a bunch of
  // different ways this could go wrong
  units::foot_t resultFt = units::math::sqrt(units::square_foot_t(10.0));
  EXPECT_NEAR(units::foot_t(3.16227766017).value(),
              units::math::sqrt(units::square_foot_t(10.0)).value(), 5.0e-9);
  EXPECT_NEAR(units::foot_t(3.16227766017).value(), resultFt.value(), 5.0e-9);
  EXPECT_EQ(resultFt, units::math::sqrt(units::square_foot_t(10.0)));
}

TEST_F(UnitMathTest, Hypot) {
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::meter_t>,
                            typename std::decay_t<decltype(units::math::hypot(
                                units::meter_t(3.0), units::meter_t(4.0)))>>));
  EXPECT_NEAR(
      units::meter_t(5.0).value(),
      (units::math::hypot(units::meter_t(3.0), units::meter_t(4.0))).value(),
      5.0e-9);

  EXPECT_TRUE(
      (std::same_as<typename std::decay_t<units::foot_t>,
                    typename std::decay_t<decltype(units::math::hypot(
                        units::foot_t(3.0), units::meter_t(1.2192)))>>));
  EXPECT_NEAR(
      units::foot_t(5.0).value(),
      (units::math::hypot(units::foot_t(3.0), units::meter_t(1.2192))).value(),
      5.0e-9);
}

TEST_F(UnitMathTest, Ceil) {
  double val = 101.1;
  EXPECT_EQ(std::ceil(val), units::math::ceil(units::meter_t(val)).value());
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::meter_t>,
                            typename std::decay_t<decltype(units::math::ceil(
                                units::meter_t(val)))>>));
}

TEST_F(UnitMathTest, Floor) {
  double val = 101.1;
  EXPECT_EQ(std::floor(val), units::math::floor(units::scalar_t(val)));
}

TEST_F(UnitMathTest, Fmod) {
  EXPECT_EQ(
      std::fmod(100.0, 101.2),
      units::math::fmod(units::meter_t(100.0), units::meter_t(101.2)).value());
}

TEST_F(UnitMathTest, Trunc) {
  double val = 101.1;
  EXPECT_EQ(std::trunc(val), units::math::trunc(units::scalar_t(val)));
}

TEST_F(UnitMathTest, Round) {
  double val = 101.1;
  EXPECT_EQ(std::round(val), units::math::round(units::scalar_t(val)));
}

TEST_F(UnitMathTest, Copysign) {
  double sign = -1;
  units::meter_t val(5.0);
  EXPECT_EQ(units::meter_t(-5.0), units::math::copysign(val, sign));
  EXPECT_EQ(units::meter_t(-5.0),
            units::math::copysign(val, units::angle::radian_t(sign)));
}

TEST_F(UnitMathTest, Fdim) {
  EXPECT_EQ(units::meter_t(0.0),
            units::math::fdim(units::meter_t(8.0), units::meter_t(10.0)));
  EXPECT_EQ(units::meter_t(2.0),
            units::math::fdim(units::meter_t(10.0), units::meter_t(8.0)));
  EXPECT_NEAR(
      units::meter_t(9.3904).value(),
      units::math::fdim(units::meter_t(10.0), units::foot_t(2.0)).value(),
      5.0e-320);  // not sure why they aren't comparing exactly equal,
                  // but clearly they are.
}

TEST_F(UnitMathTest, Fmin) {
  EXPECT_EQ(units::meter_t(8.0),
            units::math::fmin(units::meter_t(8.0), units::meter_t(10.0)));
  EXPECT_EQ(units::meter_t(8.0),
            units::math::fmin(units::meter_t(10.0), units::meter_t(8.0)));
  EXPECT_EQ(units::foot_t(2.0),
            units::math::fmin(units::meter_t(10.0), units::foot_t(2.0)));
}

TEST_F(UnitMathTest, Fmax) {
  EXPECT_EQ(units::meter_t(10.0),
            units::math::fmax(units::meter_t(8.0), units::meter_t(10.0)));
  EXPECT_EQ(units::meter_t(10.0),
            units::math::fmax(units::meter_t(10.0), units::meter_t(8.0)));
  EXPECT_EQ(units::meter_t(10.0),
            units::math::fmax(units::meter_t(10.0), units::foot_t(2.0)));
}

TEST_F(UnitMathTest, Fabs) {
  EXPECT_EQ(units::meter_t(10.0), units::math::fabs(units::meter_t(-10.0)));
  EXPECT_EQ(units::meter_t(10.0), units::math::fabs(units::meter_t(10.0)));
}

TEST_F(UnitMathTest, Abs) {
  EXPECT_EQ(units::meter_t(10.0), units::math::abs(units::meter_t(-10.0)));
  EXPECT_EQ(units::meter_t(10.0), units::math::abs(units::meter_t(10.0)));
}

// Constexpr
TEST_F(ConstexprTest, Construction) {
  constexpr units::meter_t result0(0);
  constexpr auto result1 = units::make_unit<units::meter_t>(1);
  constexpr auto result2 = units::meter_t(2);
  constexpr auto result3 = -3_m;

  EXPECT_EQ(units::meter_t(0), result0);
  EXPECT_EQ(units::meter_t(1), result1);
  EXPECT_EQ(units::meter_t(2), result2);
  EXPECT_EQ(units::meter_t(-3), result3);

  EXPECT_TRUE(noexcept(result0));
  EXPECT_TRUE(noexcept(result1));
  EXPECT_TRUE(noexcept(result2));
  EXPECT_TRUE(noexcept(result3));
}

TEST_F(ConstexprTest, Constants) {
  EXPECT_TRUE(noexcept(units::constants::c()));
  EXPECT_TRUE(noexcept(units::constants::G()));
  EXPECT_TRUE(noexcept(units::constants::h()));
  EXPECT_TRUE(noexcept(units::constants::mu0()));
  EXPECT_TRUE(noexcept(units::constants::epsilon0()));
  EXPECT_TRUE(noexcept(units::constants::Z0()));
  EXPECT_TRUE(noexcept(units::constants::k_e()));
  EXPECT_TRUE(noexcept(units::constants::e()));
  EXPECT_TRUE(noexcept(units::constants::m_e()));
  EXPECT_TRUE(noexcept(units::constants::m_p()));
  EXPECT_TRUE(noexcept(units::constants::mu_B()));
  EXPECT_TRUE(noexcept(units::constants::N_A()));
  EXPECT_TRUE(noexcept(units::constants::R()));
  EXPECT_TRUE(noexcept(units::constants::k_B()));
  EXPECT_TRUE(noexcept(units::constants::F()));
  EXPECT_TRUE(noexcept(units::constants::sigma()));
}

TEST_F(ConstexprTest, Arithmetic) {
  constexpr auto result0(1_m + 1_m);
  constexpr auto result1(1_m - 1_m);
  constexpr auto result2(1_m * 1_m);
  constexpr auto result3(1_m / 1_m);
  constexpr auto result4(units::meter_t(1) + units::meter_t(1));
  constexpr auto result5(units::meter_t(1) - units::meter_t(1));
  constexpr auto result6(units::meter_t(1) * units::meter_t(1));
  constexpr auto result7(units::meter_t(1) / units::meter_t(1));
  constexpr auto result8(units::math::cpow<2>(units::meter_t(2)));
  constexpr auto result9 = units::math::cpow<3>(2_m);
  constexpr auto result10 = 2_m * 2_m;

  EXPECT_TRUE(noexcept(result0));
  EXPECT_TRUE(noexcept(result1));
  EXPECT_TRUE(noexcept(result2));
  EXPECT_TRUE(noexcept(result3));
  EXPECT_TRUE(noexcept(result4));
  EXPECT_TRUE(noexcept(result5));
  EXPECT_TRUE(noexcept(result6));
  EXPECT_TRUE(noexcept(result7));
  EXPECT_TRUE(noexcept(result8));
  EXPECT_TRUE(noexcept(result9));
  EXPECT_TRUE(noexcept(result10));

  EXPECT_EQ(8_cu_m, result9);
  EXPECT_EQ(4_sq_m, result10);
}

TEST_F(ConstexprTest, Relational) {
  constexpr bool equalityTrue = (1_m == 1_m);
  constexpr bool equalityFalse = (1_m == 2_m);
  constexpr bool lessThanTrue = (1_m < 2_m);
  constexpr bool lessThanFalse = (1_m < 1_m);
  constexpr bool lessThanEqualTrue1 = (1_m <= 1_m);
  constexpr bool lessThanEqualTrue2 = (1_m <= 2_m);
  constexpr bool lessThanEqualFalse = (1_m < 0_m);
  constexpr bool greaterThanTrue = (2_m > 1_m);
  constexpr bool greaterThanFalse = (2_m > 2_m);
  constexpr bool greaterThanEqualTrue1 = (2_m >= 1_m);
  constexpr bool greaterThanEqualTrue2 = (2_m >= 2_m);
  constexpr bool greaterThanEqualFalse = (2_m > 3_m);

  EXPECT_TRUE(equalityTrue);
  EXPECT_TRUE(lessThanTrue);
  EXPECT_TRUE(lessThanEqualTrue1);
  EXPECT_TRUE(lessThanEqualTrue2);
  EXPECT_TRUE(greaterThanTrue);
  EXPECT_TRUE(greaterThanEqualTrue1);
  EXPECT_TRUE(greaterThanEqualTrue2);
  EXPECT_FALSE(equalityFalse);
  EXPECT_FALSE(lessThanFalse);
  EXPECT_FALSE(lessThanEqualFalse);
  EXPECT_FALSE(greaterThanFalse);
  EXPECT_FALSE(greaterThanEqualFalse);
}

TEST_F(ConstexprTest, StdArray) {
  constexpr std::array<units::meter_t, 5> arr = {0_m, 1_m, 2_m, 3_m, 4_m};
  constexpr bool equal = (arr[3] == 3_m);
  EXPECT_TRUE(equal);
}

TEST_F(CompileTimeArithmeticTest, UnitValueType) {
  using mRatio = units::unit_value_t<units::meters, 3, 2>;
  EXPECT_EQ(units::meter_t(1.5), mRatio::value());
}

TEST_F(CompileTimeArithmeticTest, IsUnitValueType) {
  using mRatio = units::unit_value_t<units::meters, 3, 2>;

  EXPECT_TRUE((units::traits::is_unit_value_t_v<mRatio>));
  EXPECT_FALSE((units::traits::is_unit_value_t_v<units::meter_t>));
  EXPECT_FALSE((units::traits::is_unit_value_t_v<double>));

  EXPECT_TRUE((units::traits::is_unit_value_t_v<mRatio, units::meters>));
  EXPECT_FALSE(
      (units::traits::is_unit_value_t_v<units::meter_t, units::meters>));
  EXPECT_FALSE((units::traits::is_unit_value_t_v<double, units::meters>));
}

TEST_F(CompileTimeArithmeticTest, IsUnitValueTypeCategory) {
  using mRatio = units::unit_value_t<units::feet, 3, 2>;
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::length_unit,
                                                 mRatio>));
  EXPECT_FALSE(
      (units::traits::is_unit_value_t_category_v<units::category::angle_unit,
                                                 mRatio>));
  EXPECT_FALSE(
      (units::traits::is_unit_value_t_category_v<units::category::length_unit,
                                                 units::meter_t>));
  EXPECT_FALSE(
      (units::traits::is_unit_value_t_category_v<units::category::length_unit,
                                                 double>));
}

TEST_F(CompileTimeArithmeticTest, UnitValueAdd) {
  using mRatio = units::unit_value_t<units::meters, 3, 2>;

  using sum = units::unit_value_add<mRatio, mRatio>;
  EXPECT_EQ(units::meter_t(3.0), sum::value());
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::length_unit,
                                                 sum>));

  using ftRatio = units::unit_value_t<units::feet, 1>;

  using sumf = units::unit_value_add<ftRatio, mRatio>;
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::foot_t>,
                            typename std::decay_t<decltype(sumf::value())>>));
  EXPECT_NEAR(5.92125984, sumf::value().value(), 5.0e-8);
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::length_unit,
                                                 sumf>));

  using cRatio = units::unit_value_t<units::celsius, 1>;
  using fRatio = units::unit_value_t<units::fahrenheit, 2>;

  using sumc = units::unit_value_add<cRatio, fRatio>;
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::celsius_t>,
                            typename std::decay_t<decltype(sumc::value())>>));
  EXPECT_NEAR(2.11111111111, sumc::value().value(), 5.0e-8);
  EXPECT_TRUE((units::traits::is_unit_value_t_category_v<
               units::category::temperature_unit, sumc>));

  using rRatio = units::unit_value_t<units::angle::radian, 1>;
  using dRatio = units::unit_value_t<units::angle::degree, 3>;

  using sumr = units::unit_value_add<rRatio, dRatio>;
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::angle::radian_t>,
                            typename std::decay_t<decltype(sumr::value())>>));
  EXPECT_NEAR(1.05235988, sumr::value().value(), 5.0e-8);
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::angle_unit,
                                                 sumr>));
}

TEST_F(CompileTimeArithmeticTest, UnitValueSubtract) {
  using mRatio = units::unit_value_t<units::meters, 3, 2>;

  using diff = units::unit_value_subtract<mRatio, mRatio>;
  EXPECT_EQ(units::meter_t(0), diff::value());
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::length_unit,
                                                 diff>));

  using ftRatio = units::unit_value_t<units::feet, 1>;

  using difff = units::unit_value_subtract<ftRatio, mRatio>;
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::foot_t>,
                            typename std::decay_t<decltype(difff::value())>>));
  EXPECT_NEAR(-3.92125984, difff::value().value(), 5.0e-8);
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::length_unit,
                                                 difff>));

  using cRatio = units::unit_value_t<units::celsius, 1>;
  using fRatio = units::unit_value_t<units::fahrenheit, 2>;

  using diffc = units::unit_value_subtract<cRatio, fRatio>;
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::celsius_t>,
                            typename std::decay_t<decltype(diffc::value())>>));
  EXPECT_NEAR(-0.11111111111, diffc::value().value(), 5.0e-8);
  EXPECT_TRUE((units::traits::is_unit_value_t_category_v<
               units::category::temperature_unit, diffc>));

  using rRatio = units::unit_value_t<units::angle::radian, 1>;
  using dRatio = units::unit_value_t<units::angle::degree, 3>;

  using diffr = units::unit_value_subtract<rRatio, dRatio>;
  EXPECT_TRUE((std::same_as<typename std::decay_t<units::angle::radian_t>,
                            typename std::decay_t<decltype(diffr::value())>>));
  EXPECT_NEAR(0.947640122, diffr::value().value(), 5.0e-8);
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::angle_unit,
                                                 diffr>));
}

TEST_F(CompileTimeArithmeticTest, UnitValueMultiply) {
  using mRatio = units::unit_value_t<units::meters, 2>;
  using ftRatio = units::unit_value_t<units::feet, 656168, 100000>;  // 2 meter

  using product = units::unit_value_multiply<mRatio, mRatio>;
  EXPECT_EQ(units::square_meter_t(4), product::value());
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::area_unit,
                                                 product>));

  using productM = units::unit_value_multiply<mRatio, ftRatio>;

  EXPECT_TRUE(
      (std::same_as<typename std::decay_t<units::square_meter_t>,
                    typename std::decay_t<decltype(productM::value())>>));
  EXPECT_NEAR(4.0, productM::value().value(), 5.0e-7);
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::area_unit,
                                                 productM>));

  using productF = units::unit_value_multiply<ftRatio, mRatio>;
  EXPECT_TRUE(
      (std::same_as<typename std::decay_t<units::square_foot_t>,
                    typename std::decay_t<decltype(productF::value())>>));
  EXPECT_NEAR(43.0556444224, productF::value().value(), 5.0e-6);
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::area_unit,
                                                 productF>));

  using productF2 = units::unit_value_multiply<ftRatio, ftRatio>;
  EXPECT_TRUE(
      (std::same_as<typename std::decay_t<units::square_foot_t>,
                    typename std::decay_t<decltype(productF2::value())>>));
  EXPECT_NEAR(43.0556444224, productF2::value().value(), 5.0e-8);
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::area_unit,
                                                 productF2>));

  using nRatio = units::unit_value_t<units::force::newton, 5>;

  using productN = units::unit_value_multiply<nRatio, ftRatio>;
  EXPECT_FALSE(
      (std::same_as<typename std::decay_t<units::torque::newton_meter_t>,
                    typename std::decay_t<decltype(productN::value())>>));
  EXPECT_TRUE((std::is_convertible_v<
               typename std::decay_t<units::torque::newton_meter_t>,
               typename std::decay_t<decltype(productN::value())>>));
  EXPECT_NEAR(32.8084, productN::value().value(), 5.0e-8);
  EXPECT_NEAR(10.0, (productN::value().convert<newton_meter>().value()),
              5.0e-7);
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::torque_unit,
                                                 productN>));

  using r1Ratio = units::unit_value_t<units::angle::radian, 11, 10>;
  using r2Ratio = units::unit_value_t<units::angle::radian, 22, 10>;

  using productR = units::unit_value_multiply<r1Ratio, r2Ratio>;
  EXPECT_TRUE(
      (std::same_as<typename std::decay_t<units::steradian_t>,
                    typename std::decay_t<decltype(productR::value())>>));
  EXPECT_NEAR(2.42, productR::value().value(), 5.0e-8);
  EXPECT_NEAR(7944.39137,
              (productR::value().convert<units::degrees_squared>().value()),
              5.0e-6);
  EXPECT_TRUE((units::traits::is_unit_value_t_category_v<
               units::category::solid_angle_unit, productR>));
}

TEST_F(CompileTimeArithmeticTest, UnitValueDivide) {
  using mRatio = units::unit_value_t<units::meters, 2>;
  using ftRatio =
      units::unit_value_t<units::feet, 656168, 100000>;  // 2 units::meter

  using product = units::unit_value_divide<mRatio, mRatio>;
  EXPECT_EQ(units::scalar_t(1), product::value());
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::scalar_unit,
                                                 product>));

  using productM = units::unit_value_divide<mRatio, ftRatio>;

  EXPECT_TRUE(
      (std::same_as<typename std::decay_t<units::scalar_t>,
                    typename std::decay_t<decltype(productM::value())>>));
  EXPECT_NEAR(1, productM::value().value(), 5.0e-7);
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::scalar_unit,
                                                 productM>));

  using productF = units::unit_value_divide<ftRatio, mRatio>;
  EXPECT_TRUE(
      (std::same_as<typename std::decay_t<units::scalar_t>,
                    typename std::decay_t<decltype(productF::value())>>));
  EXPECT_NEAR(1.0, productF::value().value(), 5.0e-6);
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::scalar_unit,
                                                 productF>));

  using productF2 = units::unit_value_divide<ftRatio, ftRatio>;
  EXPECT_TRUE(
      (std::same_as<typename std::decay_t<units::scalar_t>,
                    typename std::decay_t<decltype(productF2::value())>>));
  EXPECT_NEAR(1.0, productF2::value().value(), 5.0e-8);
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::scalar_unit,
                                                 productF2>));

  using sRatio = units::unit_value_t<units::seconds, 10>;

  using productMS = units::unit_value_divide<mRatio, sRatio>;
  EXPECT_TRUE(
      (std::same_as<typename std::decay_t<units::meters_per_second_t>,
                    typename std::decay_t<decltype(productMS::value())>>));
  EXPECT_NEAR(0.2, productMS::value().value(), 5.0e-8);
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::velocity_unit,
                                                 productMS>));

  using rRatio = units::unit_value_t<units::angle::radian, 20>;

  using productRS = units::unit_value_divide<rRatio, sRatio>;
  EXPECT_TRUE(
      (std::same_as<typename std::decay_t<units::radians_per_second_t>,
                    typename std::decay_t<decltype(productRS::value())>>));
  EXPECT_NEAR(2, productRS::value().value(), 5.0e-8);
  EXPECT_NEAR(114.592,
              (productRS::value().convert<units::degrees_per_second>().value()),
              5.0e-4);
  EXPECT_TRUE((units::traits::is_unit_value_t_category_v<
               units::category::angular_velocity_unit, productRS>));
}

TEST_F(CompileTimeArithmeticTest, UnitValuePower) {
  using mRatio = units::unit_value_t<units::meters, 2>;

  using sq = units::unit_value_power<mRatio, 2>;
  EXPECT_TRUE(
      (std::is_convertible_v<typename std::decay_t<units::square_meter_t>,
                             typename std::decay_t<decltype(sq::value())>>));
  EXPECT_NEAR(4, sq::value().value(), 5.0e-8);
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::area_unit,
                                                 sq>));

  using rRatio = units::unit_value_t<units::angle::radian, 18, 10>;

  using sqr = units::unit_value_power<rRatio, 2>;
  EXPECT_TRUE(
      (std::is_convertible_v<typename std::decay_t<units::steradian_t>,
                             typename std::decay_t<decltype(sqr::value())>>));
  EXPECT_NEAR(3.24, sqr::value().value(), 5.0e-8);
  EXPECT_NEAR(10636.292574038049895092690529904,
              (sqr::value().convert<units::degrees_squared>().value()),
              5.0e-10);
  EXPECT_TRUE((units::traits::is_unit_value_t_category_v<
               units::category::solid_angle_unit, sqr>));
}

TEST_F(CompileTimeArithmeticTest, UnitValueSqrt) {
  using mRatio = units::unit_value_t<units::square_meters, 10>;

  using root = units::unit_value_sqrt<mRatio>;
  EXPECT_TRUE(
      (std::is_convertible_v<typename std::decay_t<units::meter_t>,
                             typename std::decay_t<decltype(root::value())>>));
  EXPECT_NEAR(3.16227766017, root::value().value(), 5.0e-9);
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::length_unit,
                                                 root>));

  using hRatio = units::unit_value_t<units::hectare, 51, 7>;

  using rooth = units::unit_value_sqrt<hRatio, 100000000>;
  EXPECT_TRUE(
      (std::is_convertible_v<typename std::decay_t<units::mile_t>,
                             typename std::decay_t<decltype(rooth::value())>>));
  EXPECT_NEAR(2.69920623253, rooth::value().value(), 5.0e-8);
  EXPECT_NEAR(269.920623, rooth::value().convert<units::meters>().value(),
              5.0e-6);
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::length_unit,
                                                 rooth>));

  using rRatio = units::unit_value_t<units::steradian, 18, 10>;

  using rootr = units::unit_value_sqrt<rRatio>;
  EXPECT_TRUE((units::traits::is_angle_unit_v<decltype(rootr::value())>));
  EXPECT_NEAR(1.3416407865, rootr::value().value(), 5.0e-8);
  EXPECT_NEAR(76.870352574,
              rootr::value().convert<units::angle::degrees>().value(), 5.0e-6);
  EXPECT_TRUE(
      (units::traits::is_unit_value_t_category_v<units::category::angle_unit,
                                                 rootr>));
}

TEST_F(CaseStudiesTest, RadarRangeEquation) {
  units::watt_t P_t;            // transmit power
  units::scalar_t G;            // gain
  units::meter_t lambda;        // wavelength
  units::square_meter_t sigma;  // radar cross section
  units::meter_t R;             // range
  units::kelvin_t T_s;          // system noise temp
  units::hertz_t B_n;           // bandwidth
  units::scalar_t L;            // loss

  P_t = units::megawatt_t(1.4);
  G = dB_t(33.0);
  lambda = units::constants::c / units::megahertz_t(2800);
  sigma = units::square_meter_t(1.0);
  R = units::meter_t(111000.0);
  T_s = units::kelvin_t(950.0);
  B_n = units::megahertz_t(1.67);
  L = dB_t(8.0);

  units::scalar_t SNR =
      (P_t * units::math::pow<2>(G) * units::math::pow<2>(lambda) * sigma) /
      (units::math::pow<3>(4 * units::constants::pi) * units::math::pow<4>(R) *
       units::constants::k_B * T_s * B_n * L);

  EXPECT_NEAR(1.535, SNR(), 5.0e-4);
}

TEST_F(CaseStudiesTest, PythagoreanTheorum) {
  EXPECT_EQ(units::meter_t(3), RightTriangle::a::value());
  EXPECT_EQ(units::meter_t(4), RightTriangle::b::value());
  EXPECT_EQ(units::meter_t(5), RightTriangle::c::value());
  EXPECT_TRUE(units::math::pow<2>(RightTriangle::a::value()) +
                  units::math::pow<2>(RightTriangle::b::value()) ==
              units::math::pow<2>(RightTriangle::c::value()));
}
