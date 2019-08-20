#include "gtest/gtest.h"
#include <cmath>
#include <random>
#include <chrono>
#include <frc/controller/PeriodVariantLoop.h>
#include <control/PeriodVariant/ElevatorCoeffs.h>
#include <Eigen/Core>

using namespace frc;
using namespace std::chrono;

constexpr double kNominalDt = 0.005;
constexpr double kPositionStddev = 0.0001;

void update(PeriodVariantLoop<2, 1, 1>& loop, double dt, double yNoise) {

  Eigen::Matrix<double, 1, 1> y = loop.GetPlant().UpdateY(loop.Xhat(), loop.U()) + Eigen::Matrix<double, 1, 1>(yNoise);

  loop.Correct(y);
  loop.Predict(duration_cast<nanoseconds>(duration<double>(dt)));
}

TEST(PeriodVariant, TestEnabled) {
  PeriodVariantLoop<2, 1, 1> m_loop{MakeElevatorPVLoop()};
  std::default_random_engine generator;
  std::normal_distribution<double> rand;

  Eigen::Matrix<double, 2, 1> references;
  references << 2.0, 0.0;
  m_loop.SetNextR(references);

  m_loop.Enable();

  for(int i = 0; i < 350; i++) {
    double dtNoise = rand(generator);
    double dt = std::abs(kNominalDt + (dtNoise * 0.02));
    double yNoise = rand(generator) * (kPositionStddev / kNominalDt) * dt;
    update(m_loop, dt, yNoise);
    EXPECT_GE(m_loop.U(0), -12.0);
    EXPECT_LE(m_loop.U(0), 12.0);
  }

  EXPECT_LT(abs(m_loop.Xhat(0) - 2.0), 0.05);
  EXPECT_LT(abs(m_loop.Xhat(1) - 0.0), 0.5);
}


TEST(PeriodVariant, TestDisabled) {
  PeriodVariantLoop<2, 1, 1> m_loop{MakeElevatorPVLoop()};

  Eigen::Matrix<double, 2, 1> references;
  references << 2.0, 0.0;

  m_loop.SetNextR(references);

  for(int i = 0; i < 100; i++) {
    update(m_loop, 0.0, 0.0);
  }

  auto position = m_loop.Xhat(0);
  auto velocity = m_loop.Xhat(1);

  std::cerr << position << "\n";
  std::cerr << velocity << "\n";

  EXPECT_LT(abs(position), 1E-9);
  EXPECT_LT(abs(velocity), 1E-9);
}