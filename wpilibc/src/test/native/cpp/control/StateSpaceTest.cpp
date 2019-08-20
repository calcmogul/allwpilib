#include "gtest/gtest.h"
#include <cmath>
#include <random>
#include <frc/controller/StateSpaceLoop.h>
#include <control/StateSpace/ElevatorCoeffs.h>
#include <Eigen/Core>

using namespace frc;

constexpr double kPositionStddev = 0.0001;

void update(StateSpaceLoop<2, 1, 1>& loop, double noise) {
  std::cerr << "Getting noise value " << noise << "\n";
  auto y = loop.GetPlant().UpdateY(loop.Xhat(), loop.U()) + Eigen::Matrix<double, 1, 1>(noise);
  loop.Correct(y);
  loop.Predict();
}

TEST(StateSpace, TestEnabled) {
  std::default_random_engine generator;
  std::normal_distribution<double> rand{0.0, kPositionStddev};

  StateSpaceLoop<2, 1, 1> m_loop{MakeElevatorLoop()};

  Eigen::Matrix<double, 2, 1> references;
  references << 2.0, 0.0;
  m_loop.SetNextR(references);

  
  m_loop.Enable();

  for(int i = 0; i < 350; i++) {
      update(m_loop, rand(generator));
      EXPECT_TRUE(m_loop.U(0) >= -12.0 && m_loop.U(0) <= 12.0);
  }

  EXPECT_LT(std::abs(m_loop.Xhat(0) - 2.0), 0.05);
  EXPECT_LT(std::abs(m_loop.Xhat(1) - 0.0), 0.5);
}

TEST(StateSpace, TestDisabled) {
  StateSpaceLoop<2, 1, 1> m_loop{MakeElevatorLoop()};

  Eigen::Matrix<double, 2, 1> references;
  references << 2.0, 0.0;
  m_loop.SetNextR(references);

  EXPECT_DOUBLE_EQ(m_loop.Xhat(0), 0.0);
  EXPECT_DOUBLE_EQ(m_loop.Xhat(1), 0.0);

  for(int i = 0; i < 100; i++) {
      update(m_loop, 0.0);
  }

  EXPECT_LT(std::abs(m_loop.Xhat(0)), 1E-9);
  EXPECT_LT(std::abs(m_loop.Xhat(1)), 1E-9);
}