// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "wpi/math/system/Discretization.hpp"

#include <cstdlib>
#include <functional>

#include <Eigen/Eigenvalues>
#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>

#include "wpi/math/linalg/EigenCore.hpp"
#include "wpi/math/system/NumericalIntegration.hpp"
#include "wpi/units/time.hpp"

// Check that for a simple second-order system that we can easily analyze
// analytically,
TEST_CASE("DiscretizationTest DiscretizeA", "[wpimath]") {
  wpi::math::Matrixd<2, 2> contA{{0, 1}, {0, 0}};

  wpi::math::Vectord<2> x0{1, 1};
  wpi::math::Matrixd<2, 2> discA;

  wpi::math::DiscretizeA<2>(contA, 1_s, &discA);
  wpi::math::Vectord<2> x1Discrete = discA * x0;

  // We now have pos = vel = 1 and accel = 0, which should give us:
  wpi::math::Vectord<2> x1Truth{1.0 * x0(0) + 1.0 * x0(1),
                                0.0 * x0(0) + 1.0 * x0(1)};

  CHECK(x1Truth == x1Discrete);
}

// Check that for a simple second-order system that we can easily analyze
// analytically,
TEST_CASE("DiscretizationTest DiscretizeAB", "[wpimath]") {
  wpi::math::Matrixd<2, 2> contA{{0, 1}, {0, 0}};
  wpi::math::Matrixd<2, 1> contB{0, 1};

  wpi::math::Vectord<2> x0{1, 1};
  wpi::math::Vectord<1> u{1};
  wpi::math::Matrixd<2, 2> discA;
  wpi::math::Matrixd<2, 1> discB;

  wpi::math::DiscretizeAB<2, 1>(contA, contB, 1_s, &discA, &discB);
  wpi::math::Vectord<2> x1Discrete = discA * x0 + discB * u;

  // We now have pos = vel = accel = 1, which should give us:
  wpi::math::Vectord<2> x1Truth{1.0 * x0(0) + 1.0 * x0(1) + 0.5 * u(0),
                                0.0 * x0(0) + 1.0 * x0(1) + 1.0 * u(0)};

  CHECK(x1Truth == x1Discrete);
}

// Test that DiscretizeAB() matches the dense matrix exponential for each Padé
// approximant degree and with scaling and squaring
TEST_CASE("DiscretizationTest DiscretizeABMatchesDenseExp", "[wpimath]") {
  constexpr int States = 4;
  constexpr int Inputs = 2;

  std::srand(0);
  for (double dt : {1e-4, 5e-3, 0.02, 0.1, 0.5, 2.0, 10.0}) {
    for (int trial = 0; trial < 10; ++trial) {
      wpi::math::Matrixd<States, States> contA =
          wpi::math::Matrixd<States, States>::Random();
      wpi::math::Matrixd<States, Inputs> contB =
          wpi::math::Matrixd<States, Inputs>::Random();

      wpi::math::Matrixd<States, States> discA;
      wpi::math::Matrixd<States, Inputs> discB;
      wpi::math::DiscretizeAB<States, Inputs>(
          contA, contB, wpi::units::second_t{dt}, &discA, &discB);

      wpi::math::Matrixd<States + Inputs, States + Inputs> M;
      M.setZero();
      M.block<States, States>(0, 0) = contA;
      M.block<States, Inputs>(0, States) = contB;
      wpi::math::Matrixd<States + Inputs, States + Inputs> phi = (M * dt).exp();

      INFO("dt = " << dt);
      CHECK(discA.isApprox(phi.block<States, States>(0, 0), 1e-12));
      CHECK(discB.isApprox(phi.block<States, Inputs>(0, States), 1e-12));
    }
  }
}

// Test that DiscretizeAQ() matches the dense matrix exponential for each Padé
// approximant degree and with scaling and squaring
TEST_CASE("DiscretizationTest DiscretizeAQMatchesDenseExp", "[wpimath]") {
  constexpr int States = 4;

  std::srand(0);
  // Larger dt are omitted since Q_d becomes ill-conditioned and both methods
  // lose accuracy beyond the tolerance
  for (double dt : {1e-4, 5e-3, 0.02, 0.1, 0.5, 2.0}) {
    for (int trial = 0; trial < 10; ++trial) {
      wpi::math::Matrixd<States, States> contA =
          wpi::math::Matrixd<States, States>::Random();
      wpi::math::Matrixd<States, States> L =
          wpi::math::Matrixd<States, States>::Random();
      wpi::math::Matrixd<States, States> contQ = L * L.transpose();

      wpi::math::Matrixd<States, States> discA;
      wpi::math::Matrixd<States, States> discQ;
      wpi::math::DiscretizeAQ<States>(contA, contQ, wpi::units::second_t{dt},
                                      &discA, &discQ);

      wpi::math::Matrixd<2 * States, 2 * States> M;
      M.setZero();
      M.block<States, States>(0, 0) = -contA;
      M.block<States, States>(0, States) = contQ;
      M.block<States, States>(States, States) = contA.transpose();
      wpi::math::Matrixd<2 * States, 2 * States> phi = (M * dt).exp();

      wpi::math::Matrixd<States, States> expectedA =
          phi.block<States, States>(States, States).transpose();
      wpi::math::Matrixd<States, States> expectedQ =
          expectedA * phi.block<States, States>(0, States);
      expectedQ = (expectedQ + expectedQ.transpose()) / 2.0;

      INFO("dt = " << dt);
      CHECK(discA.isApprox(expectedA, 1e-12));
      CHECK(discQ.isApprox(expectedQ, 1e-12));
    }
  }
}

// Test that dynamic-size DiscretizeAB() and DiscretizeAQ() match their
// fixed-size versions on both sides of the dense exponential size threshold
TEST_CASE("DiscretizationTest DiscretizeDynamicSize", "[wpimath]") {
  std::srand(0);

  auto checkAB = [](auto contA, auto contB) {
    decltype(contA) discA;
    decltype(contB) discB;
    wpi::math::DiscretizeAB(contA, contB, 0.5_s, &discA, &discB);

    Eigen::MatrixXd dynA = contA;
    Eigen::MatrixXd dynB = contB;
    Eigen::MatrixXd dynDiscA;
    Eigen::MatrixXd dynDiscB;
    wpi::math::DiscretizeAB<Eigen::Dynamic, Eigen::Dynamic>(
        dynA, dynB, 0.5_s, &dynDiscA, &dynDiscB);

    CHECK(dynDiscA.isApprox(discA, 1e-12));
    CHECK(dynDiscB.isApprox(discB, 1e-12));
  };
  checkAB(wpi::math::Matrixd<2, 2>::Random().eval(),
          wpi::math::Matrixd<2, 1>::Random().eval());
  checkAB(wpi::math::Matrixd<4, 4>::Random().eval(),
          wpi::math::Matrixd<4, 2>::Random().eval());

  auto checkAQ = [](auto contA, auto L) {
    decltype(contA) contQ = L * L.transpose();
    decltype(contA) discA;
    decltype(contA) discQ;
    wpi::math::DiscretizeAQ(contA, contQ, 0.5_s, &discA, &discQ);

    Eigen::MatrixXd dynA = contA;
    Eigen::MatrixXd dynQ = contQ;
    Eigen::MatrixXd dynDiscA;
    Eigen::MatrixXd dynDiscQ;
    wpi::math::DiscretizeAQ<Eigen::Dynamic>(dynA, dynQ, 0.5_s, &dynDiscA,
                                            &dynDiscQ);

    CHECK(dynDiscA.isApprox(discA, 1e-12));
    CHECK(dynDiscQ.isApprox(discQ, 1e-12));
  };
  checkAQ(wpi::math::Matrixd<3, 3>::Random().eval(),
          wpi::math::Matrixd<3, 3>::Random().eval());
  checkAQ(wpi::math::Matrixd<9, 9>::Random().eval(),
          wpi::math::Matrixd<9, 9>::Random().eval());
}

//                                               T
// Test that the discrete approximation of Q_d ≈ ∫ e^(Aτ) Q e^(Aᵀτ) dτ
//                                               0
TEST_CASE("DiscretizationTest DiscretizeSlowModelAQ", "[wpimath]") {
  wpi::math::Matrixd<2, 2> contA{{0, 1}, {0, 0}};
  wpi::math::Matrixd<2, 2> contQ{{1, 0}, {0, 1}};

  constexpr wpi::units::second_t dt = 1_s;

  //       T
  // Q_d ≈ ∫ e^(Aτ) Q e^(Aᵀτ) dτ
  //       0
  wpi::math::Matrixd<2, 2> discQIntegrated = wpi::math::RKDP<
      std::function<wpi::math::Matrixd<2, 2>(wpi::units::second_t,
                                             const wpi::math::Matrixd<2, 2>&)>,
      wpi::math::Matrixd<2, 2>>(
      [&](wpi::units::second_t t, const wpi::math::Matrixd<2, 2>&) {
        return wpi::math::Matrixd<2, 2>((contA * t.value()).exp() * contQ *
                                        (contA.transpose() * t.value()).exp());
      },
      0_s, wpi::math::Matrixd<2, 2>::Zero(), dt);

  wpi::math::Matrixd<2, 2> discA;
  wpi::math::Matrixd<2, 2> discQ;
  wpi::math::DiscretizeAQ<2>(contA, contQ, dt, &discA, &discQ);

  UNSCOPED_INFO("Expected these to be nearly equal:\ndiscQ:\n"
                << discQ << "\ndiscQIntegrated:\n"
                << discQIntegrated);
  CHECK((discQIntegrated - discQ).norm() < 1e-10);
}

//                                               T
// Test that the discrete approximation of Q_d ≈ ∫ e^(Aτ) Q e^(Aᵀτ) dτ
//                                               0
TEST_CASE("DiscretizationTest DiscretizeFastModelAQ", "[wpimath]") {
  wpi::math::Matrixd<2, 2> contA{{0, 1}, {0, -1406.29}};
  wpi::math::Matrixd<2, 2> contQ{{0.0025, 0}, {0, 1}};

  constexpr wpi::units::second_t dt = 5_ms;

  //       T
  // Q_d = ∫ e^(Aτ) Q e^(Aᵀτ) dτ
  //       0
  wpi::math::Matrixd<2, 2> discQIntegrated = wpi::math::RKDP<
      std::function<wpi::math::Matrixd<2, 2>(wpi::units::second_t,
                                             const wpi::math::Matrixd<2, 2>&)>,
      wpi::math::Matrixd<2, 2>>(
      [&](wpi::units::second_t t, const wpi::math::Matrixd<2, 2>&) {
        return wpi::math::Matrixd<2, 2>((contA * t.value()).exp() * contQ *
                                        (contA.transpose() * t.value()).exp());
      },
      0_s, wpi::math::Matrixd<2, 2>::Zero(), dt);

  wpi::math::Matrixd<2, 2> discA;
  wpi::math::Matrixd<2, 2> discQ;
  wpi::math::DiscretizeAQ<2>(contA, contQ, dt, &discA, &discQ);

  UNSCOPED_INFO("Expected these to be nearly equal:\ndiscQ:\n"
                << discQ << "\ndiscQIntegrated:\n"
                << discQIntegrated);
  CHECK((discQIntegrated - discQ).norm() < 1e-3);
}

// Test that DiscretizeR() works
TEST_CASE("DiscretizationTest DiscretizeR", "[wpimath]") {
  wpi::math::Matrixd<2, 2> contR{{2.0, 0.0}, {0.0, 1.0}};
  wpi::math::Matrixd<2, 2> discRTruth{{4.0, 0.0}, {0.0, 2.0}};

  wpi::math::Matrixd<2, 2> discR = wpi::math::DiscretizeR<2>(contR, 500_ms);

  UNSCOPED_INFO("Expected these to be nearly equal:\ndiscR:\n"
                << discR << "\ndiscRTruth:\n"
                << discRTruth);
  CHECK((discRTruth - discR).norm() < 1e-10);
}
