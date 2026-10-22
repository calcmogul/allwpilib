// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <jni.h>

#include <Eigen/Core>

#include "org_wpilib_math_jni_DiscretizationJNI.h"
#include "wpi/math/system/Discretization.hpp"
#include "wpi/units/time.hpp"
#include "wpi/util/jni_util.hpp"

using namespace wpi::util::java;

using RowMajorMatrix =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

extern "C" {

/*
 * Class:     org_wpilib_math_jni_DiscretizationJNI
 * Method:    discretizeAB
 * Signature: ([D[DIID[D[D)V
 */
JNIEXPORT void JNICALL
Java_org_wpilib_math_jni_DiscretizationJNI_discretizeAB
  (JNIEnv* env, jclass, jdoubleArray contA, jdoubleArray contB, jint states,
   jint inputs, jdouble dt, jdoubleArray discA, jdoubleArray discB)
{
  JSpan<const jdouble> nativeContA{env, contA};
  JSpan<const jdouble> nativeContB{env, contB};

  Eigen::MatrixXd contAmat =
      Eigen::Map<const RowMajorMatrix>{nativeContA.data(), states, states};
  Eigen::MatrixXd contBmat =
      Eigen::Map<const RowMajorMatrix>{nativeContB.data(), states, inputs};

  Eigen::MatrixXd discAmat;
  Eigen::MatrixXd discBmat;
  wpi::math::DiscretizeAB<Eigen::Dynamic, Eigen::Dynamic>(
      contAmat, contBmat, wpi::units::second_t{dt}, &discAmat, &discBmat);

  RowMajorMatrix discAresult = discAmat;
  RowMajorMatrix discBresult = discBmat;
  env->SetDoubleArrayRegion(discA, 0, states * states, discAresult.data());
  env->SetDoubleArrayRegion(discB, 0, states * inputs, discBresult.data());
}

/*
 * Class:     org_wpilib_math_jni_DiscretizationJNI
 * Method:    discretizeAQ
 * Signature: ([D[DID[D[D)V
 */
JNIEXPORT void JNICALL
Java_org_wpilib_math_jni_DiscretizationJNI_discretizeAQ
  (JNIEnv* env, jclass, jdoubleArray contA, jdoubleArray contQ, jint states,
   jdouble dt, jdoubleArray discA, jdoubleArray discQ)
{
  JSpan<const jdouble> nativeContA{env, contA};
  JSpan<const jdouble> nativeContQ{env, contQ};

  Eigen::MatrixXd contAmat =
      Eigen::Map<const RowMajorMatrix>{nativeContA.data(), states, states};
  Eigen::MatrixXd contQmat =
      Eigen::Map<const RowMajorMatrix>{nativeContQ.data(), states, states};

  Eigen::MatrixXd discAmat;
  Eigen::MatrixXd discQmat;
  wpi::math::DiscretizeAQ<Eigen::Dynamic>(
      contAmat, contQmat, wpi::units::second_t{dt}, &discAmat, &discQmat);

  RowMajorMatrix discAresult = discAmat;
  RowMajorMatrix discQresult = discQmat;
  env->SetDoubleArrayRegion(discA, 0, states * states, discAresult.data());
  env->SetDoubleArrayRegion(discQ, 0, states * states, discQresult.data());
}

}  // extern "C"
