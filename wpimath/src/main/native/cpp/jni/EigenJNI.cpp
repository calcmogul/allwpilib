// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <jni.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include "org_wpilib_math_jni_EigenJNI.h"
#include "wpi/util/jni_util.hpp"

using namespace wpi::util::java;

extern "C" {

/*
 * Class:     org_wpilib_math_jni_EigenJNI
 * Method:    rankUpdate
 * Signature: ([DI[DDZ)V
 */
JNIEXPORT void JNICALL
Java_org_wpilib_math_jni_EigenJNI_rankUpdate
  (JNIEnv* env, jclass, jdoubleArray mat, jint rows, jdoubleArray vec,
   jdouble sigma, jboolean lowerTriangular)
{
  JSpan<jdouble> matBody{env, mat};
  JSpan<const jdouble> vecBody{env, vec};

  Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      L{matBody.data(), rows, rows};
  Eigen::Map<const Eigen::Vector<double, Eigen::Dynamic>> v{vecBody.data(),
                                                            rows};

  if (lowerTriangular == JNI_TRUE) {
    Eigen::internal::llt_inplace<double, Eigen::Lower>::rankUpdate(L, v, sigma);
  } else {
    Eigen::internal::llt_inplace<double, Eigen::Upper>::rankUpdate(L, v, sigma);
  }
}

}  // extern "C"
