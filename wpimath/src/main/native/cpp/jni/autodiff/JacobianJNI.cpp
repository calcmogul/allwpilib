// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <jni.h>

#include <sleipnir/autodiff/jacobian.hpp>
#include <wpi/jni_util.h>

#include "edu_wpi_first_math_autodiff_JacobianJNI.h"

using namespace wpi::java;

extern "C" {

/*
 * Class:     edu_wpi_first_math_autodiff_JacobianJNI
 * Method:    createSM
 * Signature: (J[J)J
 */
JNIEXPORT jlong JNICALL
Java_edu_wpi_first_math_autodiff_JacobianJNI_createSM
  (JNIEnv* env, jclass, jlong variable, jlongArray wrt)
{}

/*
 * Class:     edu_wpi_first_math_jni_autodiff_VariableJNI
 * Method:    createMM
 * Signature: ([J[J)J
 */
JNIEXPORT jlong JNICALL
Java_edu_wpi_first_math_jni_autodiff_VariableJNI_createMM
  (JNIEnv* env, jclass, jlongArray variables, jlongArray wrt)
{}

/*
 * Class:     edu_wpi_first_math_jni_autodiff_JacobianJNI
 * Method:    destroy
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_math_jni_autodiff_JacobianJNI_destroy
  (JNIEnv* env, jclass, jlong handle)
{
  delete reinterpret_cast<slp::Jacobian*>(handle);
}

/*
 * Class:     edu_wpi_first_math_jni_autodiff_JacobianJNI
 * Method:    get
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_edu_wpi_first_math_jni_autodiff_JacobianJNI_get
  (JNIEnv* env, jclass, jlong handle)
{}

/*
 * Class:     edu_wpi_first_math_jni_autodiff_JacobianJNI
 * Method:    value
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_edu_wpi_first_math_jni_autodiff_JacobianJNI_value
  (JNIEnv* env, jclass, jlong handle)
{}

}  // extern "C"
