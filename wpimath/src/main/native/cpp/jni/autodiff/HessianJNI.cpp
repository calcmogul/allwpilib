// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <jni.h>

#include <sleipnir/autodiff/hessian.hpp>
#include <wpi/jni_util.h>

#include "edu_wpi_first_math_autodiff_HessianJNI.h"

using namespace wpi::java;

extern "C" {

/*
 * Class:     edu_wpi_first_math_autodiff_HessianJNI
 * Method:    create
 * Signature: (J[J)J
 */
JNIEXPORT jlong JNICALL
Java_edu_wpi_first_math_autodiff_HessianJNI_create
  (JNIEnv* env, jclass, jlong variable, jlongArray wrt)
{}

/*
 * Class:     edu_wpi_first_math_jni_autodiff_HessianJNI
 * Method:    destroy
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_math_jni_autodiff_HessianJNI_destroy
  (JNIEnv* env, jclass, jlong handle)
{
  delete reinterpret_cast<slp::Hessian<>*>(handle);
}

/*
 * Class:     edu_wpi_first_math_jni_autodiff_HessianJNI
 * Method:    get
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_edu_wpi_first_math_jni_autodiff_HessianJNI_get
  (JNIEnv* env, jclass, jlong handle)
{}

/*
 * Class:     edu_wpi_first_math_jni_autodiff_HessianJNI
 * Method:    value
 * Signature: (J)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_edu_wpi_first_math_jni_autodiff_HessianJNI_value
  (JNIEnv* env, jclass, jlong handle)
{}

}  // extern "C"
