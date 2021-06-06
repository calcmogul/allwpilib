// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <jni.h>

#include "HALUtil.h"
#include "edu_wpi_first_hal_ConstantsJNI.h"
#include "hal/Constants.h"
#include "jni_md.h"

using namespace hal;

extern "C" {
/*
 * Class:     edu_wpi_first_hal_ConstantsJNI
 * Method:    getSystemClockTicksPerMicrosecond
 * Signature: ()I
 */
JNIEXPORT jint JNICALL
Java_edu_wpi_first_hal_ConstantsJNI_getSystemClockTicksPerMicrosecond
  (JNIEnv* env, jclass)
{
  jint value = HAL_GetSystemClockTicksPerMicrosecond();
  return value;
}
}  // extern "C"
