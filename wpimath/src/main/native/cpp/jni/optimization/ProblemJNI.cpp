// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <jni.h>

#include <sleipnir/optimization/problem.hpp>
#include <wpi/jni_util.h>

#include "edu_wpi_first_math_optimization_ProblemJNI.h"

using namespace wpi::java;

extern "C" {

/*
 * Class:     edu_wpi_first_math_optimization_ProblemJNI
 * Method:    create
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL
Java_edu_wpi_first_math_optimization_ProblemJNI_create
  (JNIEnv* env, jclass)
{
  return reinterpret_cast<jlong>(new slp::Problem);
}

/*
 * Class:     edu_wpi_first_math_optimization_ProblemJNI
 * Method:    destroy
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_math_optimization_ProblemJNI_destroy
  (JNIEnv* env, jclass, jlong handle)
{
  delete reinterpret_cast<slp::Problem*>(handle);
}

/*
 * Class:     edu_wpi_first_math_optimization_ProblemJNI
 * Method:    decisionVariableScalar
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL
Java_edu_wpi_first_math_optimization_ProblemJNI_decisionVariableScalar
  (JNIEnv* env, jclass, jlong handle)
{
  auto& problem = *reinterpret_cast<slp::Problem*>(handle);
  return reinterpret_cast<jlong>(
      new slp::Variable{problem.decision_variable()});
}

/*
 * Class:     edu_wpi_first_math_optimization_ProblemJNI
 * Method:    decisionVariableColumnVector
 * Signature: (JI)J
 */
JNIEXPORT jlong JNICALL
Java_edu_wpi_first_math_optimization_ProblemJNI_decisionVariableColumnVector
  (JNIEnv* env, jclass, jlong handle, jint rows)
{}

/*
 * Class:     edu_wpi_first_math_optimization_ProblemJNI
 * Method:    decisionVariableMatrix
 * Signature: (JII)J
 */
JNIEXPORT jlong JNICALL
Java_edu_wpi_first_math_optimization_ProblemJNI_decisionVariableMatrix
  (JNIEnv* env, jclass, jlong handle, jint rows, jint cols)
{}

/*
 * Class:     edu_wpi_first_math_optimization_ProblemJNI
 * Method:    symmetricDecisionVariable
 * Signature: (JI)J
 */
JNIEXPORT jlong JNICALL
Java_edu_wpi_first_math_optimization_ProblemJNI_symmetricDecisionVariable
  (JNIEnv* env, jclass, jlong handle, jint rows)
{}

/*
 * Class:     edu_wpi_first_math_optimization_ProblemJNI
 * Method:    minimize
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_math_optimization_ProblemJNI_minimize
  (JNIEnv* env, jclass, jlong handle, jlong costHandle)
{}

/*
 * Class:     edu_wpi_first_math_optimization_ProblemJNI
 * Method:    maximize
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_math_optimization_ProblemJNI_maximize
  (JNIEnv* env, jclass, jlong handle, jlong objectiveHandle)
{}

/*
 * Class:     edu_wpi_first_math_optimization_ProblemJNI
 * Method:    subjectToEq
 * Signature: (J?)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_math_optimization_ProblemJNI_subjectToEq
  (JNIEnv* env, jclass, jlong handle, EqualityConstraints constraint)
{}

/*
 * Class:     edu_wpi_first_math_optimization_ProblemJNI
 * Method:    subjectToIneq
 * Signature: (J?)V
 */
JNIEXPORT void JNICALL
Java_edu_wpi_first_math_optimization_ProblemJNI_subjectToIneq
  (JNIEnv* env, jclass, jlong handle, InequalityConstraints constraint)
{}

/*
 * Class:     edu_wpi_first_math_optimization_ProblemJNI
 * Method:    costFunctionType
 * Signature: (J)?
 */
JNIEXPORT ExpressionType JNICALL
Java_edu_wpi_first_math_optimization_ProblemJNI_costFunctionType
  (JNIEnv* env, jclass, jlong handle)
{
}

/*
 * Class:     edu_wpi_first_math_optimization_ProblemJNI
 * Method:    equalityConstraintType
 * Signature: (J)?
 */
JNIEXPORT ExpressionType JNICALL
Java_edu_wpi_first_math_optimization_ProblemJNI_equalityConstraintType
  (JNIEnv* env, jclass, jlong handle)
{}

/*
 * Class:     edu_wpi_first_math_optimization_ProblemJNI
 * Method:    inequalityConstraintType
 * Signature: (J)?
 */
JNIEXPORT ExpressionType JNICALL
Java_edu_wpi_first_math_optimization_ProblemJNI_inequalityConstraintType
  (JNIEnv* env, jclass, jlong handle)
{}

/*
 * Class:     edu_wpi_first_math_optimization_ProblemJNI
 * Method:    solve
 * Signature: (J)?
 */
JNIEXPORT ExitStatus JNICALL
Java_edu_wpi_first_math_optimization_ProblemJNI_solve
  (JNIEnv* env, jclass, jlong handle)
{}

/*
 * Class:     edu_wpi_first_math_optimization_ProblemJNI
 * Method:    solveWithOptions
 * Signature: (J?)?
 */
JNIEXPORT ExitStatus JNICALL
Java_edu_wpi_first_math_optimization_ProblemJNI_solveWithOptions
  (JNIEnv* env, jclass, jlong handle, Options options)
{}

/*
 * Class:     edu_wpi_first_math_optimization_ProblemJNI
 * Method:    solveWithOptionsSpy
 * Signature: (J?Z)?
 */
JNIEXPORT ExitStatus JNICALL
Java_edu_wpi_first_math_optimization_ProblemJNI_solveWithOptionsSpy
  (JNIEnv* env, jclass, jlong handle, Options options, jboolean spy)
{}

}  // extern "C"
