/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class com_ctre_phoenix6_controls_jni_ControlJNI */

#ifndef _Included_com_ctre_phoenix6_controls_jni_ControlJNI
#define _Included_com_ctre_phoenix6_controls_jni_ControlJNI
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestConfigApply
 * Signature: (Ljava/lang/String;IDLjava/lang/String;Z)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestConfigApply
  (JNIEnv *, jclass, jstring, jint, jdouble, jstring, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlDutyCycleOut
 * Signature: (Ljava/lang/String;IDZDZZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlDutyCycleOut
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jdouble, jboolean, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlTorqueCurrentFOC
 * Signature: (Ljava/lang/String;IDZDDDZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlTorqueCurrentFOC
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jdouble, jdouble, jdouble, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlVoltageOut
 * Signature: (Ljava/lang/String;IDZDZZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlVoltageOut
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jdouble, jboolean, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlPositionDutyCycle
 * Signature: (Ljava/lang/String;IDZDZDIZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlPositionDutyCycle
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jdouble, jboolean, jdouble, jint, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlPositionVoltage
 * Signature: (Ljava/lang/String;IDZDZDIZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlPositionVoltage
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jdouble, jboolean, jdouble, jint, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlPositionTorqueCurrentFOC
 * Signature: (Ljava/lang/String;IDZDDIZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlPositionTorqueCurrentFOC
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jdouble, jdouble, jint, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlVelocityDutyCycle
 * Signature: (Ljava/lang/String;IDZDZDIZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlVelocityDutyCycle
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jdouble, jboolean, jdouble, jint, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlVelocityVoltage
 * Signature: (Ljava/lang/String;IDZDZDIZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlVelocityVoltage
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jdouble, jboolean, jdouble, jint, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlVelocityTorqueCurrentFOC
 * Signature: (Ljava/lang/String;IDZDDIZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlVelocityTorqueCurrentFOC
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jdouble, jdouble, jint, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlMotionMagicDutyCycle
 * Signature: (Ljava/lang/String;IDZDZDIZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlMotionMagicDutyCycle
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jdouble, jboolean, jdouble, jint, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlMotionMagicVoltage
 * Signature: (Ljava/lang/String;IDZDZDIZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlMotionMagicVoltage
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jdouble, jboolean, jdouble, jint, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlMotionMagicTorqueCurrentFOC
 * Signature: (Ljava/lang/String;IDZDDIZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlMotionMagicTorqueCurrentFOC
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jdouble, jdouble, jint, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlFollower
 * Signature: (Ljava/lang/String;IDZIZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlFollower
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jint, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlStrictFollower
 * Signature: (Ljava/lang/String;IDZI)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlStrictFollower
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jint);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlNeutralOut
 * Signature: (Ljava/lang/String;IDZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlNeutralOut
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlCoastOut
 * Signature: (Ljava/lang/String;IDZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlCoastOut
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlStaticBrake
 * Signature: (Ljava/lang/String;IDZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlStaticBrake
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlBalanceBattery
 * Signature: (Ljava/lang/String;IDZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlBalanceBattery
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlBMSManualIsolator
 * Signature: (Ljava/lang/String;IDZZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlBMSManualIsolator
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jboolean);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlBMSManualVboost
 * Signature: (Ljava/lang/String;IDZZDD)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlBMSManualVboost
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jboolean, jdouble, jdouble);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlBMSManualPwmJunction
 * Signature: (Ljava/lang/String;IDZID)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlBMSManualPwmJunction
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean, jint, jdouble);

/*
 * Class:     com_ctre_phoenix6_controls_jni_ControlJNI
 * Method:    JNI_RequestControlBMSClearFault
 * Signature: (Ljava/lang/String;IDZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_controls_jni_ControlJNI_JNI_1RequestControlBMSClearFault
  (JNIEnv *, jclass, jstring, jint, jdouble, jboolean);

#ifdef __cplusplus
}
#endif
#endif
