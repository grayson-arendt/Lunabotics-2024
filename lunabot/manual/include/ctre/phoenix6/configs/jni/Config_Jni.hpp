/*
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class com_ctre_phoenix6_configs_jni_ConfigJNI */

#ifndef _Included_com_ctre_phoenix6_configs_jni_ConfigJNI
#define _Included_com_ctre_phoenix6_configs_jni_ConfigJNI
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     com_ctre_phoenix6_configs_jni_ConfigJNI
 * Method:    SetConfigs
 * Signature: (Ljava/lang/String;IDZZ)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_configs_jni_ConfigJNI_SetConfigs
  (JNIEnv *, jobject, jstring, jint, jdouble, jboolean, jboolean);

/*
 * Class:     com_ctre_phoenix6_configs_jni_ConfigJNI
 * Method:    GetConfigs
 * Signature: (Ljava/lang/String;ID)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_configs_jni_ConfigJNI_GetConfigs
  (JNIEnv *, jobject, jstring, jint, jdouble);

/*
 * Class:     com_ctre_phoenix6_configs_jni_ConfigJNI
 * Method:    Serializedouble
 * Signature: (ID)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_com_ctre_phoenix6_configs_jni_ConfigJNI_Serializedouble
  (JNIEnv *, jclass, jint, jdouble);

/*
 * Class:     com_ctre_phoenix6_configs_jni_ConfigJNI
 * Method:    Serializeint
 * Signature: (II)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_com_ctre_phoenix6_configs_jni_ConfigJNI_Serializeint
  (JNIEnv *, jclass, jint, jint);

/*
 * Class:     com_ctre_phoenix6_configs_jni_ConfigJNI
 * Method:    Serializeboolean
 * Signature: (IZ)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_com_ctre_phoenix6_configs_jni_ConfigJNI_Serializeboolean
  (JNIEnv *, jclass, jint, jboolean);

/*
 * Class:     com_ctre_phoenix6_configs_jni_ConfigJNI
 * Method:    Deserializedouble
 * Signature: (ILjava/lang/String;)D
 */
JNIEXPORT jdouble JNICALL Java_com_ctre_phoenix6_configs_jni_ConfigJNI_Deserializedouble
  (JNIEnv *, jclass, jint, jstring);

/*
 * Class:     com_ctre_phoenix6_configs_jni_ConfigJNI
 * Method:    Deserializeint
 * Signature: (ILjava/lang/String;)I
 */
JNIEXPORT jint JNICALL Java_com_ctre_phoenix6_configs_jni_ConfigJNI_Deserializeint
  (JNIEnv *, jclass, jint, jstring);

/*
 * Class:     com_ctre_phoenix6_configs_jni_ConfigJNI
 * Method:    Deserializeboolean
 * Signature: (ILjava/lang/String;)Z
 */
JNIEXPORT jboolean JNICALL Java_com_ctre_phoenix6_configs_jni_ConfigJNI_Deserializeboolean
  (JNIEnv *, jclass, jint, jstring);

#ifdef __cplusplus
}
#endif
#endif
