/****************************************************************************
 *
 * Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include <QtAndroidExtras/QtAndroidExtras>
#include <QtAndroidExtras/QAndroidJniObject>
#include "QGCApplication.h"
#include "AndroidInterface.h"
#include <QAndroidJniObject>

void AndroidInterface::acquireScreenWakeLock()
{
    QAndroidJniObject::callStaticMethod<void>("org/mavlink/qgroundcontrol/QGCActivity", "acquireScreenWakeLock");
}

void AndroidInterface::releaseScreenWakeLock() {
    QAndroidJniObject::callStaticMethod<void>("org/mavlink/qgroundcontrol/QGCActivity", "releaseScreenWakeLock");
}

QString AndroidInterface::getSystemProperty(QString& prop_name, QString& default_value)
{
    QAndroidJniObject prop = QAndroidJniObject::fromString(prop_name);
    QAndroidJniObject defaultValue = QAndroidJniObject::fromString(default_value);
    QAndroidJniObject value = QAndroidJniObject::callStaticObjectMethod("android/os/SystemProperties", "get",
                            "(Ljava/lang/String;Ljava/lang/String;)Ljava/lang/String;", prop.object<jstring>(), defaultValue.object<jstring>());
    return value.toString();
}

void AndroidInterface::setSystemProperty(QString& prop_name, QString& prop_value)
{
    QAndroidJniObject prop = QAndroidJniObject::fromString(prop_name);
    QAndroidJniObject value = QAndroidJniObject::fromString(prop_value);
     QAndroidJniObject::callStaticObjectMethod("android/os/SystemProperties", "set", "(Ljava/lang/String;Ljava/lang/String;)V",
                                               prop.object<jstring>(), value.object<jstring>());
}

QString AndroidInterface::getSdcardPath()
{
    QAndroidJniObject value = QAndroidJniObject::callStaticObjectMethod("org/mavlink/qgroundcontrol/QGCActivity", "getSdcardPath",
                            "()Ljava/lang/String;");
    return value.toString();
}

void AndroidInterface::triggerMediaScannerScanFile(QString& file_path)
{
    QAndroidJniObject path = QAndroidJniObject::fromString(file_path);
     QAndroidJniObject::callStaticObjectMethod("org/mavlink/qgroundcontrol/QGCActivity", "triggerMediaScannerScanFile",
                                               "(Ljava/lang/String;)V",
                                               path.object<jstring>());
}
