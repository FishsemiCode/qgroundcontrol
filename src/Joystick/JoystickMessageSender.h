/****************************************************************************
 *
 * Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include "QGCLoggingCategory.h"
#include "Joystick.h"

class JoystickManager;
class UDPLink;

Q_DECLARE_LOGGING_CATEGORY(JoystickMessageSenderLog)

class JoystickMessageSender : public QObject
{
    Q_OBJECT

public:
    JoystickMessageSender(JoystickManager* joystickManager);
    ~JoystickMessageSender();

private slots:
    void _handleManualControl(float roll, float pitch, float yaw, float thrust, quint16 buttons, int joystickMode);
    void _activeJoystickChanged(Joystick* joystick);
    void _setupJoystickLink();

private:
    Joystick* _activeJoystick;
    UDPLink* _udpLink;
    uint _joystickPortNumber;
    uint8_t _joystickCompId;
    QString _remoteHostIp;
    int _mavlinkChannel;
};
