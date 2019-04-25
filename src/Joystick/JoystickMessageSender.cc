/****************************************************************************
 *
 * Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "JoystickManager.h"
#include "MAVLinkProtocol.h"
#include "QGCApplication.h"
#include "UDPLink.h"
#include "JoystickMessageSender.h"
#include "KeyConfiguration.h"

#define SBUS_MODE             0x20
#define CHANNEL_IDEX          0x01
#define SBUS_DATA_LEN         25
#define SBUS_STARTBYTE        0x0f
#define SBUS_ENDBYTE          0x00

struct rc_msg {
    /* typs is MSB, idex is LSB */
    uint8_t type_idex;
    /* sbus or ppm data */
    uint8_t rc_data[SBUS_DATA_LEN];
};

static void pack_rc_channels_msg(int sbus, uint16_t (&channels)[16], struct rc_msg *msg);

QGC_LOGGING_CATEGORY(JoystickMessageSenderLog, "JoystickMessageSenderLog")

uint16_t JoystickMessageSender::_sbus0ChannelValues[12] = { 0 };
uint16_t JoystickMessageSender::_sbus1ChannelValues[16] = { 0 };

JoystickMessageSender::JoystickMessageSender(JoystickManager* joystickManager)
    : QObject()
    , _activeJoystick(NULL)
    , _joystickManager(joystickManager)
    , _udpLink(NULL)
    , _joystickPortNumber(16666)
    , _joystickCompId(66)
    , _remoteHostIp("192.168.0.10")
    , _mavlinkChannel(0)
    , _channelCount(16)
    , _sbus1Enable(false)
{
    connect(joystickManager, &JoystickManager::activeJoystickChanged, this, &JoystickMessageSender::_activeJoystickChanged);

    if (_joystickManager->getRCSetting("SbusCtrl/Sbus1SendbyApp").toBool()) {
        _sbus1Enable = true;
    }
}

JoystickMessageSender::~JoystickMessageSender()
{
}

void JoystickMessageSender::_setupJoystickLink()
{
    UDPConfiguration* config = new UDPConfiguration(QString("Joystick"));
    config->addHost(_remoteHostIp, _joystickPortNumber);
    LinkManager* linkMgr = qgcApp()->toolbox()->linkManager();
    config->setDynamic(true);
    SharedLinkConfigurationPointer linkConfig = linkMgr->addConfiguration(config);
    _udpLink = new UDPLink(linkConfig);
    _udpLink->_connect();

    _mavlinkChannel = qgcApp()->toolbox()->linkManager()->_reserveMavlinkChannel();
    if (_mavlinkChannel == 0) {
        qWarning() << "No mavlink channels available";
        return;
    }
    // use Mavlink 2.0
    mavlink_status_t* mavlinkStatus = mavlink_get_channel_status(_mavlinkChannel);
    mavlinkStatus->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
}

void JoystickMessageSender::_handleManualControl(float roll, float pitch, float yaw, float thrust, float wheel, quint16 buttons, int joystickMode)
{
    if (joystickMode != Vehicle::JoystickModeRC || _mavlinkChannel == 0) {
        return;
    }
    // Store the previous manual commands
    static float manualRollAngle = 0.0;
    static float manualPitchAngle = 0.0;
    static float manualYawAngle = 0.0;
    static float manualThrust = 0.0;
    static float manualWheel = 0.0;
    static quint16 manualButtons = 0;
    static quint8 countSinceLastTransmission = 0; // Track how many calls to this function have occurred since the last MAVLink transmission

    // Transmit the external setpoints only if they've changed OR if it's been a little bit since they were last transmit. To make sure there aren't issues with
    // response rate, we make sure that a message is transmit when the commands have changed, then one more time, and then switch to the lower transmission rate
    // if no command inputs have changed.

    // The default transmission rate is 25Hz, but when no inputs have changed it drops down to 5Hz.
    bool sendCommand = false;
    if (countSinceLastTransmission++ >= 5) {
        sendCommand = true;
        countSinceLastTransmission = 0;
    } else if ((!qIsNaN(roll) && roll != manualRollAngle) || (!qIsNaN(pitch) && pitch != manualPitchAngle) ||
             (!qIsNaN(yaw) && yaw != manualYawAngle) || (!qIsNaN(thrust) && thrust != manualThrust) ||
             (!qIsNaN(wheel) && wheel != manualWheel) || buttons != manualButtons) {
        sendCommand = true;

        // Ensure that another message will be sent the next time this function is called
        countSinceLastTransmission = 10;
    }

    if (!sendCommand) {
        return;
    }

    // Now if we should trigger an update, let's do that
    // Save the new manual control inputs
    manualRollAngle = roll;
    manualPitchAngle = pitch;
    manualYawAngle = yaw;
    manualThrust = thrust;
    manualThrust = wheel;
    manualButtons = buttons;

    int sbus, channel, channelValue;
    if(KeyConfiguration::getScrollWheelSetting(&sbus, &channel)) {
        channelValue = (int)(1000 + wheel * 1000.f);
        setChannelValue(sbus, channel, channelValue);
    }

    // Store scaling values for all 3 axes
    const float axesScaling = 1.0 * 1000.0;

    const float ch1 = (roll+1) * axesScaling;
    const float ch2 = (-pitch+1) * axesScaling;
    const float ch3 = (yaw+1) * axesScaling;
    const float ch4 = thrust * axesScaling;

    if (_sbus1Enable) {
        struct rc_msg message;
        pack_rc_channels_msg(1, _sbus1ChannelValues, &message);

        _udpLink->writeBytesSafe((const char*)&message, sizeof(struct rc_msg));
    }
}

void JoystickMessageSender::_activeJoystickChanged(Joystick* joystick)
{
    if (_activeJoystick) {
        disconnect(_activeJoystick, &Joystick::manualControl, this, &JoystickMessageSender::_handleManualControl);
        _activeJoystick = NULL;
        if (_mavlinkChannel > 0) {
            qgcApp()->toolbox()->linkManager()->_freeMavlinkChannel(_mavlinkChannel);
        }
    }
    if (joystick) {
        if(_udpLink == NULL) {
            _setupJoystickLink();
        }
        _activeJoystick = joystick;
        connect(_activeJoystick, &Joystick::manualControl, this, &JoystickMessageSender::_handleManualControl);
    }
}

QVariantList JoystickMessageSender::sbusChannelStatus()
{
    KeyConfiguration* conf;
    QVariantList list;

    for(int i = 0; i < 2; i++) {
        QString status;
        conf = _joystickManager->getKeyConfiguration(i);
        for(int j = 0; j < conf->channelCount(); j++) {
            if(conf->getControlMode(j + conf->getChannelMinNum()) > 1) {
                status += QString("C%1: %2/%3 ").arg(j + conf->getChannelMinNum(), 2, 10, QChar('0'))
                                                .arg(conf->getSeqInChannel(j + conf->getChannelMinNum(), i == 0 ? _sbus0ChannelValues[j] : _sbus1ChannelValues[j]))
                                                .arg(conf->getChannelValueCount(j + conf->getChannelMinNum()));
            } else if(conf->getControlMode(j + conf->getChannelMinNum()) == 1) {//scroll wheel
                status += QString("C%1: ").arg(j + conf->getChannelMinNum(), 2, 10, QChar('0')) + "SW ";
            }
        }
        list += QVariant::fromValue(status);
    }
    return list;
}

int JoystickMessageSender::getChannelValue(int sbus, int ch)
{
    if(sbus == 1) {
        return _sbus0ChannelValues[ch - 5];
    } else if(sbus == 2) {
        return _sbus1ChannelValues[ch - 1];
    }

    return 0;
}

void JoystickMessageSender::setChannelValue(int sbus, int ch, uint16_t value)
{
    if (ch > _channelCount) {
        qWarning() << "invalid channel id" << ch;
        return;
    }
    if(sbus == 1) {
        _sbus0ChannelValues[ch - 5] = value;
    } else if(sbus == 2) {
        _sbus1ChannelValues[ch - 1] = value;
    }
    emit sbusChannelStatusChanged();
}

int JoystickMessageSender::channelCount()
{
    return _channelCount;
}

static void pack_rc_channels_msg(int sbus, uint16_t (&channels)[16], struct rc_msg *msg)
{
    msg->type_idex = (sbus & CHANNEL_IDEX) | SBUS_MODE;
    /* sbus protocol start byte:0xF0 */
    msg->rc_data[0] = SBUS_STARTBYTE;
    /* sbus protocol data1: channel1 0-7 bit */
    msg->rc_data[1] =(channels[0] & 0xff);
    /* sbus protocol data2: channel1 8-10 bit and channel2 0-4 */
    msg->rc_data[2] =(((channels[0] >> 8) | (channels[1]<< 3)) & 0xff);
    /* sbus protocol data3: channel2 5-10 bit and channel3 0-1 */
    msg->rc_data[3] =(((channels[1] >> 5) | (channels[2] << 6)) & 0xff);
    /* sbus protocol data4: channel3 2-9 bit */
    msg->rc_data[4] =((channels[2] >> 2) & 0xff);
    /* sbus protocol data5: channel3 10 bit and channel4 0-6 */
    msg->rc_data[5] =(((channels[2] >> 10) | (channels[3] << 1)) & 0xff);
    /* sbus protocol data6: channel4 7-10 bit and channel5 0-3 */
    msg->rc_data[6] =(((channels[3] >> 7) | (channels[4] << 4)) & 0xff);
    /* sbus protocol data7: channel5 4-10 bit and channel6 0 */
    msg->rc_data[7] =(((channels[4] >> 4) | (channels[5] << 7)) & 0xff);
    /* sbus protocol data8: channel6 1-8 bit */
    msg->rc_data[8] =((channels[5] >> 1) & 0xff);
    /* sbus protocol data9: channel6 9-10 bit and channel7 0-5 */
    msg->rc_data[9] =(((channels[5] >> 9) | (channels[6] << 2)) & 0xff);
    /* sbus protocol data10: channel7 6-10 bit and channel8 0-2 */
    msg->rc_data[10] =(((channels[6] >> 6) | (channels[7] << 5)) & 0xff);
    /* sbus protocol data11: channel8 3-10 bit */
    msg->rc_data[11] =(((channels[7] >> 3)) & 0xff);

    /* sbus protocol data12: channel9 0-7 bit */
    msg->rc_data[12] =(channels[8] & 0xff);
    /* sbus protocol data13: channel9 8-10 bit and channel10 0-4 */
    msg->rc_data[13] =(((channels[8] >> 8) | (channels[9] << 3)) & 0xff);
    /* sbus protocol data14: channel10 5-10 bit and channel11 0-1 */
    msg->rc_data[14] =(((channels[9] >> 5) | (channels[10] << 6)) & 0xff);
    /* sbus protocol data15: channel11 2-9 bit */
    msg->rc_data[15] =((channels[10] >> 2) & 0xff);
    /* sbus protocol data16: channel11 10 bit and channel12 0-6 */
    msg->rc_data[16] =(((channels[10] >> 10) | (channels[11] << 1)) & 0xff);
    /* sbus protocol data17: channel12 7-10 bit and channel13 0-3 */
    msg->rc_data[17] =(((channels[11] >> 7) | (channels[12] << 4)) & 0xff);
    /* sbus protocol data18: channel13 4-10 bit and channel14 0 */
    msg->rc_data[18] =(((channels[12] >> 4) | (channels[13] << 7)) & 0xff);
    /* sbus protocol data19: channel14 1-8 bit */
    msg->rc_data[19] =((channels[13] >> 1) & 0xff);
    /* sbus protocol data20: channel14 9-10 bit and channel15 0-5 */
    msg->rc_data[20] =(((channels[13] >> 9) | (channels[14] << 2)) & 0xff);
    /* sbus protocol data21: channel15 6-10 bit and channel16 0-2 */
    msg->rc_data[21] =(((channels[14] >> 6) | (channels[15] << 5)) & 0xff);
    /* sbus protocol data22: channel16 3-10 bit */
    msg->rc_data[22] =((channels[15] >> 3) & 0xff);

    msg->rc_data[23] = 0x00;
    msg->rc_data[24] = SBUS_ENDBYTE;
}
