/****************************************************************************
 *
 *   (c) 2009-2016 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include "VideoSettings.h"

#include <QQmlEngine>
#include <QtQml>
#include <QVariantList>

#ifdef __android__
#include <QtAndroidExtras/QAndroidJniObject>
#define VIDEO_SHARE_PROP_NAME "persist.fpv.ext.disp"
#endif

#ifndef QGC_DISABLE_UVC
#include <QCameraInfo>
#endif

const char* VideoSettings::videoSettingsGroupName = "Video";

const char* VideoSettings::videoSourceName =        "VideoSource";
const char* VideoSettings::udpPortName =            "VideoUDPPort";
const char* VideoSettings::rtspUrlName =            "VideoRTSPUrl";
const char* VideoSettings::tcpUrlName =             "VideoTCPUrl";
const char* VideoSettings::videoAspectRatioName =   "VideoAspectRatio";
const char* VideoSettings::videoGridLinesName =     "VideoGridLines";
const char* VideoSettings::showRecControlName =     "ShowRecControl";
const char* VideoSettings::recordingFormatName =    "RecordingFormat";
const char* VideoSettings::maxVideoSizeName =       "MaxVideoSize";
const char* VideoSettings::enableStorageLimitName = "EnableStorageLimit";
const char* VideoSettings::rtspTimeoutName =        "RtspTimeout";
const char* VideoSettings::streamEnabledName =      "StreamEnabled";
const char* VideoSettings::disableWhenDisarmedName ="DisableWhenDisarmed";
const char* VideoSettings::videoResolutionName =    "VideoResolution";
const char* VideoSettings::cameraIdName =           "cameraId";
const char* VideoSettings::videoShareEnableName =   "VideoShareEnable";
const char* VideoSettings::saveSdCardEnableName =   "SaveSdCardEnable";

const char* VideoSettings::videoSourceNoVideo =     "No Video Available";
const char* VideoSettings::videoDisabled =          "Video Stream Disabled";
const char* VideoSettings::videoSourceUDP =         "UDP Video Stream";
const char* VideoSettings::videoSourceRTSP =        "RTSP Video Stream";
const char* VideoSettings::videoSourceTCP =         "TCP-MPEG2 Video Stream";
const char* VideoSettings::videoSourceAuto =        "Auto Connection Video Stream";

VideoSettings::VideoSettings(QObject* parent)
    : SettingsGroup(videoSettingsGroupName, QString() /* root settings group */, parent)
    , _videoSourceFact(NULL)
    , _udpPortFact(NULL)
    , _tcpUrlFact(NULL)
    , _rtspUrlFact(NULL)
    , _videoAspectRatioFact(NULL)
    , _gridLinesFact(NULL)
    , _showRecControlFact(NULL)
    , _recordingFormatFact(NULL)
    , _maxVideoSizeFact(NULL)
    , _enableStorageLimitFact(NULL)
    , _rtspTimeoutFact(NULL)
    , _streamEnabledFact(NULL)
    , _disableWhenDisarmedFact(NULL)
    , _videoResolutionFact(NULL)
    , _cameraIdFact(NULL)
    , _videoShareEnableFact(NULL)
    , _videoShareSettings(NULL)
{
    QQmlEngine::setObjectOwnership(this, QQmlEngine::CppOwnership);
    qmlRegisterUncreatableType<VideoSettings>("QGroundControl.SettingsManager", 1, 0, "VideoSettings", "Reference only");

    // Setup enum values for videoSource settings into meta data
    bool noVideo = false;
    QStringList videoSourceList;
#ifdef QGC_GST_STREAMING
#ifndef NO_UDP_VIDEO
    videoSourceList.append(videoSourceUDP);
#endif
    videoSourceList.append(videoSourceRTSP);
    videoSourceList.append(videoSourceTCP);
    videoSourceList.append(videoSourceAuto);
#endif
#ifndef QGC_DISABLE_UVC
    QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
    foreach (const QCameraInfo &cameraInfo, cameras) {
        videoSourceList.append(cameraInfo.description());
    }
#endif
    if (videoSourceList.count() == 0) {
        noVideo = true;
        videoSourceList.append(videoSourceNoVideo);
    } else {
        videoSourceList.insert(0, videoDisabled);
    }
    QVariantList videoSourceVarList;
    foreach (const QString& videoSource, videoSourceList) {
        videoSourceVarList.append(QVariant::fromValue(videoSource));
    }
    _nameToMetaDataMap[videoSourceName]->setEnumInfo(videoSourceList, videoSourceVarList);

    // Set default value for videoSource
    if (noVideo) {
        _nameToMetaDataMap[videoSourceName]->setRawDefaultValue(videoSourceNoVideo);
    } else {
        _nameToMetaDataMap[videoSourceName]->setRawDefaultValue(videoSourceAuto);
    }

    _videoShareSettings = new WifiSettings();
}

Fact* VideoSettings::videoSource(void)
{
    if (!_videoSourceFact) {
        _videoSourceFact = _createSettingsFact(videoSourceName);
        connect(_videoSourceFact, &Fact::valueChanged, this, &VideoSettings::_configChanged);
    }
    return _videoSourceFact;
}

Fact* VideoSettings::udpPort(void)
{
    if (!_udpPortFact) {
        _udpPortFact = _createSettingsFact(udpPortName);
        connect(_udpPortFact, &Fact::valueChanged, this, &VideoSettings::_configChanged);
    }
    return _udpPortFact;
}

Fact* VideoSettings::rtspUrl(void)
{
    if (!_rtspUrlFact) {
        _rtspUrlFact = _createSettingsFact(rtspUrlName);
        connect(_rtspUrlFact, &Fact::valueChanged, this, &VideoSettings::_configChanged);
    }
    return _rtspUrlFact;
}

Fact* VideoSettings::tcpUrl(void)
{
    if (!_tcpUrlFact) {
        _tcpUrlFact = _createSettingsFact(tcpUrlName);
        connect(_tcpUrlFact, &Fact::valueChanged, this, &VideoSettings::_configChanged);
    }
    return _tcpUrlFact;
}

Fact* VideoSettings::aspectRatio(void)
{
    if (!_videoAspectRatioFact) {
        _videoAspectRatioFact = _createSettingsFact(videoAspectRatioName);
    }
    return _videoAspectRatioFact;
}

Fact* VideoSettings::gridLines(void)
{
    if (!_gridLinesFact) {
        _gridLinesFact = _createSettingsFact(videoGridLinesName);
    }
    return _gridLinesFact;
}

Fact* VideoSettings::showRecControl(void)
{
    if (!_showRecControlFact) {
        _showRecControlFact = _createSettingsFact(showRecControlName);
    }
    return _showRecControlFact;
}

Fact* VideoSettings::recordingFormat(void)
{
    if (!_recordingFormatFact) {
        _recordingFormatFact = _createSettingsFact(recordingFormatName);
    }
    return _recordingFormatFact;
}

Fact* VideoSettings::maxVideoSize(void)
{
    if (!_maxVideoSizeFact) {
        _maxVideoSizeFact = _createSettingsFact(maxVideoSizeName);
    }
    return _maxVideoSizeFact;
}

Fact* VideoSettings::enableStorageLimit(void)
{
    if (!_enableStorageLimitFact) {
        _enableStorageLimitFact = _createSettingsFact(enableStorageLimitName);
    }
    return _enableStorageLimitFact;
}

Fact* VideoSettings::rtspTimeout(void)
{
    if (!_rtspTimeoutFact) {
        _rtspTimeoutFact = _createSettingsFact(rtspTimeoutName);
    }
    return _rtspTimeoutFact;
}

Fact* VideoSettings::streamEnabled(void)
{
    if (!_streamEnabledFact) {
        _streamEnabledFact = _createSettingsFact(streamEnabledName);
    }
    return _streamEnabledFact;
}

Fact* VideoSettings::disableWhenDisarmed(void)
{
    if (!_disableWhenDisarmedFact) {
        _disableWhenDisarmedFact = _createSettingsFact(disableWhenDisarmedName);
    }
    return _disableWhenDisarmedFact;
}

Fact* VideoSettings::videoResolution(void)
{
    if (!_videoResolutionFact) {
        _videoResolutionFact = _createSettingsFact(videoResolutionName);
    }
    return _videoResolutionFact;
}

Fact* VideoSettings::cameraId(void)
{
    if (!_cameraIdFact) {
        _cameraIdFact = _createSettingsFact(cameraIdName);
    }
    return _cameraIdFact;
}

bool VideoSettings::streamConfigured(void)
{
#if !defined(QGC_GST_STREAMING)
    return false;
#endif
    QString vSource = videoSource()->rawValue().toString();
    if(vSource == videoSourceNoVideo || vSource == videoDisabled) {
        return false;
    }
    //-- If UDP, check if port is set
    if(vSource == videoSourceUDP) {
        return udpPort()->rawValue().toInt() != 0;
    }
    //-- If RTSP, check for URL
    if(vSource == videoSourceRTSP) {
        return !rtspUrl()->rawValue().toString().isEmpty();
    }
    //-- If TCP, check for URL
    if(vSource == videoSourceTCP) {
        return !tcpUrl()->rawValue().toString().isEmpty();
    }
    if(vSource == videoSourceAuto) {
        return true;
    }
    return false;
}

Fact* VideoSettings::videoShareEnable(void)
{
    if (!_videoShareEnableFact) {
        _videoShareEnableFact = _createSettingsFact(videoShareEnableName);

#ifdef __android__
        //Set video share state by property
        QAndroidJniObject prop = QAndroidJniObject::fromString(VIDEO_SHARE_PROP_NAME);
        QAndroidJniObject defaultValue = QAndroidJniObject::fromString("0");
        QAndroidJniObject value = QAndroidJniObject::callStaticObjectMethod("android/os/SystemProperties", "get",
                                    "(Ljava/lang/String;Ljava/lang/String;)Ljava/lang/String;", prop.object<jstring>(), defaultValue.object<jstring>());
        _videoShareEnableFact->setRawValue(value.toString().toInt());
        if(_videoShareEnableFact->rawValue().toBool()) {
            _videoShareSettings->setVideoShareApEnabled(true);
        }
#endif
    }
    return _videoShareEnableFact;
}

/*
 *set android property persist.fpv.ext.disp
 */
bool VideoSettings::setVideoShareEnabled(bool enabled)
{
#ifdef __android__
    if(!_videoShareSettings->setVideoShareApEnabled(enabled)) {
        qWarning() << "Set wifi AP hotspot failed.";
        return false;
    }

    QAndroidJniObject prop = QAndroidJniObject::fromString(VIDEO_SHARE_PROP_NAME);
    QAndroidJniObject value;
    if(enabled) {
        value = QAndroidJniObject::fromString("1");
    } else {
        value = QAndroidJniObject::fromString("0");
    }
    QAndroidJniObject::callStaticObjectMethod("android/os/SystemProperties", "set", "(Ljava/lang/String;Ljava/lang/String;)V",
                                                                                prop.object<jstring>(), value.object<jstring>());
    qDebug() << "Andoid property" << prop.toString() << "is be set to" << value.toString();
#endif
    return true;
}

Fact* VideoSettings::saveSdCardEnable(void)
{
    if (!_saveSdCardEnableFact) {
        _saveSdCardEnableFact = _createSettingsFact(saveSdCardEnableName);
    }
    return _saveSdCardEnableFact;
}

void VideoSettings::_configChanged(QVariant)
{
    emit streamConfiguredChanged();
}

