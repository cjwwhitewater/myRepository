#include <iostream>
#include "dji_camera_manager.h"
#include "VideoReader.h"

using namespace std;

VideoDataProcessor VideoReader::videoDataProcessor = nullptr;

VideoReader::VideoReader()
{
}

VideoReader::~VideoReader()
{
    T_DjiReturnCode returnCode;
    returnCode = DjiLiveview_StopH264Stream(DJI_LIVEVIEW_CAMERA_POSITION_NO_1,
                                            DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "stop h30 h264 stream failed, error code" << returnCode << endl;
        return;
    }
}

void VideoReader::startVideoStream()
{
    T_DjiReturnCode returnCode;
    // Thi calling can only happend after calling to DjiCore_Init()
    returnCode = DjiLiveview_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Liveview init failed, error code" << returnCode << endl;
        return;
    }

    returnCode = DjiLiveview_StartH264Stream(
                DJI_LIVEVIEW_CAMERA_POSITION_NO_1,
                DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT,
                djiLiveviewH264Callback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "start h30 h264 stream failed, error code" << returnCode << endl;
        return;
    }

    returnCode = DjiCameraManager_SetStreamSource(
                            DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1,
                            DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "set stream source failed, error code" << returnCode << endl;
        return;
    }
}

void VideoReader::djiLiveviewH264Callback(E_DjiLiveViewCameraPosition position,
                                          const uint8_t *buf, uint32_t len)
{
    if (videoDataProcessor){
        videoDataProcessor(buf, len);
    }
}

void VideoReader::setVideoDataProcessor(VideoDataProcessor videoDataProcessor_)
{
    if (videoDataProcessor){
        cout << "WARNING: VideoReader::videoDataProcessor has already been set"
             << endl;
        return;
    }
    videoDataProcessor = videoDataProcessor_;
}

