#include <iostream>
#include <algorithm>
#include <cmath>
#include "options.h"
#include "FocalLengthController.h"
#include "nanotimer.h"
#include "dji_fc_subscription.h"
#include "dji_camera_manager.h"

using namespace std;

FocalLengthController* FocalLengthController::singleton = nullptr;

FocalLengthController::FocalLengthController()
{
    mountPosition = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
}

FocalLengthController::~FocalLengthController()
{
    T_DjiReturnCode returnCode;
    returnCode = DjiCameraManager_DeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Failed to cleanup dji camera manager" << endl;
        return;
    }
}

void FocalLengthController::initialize()
{
    T_DjiReturnCode returnCode;
    returnCode = DjiCameraManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Failed to initialize dji camera manager" << endl;
        return;
    }
}

void FocalLengthController::testZoomFactorRange()
{
    T_DjiReturnCode returnCode;

    returnCode = DjiCameraManager_StartContinuousOpticalZoom(mountPosition,
                               DJI_CAMERA_ZOOM_DIRECTION_OUT,
                               DJI_CAMERA_ZOOM_SPEED_FASTEST);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Failed to start continuous zooming" << endl;
        return;
    }
    cout << "zooming out, wait 10 seconds ..." << endl;
    millisecond_delay(10000);

    T_DjiCameraManagerOpticalZoomParam zoomParam;
    returnCode = DjiCameraManager_GetOpticalZoomParam(mountPosition, &zoomParam);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Failed to get zoom parameters" << endl;
        return;
    }
    cout << "current zoom factor: "   << zoomParam.currentOpticalZoomFactor
         << ", maximum zoom factor: " << zoomParam.maxOpticalZoomFactor
         << endl;

//    for (float factor = zoomParam.currentOpticalZoomFactor;
//         factor<zoomParam.maxOpticalZoomFactor; factor+=10){
//    for (float factor = 2.0; factor< 23;  factor++){
      int mapper[] = {2,  23,  4,  15,  6};
      for (int i=0; i < sizeof(mapper)/sizeof(mapper[0]); i++){
        float factor = mapper[i];
        returnCode = DjiCameraManager_SetOpticalZoomParam(mountPosition,
                        DJI_CAMERA_ZOOM_DIRECTION_OUT, factor);
        cout << "set zooming factor to " << factor << "..." << endl;
        millisecond_delay(10000);

        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            cout << "Failed to set focal length" << endl;
        }

        returnCode = DjiCameraManager_GetOpticalZoomParam(
                                            mountPosition, &zoomParam);
        cout << "current zoom factor: "   << zoomParam.currentOpticalZoomFactor
             << ", maximum zoom factor: " << zoomParam.maxOpticalZoomFactor
             << endl << endl;
    }
}

void FocalLengthController::setFocalLength(float focalLength)
{
    T_DjiReturnCode returnCode;

    std::clamp(focalLength, minFocalLength, maxFocalLength);
    // linear mapping from focal length to zoom factor.
    float zoomFactor = ( focalLength - minFocalLength) /
                       ( maxFocalLength - minFocalLength) *
                       (maxZoomFactor - minZoomFactor) + minZoomFactor;
    returnCode = DjiCameraManager_SetOpticalZoomParam(mountPosition,
                        DJI_CAMERA_ZOOM_DIRECTION_OUT, zoomFactor);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Failed to set focal length" << endl;
    }
}

int FocalLengthController::getFocalLength()
{
    T_DjiCameraManagerOpticalZoomParam zoomParam;
    T_DjiReturnCode code = DjiCameraManager_GetOpticalZoomParam(
                            mountPosition, &zoomParam);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Failed to get zoom parameters" << endl;
        return 0;
    }

    // linear mapping from zoom factor to focal length.
    float zoomFactor = zoomParam.currentOpticalZoomFactor;
    float focalLength = ( zoomFactor - minZoomFactor) /
                        (maxZoomFactor - minZoomFactor) *
                         ( maxFocalLength - minFocalLength) + minFocalLength;

    return focalLength;
}

void FocalLengthController::reportStatus(Json::Value& status)
{
    status["cameraFocalLength"] = getFocalLength();
}


// runtime singleton
void FocalLengthController::createInstance()
{
    if (singleton){
        cout << "the singleton of FocalLengthController has already been created, "
                "don't try to create it again" << endl;
        exit(-1);
    }
    singleton = new FocalLengthController;
    singleton->initialize();
}


FocalLengthController* FocalLengthController::get()
{
    if (singleton==nullptr){
        cout << "FocalLengthController::get() being called but the singleton of "
                "FocalLengthController has not been created" << endl;
        exit(-1);
    }
    return singleton;
}
