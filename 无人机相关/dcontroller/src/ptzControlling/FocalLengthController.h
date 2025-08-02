#pragma once
#include <string>
#include <utility>
#include <vector>
#include <jsoncpp/json/json.h>
#include "dji_typedef.h"

using std::string;
using std::vector;
using std::pair;

class FocalLengthController{
public:
    ~FocalLengthController();

    void setFocalLength(float setFocalLength);

    // The focal length will be converted to an integer when returing.
    int getFocalLength();

    // Report focal length.
    void reportStatus(Json::Value& status);

private:
    void initialize();

    // A test function to determine the range of the zoom factor for H30 device.
    void testZoomFactorRange();

private:
    E_DjiMountPosition mountPosition;

    const float minZoomFactor  =  2.0;
    const float maxZoomFactor  = 23.0;
    const float minFocalLength =  7.1;
    const float maxFocalLength = 172.0;

// runtime singleton
public:
    static void createInstance();
    static FocalLengthController* get();
private:
    FocalLengthController();
    FocalLengthController(const FocalLengthController&);
private:
    static FocalLengthController* singleton;
};
