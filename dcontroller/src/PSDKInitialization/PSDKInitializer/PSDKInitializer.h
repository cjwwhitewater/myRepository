#pragma once

#include <iostream>
#include <fstream>
#include "dji_typedef.h"
#include "dji_core.h"

using namespace std;

class PSDKInitializer {
public:
    PSDKInitializer();
    ~PSDKInitializer();

private:
    static void setupEnvironment();
    static void startPSDK();
    static T_DjiReturnCode printToConsole(const uint8_t *data, uint16_t dataLen);
    static T_DjiReturnCode localWrite(const uint8_t *data, uint16_t dataLen);
    static T_DjiReturnCode fillInUserInfo(T_DjiUserInfo *userInfo);
    static T_DjiReturnCode localWriteFsInit(const char *path);
};


