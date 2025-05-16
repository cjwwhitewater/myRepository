#include "MTPCommon.h"

using namespace std;

map<MessageType, string> messageTypeNames = {
    {MessageType::DeviceStatus, "DeviceStatus"},
    {MessageType::Image,       "Image"},
    {MessageType::Video,       "Video"}
};
