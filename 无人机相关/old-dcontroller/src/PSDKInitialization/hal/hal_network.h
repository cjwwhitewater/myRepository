#pragma once

#include "dji_platform.h"

#define LINUX_NETWORK_DEV           "eth1"

#define USB_NET_ADAPTER_VID         (0x0bda)
#define USB_NET_ADAPTER_PID         (0x8153)

#define LINUX_CMD_STR_MAX_SIZE      (128)

T_DjiReturnCode HalNetWork_Init(const char *ipAddr, const char *netMask,
                                T_DjiNetworkHandle *halObj);
T_DjiReturnCode HalNetWork_DeInit(T_DjiNetworkHandle halObj);
T_DjiReturnCode HalNetWork_GetDeviceInfo(T_DjiHalNetworkDeviceInfo *deviceInfo);

