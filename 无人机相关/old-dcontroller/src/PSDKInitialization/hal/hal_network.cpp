#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "hal_network.h"
#include "dji_logger.h"

T_DjiReturnCode HalNetWork_Init(const char *ipAddr, const char *netMask,
                                T_DjiNetworkHandle *halObj)
{
    int32_t ret;
    char cmdStr[LINUX_CMD_STR_MAX_SIZE];
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    int32_t routeIp[4] = {0};
    int32_t genMask[4] = {0};

    if (ipAddr == NULL || netMask == NULL) {
        USER_LOG_ERROR("hal network config param error");
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    sscanf(ipAddr, "%d.%d.%d.%d", &routeIp[0], &routeIp[1], &routeIp[2], &routeIp[3]);
    sscanf(netMask, "%d.%d.%d.%d", &genMask[0], &genMask[1], &genMask[2], &genMask[3]);
    routeIp[0] &= genMask[0];
    routeIp[1] &= genMask[1];
    routeIp[2] &= genMask[2];
    routeIp[3] &= genMask[3];

    //Attention: need root permission to config ip addr and netmask.
    memset(cmdStr, 0, sizeof(cmdStr));

    snprintf(cmdStr, sizeof(cmdStr), "ifconfig %s up", LINUX_NETWORK_DEV);
    USER_LOG_DEBUG("%s", cmdStr);
    ret = system(cmdStr);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Can't open the network."
                       "Probably the program not execute with root permission."
                       "Please use the root permission to execute the program.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    snprintf(cmdStr, sizeof(cmdStr), "ifconfig %s %s netmask %s",
             LINUX_NETWORK_DEV, ipAddr, netMask);
    USER_LOG_DEBUG("%s", cmdStr);
    ret = system(cmdStr);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("cmd failed: %s", cmdStr);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    osalHandler->TaskSleepMs(50);

    snprintf(cmdStr, sizeof(cmdStr), "ip route flush dev %s", LINUX_NETWORK_DEV);
    USER_LOG_DEBUG("%s", cmdStr);
    ret = system(cmdStr);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("cmd failed: %s", cmdStr);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    snprintf(cmdStr, sizeof(cmdStr), "route add -net %d.%d.%d.%d netmask %s dev %s",
        routeIp[0], routeIp[1], routeIp[2], routeIp[3], netMask, LINUX_NETWORK_DEV);
    USER_LOG_DEBUG("%s", cmdStr);
    ret = system(cmdStr);
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("cmd failed: %s", cmdStr);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    osalHandler->TaskSleepMs(50);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode HalNetWork_DeInit(T_DjiNetworkHandle halObj)
{
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode HalNetWork_GetDeviceInfo(T_DjiHalNetworkDeviceInfo *deviceInfo)
{
    deviceInfo->usbNetAdapter.vid = USB_NET_ADAPTER_VID;
    deviceInfo->usbNetAdapter.pid = USB_NET_ADAPTER_PID;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

