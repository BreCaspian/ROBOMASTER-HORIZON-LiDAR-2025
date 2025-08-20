/* 
ch：该示例程序展示相机采集图像过程中断开连接后，如何恢复采集图像。

en: This sample program shows how to reconnect a camera and continue to acquire images after the camera is disconnected during image acquisition.
*/

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MVFGControl.h"

IFHANDLE g_hInterface = NULL;
DEVHANDLE g_hDevice = NULL;
STREAMHANDLE g_hStream = NULL;
bool g_bExit = false;              // ch:停止取流 | en:Stop grabbing
bool g_bConnect = false;
bool g_bStartGrabbing = false;
unsigned char g_strDeviceID[MV_FG_MAX_DEV_INFO_SIZE] = {0};

#define MV_FG_MAX_DEV_INFO_SIZE 64 // 设备信息最大长度


// 等待用户输入enter键来结束取流或结束程序
// wait for user to input enter to stop grabbing or end the sample program
void PressEnterToExit(void)
{
    int c;
    while ((c = getchar()) != '\n' && c != EOF)
        ;
    fprintf(stderr, "\nPress enter to exit.\n");
    while (getchar() != '\n')
        ;
    sleep(1);
}


// ch:取流线程 | en:Grabbing image data thread
static void* WorkThread(void *pUser)
{
    if (pUser)
    {
        MV_FG_BUFFER_INFO stFrameInfo = {0}; // 图像信息
        const unsigned int nTimeout = 2000;
        int nRet = 0;

        g_bExit = false;

        while (!g_bExit)
        {
            if (g_bStartGrabbing)
            {
                // ch:获取一帧图像缓存信息 | en:Get one frame buffer's info
                nRet = MV_FG_GetFrameBuffer(g_hStream, &stFrameInfo, nTimeout);
                if (MV_FG_SUCCESS != nRet)
                {
                    printf("Get frame buffer info failed! %#x\n", nRet);
                    continue;
                }
                else
                {
                    printf("FrameNumber:%ld, Width:%d, Height:%d\n", stFrameInfo.nFrameID, stFrameInfo.nWidth, stFrameInfo.nHeight);
                }

                // ch:将缓存放回输入队列 | en:Put the buffer back into the input queue
                nRet = MV_FG_ReleaseFrameBuffer(g_hStream, &stFrameInfo);
                if (MV_FG_SUCCESS != nRet)
                {
                    printf("Release frame buffer failed! %#x\n", nRet);
                    break;
                }
            }
        }
    }

    return 0;
}

void __stdcall ReconnectDevice(MV_FG_EXCEPTION_TYPE enExceptionType, void *pUser)
{
    if (EXCEPTION_TYPE_CAMERA_DISCONNECT_ERR == enExceptionType)
    {
        int nRet = MV_FG_SUCCESS;
        bool bChanged = false;

        if (g_bConnect)
        {
            g_bConnect = false;
            g_bStartGrabbing = false;
            MV_FG_CloseStream(g_hStream);
            g_hStream = NULL;
            MV_FG_CloseDevice(g_hDevice);
            g_hDevice = NULL;

            while (1)
            {
                MV_FG_UpdateDeviceList(g_hInterface, &bChanged);
                nRet = MV_FG_OpenDeviceByID(g_hInterface, (char *)g_strDeviceID, &g_hDevice);
                if (MV_FG_SUCCESS == nRet)
                {
                    MV_FG_RegisterExceptionCallBack(g_hDevice, ReconnectDevice, pUser);

                    // ch:关闭触发模式 | en:Close trigger mode
                    nRet = MV_FG_SetEnumValueByString(g_hDevice, "TriggerMode", "Off");
                    if (MV_FG_SUCCESS != nRet)
                    {
                        printf("Turn off trigger mode failed! %#x\n", nRet);
                        break;
                    }

                    printf("Reconnection success\n");
                    g_bConnect = true;

                    break;
                }
                else
                {
                    usleep(10 * 1000);
                }
            }

            if (g_bConnect)
            {
                do
                {
                    // ch:获取流通道个数 | en:Get number of stream
                    unsigned int nStreamNum = 0;
                    nRet = MV_FG_GetNumStreams(g_hDevice, &nStreamNum);
                    if (MV_FG_SUCCESS != nRet || 0 == nStreamNum)
                    {
                        break;
                    }

                    // ch:打开流通道(目前只支持单个通道) | en:Open stream(Only a single stream is supported now)
                    nRet = MV_FG_OpenStream(g_hDevice, 0, &g_hStream);
                    if (MV_FG_SUCCESS != nRet)
                    {
                        break;
                    }

                    // ch:设置SDK内部缓存数量 | en:Set internal buffer number
                    const unsigned int nBufferNum = 3;
                    nRet = MV_FG_SetBufferNum(g_hStream, nBufferNum);
                    if (MV_FG_SUCCESS != nRet)
                    {
                        break;
                    }

                    // ch:开始取流 | en:Start Acquisition
                    nRet = MV_FG_StartAcquisition(g_hStream);
                    if (MV_FG_SUCCESS != nRet)
                    {
                        printf("Start acquistion failed! %#x\n", nRet);
                        break;
                    }

                    g_bStartGrabbing = true;

                } while (false);
            }
        }
    }
}

// ch:打印采集卡信息 | en:Print interface info
bool PrintInterfaceInfo(unsigned int nInterfaceNum)
{
    int nRet = 0;

    for (unsigned int i = 0; i < nInterfaceNum; i++)
    {
        MV_FG_INTERFACE_INFO stInterfaceInfo = {0};

        nRet = MV_FG_GetInterfaceInfo(i, &stInterfaceInfo);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Get info of No.%d interface failed! %#x\n", i, nRet);
            return false;
        }

        switch (stInterfaceInfo.nTLayerType)
        {
        case MV_FG_CXP_INTERFACE:
        {
            printf("[CXP]No.%d Interface: \n\tDisplayName: %s\n\tInterfaceID: %s\n\tSerialNumber:%s\n", i,
                   stInterfaceInfo.IfaceInfo.stCXPIfaceInfo.chDisplayName,
                   stInterfaceInfo.IfaceInfo.stCXPIfaceInfo.chInterfaceID,
                   stInterfaceInfo.IfaceInfo.stCXPIfaceInfo.chSerialNumber);
            break;
        }
        case MV_FG_GEV_INTERFACE:
        {
            printf("[GEV]No.%d Interface: \n\tDisplayName: %s\n\tInterfaceID: %s\n\tSerialNumber:%s\n", i,
                   stInterfaceInfo.IfaceInfo.stGEVIfaceInfo.chDisplayName,
                   stInterfaceInfo.IfaceInfo.stGEVIfaceInfo.chInterfaceID,
                   stInterfaceInfo.IfaceInfo.stGEVIfaceInfo.chSerialNumber);
            break;
        }
        case MV_FG_CAMERALINK_INTERFACE:
        {
            printf("[CML]No.%d Interface: \n\tDisplayName: %s\n\tInterfaceID: %s\n\tSerialNumber:%s\n", i,
                   stInterfaceInfo.IfaceInfo.stCMLIfaceInfo.chDisplayName,
                   stInterfaceInfo.IfaceInfo.stCMLIfaceInfo.chInterfaceID,
                   stInterfaceInfo.IfaceInfo.stCMLIfaceInfo.chSerialNumber);
            break;
        }
        case MV_FG_XoF_INTERFACE:
            {
                printf("[XoF]No.%d Interface: \n\tDisplayName: %s\n\tInterfaceID: %s\n\tSerialNumber:%s\n", i, 
                    stInterfaceInfo.IfaceInfo.stXoFIfaceInfo.chDisplayName, 
                    stInterfaceInfo.IfaceInfo.stXoFIfaceInfo.chInterfaceID, 
                    stInterfaceInfo.IfaceInfo.stXoFIfaceInfo.chSerialNumber);
                break;
            }      
        default:
        {
            printf("Unknown interface type.\n");
            return false;
        }
        }
    }

    return true;
}

// ch:打印设备信息 | en:Print device info
bool PrintDeviceInfo(IFHANDLE g_hInterface, unsigned int nDeviceNum)
{
    int nRet = 0;

    for (unsigned int i = 0; i < nDeviceNum; i++)
    {
        MV_FG_DEVICE_INFO stDeviceInfo = {0};

        nRet = MV_FG_GetDeviceInfo(g_hInterface, i, &stDeviceInfo);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Get info of No.%d device failed! %#x\n", i, nRet);
            return false;
        }

        switch (stDeviceInfo.nDevType)
        {
        case MV_FG_CXP_DEVICE:
        {
            printf("[CXP]No.%d Device: \n\tUserDefinedName: %s\n\tModelName: %s\n\tSerialNumber: %s\n", i,
                   stDeviceInfo.DevInfo.stCXPDevInfo.chUserDefinedName,
                   stDeviceInfo.DevInfo.stCXPDevInfo.chModelName,
                   stDeviceInfo.DevInfo.stCXPDevInfo.chSerialNumber);
            break;
        }
        case MV_FG_GEV_DEVICE:
        {
            printf("[GEV]No.%d Device: \n\tUserDefinedName: %s\n\tModelName: %s\n\tSerialNumber: %s\n", i,
                   stDeviceInfo.DevInfo.stGEVDevInfo.chUserDefinedName,
                   stDeviceInfo.DevInfo.stGEVDevInfo.chModelName,
                   stDeviceInfo.DevInfo.stGEVDevInfo.chSerialNumber);
            break;
        }
        case MV_FG_CAMERALINK_DEVICE:
        {
            printf("[CML]No.%d Device: \n\tUserDefinedName: %s\n\tModelName: %s\n\tSerialNumber: %s\n", i,
                   stDeviceInfo.DevInfo.stCMLDevInfo.chUserDefinedName,
                   stDeviceInfo.DevInfo.stCMLDevInfo.chModelName,
                   stDeviceInfo.DevInfo.stCMLDevInfo.chSerialNumber);
            break;
        }
        case MV_FG_XoF_DEVICE:
        {
            printf("[XoF]No.%d Device: \n\tUserDefinedName: %s\n\tModelName: %s\n\tSerialNumber: %s\n", i, 
                stDeviceInfo.DevInfo.stXoFDevInfo.chUserDefinedName,
                stDeviceInfo.DevInfo.stXoFDevInfo.chModelName,
                stDeviceInfo.DevInfo.stXoFDevInfo.chSerialNumber);
            break;
        }     
        default:
        {
            printf("Unknown device type.\n");
            return false;
        }
        }
    }

    return true;
}

int main()
{
    int nRet = 0;

    do
    {
        // ch:枚举采集卡 | en:Enum interface
        bool bChanged = false;
        nRet = MV_FG_UpdateInterfaceList(MV_FG_CXP_INTERFACE | MV_FG_GEV_INTERFACE | MV_FG_CAMERALINK_INTERFACE | MV_FG_XoF_INTERFACE, &bChanged);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Update interface list failed! %#x\n", nRet);
            break;
        }

        // ch:获取采集卡数量 | en:Get interface num
        unsigned int nInterfaceNum = 0;
        nRet = MV_FG_GetNumInterfaces(&nInterfaceNum);
        if (MV_FG_SUCCESS != nRet || 0 == nInterfaceNum)
        {
            printf("No interface found! return = %d, number = %d\n", nRet, nInterfaceNum);
            break;
        }

        // ch:显示采集卡信息 | en:Show interface info
        if (false == PrintInterfaceInfo(nInterfaceNum))
        {
            break;
        }

        // ch:选择采集卡 | en:Select interface
        int nInterfaceIndex = -1;
        printf("Select an interface: ");
        scanf("%d", &nInterfaceIndex);

        if (nInterfaceIndex < 0 || nInterfaceIndex >= (int)nInterfaceNum)
        {
            printf("Invalid interface index.\nQuit.\n");
            break;
        }

        // ch:以指定权限打开采集卡，获得采集卡句柄 | en:Open interface with specified permissions, get handle
        nRet = MV_FG_OpenInterface((unsigned int)nInterfaceIndex, &g_hInterface);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Open No.%d interface failed! %#x\n", nInterfaceIndex, nRet);
            break;
        }

        // ch:枚举采集卡上的相机 | en:Enum camera of interface
        nRet = MV_FG_UpdateDeviceList(g_hInterface, &bChanged);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Update device list failed! %#x\n", nRet);
            break;
        }

        // ch:获取设备数量 | en:Get device number
        unsigned int nDeviceNum = 0;
        nRet = MV_FG_GetNumDevices(g_hInterface, &nDeviceNum);
        if (MV_FG_SUCCESS != nRet || 0 == nDeviceNum)
        {
            printf("No device found! return = %d, number = %d\n", nRet, nDeviceNum);
            break;
        }

        // ch:显示设备信息 | en:Show device info
        if (false == PrintDeviceInfo(g_hInterface, nDeviceNum))
        {
            break;
        }

        // ch:选择设备 | en:Select device
        int nDeviceIndex = -1;
        printf("Select a device: ");
        scanf("%d", &nDeviceIndex);

        if (nDeviceIndex < 0 || nDeviceIndex >= (int)nDeviceNum)
        {
            printf("Invalid device index.\nQuit.\n");
            break;
        }

        // ch:打开设备，获得设备句柄 | en:Open device, get handle
        nRet = MV_FG_OpenDevice(g_hInterface, (unsigned int)nDeviceIndex, &g_hDevice);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Open No.%d device failed! %#x\n", nDeviceIndex, nRet);
            g_hDevice = NULL;
            break;
        }

        g_bConnect = true;

        //获取设备信息
        MV_FG_DEVICE_INFO stDevInfo = {0};
        nRet = MV_FG_GetDeviceInfo(g_hInterface, (unsigned int)nDeviceIndex, &stDevInfo);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Get device info failed! %#x\n", nRet);
            break;
        }

        switch (stDevInfo.nDevType)
        {
        case MV_FG_CXP_DEVICE:
        {
            memcpy(g_strDeviceID, stDevInfo.DevInfo.stCXPDevInfo.chDeviceID, MV_FG_MAX_DEV_INFO_SIZE);
            break;
        }
        case MV_FG_GEV_DEVICE:
        {
            memcpy(g_strDeviceID, stDevInfo.DevInfo.stGEVDevInfo.chDeviceID, MV_FG_MAX_DEV_INFO_SIZE);
            break;
        }
        case MV_FG_CAMERALINK_DEVICE:
        {
            memcpy(g_strDeviceID, stDevInfo.DevInfo.stCMLDevInfo.chDeviceID, MV_FG_MAX_DEV_INFO_SIZE);
            break;
        }
        default:
        {
            nRet = MV_FG_ERR_INVALID_ID;
        }
        }
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Unsupported device type\n");
            break;
        }

        //注册异常回调
        nRet = MV_FG_RegisterExceptionCallBack(g_hDevice, ReconnectDevice, NULL);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("register exception callback failed! %#x\n", nRet);
            break;
        }

        // ch:关闭触发模式 | en:Close trigger mode
        nRet = MV_FG_SetEnumValueByString(g_hDevice, "TriggerMode", "Off");
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Turn off trigger mode failed! %#x\n", nRet);
            break;
        }

        // ch:获取流通道个数 | en:Get number of stream
        unsigned int nStreamNum = 0;
        nRet = MV_FG_GetNumStreams(g_hDevice, &nStreamNum);
        if (MV_FG_SUCCESS != nRet || 0 == nStreamNum)
        {
            printf("No stream available! return = %d, number = %d\n", nRet, nStreamNum);
            break;
        }

        // ch:打开流通道(目前只支持单个通道) | en:Open stream(Only a single stream is supported now)
        nRet = MV_FG_OpenStream(g_hDevice, 0, &g_hStream);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Open stream failed! %#x\n", nRet);
            break;
        }

        // ch:设置SDK内部缓存数量 | en:Set internal buffer number
        const unsigned int nBufferNum = 3;
        nRet = MV_FG_SetBufferNum(g_hStream, nBufferNum);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Set buffer number failed! %#x\n", nRet);
            break;
        }

        // ch:开始取流 | en:Start Acquisition
        nRet = MV_FG_StartAcquisition(g_hStream);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Start acquistion failed! %#x\n", nRet);
            break;
        }
        g_bStartGrabbing = true;

        // ch:创建取流线程 | en:Create acquistion thread
        pthread_t nThreadID;
        nRet = pthread_create(&nThreadID, NULL, WorkThread, g_hStream);
        if (nRet != 0)
        {
            printf("thread create failed.ret = %d\n", nRet);
            break;
        }
        printf("Press Enter to stop acquisition.\n");
        PressEnterToExit();

        // ch:停止取流 | en:Stop Acquisition
        nRet = MV_FG_StopAcquisition(g_hStream);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Stop acquisition failed! %#x\n", nRet);
            break;
        }
        g_bStartGrabbing = false;

        // ch:关闭取流线程 | en:Close acquistion thread
        g_bExit = true;
        g_bConnect = false;

        // ch:等待取流线程结束 | en:Join acquistion thread
        nRet = pthread_join(nThreadID, NULL);
        if (nRet != 0)
        {
            printf("thread join failed.ret = %d\n", nRet);
            break;
        }
    } while (0);

    // ch:关闭流通道 | en:Close Stream
    if (NULL != g_hStream)
    {
        nRet = MV_FG_CloseStream(g_hStream);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Close stream failed! %#x\n", nRet);
        }
        g_hStream = NULL;
    }

    // ch:关闭设备 | en:Close device
    if (NULL != g_hDevice)
    {
        nRet = MV_FG_CloseDevice(g_hDevice);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Close device failed! %#x\n", nRet);
        }
        g_hDevice = NULL;
    }

    // ch:关闭采集卡 | en:Close interface
    if (NULL != g_hInterface)
    {
        nRet = MV_FG_CloseInterface(g_hInterface);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Close interface failed! %#x\n", nRet);
        }
        g_hInterface = NULL;
    }

    return 0;
}
