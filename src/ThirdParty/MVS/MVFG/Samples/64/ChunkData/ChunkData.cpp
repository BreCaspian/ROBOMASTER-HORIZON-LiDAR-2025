/* 
ch：该示例程序展示如何通过MV_FG_GetBufferChunkData()获取块数据，并展示如何对块数据中的曝光和时间戳解析。

en: This sample program shows how to use MV_FG_GetBufferChunkData() to get the block data, and shows how to parse the exposure and timestamp in the block data.
*/
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MVFGControl.h"

#define BUFFER_NUMBER               3               // ch:申请的缓存个数 | en:Number of requested buffer
#define CHUNK_ID_TIMESTAMP_LITTLE   0xa5a50101      // ch:时间戳 | en:Timestamp
#define CHUNK_ID_EXPOSURE_LITTLE    0xa5a50103      // ch:曝光 | en:Exposure

// 等待用户输入enter键来结束取流或结束程序
// wait for user to input enter to stop grabbing or end the sample program
void PressEnterToExit(void)
{
    int c;
    while ( (c = getchar()) != '\n' && c != EOF );
    fprintf( stderr, "\nPress enter to exit.\n");
    while( getchar() != '\n');
    sleep(1);
}

// ch:帧信息回调函数 | en:Frame info callback
void FrameCb(MV_FG_BUFFER_INFO* pstBufferInfo, void* pUser)
{
    if (pstBufferInfo)
    {
        // ch:打印图像信息 | en:Print image info
        printf("FrameNumber:%ld, Width:%d, Height:%d, Chunk Num:%d\n", 
            pstBufferInfo->nFrameID, pstBufferInfo->nWidth, pstBufferInfo->nHeight, pstBufferInfo->nNumChunks);

        int                     nRet = 0;
        STREAMHANDLE            hStream = (STREAMHANDLE)pUser;
        unsigned int            nChunkNum = pstBufferInfo->nNumChunks;      // Chunk个数
        MV_FG_CHUNK_DATA_INFO   stChunkDataInfo = { 0 };                    // Chunk信息

        // ch:打印chunk信息 | en:Print chunk info
        printf("********************\n");
        for (unsigned int i = 0; i < nChunkNum; i++)
        {
            memset(&stChunkDataInfo, 0, sizeof(MV_FG_CHUNK_DATA_INFO));
            nRet = MV_FG_GetBufferChunkData(hStream, pstBufferInfo, i, &stChunkDataInfo);
            if (MV_FG_SUCCESS != nRet)
            {
                printf("Get No.%d chunk data failed! %#x\n", i, nRet);
                printf("********************\n");
                return;
            }

            unsigned int* pIntChunkData = (unsigned int*)stChunkDataInfo.pChunkData;
            unsigned int nSecondCount = 0;
            unsigned int nCycleCount = 0;
            unsigned int nCycleOffset = 0;
            float fExposureTime = 0.0;
            switch (stChunkDataInfo.nChunkID)
            {
            case CHUNK_ID_TIMESTAMP_LITTLE:
                nSecondCount = (stChunkDataInfo.pChunkData[3] & 0xF8) >> 3;
                nCycleCount  = ((stChunkDataInfo.pChunkData[3] & 0xF7) << 10) + (stChunkDataInfo.pChunkData[2] << 2) + ((stChunkDataInfo.pChunkData[1] & 0xC0) >> 6);
                nCycleOffset = ((stChunkDataInfo.pChunkData[1] & 0x3F) << 8) + (int)stChunkDataInfo.pChunkData[0];
                printf("Chunk ID[%#x], Chunk length[%d], nSecondCount[%u], nCycleCount[%u], nCycleOffset[%u]\n", 
                    stChunkDataInfo.nChunkID, stChunkDataInfo.nChunkLen, nSecondCount, nCycleCount, nCycleOffset);
                break;
            case CHUNK_ID_EXPOSURE_LITTLE:
                fExposureTime = pIntChunkData[0];
                printf("Chunk ID[%#x], Chunk length[%d], fExposureTime[%f]\n", 
                    stChunkDataInfo.nChunkID, stChunkDataInfo.nChunkLen, fExposureTime);
                break;
            default:
                break;
            }
        }
        printf("********************\n");
    }
}

// ch:打印采集卡信息 | en:Print interface info
bool PrintInterfaceInfo(unsigned int nInterfaceNum)
{
    int nRet = 0;

    for (unsigned int i = 0; i < nInterfaceNum; i++)
    {
        MV_FG_INTERFACE_INFO stInterfaceInfo = { 0 };

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
bool PrintDeviceInfo(IFHANDLE hInterface, unsigned int nDeviceNum)
{
    int nRet = 0;

    for (unsigned int i = 0; i < nDeviceNum; i++)
    {
        MV_FG_DEVICE_INFO stDeviceInfo = { 0 };

        nRet = MV_FG_GetDeviceInfo(hInterface, i, &stDeviceInfo);
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
    int             nRet = 0;
    IFHANDLE        hInterface = NULL;
    DEVHANDLE       hDevice = NULL;
    STREAMHANDLE    hStream = NULL;

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

        // ch:打开采集卡，获得采集卡句柄 | en:Open interface, get handle
        nRet = MV_FG_OpenInterface((unsigned int)nInterfaceIndex, &hInterface);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Open No.%d interface failed! %#x\n", nInterfaceIndex, nRet);
            break;
        }

        // ch:枚举采集卡上的相机 | en:Enum camera of interface
        nRet = MV_FG_UpdateDeviceList(hInterface, &bChanged);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Update device list failed! %#x\n", nRet);
            break;
        }

        // ch:获取设备数量 | en:Get device number
        unsigned int nDeviceNum = 0;
        nRet = MV_FG_GetNumDevices(hInterface, &nDeviceNum);
        if (MV_FG_SUCCESS != nRet || 0 == nDeviceNum)
        {
            printf("No device found! return = %d, number = %d\n", nRet, nDeviceNum);
            break;
        }

        // ch:显示设备信息 | en:Show device info
        if (false == PrintDeviceInfo(hInterface, nDeviceNum))
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
        nRet = MV_FG_OpenDevice(hInterface, (unsigned int)nDeviceIndex, &hDevice);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Open No.%d device failed! %#x\n", nDeviceIndex, nRet);
            hDevice = NULL;
            break;
        }

        // ch:开启Chunk Mode | en:Open Chunk Mode
        nRet = MV_FG_SetBoolValue(hDevice, "ChunkModeActive", true);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Set chunk mode failed! %#x\n", nRet);
            break;
        }

        // ch:Chunk Selector设为Exposure | en: Chunk Selector set as Exposure
        nRet = MV_FG_SetEnumValueByString(hDevice, "ChunkSelector", "Exposure");
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Set exposure chunk failed! %#x\n", nRet);
            break;
        }

        // ch:开启Chunk Enable | en:Open Chunk Enable
        nRet = MV_FG_SetBoolValue(hDevice, "ChunkEnable", true);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Set exposure chunk enable failed! %#x\n", nRet);
            break;
        }

        // ch:Chunk Selector设为Timestamp | en: Chunk Selector set as Timestamp
        nRet = MV_FG_SetEnumValueByString(hDevice, "ChunkSelector", "Timestamp");
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Set timestamp chunk failed! %#x\n", nRet);
            break;
        }

        // ch:开启Chunk Enable | en:Open Chunk Enable
        nRet = MV_FG_SetBoolValue(hDevice, "ChunkEnable", true);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Set timestamp chunk enable failed! %#x\n", nRet);
            break;
        }

        // ch:关闭触发模式 | en:Close trigger mode
        nRet = MV_FG_SetEnumValueByString(hDevice, "TriggerMode", "Off");
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Turn off trigger mode failed! %#x\n", nRet);
            break;
        }

        // ch:获取流通道个数 | en:Get number of stream
        unsigned int nStreamNum = 0;
        nRet = MV_FG_GetNumStreams(hDevice, &nStreamNum);
        if (MV_FG_SUCCESS != nRet || 0 == nStreamNum)
        {
            printf("No stream available! return = %d, number = %d\n", nRet, nStreamNum);
            break;
        }

        // ch:打开流通道(目前只支持单个通道) | en:Open stream(Only a single stream is supported now)
        nRet = MV_FG_OpenStream(hDevice, 0, &hStream);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Open stream failed! %#x\n", nRet);
            break;
        }

        // ch:设置SDK内部缓存数量 | en:Set internal buffer number
        nRet = MV_FG_SetBufferNum(hStream, BUFFER_NUMBER);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Set buffer number failed! %#x\n", nRet);
            break;
        }

        // ch:注册帧缓存信息回调函数 | en:Register frame info callback
        nRet = MV_FG_RegisterFrameCallBackEx(hStream, FrameCb, hStream, true);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Register frame callback Ex failed! %#x\n", nRet);
            break;
        }

        // ch:开始取流 | en:Start Acquisition
        nRet = MV_FG_StartAcquisition(hStream);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Start acquistion failed! %#x\n", nRet);
            break;
        }

        printf("Press any key to stop acquisition.\n");
        PressEnterToExit();

        // ch:停止取流 | en:Stop Acquisition
        nRet = MV_FG_StopAcquisition(hStream);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Stop acquisition failed! %#x\n", nRet);
            break;
        }
    } while (0);

    // ch:关闭流通道 | en:Close Stream
    if (NULL != hStream)
    {
        nRet = MV_FG_CloseStream(hStream);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Close stream failed! %#x\n", nRet);
        }
        hStream = NULL;
    }

    // ch:关闭设备 | en:Close device
    if (NULL != hDevice)
    {
        nRet = MV_FG_CloseDevice(hDevice);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Close device failed! %#x\n", nRet);
        }
        hDevice = NULL;
    }

    // ch:关闭采集卡 | en:Close interface
    if (NULL != hInterface)
    {
        nRet = MV_FG_CloseInterface(hInterface);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Close interface failed! %#x\n", nRet);
        }
        hInterface = NULL;
    }

    return 0;
}
