/* 
ch：该示例程序展示如何使用MV_FG_ConvertPixelType（）接口进行图像格式转化。示例中统一将Mono格式的图像转为Mono8格式，彩色格式的图像转为RGB8格式。

en: This sample program shows how to use the API MV_FG_ConvertPixelType() for image format conversion.
	In the example, the images in Mono format are converted to Mono8 format, and the images in color format are converted to RGB8 format.
*/
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MVFGControl.h"

#define BUFFER_NUMBER       3       // ch:申请的缓存个数 | en:Number of requested buffer
#define FILE_NAME_LEN       256     // ch:最大文件名长度 | en:Max of file name length
#define SAVE_IMAGE_NUM      10      // ch:最大存储图像数量 | en:Max number of stored images
#define TIMEOUT             1000    // ch:超时时间，单位毫秒 | en:Timeout, unit ms
bool g_bExit = false;               // ch:停止取流 | en:Stop grabbing

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

bool IsColorPixelFormat(MV_FG_PIXEL_TYPE enPixelType)
{
    switch(enPixelType)
    {
    case MV_FG_PIXEL_TYPE_RGBA8_Packed:
    case MV_FG_PIXEL_TYPE_BGRA8_Packed:
    case MV_FG_PIXEL_TYPE_BayerGR8:
    case MV_FG_PIXEL_TYPE_BayerRG8:
    case MV_FG_PIXEL_TYPE_BayerGB8:
    case MV_FG_PIXEL_TYPE_BayerBG8:
    case MV_FG_PIXEL_TYPE_BayerGB10:
    case MV_FG_PIXEL_TYPE_BayerGB10_Packed:
    case MV_FG_PIXEL_TYPE_BayerBG10:
    case MV_FG_PIXEL_TYPE_BayerBG10_Packed:
    case MV_FG_PIXEL_TYPE_BayerRG10:
    case MV_FG_PIXEL_TYPE_BayerRG10_Packed:
    case MV_FG_PIXEL_TYPE_BayerGR10:
    case MV_FG_PIXEL_TYPE_BayerGR10_Packed:
    case MV_FG_PIXEL_TYPE_BayerGB12:
    case MV_FG_PIXEL_TYPE_BayerGB12_Packed:
    case MV_FG_PIXEL_TYPE_BayerBG12:
    case MV_FG_PIXEL_TYPE_BayerBG12_Packed:
    case MV_FG_PIXEL_TYPE_BayerRG12:
    case MV_FG_PIXEL_TYPE_BayerRG12_Packed:
    case MV_FG_PIXEL_TYPE_BayerGR12:
    case MV_FG_PIXEL_TYPE_BayerGR12_Packed:
    case MV_FG_PIXEL_TYPE_BayerGR16:
    case MV_FG_PIXEL_TYPE_BayerRG16:
    case MV_FG_PIXEL_TYPE_BayerGB16:
    case MV_FG_PIXEL_TYPE_BayerBG16:
    case MV_FG_PIXEL_TYPE_YUV422_Packed:
    case MV_FG_PIXEL_TYPE_YUV422_YUYV_Packed:
        return true;
    default:
        return false;
    }
}

// 判断黑白格式图像
bool IsMonoPixelFormat(MV_FG_PIXEL_TYPE enPixelType)
{
    switch(enPixelType)
    {
    case MV_FG_PIXEL_TYPE_Mono10:
    case MV_FG_PIXEL_TYPE_Mono10_Packed:
    case MV_FG_PIXEL_TYPE_Mono12:
    case MV_FG_PIXEL_TYPE_Mono12_Packed:
    case MV_FG_PIXEL_TYPE_Mono16:
        return true;
    default:
        return false;
    }
}

// ch:取流线程 | en:Grabbing image data thread
static void* GrabbingThread(void* pUser)
{
    if (pUser)
    {
        STREAMHANDLE        hStream = (STREAMHANDLE)pUser;
        MV_FG_BUFFER_INFO   stFrameInfo = { 0 };    // 图像信息
        int                 nRet = 0;
        int                 nSaveNum = 0;           // 存图数量
        MV_FG_CONVERT_PIXEL_INFO stConvertPixelInfo = {0}; // 图像转换信息
        memset(&stConvertPixelInfo, 0, sizeof(MV_FG_CONVERT_PIXEL_INFO));

        // ch:开始取流 | en:Start Acquisition
        nRet = MV_FG_StartAcquisition(hStream);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Start acquistion failed! %#x\n", nRet);
            return NULL;
        }
        g_bExit = false;

        while (!g_bExit)
        {
            // ch:获取一帧图像缓存信息 | en:Get one frame buffer's info
            nRet = MV_FG_GetFrameBuffer(hStream, &stFrameInfo, TIMEOUT);
            if (MV_FG_SUCCESS != nRet)
            {
                printf("Get frame buffer info failed! %#x\n", nRet);
                continue;
            }
            printf("FrameNumber:%ld, Width:%d, Height:%d\n", stFrameInfo.nFrameID, stFrameInfo.nWidth, stFrameInfo.nHeight);

            if ((stFrameInfo.pBuffer) && (0 < stFrameInfo.nFilledSize) && (SAVE_IMAGE_NUM > nSaveNum))
            {
                MV_FG_PIXEL_TYPE enDstPixelType = MV_FG_PIXEL_TYPE_Undefined;
                unsigned int nChannelNum = 0;
                char szFileName[FILE_NAME_LEN] = { 0 };
                FILE* pImageFile = NULL;

				time_t rawtime; 
				struct tm  *timeinfo; 
				time ( &rawtime ); 
				timeinfo = localtime(&rawtime);

                //如果是彩色则转成RGB8
                if (IsColorPixelFormat(stFrameInfo.enPixelType))
                {
                    nChannelNum = 3;
                    enDstPixelType = MV_FG_PIXEL_TYPE_RGB8_Packed;
                    sprintf(szFileName, "After%d_%04d%02d%02d%02d%02d%02d.rgb", ++nSaveNum, timeinfo->tm_year + 1900, timeinfo->tm_mon + 1,
						timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
                }
                //如果是黑白则转换成Mono8
                else if (IsMonoPixelFormat(stFrameInfo.enPixelType))
                {
                    nChannelNum = 1;
                    enDstPixelType = MV_FG_PIXEL_TYPE_Mono8;
                    sprintf(szFileName, "After%d_%04d%02d%02d%02d%02d%02d.gray", ++nSaveNum, timeinfo->tm_year + 1900, timeinfo->tm_mon + 1,
						timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
                }
                else
                {
                    printf("Don't need to convert!\n");
                }

                if (enDstPixelType != MV_FG_PIXEL_TYPE_Undefined)
                {

                    stConvertPixelInfo.stInputImageInfo.pImageBuf = (unsigned char*)stFrameInfo.pBuffer;
                    stConvertPixelInfo.stInputImageInfo.nImageBufLen = stFrameInfo.nFilledSize;
                    stConvertPixelInfo.stInputImageInfo.nHeight = stFrameInfo.nHeight;
                    stConvertPixelInfo.stInputImageInfo.nWidth = stFrameInfo.nWidth;
                    stConvertPixelInfo.stInputImageInfo.enPixelType = stFrameInfo.enPixelType;
                    
                    unsigned int nSize = stFrameInfo.nHeight * stFrameInfo.nWidth * nChannelNum;
                    if (nSize > stConvertPixelInfo.stOutputImageInfo.nImageBufSize)
                    {
                        if (stConvertPixelInfo.stOutputImageInfo.pImageBuf)
                        {
                            free(stConvertPixelInfo.stOutputImageInfo.pImageBuf);
                            stConvertPixelInfo.stOutputImageInfo.pImageBuf = NULL;
                        }
                        stConvertPixelInfo.stOutputImageInfo.pImageBuf = (unsigned char*)malloc(nSize);
                        if (NULL == stConvertPixelInfo.stOutputImageInfo.pImageBuf)
                        {
                            printf("malloc pConvertData fail!\n");
                            nRet = MV_FG_ERR_RESOURCE_EXHAUSTED;
                            break;
                        }
                        stConvertPixelInfo.stOutputImageInfo.nImageBufSize = nSize;
                    }
                    
                    stConvertPixelInfo.stOutputImageInfo.nImageBufLen = 0;
                    stConvertPixelInfo.stOutputImageInfo.enPixelType = enDstPixelType;
                    stConvertPixelInfo.enCfaMethod = MV_FG_CFA_METHOD_OPTIMAL;

                    nRet = MV_FG_ConvertPixelType(hStream, &stConvertPixelInfo);
                    if (MV_FG_SUCCESS == nRet)
                    {
                        if (NULL != (pImageFile = fopen(szFileName, "wb")))
                        {
                            fwrite(stConvertPixelInfo.stOutputImageInfo.pImageBuf, 1, stConvertPixelInfo.stOutputImageInfo.nImageBufLen, pImageFile);
                            fclose(pImageFile);
                            printf("Convert pixeltype succeed\n");
                        }
                    }
                    else
                    {
                        printf("Convert Pixel Type fail! nRet [0x%x]\n", nRet);
                    }
                }
            }

            // ch:将缓存放回输入队列 | en:Put the buffer back into the input queue
            nRet = MV_FG_ReleaseFrameBuffer(hStream, &stFrameInfo);
            if (MV_FG_SUCCESS != nRet)
            {
                printf("Release frame buffer failed! %#x\n", nRet);
                break;
            }
        }

        if (stConvertPixelInfo.stOutputImageInfo.pImageBuf)
        {
            free(stConvertPixelInfo.stOutputImageInfo.pImageBuf);
            stConvertPixelInfo.stOutputImageInfo.pImageBuf = NULL;
        }

        // ch:停止取流 | en:Stop Acquisition
        nRet = MV_FG_StopAcquisition(hStream);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Stop acquisition failed! %#x\n", nRet);
            return NULL;
        }
    }
	return 0;
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

        // ch:以指定权限打开采集卡，获得采集卡句柄 | en:Open interface with specified permissions, get handle
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

        // ch:创建取流线程 | en:Create acquistion thread
		pthread_t nThreadID;
        nRet = pthread_create(&nThreadID, NULL ,GrabbingThread, hStream);
        if (nRet != 0)
        {
            printf("thread create failed.ret = %d\n",nRet);
            break;
        }

        printf("Press Enter to stop acquisition.\n");
        PressEnterToExit();

        // ch:关闭取流线程 | en:Close acquistion thread
        g_bExit = true;

        // ch:等待取流线程结束 | en:Join acquistion thread
        nRet = pthread_join(nThreadID, NULL);
        if (nRet != 0)
        {
            printf("thread join failed.ret = %d\n",nRet);
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
