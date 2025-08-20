#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "MVFGControl.h"

#define BUFFER_NUMBER       3       // ch:申请的缓存个数 | en:Number of requested buffer

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
    char            szFeatureFile[256] = {0};

    do
    {
        // ch:枚举采集卡 | en:Enum interface
        bool bChanged = false;
		/* ch:提供一份由GEV采集卡导出的配置文件用于演示 OpenInterfaceWithConfig功能，
			  用户要使用其它类型的卡可自行打开以下枚举类型并导出对应的配置文件
			  [注]配置文件导入时会对DeviceFirmwareVersion节点做校验，推荐用户用自己导出的配置文件替换Demo.hcf
		   en:Provide a configuration file exported by the GEV FramGrabber card to demonstrate the OpenInterfaceWithConfig function,
		      If users want to use other types of cards, they can open the following enumeration types themselves and export the corresponding configuration files
			  [Note] When importing the configuration file, the DeviceFirmwareVersion node will be verified. It is recommended that users replace Demo.hcf with their own exported configuration file
		*/        
        nRet = MV_FG_UpdateInterfaceList(MV_FG_GEV_INTERFACE /* | MV_FG_CXP_INTERFACE | MV_FG_CAMERALINK_INTERFACE | MV_FG_XoF_INTERFACE */, &bChanged);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Update interface list failed! %#x\r\n", nRet);
            break;
        }

        // ch:获取采集卡数量 | en:Get interface num
        unsigned int nInterfaceNum = 0;
        nRet = MV_FG_GetNumInterfaces(&nInterfaceNum);
        if (MV_FG_SUCCESS != nRet || 0 == nInterfaceNum)
        {
            printf("No interface found! return = %d, number = %d\r\n", nRet, nInterfaceNum);
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
            printf("Invalid interface index.\nQuit.\r\n");
            break;
        }

        printf("******\r\n");
        printf("\t\t Chose Method To Open Interface\r\n");
        printf("\t\t 0. Open Interface With Demo Config\r\n");
		printf("\t\t 1. Open Interface\r\n");
		printf("\t\t other. Open Interface default\r\n");
        printf("******\r\n");
        int nMethod = -1;
		int bOpenWithConfig = false;
		const char* pFileName = szFeatureFile;        
		scanf("%d", &nMethod);

        switch (nMethod)
        {
        case 0:
			// ch:以指定权限打开采集卡，获得采集卡句柄 | en:Open interface with specified permissions, get handle
            nRet = MV_FG_OpenInterfaceWithConfig((unsigned int)nInterfaceIndex, "Demo.hcf", &hInterface);
            bOpenWithConfig = true;
			break;
		case 1:
        default:
			// ch:打开采集卡，获得采集卡句柄 | en:Open interface, get handle
            nRet = MV_FG_OpenInterface((unsigned int)nInterfaceIndex, &hInterface);
            break;
        }
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Open No.%d interface failed! %#x\r\n", nInterfaceIndex, nRet);
            break;
        }

		if(!bOpenWithConfig)
		{
			sprintf(szFeatureFile, "Interface%d.hcf", nInterfaceIndex);
			printf("\t\tInterface FeatureSave Start\r\n");

			// ch:对采集卡执行FeatureSave | en: Execute FeatureSave on the Interface
			nRet = MV_FG_FeatureSave(hInterface, pFileName);
			if (MV_FG_SUCCESS != nRet)
			{
				printf("Feature Save interface failed! %#x\r\n", nRet);
				break;
			}

			printf("\t\tInterface FeatureSave Success\r\n");
			printf("\t\tInterface FeatureLoad Start\r\n");

			// ch:对采集卡执行FeatureLoad | en: Execute FeatureLoad on the Interface
			nRet = MV_FG_FeatureLoad(hInterface, pFileName);
			if (MV_FG_SUCCESS != nRet)
			{
				printf("Feature Load Interface method failed! %#x\r\n", nRet);
				break;
			}
			printf("\t\tInterface FeatureLoad Success, UpdateDeviceList\r\n");

		}
        
        // ch:枚举采集卡上的相机 | en:Enum camera of interface
        nRet = MV_FG_UpdateDeviceList(hInterface, &bChanged);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Update device list failed! %#x\r\n", nRet);
            break;
        }

        // ch:获取设备数量 | en:Get device number
        unsigned int nDeviceNum = 0;
        nRet = MV_FG_GetNumDevices(hInterface, &nDeviceNum);
        if (MV_FG_SUCCESS != nRet || 0 == nDeviceNum)
        {
            printf("No device found! return = %d, number = %d\r\n", nRet, nDeviceNum);
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
            printf("Invalid device index.\nQuit.\r\n");
            break;
        }

        // ch:打开设备，获得设备句柄 | en:Open device, get handle
        nRet = MV_FG_OpenDevice(hInterface, (unsigned int)nDeviceIndex, &hDevice);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Open No.%d device failed! %#x\r\n", nDeviceIndex, nRet);
            hDevice = NULL;
            break;
        }

		sprintf(szFeatureFile, "Device%d.hcf", nDeviceIndex);
		printf("\t\tDevice FeatureSave Start\r\n");

		// ch:对相机执行FeatureSave | en: Execute FeatureSave on the Device
        nRet = MV_FG_FeatureSave(hDevice, pFileName); // [IN] 配置文件文件名
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Feature Save Device failed! %#x\r\n", nRet);
            break;
        }
		printf("\t\tDevice FeatureSave success\r\n");
		printf("\t\tDevice FeatureLoad Start\r\n");

		// ch:对相机执行 FeatureLoad | en: Execute FeatureLoad on the Device
		nRet = MV_FG_FeatureLoad(hDevice, pFileName);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Feature Load Device failed! %#x\r\n", nRet);
            break;
        }
		printf("\t\tDevice FeatureLoad success\r\n");


    } while (0);


    // ch:关闭设备 | en:Close device
    if (NULL != hDevice)
    {
        nRet = MV_FG_CloseDevice(hDevice);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Close device failed! %#x\r\n", nRet);
        }
        hDevice = NULL;
    }

    // ch:关闭采集卡 | en:Close interface
    if (NULL != hInterface)
    {
        nRet = MV_FG_CloseInterface(hInterface);
        if (MV_FG_SUCCESS != nRet)
        {
            printf("Close interface failed! %#x\r\n", nRet);
        }
        hInterface = NULL;
    }

    printf("Press Enter to exit.\r\n");
    PressEnterToExit();

    return 0;
}
