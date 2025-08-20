#ifndef __PLATFORM_H__
#define __PLATFORM_H__

// 平台设备
typedef struct _MV_PLATFORM_DEVICE_
{
    void*           pPlatformDevice;
    bool            bRegister;
}MV_PLATFORM_DEVICE, *PMV_PLATFORM_DEVICE;

// 平台设备
typedef struct _MV_PLATFORM_DEVICE_RESOURCE_
{
    const char*         name;
    unsigned int        nNum;
    unsigned long       startMem;
    unsigned long       endMem;
    unsigned long       flagsMem;
    unsigned long       startIrq;
    unsigned long       endIrq;
    unsigned long       flagsIrq;
}MV_PLATFORM_DEVICE_RESOURCE, *PMV_PLATFORM_DEVICE_RESOURCE;

extern int MV_InitPlatformDevice(PMV_PLATFORM_DEVICE pstPlatformDeviceInfo);

extern int MV_DeInitPlatformDevice(PMV_PLATFORM_DEVICE pstPlatformDeviceInfo);

extern int MV_RegisterPlatformDevice(PMV_PLATFORM_DEVICE pstPlatformDevice);

extern int MV_UnRegisterPlatformDevice(PMV_PLATFORM_DEVICE pstPlatformDevice);

#endif	//__PLATFORM_H__