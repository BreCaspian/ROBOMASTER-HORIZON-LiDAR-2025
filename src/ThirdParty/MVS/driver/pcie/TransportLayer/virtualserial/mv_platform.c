#include "linux/kernel.h"
#include <asm-generic/errno.h>
#include <asm-generic/errno-base.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include "linux/platform_device.h"

#include "mv_platform.h"
#include "MvErrorDefine.h"

void PlatformDeviceRelease(struct device *dev)
{

}

extern int MV_InitPlatformDevice(PMV_PLATFORM_DEVICE pstPlatformDeviceInfo)
{
    struct platform_device*         pstPlatformDevice = NULL;

    if (NULL == pstPlatformDeviceInfo)
    {
        return -ENAVAIL;
    }
    
    if (pstPlatformDeviceInfo->pPlatformDevice)
    {
        return -ENAVAIL;
    }
    
    pstPlatformDeviceInfo->pPlatformDevice = kzalloc(sizeof(struct platform_device), GFP_KERNEL);

    if (NULL == pstPlatformDeviceInfo->pPlatformDevice)
    {
        return -ENOMEM;
    }

    pstPlatformDevice = pstPlatformDeviceInfo->pPlatformDevice;
    pstPlatformDevice->id = 110;
    pstPlatformDevice->dev.release = PlatformDeviceRelease;

    pstPlatformDeviceInfo->bRegister = false;

    return MV_OK;
}

extern int MV_DeInitPlatformDevice(PMV_PLATFORM_DEVICE pstPlatformDeviceInfo)
{
    struct platform_device*         pstPlatformDevice = NULL;

    if (NULL == pstPlatformDeviceInfo)
    {
        return -ENAVAIL;
    }
    
    if (NULL == pstPlatformDeviceInfo->pPlatformDevice)
    {
        return -ENAVAIL;
    }
    pstPlatformDevice = pstPlatformDeviceInfo->pPlatformDevice;
    if (pstPlatformDevice->resource)
    {
        kfree(pstPlatformDevice->resource);
        pstPlatformDevice->resource = NULL;
    }
    

    kfree(pstPlatformDeviceInfo->pPlatformDevice);
    pstPlatformDeviceInfo->pPlatformDevice = NULL;

    return MV_OK;
}

extern int MV_RegisterPlatformDevice(PMV_PLATFORM_DEVICE pstPlatformDevice)
{
    int                     nRet = MV_OK;

    if (NULL == pstPlatformDevice)
    {
        return -ENAVAIL;
    }
    
    if (NULL == pstPlatformDevice->pPlatformDevice)
    {
        return -ENAVAIL;
    }

    nRet = platform_device_register(pstPlatformDevice->pPlatformDevice);
    if (MV_OK == nRet)
    {
        pstPlatformDevice->bRegister = true;
    }
    
    return nRet;
}

extern int MV_UnRegisterPlatformDevice(PMV_PLATFORM_DEVICE pstPlatformDevice)
{

    if (NULL == pstPlatformDevice)
    {
        return -ENAVAIL;
    }
    
    if (NULL == pstPlatformDevice->pPlatformDevice)
    {
        return -ENAVAIL;
    }

    platform_device_unregister(pstPlatformDevice->pPlatformDevice);
    pstPlatformDevice->bRegister = false;
    
    return MV_OK;
}