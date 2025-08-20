

#ifndef __MV_DEVICE_H__
#define __MV_DEVICE_H__

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/signal.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/interrupt.h> 
#include <asm/uaccess.h> 
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/version.h>
#include "linux/interrupt.h"
#include "linux/dmaengine.h"
#include "linux/mmzone.h"

typedef struct work_struct MV_WORK_STRUCT;

typedef struct tasklet_struct MV_TASKLET;

typedef struct _MV_DEV_T_
{
    unsigned int devt;
}MV_DEV_T;

extern int MV_ClassCreate(void** ppOutClass, const char* name);

// extern int MV_DeviceCreate(void** ppOutDevice, void** ppInPutClass, const void* pDeviceParent, const void* pDev, const char *fmt, ...);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // __MV_DEVICE_H__