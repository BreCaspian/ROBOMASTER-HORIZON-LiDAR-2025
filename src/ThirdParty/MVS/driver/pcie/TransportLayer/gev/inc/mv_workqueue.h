#ifndef     __MV_WORKQUEUE_H__
#define     __MV_WORKQUEUE_H__

#ifdef __cplusplus
extern "C"
{
#endif 

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

extern void MV_WorkInit(void* pWorkQueue, void (*_func) (struct work_struct* pstWorkStruct));

extern void MV_WorkSchedule(void* pWorkQueue);

extern void MV_WorkFree(void* pWorkQueue);

extern void MV_TaskletInit(void* pTasklet, void (*_func) (unsigned long data), unsigned long nData);

extern void MV_TaskletSchedule(void* pTasklet);

extern void MV_TaskletKill(void* pTasklet);

#ifdef __cplusplus
}
#endif 

#endif  // __MV_WORKQUEUE_H__
