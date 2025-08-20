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

// sg dma read dma-name
#include "linux/of.h"

#include <asm/current.h>
#include <asm/page.h>
#include <linux/mm.h>
#include <linux/page-flags.h>
#include <linux/pagemap.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/gfp.h>

// typedef void* (*MV_Func)             (struct work_struct* pstWorkStruct);

void MV_WorkInit(void* pWorkQueue, void (*_func) (struct work_struct* pstWorkStruct))
{
    struct work_struct*             pstWork = NULL;
    if (NULL == pWorkQueue)
    {
        return;
    }
    
    pstWork = (struct work_struct*)pWorkQueue;
    INIT_WORK(pstWork, _func);
}

void MV_WorkSchedule(void* pWorkQueue)
{
    struct work_struct*             pstWork = NULL;
    if (NULL == pWorkQueue)
    {
        return;
    }
    

    pstWork = (struct work_struct*)pWorkQueue;
    
    schedule_work(pstWork);
}

void MV_WorkFree(void* pWorkQueue)
{
    struct work_struct*             pstWork = NULL;
    if (NULL == pWorkQueue)
    {
        return;
    }
    pstWork = (struct work_struct*)pWorkQueue;

    flush_work(pstWork);
}

//typedef void (*MV_TaskletFunc)             (struct tasklet_struct* pstTaskletStruct);

void MV_TaskletInit(void* pTasklet, void (*_func) (unsigned long data), unsigned long nData)
{
    struct tasklet_struct*          pstTasklet = NULL;
    if (NULL == pTasklet)
    {
        return;
    }
    pstTasklet = (struct tasklet_struct*)pTasklet;
    tasklet_init(pstTasklet, _func, nData);
}

void MV_TaskletSchedule(void* pTasklet)
{

    struct tasklet_struct*          pstTasklet = NULL;
    if (NULL == pTasklet)
    {
        return;
    }
    pstTasklet = (struct tasklet_struct*)pTasklet;
    tasklet_schedule(pstTasklet);
}

void MV_TaskletKill(void* pTasklet)
{
    struct tasklet_struct* pstTasklet = NULL;
    if (NULL == pTasklet)
    {
        return;
    }
    pstTasklet = (struct tasklet_struct*)pTasklet;
    tasklet_kill(pstTasklet);
}