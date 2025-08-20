#include "linux/kernel.h"
#include <asm-generic/errno.h>
#include <asm-generic/errno-base.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include "linux/timer.h"
#include "linux/jiffies.h"

#include "mv_timer.h"
#include "MvErrorDefine.h"

#define CML_PORT_STATUS_CHECK_PERIOD    1000            // ms


extern int MV_InitTimerList(MV_TIMER_LIST* timer, void (*function)(struct timer_list*), void* pData)
{
    struct timer_list*          pstTimer = NULL;
    if (NULL == timer)
    {
        return -EINVAL;
    }

    timer->pTimerList = kmalloc(sizeof(struct timer_list) + sizeof(pData), GFP_KERNEL);

    if (NULL == timer->pTimerList)
    {
        return -ENOMEM;
    }

    pstTimer = (struct timer_list*)timer->pTimerList;

    memset(timer->pTimerList, 0, sizeof(struct timer_list) + sizeof(pData));

    *(void**)((char*)pstTimer + sizeof(struct timer_list)) = pData;

    pstTimer->expires = jiffies + msecs_to_jiffies(CML_PORT_STATUS_CHECK_PERIOD);
    timer_setup(timer->pTimerList, function, 0);

    return MV_OK;
}


extern int MV_AddTimerList(MV_TIMER_LIST* timer)
{
    struct timer_list*          pstTimer = NULL;
    if (NULL == timer || NULL == timer->pTimerList)
    {
        return -EINVAL;
    }

    pstTimer = (struct timer_list*)timer->pTimerList;

    add_timer(pstTimer);

    return MV_OK;
}

extern int MV_DelTimerList(MV_TIMER_LIST* timer)
{
    struct timer_list*          pstTimer = NULL;
    if (NULL == timer || NULL == timer->pTimerList)
    {
        return -EINVAL;
    }

    pstTimer = (struct timer_list*)timer->pTimerList;

    del_timer_sync(pstTimer);

    if (pstTimer)
    {
        kfree(pstTimer);
        pstTimer = NULL;
    }
    

    return MV_OK;
}

extern int MV_ModTimerList(MV_TIMER_LIST* timer)
{
    return MV_OK;
}

extern int MV_GetTimerSize(MV_TIMER_LIST* timer)
{
    return sizeof(struct timer_list);
}

extern int MV_SetTimerJiffies(MV_TIMER_LIST* timer)
{
    struct timer_list*          pstTimer = NULL;
    if (NULL == timer || NULL == timer->pTimerList)
    {
        return -EINVAL;
    }

    pstTimer = (struct timer_list*)timer->pTimerList;

    pstTimer->expires = jiffies + msecs_to_jiffies(CML_PORT_STATUS_CHECK_PERIOD);
    return MV_OK;
}